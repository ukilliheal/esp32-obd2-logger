"""Read OBD2 diagnostics from a car and save a report.

Connection (pick one):
    --esp32 [auto|wifi|bt|N]  ESP32 bridge plugged in by USB (firmware/esp32_obd_bridge).
                         It connects to whichever listed reader it finds (Wi-Fi first,
                         then Bluetooth). The Mac stays on its normal Wi-Fi.
    --wifi [HOST:PORT]   Wi-Fi ELM327 (default 192.168.0.10:35000). Join the adapter's
                         Wi-Fi network on your Mac first.
    --ble [NAME]         Bluetooth LE ELM327 (optionally part of its name, e.g. vLinker)
    --port PATH          Serial / USB / paired Bluetooth Classic device (/dev/tty.xxx)
    --url URL            Any pyserial URL, e.g. socket://127.0.0.1:35000

Examples:
    .venv/bin/python tools/diagnose.py ranger --ble
    .venv/bin/python tools/diagnose.py avalanche --wifi
    .venv/bin/python tools/diagnose.py --list
    .venv/bin/python tools/diagnose.py camaro --ble --clear-codes   # asks before clearing

Reports are written to reports/<vehicle>_<timestamp>.md and .json
"""
import argparse
import datetime as dt
import hashlib
import json
import pathlib
import subprocess
import sys
import threading
import time

import obd
from obd import OBDStatus
from obd.OBDResponse import StatusTest

from vehicles import VEHICLES

HERE = pathlib.Path(__file__).parent        # tools/
ROOT = HERE.parent                          # repo root
REPORTS = ROOT / "reports"
LIVE_LOGS = ROOT / "logs" / "live"

# Live-data PIDs worth reading on a pre-CAN gas/diesel truck, in report order.
LIVE_PIDS = [
    "RPM", "SPEED", "COOLANT_TEMP", "INTAKE_TEMP", "ENGINE_LOAD", "THROTTLE_POS",
    "MAF", "INTAKE_PRESSURE", "TIMING_ADVANCE", "FUEL_STATUS",
    "SHORT_FUEL_TRIM_1", "LONG_FUEL_TRIM_1", "SHORT_FUEL_TRIM_2", "LONG_FUEL_TRIM_2",
    "O2_B1S1", "O2_B1S2", "O2_B2S1", "O2_B2S2",
    "FUEL_LEVEL", "CONTROL_MODULE_VOLTAGE", "RUN_TIME",
    "DISTANCE_W_MIL", "DISTANCE_SINCE_DTC_CLEAR", "WARMUPS_SINCE_DTC_CLEAR",
]

PROTOCOL_NAMES = {"1": "J1850 PWM", "2": "J1850 VPW", "3": "ISO 9141-2", None: "auto-detect"}


HELPER_TIMEOUT = 90  # seconds the ESP32 may need to find a reader (several search rounds)


def _start_helper(script, extra, port, tag):
    """Run a bridge helper in the background and wait for it to print READY.
    A thread keeps reading its output afterwards, so the helper never blocks on a full pipe."""
    cmd = [sys.executable, "-u", str(HERE / script), "--port", str(port)] + extra
    if script == "esp32_bridge.py":
        cmd += ["--timeout", str(HELPER_TIMEOUT)]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    ready = threading.Event()
    failed = threading.Event()

    def reader():
        for line in proc.stdout:
            if not ready.is_set():
                print(f"  [{tag}]", line.rstrip(), flush=True)
            if line.startswith("READY"):
                ready.set()
            elif line.startswith("ERROR"):
                failed.set()
        failed.set()  # helper exited
    threading.Thread(target=reader, daemon=True).start()
    deadline = time.time() + HELPER_TIMEOUT + 15
    while time.time() < deadline and not ready.is_set() and not failed.is_set():
        time.sleep(0.1)
    if ready.is_set():
        return proc, f"socket://127.0.0.1:{port}"
    proc.terminate()
    sys.exit(f"Could not start the {tag} bridge.")


def start_ble_bridge(name, port=35000):
    return _start_helper("ble_bridge.py", ["--name", name] if name else [], port, "ble")


def start_esp32_bridge(choice, port=35002):
    """choice: "auto"/"wifi"/"bt" or a reader number, a /dev path for the board, or None
    (use the mode saved on the board)."""
    choice = (choice or "").strip()
    if choice.lower() in ("auto", "wifi", "bt") or choice.isdigit():
        extra = ["--mode", choice.lower()]
    elif choice.startswith("/dev/"):
        extra = ["--device", choice]
    elif choice:
        sys.exit(f"--esp32 takes auto, wifi, bt, a reader number, or a /dev/ path, not {choice!r}")
    else:
        extra = []
    return _start_helper("esp32_bridge.py", extra, port, "esp32")


def open_link(args):
    """Turn the --ble/--esp32/--wifi/--port/--url options into (bridge process, url)."""
    if getattr(args, "ble", None) is not None:
        return start_ble_bridge(args.ble or None)
    if getattr(args, "esp32", None) is not None:
        return start_esp32_bridge(args.esp32 or None)
    if getattr(args, "wifi", None):
        return None, "socket://" + args.wifi
    if getattr(args, "port", None):
        return None, args.port
    if getattr(args, "url", None):
        return None, args.url
    sys.exit("Choose a connection: --esp32, --ble, --wifi, --port or --url")


def add_link_options(parser):
    g = parser.add_mutually_exclusive_group()
    g.add_argument("--esp32", nargs="?", const="", metavar="auto|wifi|bt|N",
                   help="ESP32 bridge on USB; it finds a listed reader on its own. "
                        "'wifi', 'bt' or a reader number restricts it (saved on the board)")
    g.add_argument("--wifi", nargs="?", const="192.168.0.10:35000", metavar="HOST:PORT")
    g.add_argument("--ble", nargs="?", const="", metavar="NAME")
    g.add_argument("--port", metavar="PATH")
    g.add_argument("--url")


def connect(url, protocol):
    """Connect, trying the vehicle's known protocol first and auto-detect second."""
    attempts = [protocol, None] if protocol else [None]
    for proto in attempts:
        print(f"Connecting via {url} using {PROTOCOL_NAMES.get(proto, proto)}...")
        # Baud-rate auto-detection fails over TCP sockets (Wi-Fi and the BLE bridge).
        baud = 38400 if url.startswith("socket://") else None
        conn = obd.OBD(url, baudrate=baud, protocol=proto, fast=False, timeout=5,
                       check_voltage=True)
        if conn.status() == OBDStatus.CAR_CONNECTED:
            return conn
        print(f"  -> status: {conn.status()}")
        conn.close()
    status = conn.status()
    hints = {
        OBDStatus.NOT_CONNECTED: "Can't reach the adapter. Check Wi-Fi/Bluetooth/port.",
        OBDStatus.ELM_CONNECTED: "Adapter found but no 12V at the OBD port. Is it fully seated?",
        OBDStatus.OBD_CONNECTED: "Adapter powered but the car didn't answer. Turn the "
                                 "ignition to ON (or start the engine). On the Ford, a clone "
                                 "adapter without J1850 PWM support is a common cause.",
    }
    sys.exit("Connection failed: " + hints.get(status, str(status)))


def q(conn, name):
    cmd = obd.commands[name]
    if not conn.supports(cmd):
        return None
    r = conn.query(cmd)
    return None if r.is_null() else r.value


def fmt(v):
    if v is None:
        return "-"
    if isinstance(v, (bytes, bytearray)):
        return v.decode("ascii", "replace").strip("\x00 ")
    if hasattr(v, "magnitude"):
        units = str(v.units)
        to = {"degree_Celsius": "degF", "kilometer_per_hour": "mph", "kilometer": "mile"}
        if units in to:
            v = v.to(to[units])
        mag = v.magnitude
        return f"{mag:.2f} {v.units:~}" if isinstance(mag, float) else f"{mag} {v.units:~}"
    return str(v)


def vin_hash(vin):
    """VINs are never stored in plain text: keep a short SHA-256 so scans of the same
    vehicle can still be matched. Check a VIN with: printf %s VIN | shasum -a 256"""
    return hashlib.sha256(vin.encode()).hexdigest()[:16]


def read_vin(conn):
    # force=True: on J1850 python-OBD misreads the mode 09 support list and skips VIN.
    r = conn.query(obd.commands.VIN, force=True)
    raw = r.value if isinstance(r.value, (bytes, bytearray)) else \
        b"".join(bytes(m.data) for m in r.messages)
    chars = "".join(chr(b) for b in raw if chr(b).isalnum())
    return chars[-17:] if len(chars) >= 17 else None


def read_all(conn):
    data = {"protocol": conn.protocol_name(), "port": conn.port_name()}
    volt = conn.query(obd.commands.ELM_VOLTAGE)
    data["battery_voltage"] = fmt(volt.value)
    vin = read_vin(conn)
    data["vin"] = f"sha256:{vin_hash(vin)}" if vin else "not reported (normal pre-2005)"

    status = q(conn, "STATUS")
    if status:
        data["mil_on"] = status.MIL
        data["dtc_count"] = status.DTC_count
        data["ignition"] = status.ignition_type
        data["monitors"] = {
            k: ("complete" if t.complete else "INCOMPLETE")
            for k, t in vars(status).items()
            if isinstance(t, StatusTest) and t.available
        }

    def codes(cmd):
        r = conn.query(obd.commands[cmd])
        return [{"code": c, "description": d or "(manufacturer-specific)"} for c, d in (r.value or [])]

    data["stored_codes"] = codes("GET_DTC")
    data["pending_codes"] = codes("GET_CURRENT_DTC")

    freeze_dtc = q(conn, "DTC_FREEZE_DTC") if "DTC_FREEZE_DTC" in obd.commands else None
    if freeze_dtc:
        data["freeze_frame"] = {"triggered_by": fmt(freeze_dtc)}
        for name in LIVE_PIDS:
            key = "DTC_" + name
            if key in obd.commands:
                v = q(conn, key)
                if v is not None:
                    data["freeze_frame"][name] = fmt(v)

    data["live"] = {}
    for name in LIVE_PIDS:
        v = q(conn, name)
        if v is not None:
            data["live"][name] = fmt(v)
    data["supported_pids"] = sorted(c.name for c in conn.supported_commands if c.mode in (1, 9))
    return data


def write_report(key, vehicle, data):
    REPORTS.mkdir(exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    base = REPORTS / f"{key}_{stamp}"
    base.with_suffix(".json").write_text(json.dumps({"vehicle": vehicle["name"], **data},
                                                    indent=2, default=str))
    L = [f"# {vehicle['name']}", f"_Scanned {dt.datetime.now():%Y-%m-%d %H:%M}_", "",
         f"- **VIN:** {data['vin']}", f"- **Protocol:** {data['protocol']}",
         f"- **Battery:** {data['battery_voltage']}"]
    if "mil_on" in data:
        L.append(f"- **Check engine light:** {'ON' if data['mil_on'] else 'off'} "
                 f"({data['dtc_count']} stored code(s))")
    for title, k in [("Stored trouble codes", "stored_codes"), ("Pending codes", "pending_codes")]:
        L += ["", f"## {title}"]
        L += [f"- **{c['code']}** - {c['description']}" for c in data[k]] or ["- none"]
    if data.get("monitors"):
        L += ["", "## Readiness monitors", "| Monitor | Status |", "|---|---|"]
        L += [f"| {k} | {v} |" for k, v in data["monitors"].items()]
    if data.get("freeze_frame"):
        L += ["", "## Freeze frame", "| PID | Value |", "|---|---|"]
        L += [f"| {k} | {v} |" for k, v in data["freeze_frame"].items()]
    L += ["", "## Live data", "| PID | Value |", "|---|---|"]
    L += [f"| {k} | {v} |" for k, v in data["live"].items()]
    L += ["", "## Notes for this vehicle"] + [f"- {n}" for n in vehicle["notes"]]
    md = "\n".join(L) + "\n"
    base.with_suffix(".md").write_text(md)
    return base.with_suffix(".md"), md


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("vehicle", nargs="?", choices=list(VEHICLES))
    ap.add_argument("--list", action="store_true", help="list vehicle profiles")
    add_link_options(ap)
    ap.add_argument("--clear-codes", action="store_true",
                    help="after the report, clear codes (asks for confirmation)")
    ap.add_argument("--debug", action="store_true", help="show raw adapter traffic")
    args = ap.parse_args()

    if args.list or not args.vehicle:
        for k, v in VEHICLES.items():
            print(f"  {k:<10} {v['name']}  [{PROTOCOL_NAMES.get(v['protocol'])}]")
        return
    if args.debug:
        obd.logger.setLevel(obd.logging.DEBUG)

    vehicle = VEHICLES[args.vehicle]
    bridge, url = open_link(args)

    try:
        conn = connect(url, vehicle["protocol"])
        print(f"Connected: {conn.protocol_name()}. Reading data...")
        data = read_all(conn)
        path, md = write_report(args.vehicle, vehicle, data)
        print("\n" + md)
        print(f"Report saved: {path}")

        if args.clear_codes:
            print("\nClearing codes also erases freeze-frame data and resets all readiness "
                  "monitors (the car will fail an emissions test until it's driven enough).")
            if input("Type CLEAR to confirm: ").strip() == "CLEAR":
                conn.query(obd.commands.CLEAR_DTC)
                print("Codes cleared. Re-reading...")
                left = conn.query(obd.commands.GET_DTC).value or []
                print("Remaining codes:", [c for c, _ in left] or "none")
            else:
                print("Not cleared.")
        conn.close()
    finally:
        if bridge:
            bridge.terminate()


if __name__ == "__main__":
    main()
