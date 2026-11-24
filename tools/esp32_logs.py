"""Download and decode the ESP32's engine-data logs.

    .venv/bin/python tools/esp32_logs.py list              # files on the board and space used
    .venv/bin/python tools/esp32_logs.py download          # copy new/changed files to logs/esp32/
    .venv/bin/python tools/esp32_logs.py decode            # turn downloaded files into CSV
    .venv/bin/python tools/esp32_logs.py clear             # delete all logs on the board (asks)

Download runs at 115200 baud (about 8 KB/s of log data), so a full 2 MB log takes ~4-5 min.
Decoded CSVs (one per session) land next to the .bin files with readable units.
"""
import argparse
import base64
import binascii
import csv
import datetime as dt
import pathlib
import struct
import sys
import time

from esp32_bridge import Bridge, find_port

HERE = pathlib.Path(__file__).parent        # tools/
OUT = HERE.parent / "logs" / "esp32"

def trim(a):
    return round((a - 128) * 100 / 128, 2)


def c_to_f(c):
    return round(c * 9 / 5 + 32, 1)


# Standard SAE J1979 mode 01 formulas: pid -> (column name, function of (A, B)).
# The log header says which PIDs a file contains, so this is just a lookup table.
DECODE = {
    0x04: ("load_pct", lambda a, b: round(a * 100 / 255, 1)),
    0x05: ("coolant_f", lambda a, b: c_to_f(a - 40)),
    0x06: ("stft_b1_pct", lambda a, b: trim(a)),
    0x07: ("ltft_b1_pct", lambda a, b: trim(a)),
    0x08: ("stft_b2_pct", lambda a, b: trim(a)),
    0x09: ("ltft_b2_pct", lambda a, b: trim(a)),
    0x0B: ("map_kpa", lambda a, b: a),
    0x0C: ("rpm", lambda a, b: (a * 256 + b) / 4),
    0x0D: ("speed_mph", lambda a, b: round(a * 0.621371, 1)),
    0x0E: ("timing_deg", lambda a, b: a / 2 - 64),
    0x0F: ("intake_f", lambda a, b: c_to_f(a - 40)),
    0x10: ("maf_gps", lambda a, b: (a * 256 + b) / 100),
    0x11: ("throttle_pct", lambda a, b: round(a * 100 / 255, 1)),
    0x14: ("o2_b1s1_v", lambda a, b: a / 200),
    0x15: ("o2_b1s2_v", lambda a, b: a / 200),
    0x18: ("o2_b2s1_v", lambda a, b: a / 200),
    0x23: ("fuel_rail_kpa", lambda a, b: (a * 256 + b) * 10),
    0x24: ("o2_s1_lambda", lambda a, b: round((a * 256 + b) * 2 / 65536, 3)),
    0x28: ("o2_s5_lambda", lambda a, b: round((a * 256 + b) * 2 / 65536, 3)),
    0x2C: ("egr_cmd_pct", lambda a, b: round(a * 100 / 255, 1)),
    0x2F: ("fuel_level_pct", lambda a, b: round(a * 100 / 255, 1)),
    0x31: ("miles_since_clear", lambda a, b: round((a * 256 + b) * 0.621371)),
    0x33: ("baro_kpa", lambda a, b: a),
    0x3C: ("cat_temp_b1s1_f", lambda a, b: c_to_f((a * 256 + b) / 10 - 40)),
    0x42: ("module_volts", lambda a, b: (a * 256 + b) / 1000),
    0x44: ("cmd_lambda", lambda a, b: round((a * 256 + b) * 2 / 65536, 3)),
    0x46: ("ambient_f", lambda a, b: c_to_f(a - 40)),
    0x49: ("pedal_pct", lambda a, b: round(a * 100 / 255, 1)),
    0x5C: ("oil_temp_f", lambda a, b: c_to_f(a - 40)),
    0x5E: ("fuel_rate_gph", lambda a, b: round((a * 256 + b) / 20 * 0.264172, 2)),
    0x61: ("demand_torque_pct", lambda a, b: a - 125),
    0x62: ("actual_torque_pct", lambda a, b: a - 125),
}
PROTOCOLS = {"1": "SAE J1850 PWM", "2": "SAE J1850 VPW", "3": "ISO 9141-2",
             "4": "ISO 14230-4 KWP (5 baud)", "5": "ISO 14230-4 KWP (fast)",
             "6": "CAN 11-bit 500k", "7": "CAN 29-bit 500k", "8": "CAN 11-bit 250k",
             "9": "CAN 29-bit 250k", "A": "SAE J1939 CAN"}
EVENTS = {1: "engine off / no data", 2: "engine running", 3: "Mac took over the reader"}


def open_board():
    b = Bridge(find_port())
    b.on_status = lambda msg: None
    b.wait_for("MODE", 3)
    return b


def collect(b, cmd, end_word, timeout=45):
    """Send a command and gather status lines until one starts with end_word.
    The timeout is long because commands can queue behind a search step on the board."""
    lines = []
    b.on_status = lines.append
    b.command(cmd)
    t0 = time.time()
    while time.time() - t0 < timeout and not any(l.startswith(end_word) for l in lines):
        b.pump()
    return lines


def drain(b, quiet=1.0, cap=10.0):
    """Discard board output until it has been quiet for `quiet` seconds."""
    got = {"t": time.time()}
    b.on_status = lambda msg: got.update(t=time.time())
    t0 = time.time()
    while time.time() - got["t"] < quiet and time.time() - t0 < cap:
        if b.pump():
            got["t"] = time.time()


def cmd_list(_):
    b = open_board()
    lines = collect(b, "#LOGLIST", "LOGLIST done")
    for l in lines:
        if l.startswith(("LOGFILE", "LOGLIST", "ERR")):
            print(l)
    if not any(l.startswith(("LOGLIST done", "ERR")) for l in lines):
        print("No answer from the board (busy searching?). Try again.")


def cmd_download(_):
    OUT.mkdir(parents=True, exist_ok=True)
    b = open_board()
    lines = collect(b, "#LOGLIST", "LOGLIST done")
    if not any(l.startswith("LOGLIST done") for l in lines):
        sys.exit("No answer to the file list from the board (busy searching?). Try again.")
    files = [l.split()[1:3] for l in lines if l.startswith("LOGFILE")]
    if not files:
        print("No log files on the board.")
    for name, size in files:
        dest = OUT / name
        if dest.exists() and dest.stat().st_size == int(size):
            print(f"  {name}: already downloaded")
            continue
        data, state = bytearray(), {"begun": False, "end": None}

        def on(msg):
            if msg.startswith("LOGBEGIN "):
                state["begun"] = msg.split()[1] == name
            elif msg.startswith("LOGDATA ") and state["begun"]:
                try:
                    data.extend(base64.b64decode(msg[8:], validate=True))
                except (binascii.Error, ValueError):
                    state["end"] = "ERR damaged chunk"
            elif msg.startswith("LOGEND ") and state["begun"] and msg.split()[1] == name:
                state["end"] = state["end"] or msg
            elif msg.startswith(("ERR no such log file", "ERR bad file name",
                                 "ERR log storage unavailable")):
                state["end"] = msg
        b.on_status = on
        b.command("#LOGGET " + name)
        t0 = time.time()
        # ~8 KB/s of log data at 115200 baud, plus time for a search step to finish.
        while state["end"] is None and time.time() - t0 < 45 + int(size) / 4000:
            b.pump()
        end = state["end"]
        if not end or end.startswith("ERR"):
            print(f"  {name}: FAILED ({end or 'timeout'})")
            drain(b)  # don't let the rest of this file leak into the next one
            continue
        _, _, total, crc = end.split()
        if int(total) != len(data) or f"{binascii.crc32(data):08x}" != crc:
            print(f"  {name}: FAILED checksum, not saved")
            continue
        dest.write_bytes(data)
        print(f"  {name}: {len(data)} bytes OK")


HEADER_MIN = 5 + struct.calcsize("<IHBII") + 1 + 16 + 1


def parse_file(raw):
    """Yield ('header', dict) then ('rec'|'time'|'event', ...) entries."""
    if len(raw) < HEADER_MIN or raw[:4] != b"OBDL":
        raise ValueError("empty, cut off, or not an OBDL log")
    ver = raw[4]
    if ver != 2:
        raise ValueError(f"log format {ver} not supported (reflash or update this script)")
    sess, seg, reader, epoch, start_ms = struct.unpack_from("<IHBII", raw, 5)
    p = 5 + struct.calcsize("<IHBII")
    proto = chr(raw[p]); p += 1
    support = struct.unpack_from("<4I", raw, p); p += 16
    vlen = raw[p]; p += 1
    vin_hash = raw[p:p + vlen].hex(); p += vlen
    npid = raw[p]; p += 1
    if len(raw) < p + 2 * npid + 1:
        raise ValueError("header cut off")
    pids = [raw[p + 2 * i] for i in range(npid)]
    every = [raw[p + 2 * i + 1] for i in range(npid)]
    p += 2 * npid
    dlen = raw[p]; p += 1
    dtcs = raw[p:p + dlen].decode(errors="replace"); p += dlen
    yield "header", dict(version=ver, session=sess, segment=seg, reader=reader, epoch=epoch,
                         start_ms=start_ms, protocol=proto, support=support,
                         vin_hash=vin_hash, pids=pids, every=every, dtcs=dtcs)
    while p < len(raw):
        tag = raw[p]
        if tag == 0xA5 and p + 9 <= len(raw):
            t, mask = struct.unpack_from("<II", raw, p + 1)
            n = bin(mask).count("1")
            if p + 9 + 2 * n > len(raw) or mask >> npid:
                p += 1  # damaged or cut off by power loss: resync
                continue
            vals, q = {}, p + 9
            for i in range(npid):
                if mask >> i & 1:
                    vals[pids[i]] = (raw[q], raw[q + 1]); q += 2
            yield "rec", (t, vals)
            p = q
        elif tag == 0xA6 and p + 9 <= len(raw):
            t, unix = struct.unpack_from("<II", raw, p + 1)
            yield "time", (t, unix); p += 9
        elif tag == 0xA7 and p + 6 <= len(raw):
            t, ev = struct.unpack_from("<IB", raw, p + 1)
            yield "event", (t, ev); p += 6
        else:
            p += 1  # resync after a damaged byte


def cmd_decode(_):
    files = sorted(OUT.glob("s*_*.bin"))
    if not files:
        sys.exit(f"No downloaded logs in {OUT}. Run: esp32_logs.py download")
    sessions = {}
    for f in files:
        sessions.setdefault(f.name.split("_")[0], []).append(f)
    for sess, parts in sessions.items():
        rows, anchor, header = [], None, None
        for f in sorted(parts, key=lambda f: int(f.stem.split("_")[1])):
            try:
                entries = list(parse_file(f.read_bytes()))
            except (ValueError, IndexError, struct.error) as e:
                print(f"  skipped damaged file {f.name}: {e}")
                continue
            for kind, item in entries:
                if kind == "header":
                    header = header or item
                    if item["epoch"] and anchor is None:
                        anchor = (0, item["epoch"])
                elif kind == "time":
                    anchor = (item[0] / 1000, item[1])
                elif kind == "event":
                    rows.append({"t_s": item[0] / 1000, "event": EVENTS.get(item[1], item[1])})
                else:
                    t, vals = item
                    row = {"t_s": t / 1000}
                    for pid, (a, b2) in vals.items():
                        name, fn = DECODE.get(pid, (f"pid_{pid:02x}_raw", lambda x, y: x * 256 + y))
                        row[name] = fn(a, b2)
                    rows.append(row)
        if header is None:
            continue
        if anchor:
            for r in rows:
                r["time"] = dt.datetime.fromtimestamp(anchor[1] + r["t_s"] - anchor[0]).isoformat(timespec="seconds")
        names = [DECODE.get(p, (f"pid_{p:02x}_raw",))[0] for p in header["pids"]]
        cols = ["time"] * bool(anchor) + ["t_s"] + names + ["event"]
        cols = [c for c in cols if any(c in r for r in rows)]
        out = OUT / f"{sess}.csv"
        with open(out, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=cols)
            w.writeheader()
            w.writerows(rows)
        recs = sum("event" not in r for r in rows)
        span = rows[-1]["t_s"] if rows else 0
        car = (f"VIN sha256:{header['vin_hash']}" if header["vin_hash"] else "VIN not reported")
        proto = PROTOCOLS.get(header["protocol"], header["protocol"])
        print(f"  {out.name}: {recs} samples over {span / 60:.1f} min | {proto} | {car} | "
              f"stored codes: {header['dtcs'] or 'none'}"
              + ("" if anchor else " | no real-time anchor"))


def cmd_clear(_):
    if input("Delete ALL logs on the ESP32? Type DELETE to confirm: ").strip() != "DELETE":
        print("Not deleted.")
        return
    b = open_board()
    lines = collect(b, "#LOGCLEAR", "OK logs cleared")
    print(next((l for l in lines if l.startswith(("OK logs cleared", "ERR"))),
               "No answer from the board (busy searching?). Nothing confirmed."))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=["list", "download", "decode", "clear"])
    a = ap.parse_args()
    {"list": cmd_list, "download": cmd_download, "decode": cmd_decode, "clear": cmd_clear}[a.action](a)


if __name__ == "__main__":
    main()
