"""Use the ESP32 OBD bridge (firmware/esp32_obd_bridge) from the Mac.

The ESP32 is plugged in by USB and finds a reader on its own: any listed Wi-Fi reader,
then any listed Bluetooth Classic reader, repeating until one answers. The Mac stays on
its normal network. This helper opens the ESP32's serial port, sends it the time (for
log timestamps), waits until it is connected to a reader, then serves the link at
127.0.0.1:<port> for diagnose.py / log_live.py / probe.py.

    .venv/bin/python tools/esp32_bridge.py                  # wait for a reader, then serve it
    .venv/bin/python tools/esp32_bridge.py --status         # state, reader, logger, memory
    .venv/bin/python tools/esp32_bridge.py --list           # the board's reader list
    .venv/bin/python tools/esp32_bridge.py --bt-scan        # Bluetooth OBD readers nearby
    .venv/bin/python tools/esp32_bridge.py --scan           # Wi-Fi OBD readers nearby
    .venv/bin/python tools/esp32_bridge.py --add-bt "AA:BB:CC:DD:EE:FF,auto"
    .venv/bin/python tools/esp32_bridge.py --add-wifi "SSID,PASSWORD,192.168.0.10,35000"
    .venv/bin/python tools/esp32_bridge.py --delete 2
    .venv/bin/python tools/esp32_bridge.py --mode auto      # or wifi, bt, or a reader number
"""
import argparse
import glob
import re
import select
import socket
import sys
import threading
import time

import serial

STATUS_RE = re.compile(rb"#([^\n]*)\n")
# The ESP32's Bluetooth controller can print internal warnings onto the USB serial line
# ("ASSERT_WARN(1 8), in lc_task.c at line 1556"), corrupting reader replies. The
# firmware silences them; this only detects them if an older firmware is flashed.
NOISE_RE = re.compile(rb"ASSERT_WARN\(\d+ \d+\), in \w+\.c at line \d+")
PARTIAL_MAX_AGE = 0.3   # a '#' fragment older than this (or containing '>') is reader data


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbserial-*") + glob.glob("/dev/cu.SLAB_USBtoUART*")
                   + glob.glob("/dev/cu.wchusbserial*") + glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        sys.exit("ERROR no ESP32 found (no /dev/cu.usbserial-* port). Is it plugged in?")
    return ports[0]


class Bridge:
    def __init__(self, port_path):
        # Open with DTR/RTS off: with them on (pyserial's default) this CP2102 board
        # produced floods of repeated/garbage bytes. exclusive=True stops a second bridge
        # or a log download from opening the port and stealing bytes.
        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = port_path, 115200, 0.05
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.exclusive = True
        try:
            self.ser.open()
        except serial.SerialException as e:
            sys.exit(f"ERROR cannot open {port_path} ({e}). Is another bridge or log "
                     "download using the ESP32? Stop it first.")
        self.pending = b""
        self.partial_since = None
        self.on_status = lambda msg: print(f"ESP32: {msg}", flush=True)

    def pump(self):
        """Read serial, split out '#...' status lines, return ELM data bytes."""
        self.pending += self.ser.read(4096)
        if NOISE_RE.search(self.pending):
            self.on_status("WARNING Bluetooth controller noise on the serial line; "
                           "reflash the latest firmware")
            self.pending = NOISE_RE.sub(b"", self.pending)
        data = b""
        while True:
            m = STATUS_RE.search(self.pending)
            hash_at = self.pending.find(b"#")
            if m and m.start() == hash_at:
                data += self.pending[:m.start()]
                self.on_status(m.group(1).decode(errors="replace").strip())
                self.pending = self.pending[m.end():]
                self.partial_since = None
            elif hash_at >= 0:
                data += self.pending[:hash_at]
                self.pending = self.pending[hash_at:]
                frag = self.pending.split(b"\n", 1)[0]
                if self.partial_since is None:
                    self.partial_since = time.time()
                # Status lines arrive whole within a few ms; anything else is a stray '#'
                # in reader data, so release it rather than holding everything behind it.
                if (b">" in frag or b"\r" in frag or len(frag) > 600
                        or time.time() - self.partial_since > PARTIAL_MAX_AGE):
                    data += self.pending[:1]
                    self.pending = self.pending[1:]
                    self.partial_since = None
                    continue
                break
            else:
                data += self.pending
                self.pending = b""
                self.partial_since = None
                break
        return data

    def command(self, line):
        self.ser.write(line.encode() + b"\n")

    def wait_for(self, word, timeout):
        return self.wait_for_any([word], timeout) is not None

    def wait_for_any(self, words, timeout):
        """Return the first of `words` that starts a status line, or None."""
        hit = {"w": None}
        old = self.on_status

        def watch(msg):
            old(msg)
            for w in words:
                if hit["w"] is None and msg.startswith(w):
                    hit["w"] = w
        self.on_status = watch
        end = time.time() + timeout
        while time.time() < end and hit["w"] is None:
            self.pump()
        self.on_status = old
        return hit["w"]

    def wait_until_linked(self, timeout):
        """Ready when the board reports READY, or a STATUS showing the link up
        (covers the case where it was already connected before we opened the port)."""
        state = {"ok": False}
        old = self.on_status

        def watch(msg):
            old(msg)
            if msg.startswith("READY") or (msg.startswith("STATUS") and
                                           " state=linked" in msg):
                state["ok"] = True
        self.on_status = watch
        end, next_ask = time.time() + timeout, 0
        while time.time() < end and not state["ok"]:
            if time.time() > next_ask:
                self.command("#STATUS")
                next_ask = time.time() + 5
            self.pump()
        self.on_status = old
        return state["ok"]


def serve(bridge, port):
    """Serve the reader to one TCP client at a time. #HOLD/#RELEASE tell the board when a
    program is using the reader, so its logger stays out of the way until it disconnects."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(1)
    print(f"READY socket://127.0.0.1:{port}", flush=True)
    while True:
        # Between clients keep reading the board (status lines; reader bytes are discarded).
        while not select.select([srv], [], [], 0.05)[0]:
            bridge.pump()
        conn, _ = srv.accept()
        conn.settimeout(0.02)
        bridge.command("#HOLD")
        bridge.wait_for("OK held", 3)
        end = time.time() + 0.4          # drop anything left over from before this client
        while time.time() < end:
            bridge.pump()
        alive = True
        while alive:
            out = bridge.pump()
            if out:
                try:
                    conn.sendall(out)
                except OSError:
                    break
            try:
                inp = conn.recv(4096)
                if not inp:
                    alive = False
                else:
                    bridge.ser.write(inp.replace(b"#", b""))  # '#' is reserved for bridge commands
            except socket.timeout:
                pass
            except OSError:
                alive = False
        conn.close()
        bridge.command("#RELEASE")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", help="serial port (default: auto-detect)")
    ap.add_argument("--port", type=int, default=35002, help="local TCP port to serve")
    ap.add_argument("--mode", help="auto (default), wifi, bt, or a reader number from --list; "
                                   "saved on the board")
    ap.add_argument("--status", action="store_true", help="print ESP32 status and exit")
    ap.add_argument("--list", action="store_true", help="print the reader list and exit")
    ap.add_argument("--add-bt", metavar="MAC_OR_NAME[,PIN]",
                    help='add a Bluetooth Classic reader, e.g. "AA:BB:CC:DD:EE:FF,auto"')
    ap.add_argument("--add-wifi", metavar="SSID[,PASS,HOST,PORT]", help="add a Wi-Fi reader")
    ap.add_argument("--delete", metavar="N", help="remove reader N (see --list)")
    ap.add_argument("--scan", action="store_true", help="list Wi-Fi OBD readers nearby and exit")
    ap.add_argument("--bt-scan", action="store_true", help="list Bluetooth OBD readers nearby and exit")
    ap.add_argument("--log", choices=["on", "off"], help="turn the board's data logging on/off")
    ap.add_argument("--timeout", type=float, default=60,
                    help="seconds to wait for the board to find a reader")
    a = ap.parse_args()

    dev = a.device or find_port()
    print(f"Opening ESP32 on {dev}...", flush=True)
    b = Bridge(dev)
    b.wait_for("MODE", 3)  # boot banner, if opening the port reset the board
    b.command(f"#TIME {int(time.time())}")  # lets the board timestamp its logs
    b.wait_for_any(["OK", "ERR"], 3)

    def run(cmd, until, timeout=45):
        b.command(cmd)
        if b.wait_for_any(until, timeout) is None:
            print(f"(no answer to {cmd.split()[0]} within {timeout:.0f} s; the board may be "
                  "busy searching)", flush=True)

    if a.mode:
        run("#MODE " + a.mode, ["OK", "ERR"])
    if a.add_bt:
        run("#ADDBT " + a.add_bt, ["OK", "ERR"])
    if a.add_wifi:
        run("#ADDWIFI " + a.add_wifi, ["OK", "ERR"])
    if a.delete:
        run("#DEL " + a.delete, ["OK", "ERR"])
    if a.log:
        run("#LOG " + a.log, ["OK", "ERR"])
    if a.list or a.add_bt or a.add_wifi or a.delete:
        run("#LIST", ["LIST done"])
    # Commands can queue behind a search step on the board (up to ~40 s).
    if a.scan:
        run("#SCAN", ["SCAN done", "ERR"], 60)
    if a.bt_scan:
        run("#BTSCAN", ["BTSCAN done", "ERR"], 60)
    if a.status:
        run("#STATUS", ["STATUS"], 45)
    if any([a.status, a.list, a.add_bt, a.add_wifi, a.delete, a.scan, a.bt_scan, a.log]):
        return
    if not b.wait_until_linked(a.timeout):
        sys.exit("ERROR the ESP32 could not reach an OBD reader. Is a listed reader plugged "
                 "into a car and in range? Check with --status and --list.")
    serve(b, a.port)


if __name__ == "__main__":
    main()
