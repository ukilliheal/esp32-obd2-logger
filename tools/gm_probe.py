"""Read-only probe of a 1996-2007 GM truck/car on J1850 VPW (GM "Class 2").

Generic OBD2 only talks to the engine computer. GM's own Class 2 messages can
reach the other modules (ABS, airbag, body, cluster, radio, HVAC). This script
sends ONLY read requests:
    mode $09  - standard vehicle info (VIN, calibration IDs)
    mode $1A  - GM read-data-by-identifier (VIN, part numbers, etc.)
    mode $19  - GM read trouble codes (per module)
It never sends clear, write, security or programming commands.

    .venv/bin/python tools/gm_probe.py [HOST:PORT]
"""
import hashlib
import socket
import sys
import time

target = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.10:35000"
host, port = target.split(":")
s = socket.create_connection((host, int(port)), timeout=10)

MODULES = {
    "10": "Engine/transmission (PCM)",
    "28": "ABS / brakes (EBCM)",
    "40": "Body control (BCM)",
    "58": "Airbag (SDM)",
    "60": "Instrument cluster (IPC)",
    "80": "Radio",
    "99": "HVAC",
    "A0": "Transfer case / 4WD",
}
ALLOWED_MODES = {"09", "19", "1A"}  # safety: refuse anything that isn't a read


def cmd(c, wait=4.0, show=True):
    s.sendall(c.encode() + b"\r")
    buf, end = b"", time.time() + wait
    while time.time() < end and not buf.rstrip().endswith(b">"):
        try:
            chunk = s.recv(4096)
            if not chunk:
                break
            buf += chunk
        except socket.timeout:
            break
    out = buf.decode(errors="replace").replace(">", "").strip()
    lines = [ln.strip() for ln in out.splitlines() if ln.strip()]
    if show:
        print(f"  {c:<14} -> {' | '.join(lines) or '(nothing)'}")
    return lines


def obd(req, wait=4.0):
    assert req[:2].upper() in ALLOWED_MODES, f"refusing non-read request {req}"
    return cmd(req, wait)


def ascii_from(lines, skip):
    """Decode ASCII payload bytes from header-on response lines."""
    out = ""
    for ln in lines:
        b = ln.split()
        if len(b) > skip:
            out += "".join(chr(int(x, 16)) for x in b[skip:-1]  # drop checksum
                           if 0x20 <= int(x, 16) < 0x7F)
    return out


for c in ["ATZ", "ATE0", "ATL0", "ATS1", "ATSP2", "ATH1", "ATAT1", "ATST96"]:
    cmd(c, show=False)

print("\n== Standard vehicle info (mode 09, functional request) ==")
cmd("ATSH686AF1", show=False)
obd("0900")
vin = ascii_from(cmd("0902", wait=6, show=False), 6)  # skip header+mode+pid+seq
print(f"  VIN: sha256:{hashlib.sha256(vin[-17:].encode()).hexdigest()[:16]}"
      if len(vin) >= 17 else "  VIN: not reported")
for pid, what in [("04", "Calibration ID"), ("06", "Calibration verification")]:
    print(f"  {what}:")
    obd("09" + pid, wait=6)

print("\n== Which modules answer, and their trouble codes (GM mode $19) ==")
for addr, name in MODULES.items():
    print(f"\n[{addr}] {name}")
    cmd(f"ATSH6C{addr}F1", show=False)
    vin = cmd("1A90", show=False)           # VIN as stored in that module (not printed)
    print("  1A90 (VIN)     ->", "answered" if vin and "NO DATA" not in vin[0] else "NO DATA")
    obd("1AC1")                             # part number (end model)
    obd("19FFFF00", wait=6)                 # all DTCs, any status
s.close()
