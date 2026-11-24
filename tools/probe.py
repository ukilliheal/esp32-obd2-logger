"""Talk to an ELM327 adapter directly and report what it is.

    .venv/bin/python tools/probe.py [HOST:PORT]      default 192.168.0.10:35000
"""
import socket
import sys
import time

if len(sys.argv) > 1 and sys.argv[1] in ("-h", "--help"):
    print(__doc__.strip())
    sys.exit(0)
target = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.10:35000"
if ":" not in target:
    sys.exit(f"Expected HOST:PORT, got {target!r}")
host, port = target.split(":", 1)
s = socket.create_connection((host, int(port)), timeout=10)


def cmd(c, wait=10.0):
    s.sendall(c.encode() + b"\r")
    buf, end = b"", time.time() + wait
    while time.time() < end and not buf.rstrip().endswith(b">"):
        try:
            chunk = s.recv(1024)
            if not chunk:
                break
            buf += chunk
        except socket.timeout:
            break
    out = buf.decode(errors="replace").replace("\r", " ").replace(">", "").strip()
    print(f"{c:<8} -> {out}")
    return out


for c in ["ATZ", "ATE0", "ATL0", "ATS0", "ATI", "AT@1", "STI", "ATRV"]:
    cmd(c)
# Ask the car: protocol auto-detect happens on the first OBD request.
cmd("ATSP0")
cmd("0100", wait=20)
cmd("ATDP")
cmd("ATDPN")
for proto in ["1", "2", "3"]:  # J1850 PWM, J1850 VPW, ISO 9141-2: does the firmware accept them?
    cmd("ATSP" + proto)
cmd("ATSP0")

# --- Clone checks -----------------------------------------------------------
print("\nClone checks:")
version = cmd("ATI")
amc = cmd("ATAMC")          # real v2.0+ returns a hex count, clones often just say OK
cmd("0100", wait=20)        # make sure a protocol is active before monitoring
s.sendall(b"ATMA\r")        # passive bus monitor (read-only), stopped by any keypress
time.sleep(3)
s.sendall(b"\r")
time.sleep(1)
buf = b""
s.settimeout(1)
try:
    while True:
        chunk = s.recv(65536)
        if not chunk:
            break
        buf += chunk
except socket.timeout:
    pass
frames = [ln for ln in buf.decode(errors="replace").splitlines()
          if ln.strip() and ln.strip() not in ("ATMA", "STOPPED", ">")]
signs = []
if "v1.5" in version:
    signs.append("reports v1.5, a version Elm Electronics never released")
if amc.strip() == "OK":
    signs.append("ATAMC answers OK instead of a count (command not really implemented)")
if not frames:
    signs.append("ATMA bus monitor captured nothing")
print(f"  ATMA captured {len(frames)} frames in 3 s")
print("  Verdict:", "CLONE - " + "; ".join(signs) if signs else "no clone signs found")
s.close()
