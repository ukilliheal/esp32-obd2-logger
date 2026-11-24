"""Log live engine data to CSV and summarize it (O2 switching, fuel trims, warm-up).

    .venv/bin/python tools/log_live.py avalanche --wifi --seconds 180
    .venv/bin/python tools/log_live.py ranger --esp32 --seconds 300
"""
import argparse
import csv
import datetime as dt
import statistics as st
import sys
import time

import obd

from diagnose import LIVE_LOGS, add_link_options, connect, open_link
from vehicles import VEHICLES

PIDS = ["RPM", "COOLANT_TEMP", "SPEED", "MAF", "TIMING_ADVANCE", "THROTTLE_POS",
        "SHORT_FUEL_TRIM_1", "LONG_FUEL_TRIM_1", "SHORT_FUEL_TRIM_2", "LONG_FUEL_TRIM_2",
        "O2_B1S1", "O2_B2S1", "O2_B1S2", "O2_B2S2"]
UNITS = {"COOLANT_TEMP": "degF", "SPEED": "mph"}


def switches(values, lo=0.3, hi=0.6):
    """Count lean<->rich crossings of an upstream O2 sensor."""
    state, n = None, 0
    for v in values:
        s = "rich" if v > hi else "lean" if v < lo else None
        if s and state and s != state:
            n += 1
        state = s or state
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("vehicle", choices=list(VEHICLES))
    add_link_options(ap)
    ap.add_argument("--seconds", type=int, default=180)
    a = ap.parse_args()

    bridge, url = open_link(a)
    try:
        run(a, url)
    finally:
        if bridge:
            bridge.terminate()
            bridge.wait(timeout=5)


def run(a, url):
    conn = connect(url, VEHICLES[a.vehicle]["protocol"])
    pids = [p for p in PIDS if conn.supports(obd.commands[p])]

    LIVE_LOGS.mkdir(parents=True, exist_ok=True)
    path = LIVE_LOGS / f"{a.vehicle}_live_{dt.datetime.now():%Y%m%d_%H%M%S}.csv"
    rows, t0 = [], time.time()
    print(f"Logging {len(pids)} PIDs for {a.seconds}s -> {path}")
    while time.time() - t0 < a.seconds:
        row = {"t": round(time.time() - t0, 1)}
        for p in pids:
            r = conn.query(obd.commands[p])
            if not r.is_null():
                v = r.value.to(UNITS[p]) if p in UNITS else r.value
                row[p] = round(float(v.magnitude), 3)
        rows.append(row)
        if len(rows) % 10 == 1:
            print(f"  t={row['t']:>6}s  rpm={row.get('RPM')}  coolant={row.get('COOLANT_TEMP')}F")
    conn.close()

    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["t"] + pids)
        w.writeheader()
        w.writerows(rows)

    if not rows:
        print("No samples were collected.")
        return
    dur = rows[-1]["t"] - rows[0]["t"] or 1
    print(f"\n{len(rows)} samples over {dur:.0f}s ({len(rows) / dur:.2f}/s)")
    for p in pids:
        vals = [r[p] for r in rows if p in r]
        if vals:
            print(f"  {p:<18} min {min(vals):>8.2f}  avg {st.mean(vals):>8.2f}  max {max(vals):>8.2f}")
    for p in ("O2_B1S1", "O2_B2S1"):
        vals = [r[p] for r in rows if p in r]
        if vals:
            print(f"  {p} lean/rich switches: {switches(vals)} "
                  f"({switches(vals) / dur * 10:.1f} per 10s)")
    print(f"\nSaved {path}")


if __name__ == "__main__":
    sys.exit(main())
