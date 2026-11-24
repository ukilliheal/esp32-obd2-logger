"""Analyze ESP32 OBD log sessions (logs/esp32/sNNNNNN_GGGGG.bin) and write a report.

Run from the repo root with the project's Python:

    .venv/bin/python .claude/skills/esp32-log-analysis/scripts/analyze_session.py --list
    .venv/bin/python .claude/skills/esp32-log-analysis/scripts/analyze_session.py --session 3
    .venv/bin/python .claude/skills/esp32-log-analysis/scripts/analyze_session.py --new
    .venv/bin/python .claude/skills/esp32-log-analysis/scripts/analyze_session.py --session 3 --assign ranger

--list      sessions on disk, their car fingerprint, and the vehicle it is assigned to
--session   analyze one session (number) ; --new analyzes sessions without a report yet
--assign    remember that this session's car fingerprint is vehicle KEY (from vehicles.py)
--json      also print the machine-readable findings

Writes reports/<vehicle>_esp32_s<session>.md and .json. Vehicle matching: VIN hash when
the car reports one, otherwise protocol + supported-value bitmaps, stored in
logs/esp32/vehicles.json. The plain VIN is never available here (the board only stores
a hash).
"""
import argparse
import json
import math
import pathlib
import statistics as st
import sys

ROOT = pathlib.Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "tools"))
from esp32_logs import DECODE, EVENTS, OUT, PROTOCOLS, parse_file  # noqa: E402
from vehicles import VEHICLES  # noqa: E402

REPORTS = ROOT / "reports"
REGISTRY = OUT / "vehicles.json"

# Thresholds (documented so the report can explain its verdicts).
TRIM_OK, TRIM_WATCH = 5.0, 10.0        # |total fuel trim| %: <=5 ok, <=10 watch, >10 problem
WARM_F = 170.0                          # coolant above this = warm engine
IDLE_RPM_HIGH = 1000                    # warm idle above this is high for most engines
IDLE_STD_WATCH = 75                     # rpm std-dev at warm idle
LOAD_HIGH = 50.0                        # load % treated as "under load"


# ------------------------------------------------------------------ loading

def sessions_on_disk():
    out = {}
    for f in sorted(OUT.glob("s*_*.bin")):
        try:
            sess = int(f.stem.split("_")[0][1:])
        except ValueError:
            continue
        out.setdefault(sess, []).append(f)
    return out


def load_session(files):
    header, rows, anchor, events, damaged = None, [], None, [], []
    for f in sorted(files, key=lambda p: int(p.stem.split("_")[1])):
        try:
            entries = list(parse_file(f.read_bytes()))
        except Exception as e:  # damaged / cut-off file
            damaged.append(f"{f.name}: {e}")
            continue
        for kind, item in entries:
            if kind == "header":
                header = header or item
                if item["epoch"] and anchor is None:
                    anchor = (0.0, item["epoch"])
            elif kind == "time":
                anchor = (item[0] / 1000, item[1])
            elif kind == "event":
                events.append((item[0] / 1000, EVENTS.get(item[1], str(item[1]))))
            else:
                t, vals = item
                row = {"t": t / 1000}
                for pid, (a, b) in vals.items():
                    if pid in DECODE:
                        name, fn = DECODE[pid]
                        row[name] = fn(a, b)
                rows.append(row)
    rows.sort(key=lambda r: r["t"])
    return header, rows, anchor, events, damaged


def fingerprint(h):
    if h.get("vin_hash"):
        return "vin:" + h["vin_hash"]
    return f"proto:{h['protocol']}:" + ":".join(f"{s:08x}" for s in h["support"])


def registry():
    if REGISTRY.exists():
        return json.loads(REGISTRY.read_text())
    return {}


def save_registry(reg):
    REGISTRY.parent.mkdir(parents=True, exist_ok=True)
    REGISTRY.write_text(json.dumps(reg, indent=2, sort_keys=True) + "\n")


# ------------------------------------------------------------------ helpers

def vals(rows, key, pred=None):
    return [r[key] for r in rows if key in r and (pred is None or pred(r))]


def stats(v):
    if not v:
        return None
    return {"n": len(v), "min": min(v), "avg": st.mean(v), "max": max(v),
            "std": st.pstdev(v) if len(v) > 1 else 0.0}


def verdict_trim(x):
    a = abs(x)
    return "ok" if a <= TRIM_OK else "watch" if a <= TRIM_WATCH else "problem"


def total_trim(r, bank):
    s, l = r.get(f"stft_{bank}_pct"), r.get(f"ltft_{bank}_pct")
    return None if s is None or l is None else s + l


def fmt(x, nd=1):
    return "-" if x is None else f"{x:.{nd}f}"


# ------------------------------------------------------------------ analysis

def analyze(sess, header, rows, anchor, events, damaged):
    f = {"session": sess, "checks": [], "notes": []}
    check = lambda name, level, text: f["checks"].append({"check": name, "level": level, "text": text})

    f["protocol"] = PROTOCOLS.get(header["protocol"], header["protocol"])
    f["vin"] = ("sha256:" + header["vin_hash"]) if header["vin_hash"] else "not reported"
    f["stored_codes"] = header["dtcs"] or "none"
    f["fingerprint"] = fingerprint(header)
    f["start_unix"] = (anchor[1] - anchor[0]) if anchor else None
    if damaged:
        f["notes"].append("Damaged/cut-off files skipped: " + "; ".join(damaged))
    if not rows:
        f["duration_min"] = 0
        f["samples"] = 0
        check("data", "problem", "No samples in this session (the car never answered?).")
        return f

    ts = [r["t"] for r in rows]
    gaps = [b - a for a, b in zip(ts, ts[1:])]
    f["duration_min"] = (ts[-1] - ts[0]) / 60
    f["samples"] = len(rows)
    f["interval_s"] = st.median(gaps) if gaps else None
    f["values_logged"] = sorted({k for r in rows for k in r if k != "t"})
    big = [g for g in gaps if g > 15]
    if big:
        f["notes"].append(f"{len(big)} gap(s) over 15 s in the data (longest {max(big):.0f} s): "
                          "engine off, Mac using the reader, or reader dropouts.")
    f["events"] = [{"t_min": t / 60, "event": e} for t, e in events]

    running = lambda r: r.get("rpm", 0) > 400
    moving = lambda r: running(r) and r.get("speed_mph", 0) > 3
    idle = lambda r: running(r) and r.get("speed_mph", 0) <= 1
    warm = lambda r: r.get("_coolant", 0) >= WARM_F

    # Carry the last coolant reading forward (it's read every 5th cycle).
    last = None
    for r in rows:
        if "coolant_f" in r:
            last = r["coolant_f"]
        r["_coolant"] = last if last is not None else 0

    # Checks that need a warm engine fall back to "after the first 3 minutes" when the
    # engine never reached WARM_F (e.g. a thermostat stuck open), and say so.
    if not any(r["_coolant"] >= WARM_F for r in rows):
        t0 = rows[0]["t"]
        warm = lambda r: r["t"] - t0 >= 180
        if any("coolant_f" in r for r in rows):
            f["notes"].append(f"The engine never reached {WARM_F:.0f} F, so warm-engine checks use "
                              "readings after the first 3 minutes instead.")

    n_run = sum(map(running, rows))
    n_move = sum(map(moving, rows))
    f["running_pct"] = 100 * n_run / len(rows)
    f["moving_pct"] = 100 * n_move / len(rows)
    spd = vals(rows, "speed_mph")
    if spd:
        f["max_speed_mph"] = max(spd)
        dist = 0.0
        for a, b in zip(rows, rows[1:]):
            if "speed_mph" in a:
                dist += a["speed_mph"] * min(b["t"] - a["t"], 10) / 3600
        f["distance_mi"] = dist
    f["stats"] = {k: stats(vals(rows, k)) for k in f["values_logged"] if not k.startswith("_")}

    if n_run == 0:
        check("engine", "watch", "The engine was never running (rpm 0 throughout): ignition on only.")
        return f

    # --- fuel trims: idle (warm) vs under load
    f["trims"] = {}
    for bank in ("b1", "b2"):
        all_t = [x for x in (total_trim(r, bank) for r in rows if running(r)) if x is not None]
        if not all_t:
            continue
        idle_t = [x for x in (total_trim(r, bank) for r in rows if idle(r) and warm(r)) if x is not None]
        load_t = [x for x in (total_trim(r, bank) for r in rows
                              if running(r) and (r.get("load_pct", 0) >= LOAD_HIGH or r.get("rpm", 0) >= 2000))
                  if x is not None]
        d = {"all": st.mean(all_t), "idle_warm": st.mean(idle_t) if idle_t else None,
             "load": st.mean(load_t) if load_t else None, "n_load": len(load_t),
             "max": max(all_t), "min": min(all_t),
             "ltft_avg": st.mean(vals(rows, f"ltft_{bank}_pct", running) or [0])}
        f["trims"][bank] = d
        level = verdict_trim(d["all"])
        side = "lean: computer adding fuel" if d["all"] > 0 else "rich: computer removing fuel"
        sg = lambda x: "n/a" if x is None else f"{x:+.1f}%"
        check(f"fuel trim {bank}", level, f"Total fuel trim averages {d['all']:+.1f}% ({side}); "
              f"warm idle {sg(d['idle_warm'])}, under load {sg(d['load'])} ({len(load_t)} samples).")
    t = f["trims"]
    if "b1" in t and "b2" in t:
        diff = abs(t["b1"]["all"] - t["b2"]["all"])
        if diff > 5:
            check("bank balance", "watch", f"Banks differ by {diff:.1f}% total trim: points to a "
                  "one-sided cause (injector, intake leak on one side, O2 sensor) rather than a shared one.")
        bl = [b for b in t.values()]
        lean_idle = [b["idle_warm"] for b in bl if b["idle_warm"] is not None]
        lean_load = [b["load"] for b in bl if b["load"] is not None and b["n_load"] >= 10]
        if lean_idle and lean_load:
            li, ll = st.mean(lean_idle), st.mean(lean_load)
            if li > TRIM_OK and ll < li - 3:
                f["notes"].append(f"Lean correction is larger at warm idle ({li:+.1f}%) than under load "
                                  f"({ll:+.1f}%): the classic vacuum-leak pattern.")
            elif ll > TRIM_OK and ll > li + 3:
                f["notes"].append(f"Lean correction grows under load ({li:+.1f}% idle -> {ll:+.1f}% load): "
                                  "points to fuel delivery (pump, filter, pressure) or a MAF reading low.")
        elif not lean_load:
            f["notes"].append("Not enough under-load data (load >= 50% or rpm >= 2000) to tell a vacuum "
                              "leak from a fuel-delivery/MAF problem; a drive log would.")

    # --- upstream narrowband O2: stuck sensors (sampling is too slow to count switching)
    for key in ("o2_b1s1_v", "o2_b2s1_v"):
        v = vals(rows, key, lambda r: running(r) and warm(r))
        if len(v) < 20:
            continue
        lean = sum(x < 0.3 for x in v) / len(v)
        rich = sum(x > 0.6 for x in v) / len(v)
        spread = max(v) - min(v)
        if spread < 0.3 or lean > 0.9 or rich > 0.9:
            check(key, "problem", f"Upstream O2 looks stuck: range {min(v):.2f}-{max(v):.2f} V, "
                  f"{lean:.0%} of samples lean, {rich:.0%} rich (warm engine).")
        else:
            check(key, "ok", f"Upstream O2 swinging normally {min(v):.2f}-{max(v):.2f} V "
                  f"({lean:.0%} lean / {rich:.0%} rich samples).")
    for key in ("o2_s1_lambda", "o2_s5_lambda"):
        v = vals(rows, key, lambda r: running(r) and warm(r))
        if len(v) >= 20:
            a = st.mean(v)
            level = "ok" if abs(a - 1) <= 0.03 else "watch"
            check(key, level, f"Wideband lambda averages {a:.3f} (1.000 = ideal mixture).")

    # --- downstream O2 (after the catalyst), warm engine; idle and driving judged separately
    v_idle = vals(rows, "o2_b1s2_v", lambda r: idle(r) and warm(r))
    v_move = vals(rows, "o2_b1s2_v", lambda r: moving(r) and warm(r))
    v_all = vals(rows, "o2_b1s2_v", lambda r: running(r) and warm(r))
    if len(v_all) >= 10:
        parts = {name: v for name, v in (("idle", v_idle), ("driving", v_move)) if len(v) >= 8}
        if not parts:
            parts = {"running": v_all}
        desc = ", ".join(f"{k} avg {st.mean(v):.2f} V (std {st.pstdev(v):.2f})" for k, v in parts.items())
        swingy = [k for k, v in parts.items() if st.pstdev(v) > 0.25]
        low = [k for k, v in parts.items() if st.mean(v) < 0.2]
        if swingy:
            check("rear O2", "watch", f"Rear O2 swings a lot while {' and '.join(swingy)} ({desc}): if it "
                  "mirrors the front sensors the catalyst may be losing efficiency.")
        elif low == ["idle"] and "driving" in parts:
            check("rear O2", "watch", f"Rear O2 reads low only at idle ({desc}). Low at idle but normal "
                  "while driving is the classic sign of a small exhaust leak near the rear sensor "
                  "(leaks pull in outside air at idle).")
        elif low:
            check("rear O2", "watch", f"Rear O2 reads low/lean on a warm engine ({desc}). Common causes: "
                  "exhaust leak near the rear sensor, a weak sensor, or a truly lean mixture.")
        else:
            check("rear O2", "ok", f"Rear O2 steady ({desc}): catalyst storing oxygen normally.")

    # --- coolant / warm-up
    cool = [(r["t"], r["coolant_f"]) for r in rows if "coolant_f" in r]
    if cool:
        start, peak = cool[0][1], max(c for _, c in cool)
        f["coolant"] = {"start_f": start, "max_f": peak, "end_f": cool[-1][1]}
        if start < 120:
            for target in (160, 180):
                hit = next((tt for tt, c in cool if c >= target), None)
                f["coolant"][f"min_to_{target}f"] = None if hit is None else (hit - cool[0][0]) / 60
        run_min = (cool[-1][0] - cool[0][0]) / 60
        if peak >= 230:
            check("coolant", "problem", f"Coolant reached {peak:.0f} F: overheating.")
        elif peak < 160 and run_min >= 15:
            check("coolant", "watch", f"Coolant never passed {peak:.0f} F in {run_min:.0f} min: "
                  "possible thermostat stuck open.")
        else:
            check("coolant", "ok", f"Coolant {start:.0f} F -> peak {peak:.0f} F.")

    # --- idle quality (warm)
    rpm_idle = vals(rows, "rpm", lambda r: idle(r) and warm(r))
    if len(rpm_idle) >= 20:
        a, sd = st.mean(rpm_idle), st.pstdev(rpm_idle)
        f["idle_rpm"] = {"avg": a, "std": sd}
        level = "watch" if a > IDLE_RPM_HIGH or sd > IDLE_STD_WATCH else "ok"
        check("idle", level, f"Warm idle {a:.0f} rpm (std {sd:.0f}).")

    # --- charging
    volts = vals(rows, "module_volts", running)
    if len(volts) >= 5:
        a = st.mean(volts)
        level = "ok" if 13.2 <= a <= 14.8 else "watch"
        check("charging", level, f"Module voltage averages {a:.2f} V with the engine running.")

    if f.get("interval_s") and f["interval_s"] > 1.5:
        f["notes"].append(f"One full set of values every ~{f['interval_s']:.1f} s: good for trends, "
                          "too slow to judge O2 switching speed or brief misfires.")
    if f["stored_codes"] != "none":
        f["notes"].append(f"Stored trouble codes at session start: {f['stored_codes']}.")
    return f


def history_row(f):
    t = f.get("trims", {})
    return {"session": f["session"], "duration_min": f.get("duration_min"),
            "moving_pct": f.get("moving_pct"),
            "trim_b1": t.get("b1", {}).get("all"), "trim_b2": t.get("b2", {}).get("all"),
            "idle_trim_b1": t.get("b1", {}).get("idle_warm"), "idle_trim_b2": t.get("b2", {}).get("idle_warm"),
            "idle_rpm": f.get("idle_rpm", {}).get("avg"), "coolant_max": f.get("coolant", {}).get("max_f"),
            "codes": f["stored_codes"]}


# ------------------------------------------------------------------ report

LEVEL_ICON = {"ok": "OK", "watch": "WATCH", "problem": "PROBLEM"}


def report_md(f, vehicle_key, history):
    import datetime as dt
    name = VEHICLES.get(vehicle_key, {}).get("name", "Unassigned vehicle") if vehicle_key else "Unassigned vehicle"
    when = (dt.datetime.fromtimestamp(f["start_unix"]).strftime("%Y-%m-%d %H:%M")
            if f.get("start_unix") else "unknown (board had no clock; logged on battery/power bank)")
    L = [f"# {name}: ESP32 log session {f['session']}", "",
         f"- **Started:** {when}",
         f"- **Length:** {f.get('duration_min', 0):.1f} min, {f.get('samples', 0)} samples"
         + (f", one every ~{f['interval_s']:.1f} s" if f.get("interval_s") else ""),
         f"- **Protocol:** {f['protocol']} | **VIN:** {f['vin']} | **Stored codes:** {f['stored_codes']}",
         f"- **Engine running:** {f.get('running_pct', 0):.0f}% of samples | **moving:** {f.get('moving_pct', 0):.0f}%"
         + (f" | max {f['max_speed_mph']:.0f} mph, ~{f.get('distance_mi', 0):.1f} mi" if f.get("max_speed_mph") else ""),
         f"- **Car fingerprint:** `{f['fingerprint']}`", ""]
    L += ["## Checks", "| Check | Verdict | Detail |", "|---|---|---|"]
    L += [f"| {c['check']} | {LEVEL_ICON[c['level']]} | {c['text']} |" for c in f["checks"]]
    if f["notes"]:
        L += ["", "## Notes"] + [f"- {n}" for n in f["notes"]]
    if f.get("events"):
        L += ["", "## Events"] + [f"- {e['t_min']:.1f} min: {e['event']}" for e in f["events"]]
    if f.get("stats"):
        L += ["", "## Values (whole session)", "| Value | Min | Avg | Max |", "|---|---|---|---|"]
        for k, s in f["stats"].items():
            if s:
                L.append(f"| {k} | {s['min']:.2f} | {s['avg']:.2f} | {s['max']:.2f} |")
    if len(history) > 1:
        L += ["", "## Compared with earlier sessions of this vehicle",
              "| Session | Min | Moving % | Trim B1 | Trim B2 | Idle trim B1 | Idle trim B2 | Idle rpm | Max coolant | Codes |",
              "|---|---|---|---|---|---|---|---|---|---|"]
        for h in history:
            L.append(f"| {h['session']} | {fmt(h['duration_min'], 0)} | {fmt(h['moving_pct'], 0)} | "
                     f"{fmt(h['trim_b1'])} | {fmt(h['trim_b2'])} | {fmt(h['idle_trim_b1'])} | "
                     f"{fmt(h['idle_trim_b2'])} | {fmt(h['idle_rpm'], 0)} | {fmt(h['coolant_max'], 0)} | {h['codes']} |")
    L += ["", "_Thresholds: total fuel trim within ±5% OK, ±10% watch, beyond that a problem; warm = "
          f"coolant ≥ {WARM_F:.0f} F; warm idle above {IDLE_RPM_HIGH} rpm or std > {IDLE_STD_WATCH} rpm = watch._"]
    return "\n".join(L) + "\n"


# ------------------------------------------------------------------ main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--list", action="store_true")
    g.add_argument("--session", type=int)
    g.add_argument("--new", action="store_true", help="analyze sessions that have no report yet")
    ap.add_argument("--assign", metavar="VEHICLE", help="assign the session's car fingerprint to a vehicle key")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()

    on_disk = sessions_on_disk()
    reg = registry()
    if not on_disk:
        sys.exit(f"No downloaded sessions in {OUT}. Download first: .venv/bin/python esp32_logs.py download")

    loaded = {}

    def get(sess):
        if sess not in loaded:
            loaded[sess] = load_session(on_disk[sess])
        return loaded[sess]

    if a.list:
        for sess in sorted(on_disk):
            h, rows, *_ = get(sess)
            if h is None:
                print(f"  session {sess}: unreadable")
                continue
            fp = fingerprint(h)
            dur = (rows[-1]["t"] - rows[0]["t"]) / 60 if rows else 0
            print(f"  session {sess}: {len(rows)} samples, {dur:.1f} min, "
                  f"{PROTOCOLS.get(h['protocol'], h['protocol'])}, codes {h['dtcs'] or 'none'}, "
                  f"vehicle={reg.get(fp, 'UNASSIGNED')}  [{fp}]")
        return

    if a.assign and a.assign not in VEHICLES:
        sys.exit(f"Unknown vehicle {a.assign!r}. Choose one of: {', '.join(VEHICLES)}")

    targets = [a.session] if a.session else [
        s for s in sorted(on_disk) if not list(REPORTS.glob(f"*_esp32_s{s:06d}.md"))]
    if a.session and a.session not in on_disk:
        sys.exit(f"Session {a.session} is not in {OUT}. Sessions on disk: {sorted(on_disk)}")
    if not targets:
        print("All downloaded sessions already have reports.")
        return

    REPORTS.mkdir(exist_ok=True)
    for sess in targets:
        header, rows, anchor, events, damaged = get(sess)
        if header is None:
            print(f"session {sess}: no readable header, skipped ({'; '.join(damaged)})")
            continue
        fp = fingerprint(header)
        if a.assign:
            reg[fp] = a.assign
            save_registry(reg)
        vkey = reg.get(fp)
        f = analyze(sess, header, rows, anchor, events, damaged)
        # History: every session on disk with the same fingerprint (oldest first).
        history = []
        for other in sorted(on_disk):
            oh = get(other)[0]
            if oh is not None and fingerprint(oh) == fp:
                history.append(history_row(f if other == sess else analyze(other, *get(other))))
        md = report_md(f, vkey, history)
        stem = f"{vkey or 'unassigned'}_esp32_s{sess:06d}"
        for old in REPORTS.glob(f"*_esp32_s{sess:06d}.*"):  # replace an earlier (e.g. unassigned) report
            old.unlink()
        (REPORTS / f"{stem}.md").write_text(md)
        (REPORTS / f"{stem}.json").write_text(json.dumps({"vehicle": vkey, **f, "history": history},
                                                         indent=2, default=str))
        print(md)
        print(f"Report: reports/{stem}.md")
        if vkey is None:
            print(f"VEHICLE UNKNOWN for fingerprint {fp}. Ask the user which vehicle this was, then rerun "
                  f"with --session {sess} --assign <key> (keys: {', '.join(VEHICLES)}).")
        if a.json:
            print(json.dumps(f, indent=2, default=str))


if __name__ == "__main__":
    main()
