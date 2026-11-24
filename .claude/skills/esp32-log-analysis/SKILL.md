---
name: esp32-log-analysis
description: Retrieve, decode and analyze the engine-data logs recorded by the project's ESP32 OBD2 logger, match each session to the right vehicle, compare with that vehicle's earlier sessions, and write a diagnostic report. Use this whenever the user mentions the ESP32's logs or log data, a drive/idle test the logger recorded, "check/pull/download/look at the logs", "what did the truck/car do", comparing sessions, or wiping the board's log after a download, even if they don't say "ESP32" (e.g. "I left it idling with the logger in the truck, check it").
---

# ESP32 log retrieval and analysis

The ESP32 (firmware in `firmware/esp32_obd_bridge/`) logs standard OBD2 values to flash
whenever it's connected to a reader and no Mac program is using it. Logs are 64 KB binary
files (`sNNNNNN_GGGGG.bin` = session, segment) in a 2 MB rotating store. The project tools
already download and decode them; this skill strings them together and adds a consistent
analysis, so findings are comparable from one session to the next.

All commands run from the repo root with the project's Python (`.venv/bin/python`).
`S` below means `.claude/skills/esp32-log-analysis/scripts/analyze_session.py`.

## 1. Get the logs onto the Mac

1. Check the board is plugged in: `ls /dev/cu.usbserial*`. If it isn't, say so and offer to
   analyze whatever is already in `logs/esp32/` instead.
2. Only one program can hold the ESP32's USB port. If `pgrep -fl esp32_bridge` shows a
   running bridge, ask the user before stopping it (they may be mid-diagnosis).
3. `.venv/bin/python tools/esp32_logs.py download`. Every file must print `bytes OK`; files already
   on disk are skipped. A `FAILED` file: run download once more (it resumes); if it still fails,
   report which file and continue with the rest. The board may answer slowly (up to ~40 s)
   while it is searching for a reader; that's normal.
4. `.venv/bin/python tools/esp32_logs.py decode` writes one CSV per session to `logs/esp32/` for the
   user to open in a spreadsheet.

## 2. Analyze

- `S --list` shows each session: samples, length, protocol, codes, and the vehicle it is
  assigned to (`UNASSIGNED` if the fingerprint hasn't been seen before).
- `S --new` analyzes every session without a report yet; `S --session N` (re)analyzes one.
  Each writes `reports/<vehicle>_esp32_sNNNNNN.md` (+ `.json`) and prints the markdown.

**Vehicle matching.** A session is tied to a car by its VIN hash when the car reports a VIN
(most 2005+), otherwise by protocol + the car's supported-value bitmaps. Two cars of the same
make/era can share that fingerprint, so treat an automatic match on a non-VIN fingerprint as
likely rather than certain if the data looks different from that vehicle's history. When a
session is UNASSIGNED, ask the user which vehicle it was; don't guess from the data. You can
offer a hint (J1850 PWM is the Fords; J1850 VPW is GM and most 2000s Chrysler). Then
`S --session N --assign <key>` with a key from `tools/vehicles.py` (`unknown` is fine for a one-off
car). The assignment is saved in `logs/esp32/vehicles.json` for next time.

## 3. Interpret and report

The script's verdicts (OK / WATCH / PROBLEM) use fixed thresholds that are printed at the
bottom of each report. Your job is to turn them into a diagnosis the user can act on:

- Read the vehicle's notes in `tools/vehicles.py` and any earlier summaries for it in `reports/`
  (e.g. `ranger_summary_*.md`) so the new findings build on what's already known. Say
  explicitly whether things got better, worse or stayed the same, and quote the numbers.
- Fuel trims: total trim = short + long term. Lean correction biggest at warm idle and
  shrinking under load points to a vacuum leak; growing under load points to fuel delivery or
  the MAF. Without under-load data (load >= 50% or rpm >= 2000) the script says it can't tell;
  pass that on and suggest a drive log rather than guessing.
- Sampling is one full set every ~2-4 s on older cars, so don't draw conclusions about O2
  switching speed or brief misfires from these logs.
- Generic OBD2 only covers engine/emissions data. Heavy-duty trucks (over 8,500 lb) and the
  2003 Cummins report only part of it; missing values there are normal, not a fault.
- No real timestamps means the board ran on its own power and never saw the Mac during that
  session; refer to times as "minutes into the session".

In chat, lead with the answer (is anything wrong, what changed), then the key numbers, then
the suggested next step. Link the report file. Keep the whole table in the report, not chat.

## 4. Wiping the board (only when asked)

Clear the board only when the user asks, and only after every file in `tools/esp32_logs.py list`
was downloaded with `bytes OK` (sizes match what's on disk). Then
`echo DELETE | .venv/bin/python tools/esp32_logs.py clear` and confirm with `tools/esp32_logs.py list`
(expect 0 files). The Mac copies in `logs/esp32/` are kept unless the user also asks to delete
those. If the user asks to wipe before downloading, point out that undownloaded data would be
lost and confirm first.

## Privacy

The board never stores a plain VIN, only the first 8 bytes of its SHA-256. Don't try to
recover or print a VIN; refer to the hash.
