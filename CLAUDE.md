# CLAUDE.md

OBD2 diagnostics for a small garage of 2000-2003 trucks/cars, using cheap ELM327 clone
readers. Mac-side Python tools plus an ESP32 that bridges readers to the Mac over USB and
logs engine data on its own. See README.md for the user-facing overview.

## Layout and running things

- `tools/`: Python tools. Always run from the repo root with the project venv:
  `.venv/bin/python tools/<tool>.py`. They import each other as siblings (e.g.
  `from diagnose import ...`). Paths to `reports/` and `logs/` come from the repo root
  (`ROOT = HERE.parent`), never the working directory.
- `firmware/esp32_obd_bridge/`: the Arduino sketch. The folder name must match the `.ino` name.
- `logs/esp32/` (git-ignored, created by the tools): ESP32 sessions (`sNNNNNN_GGGGG.bin`, session/segment), decoded `.csv`, and
  `vehicles.json` (car fingerprint -> vehicle key). `logs/live/`: CSVs from `tools/log_live.py`.
- `reports/` (git-ignored, created by the tools): scan reports (`<vehicle>_<timestamp>.md/.json`), ESP32 session reports
  (`<vehicle>_esp32_sNNNNNN.md`), and hand-written per-vehicle summaries
  (`<vehicle>_summary_<date>.md`). Read the summary for a vehicle before diagnosing it again.
- `.claude/skills/esp32-log-analysis/`: use this skill for anything about retrieving or
  analyzing ESP32 logs. Its `scripts/analyze_session.py` holds the analysis thresholds.
- Vehicle keys: `avalanche ranger camaro ram_van ram_2500 unknown` (`tools/vehicles.py`).

Setup: `uv venv --python 3.12 .venv && uv pip install --python .venv -r requirements.txt`.

## Testing

- No car needed: `.venv/bin/elm -n 35001 -s car` (in the background), then
  `.venv/bin/python tools/diagnose.py unknown --url socket://127.0.0.1:35001`. The emulator
  is a CAN car, so forcing J1850 profiles against it returns no data; use `unknown`.
- Firmware build (the PartitionScheme flag is needed even though `partitions.csv` overrides
  the layout; `--build-property` is accepted by compile only, not upload):
  ```bash
  cd firmware/esp32_obd_bridge
  arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=huge_app --build-property upload.maximum_size=1966080 --output-dir build .
  arduino-cli upload  --fqbn esp32:esp32:esp32:PartitionScheme=huge_app -p /dev/cu.usbserial-0001 --input-dir build .
  ```
  The app is ~90% of its 1.875 MB partition, so watch the size when adding code.
- OLED screens without hardware: `firmware/wokwi/run.sh` builds the `-DOLED_DEMO` firmware
  (fake data, no reader or Bluetooth) into `build-demo/` and screenshots each screen with
  `~/.wokwi/bin/wokwi-cli`. Needs `WOKWI_CLI_TOKEN` in the environment (the user's own).
  Keep `demoTick()`'s timeline and the screenshot times in `run.sh` in step.

## Hardware facts that bite

- **Optional OLED:** 0.91" 128x32 SSD1306 on I2C, SDA=GPIO21, SCL=GPIO22, VCC from 3V3.
  The firmware drives it with its own small driver (no display library, to save flash) and
  skips it if nothing answers at 0x3C/0x3D. Don't use GPIO 21/22 for anything else.

- **ESP32 serial port** (`/dev/cu.usbserial-0001`, CP2102): open it with DTR and RTS **off**
  (`ser.dtr = ser.rts = False` before `open()`). With them on, the board floods the line with
  garbage. Only one process may hold the port (`exclusive=True`); stop a running
  `esp32_bridge.py` before downloading logs. Commands can wait up to ~40 s while the board is
  in a blocking search step, so use long timeouts.
- **Serial protocol:** board status lines are `#...\n`. ELM327 data never contains `#`.
  Firmware commands: `#STATUS #LIST #ADDWIFI #ADDBT #DEL #MODE #SCAN #BTSCAN #HOLD #RELEASE
  #TIME #LOG #LOGLIST #LOGGET #LOGCLEAR #RESET`.
- **Log format contract:** the firmware's `PIDS[]` table and file format (version 2) must
  stay in sync with `parse_file`/`DECODE` in `tools/esp32_logs.py`. Bump the format version if
  the layout changes. Firmware settings use an NVS `ver` key (currently 4); a mismatch resets
  settings to `defaultReaders()`.
- **Readers** (all ELM327 clones): Wi-Fi `WiFi_OBDII` 192.168.0.10:35000 closes idle TCP after
  ~2 s, so the firmware reconnects on demand. Some Bluetooth Classic clones ("v1.3a") send
  replies that lag one command, so parse replies by the PID they name, not by request order.
  Reader MAC addresses and PINs are not in the source: add them with `#ADDBT` / `#ADDWIFI`. None of them can reach GM
  non-engine modules, and `ATMA` doesn't work on them.
- python-OBD over `socket://` needs `baudrate=38400`, because baud auto-detection fails on
  sockets (already handled in `diagnose.connect`).
- The Mac (M1) has one Wi-Fi interface; USB Wi-Fi dongles have no driver. Reach Wi-Fi readers
  through the ESP32, or with Ethernet for internet while the Mac's Wi-Fi joins the reader.
- These cars are pre-CAN (J1850 PWM = the Ford, J1850 VPW = GM/Chrysler). Rams over 8,500 lb
  and the 2003 Cummins only partly support generic OBD2. There is no 2005 Camaro, so
  confirm its year.

## Rules

- **Never store or print a plain VIN.** Store the first 16 hex characters of its SHA-256
  (`sha256:...`); tools already do this.
- **Ask before anything irreversible:** clearing trouble codes (`--clear-codes`, mode 04),
  wiping the ESP32 log (`#LOGCLEAR`), or reflashing. Wipe the log only after a verified download.
- Probes stay read-only: modes 01/02/03/07/09 and GM `$19/$1A/$22/$3C` only. Never send
  write, security-access, or programming modes.
- When a session's car can't be matched by VIN hash, ask the user which vehicle it was
  rather than guessing from the data.
- Commit or push only when asked.
- Never commit reader MAC addresses, PINs, Wi-Fi passwords, or anything under `logs/` or
  `reports/` (both are git-ignored here).
