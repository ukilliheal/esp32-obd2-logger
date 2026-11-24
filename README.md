# OBD Diagnostics

Read, log and analyze OBD2 engine data from older cars and trucks using cheap ELM327
readers. It has two parts:

- **ESP32 bridge + logger** (Arduino firmware). The board finds an OBD2 reader on its own
  over Wi-Fi or Bluetooth Classic, relays it to a computer over USB, and logs engine data to
  its own flash when no computer is using it. Plug it into a USB power bank, leave it in
  the car, and it records the drive with no laptop. An optional small OLED shows live values
  and trouble codes.
- **Mac tools** (Python). Scan a vehicle for trouble codes, readiness monitors, freeze frame
  and live data and save a report; download and decode the ESP32's logs. They reach a reader
  over Wi-Fi, Bluetooth LE, Bluetooth Classic/serial, or through the ESP32.

Everything is read-only except clearing trouble codes, which asks first.

## Tested hardware

| Part | Tested with |
|---|---|
| ESP32 | Original ESP32 dev board: ESP32-D0WD chip, 4 MB flash, CP2102 USB-serial (shows up on macOS as `/dev/cu.usbserial-0001`), on-board LED on GPIO 2 |
| Arduino core | `esp32:esp32` 4.0.0, built with `arduino-cli` |
| Display (optional) | 0.91" 128x32 SSD1306 I2C OLED |
| Readers | Three ELM327 clones: one Wi-Fi ("v1.5", network `WiFi_OBDII`, 192.168.0.10:35000) and two Bluetooth Classic ("v1.5" and "v1.3a") |
| Cars | 2000 Ford Ranger (J1850 PWM) and 2003 Chevrolet Avalanche (J1850 VPW), plus the ELM327 emulator's CAN car |
| Computer | macOS on Apple Silicon, Python 3.12 |

The firmware needs **Bluetooth Classic**, which only the original ESP32 has. The newer
ESP32-S2, S3, C3 and C6 chips don't, so they can't talk to Bluetooth Classic readers. The
board also needs 4 MB of flash: the firmware takes 1.875 MB and the log 2 MB.

## Folder structure

```
├── README.md
├── LICENSE                     MIT
├── requirements.txt            Python dependencies for the Mac tools
├── tools/                      Mac-side Python tools (run from the repo root)
│   ├── diagnose.py             full scan -> reports/<vehicle>_<timestamp>.md/.json
│   ├── log_live.py             live data log from the Mac -> logs/live/*.csv
│   ├── esp32_bridge.py         talk to the ESP32 over USB (status, reader list, serve link)
│   ├── esp32_logs.py           download / decode / clear the ESP32's flash logs
│   ├── ble_bridge.py           Bluetooth LE reader -> local TCP port
│   ├── probe.py                identify an adapter (firmware, protocols, clone checks)
│   ├── gm_probe.py             read-only GM Class 2 module probe (needs a genuine adapter)
│   └── vehicles.py             vehicle profiles (protocol + known issues)
├── firmware/
│   ├── esp32_obd_bridge/       ESP32 Arduino sketch (bridge + auto-connect + logger)
│   │   ├── esp32_obd_bridge.ino
│   │   └── partitions.csv      1.875 MB app / 2 MB log storage
│   └── wokwi/                  simulator setup for testing the OLED screens
├── CLAUDE.md                   project notes for Claude Code
└── .claude/skills/
    └── esp32-log-analysis/     Claude Code skill: retrieve + analyze ESP32 logs
```

The tools create `logs/` and `reports/` when they first write to them. Both are
git-ignored: they hold your own vehicles' data.

## Setup

```bash
uv venv --python 3.12 .venv
uv pip install --python .venv -r requirements.txt
```

Run every tool from the repo root as `.venv/bin/python tools/<tool>.py`. Turn the ignition
on (or start the engine) before connecting. Then build and flash the ESP32 (see
[Build and flash](#build-and-flash)).

## Connecting to your OBD2 reader

The ESP32 keeps a list of up to 8 readers in its flash and tries each one, over and over,
until one answers: first Wi-Fi, then Bluetooth Classic (strongest signal first). The blue
LED blinks fast while it searches and goes solid when connected.

Out of the box the list holds one entry: an open Wi-Fi network named `WiFi_OBDII` at
192.168.0.10:35000, which is what most ELM327 Wi-Fi clones ship with. No Bluetooth reader
is listed, so add yours one of two ways.

### Option 1: add it over USB (no reflash)

With the reader plugged into a car and powered, find it and add it:

```bash
.venv/bin/python tools/esp32_bridge.py --bt-scan      # Bluetooth OBD readers nearby, with MAC addresses
.venv/bin/python tools/esp32_bridge.py --scan         # Wi-Fi OBD readers nearby
.venv/bin/python tools/esp32_bridge.py --add-bt "AA:BB:CC:DD:EE:FF,auto"
.venv/bin/python tools/esp32_bridge.py --add-wifi "SSID,PASSWORD,192.168.0.10,35000"
```

- **Bluetooth:** give the reader's MAC address (or its device name) and its PIN. A PIN of
  `auto` tries 1234, 0000 and 6789, and the board saves whichever works.
- **Wi-Fi:** give the network name, its password (leave empty for an open network), and
  the reader's IP address and port. Host and port default to 192.168.0.10 and 35000.

The list is saved on the board and survives power cycles and reflashing. Manage it with:

```bash
.venv/bin/python tools/esp32_bridge.py --list         # reader list
.venv/bin/python tools/esp32_bridge.py --delete 2     # remove reader 2
.venv/bin/python tools/esp32_bridge.py --mode auto    # or wifi, bt, or a reader number
.venv/bin/python tools/esp32_bridge.py --status       # state, reader, logger, storage, memory
.venv/bin/python tools/esp32_bridge.py --log off      # / on
```

Any serial terminal at 115200 baud works too: send `#BTSCAN`, `#ADDBT MAC-or-name,pin`,
`#ADDWIFI ssid,password,host,port`, `#LIST`, `#DEL n`. The full command list is at the top
of the sketch.

### Option 2: build it into the firmware

To have a board come up already knowing your readers, edit `defaultReaders()` in
[esp32_obd_bridge.ino](firmware/esp32_obd_bridge/esp32_obd_bridge.ino):

```cpp
void defaultReaders() {
  readers.clear();

  Reader w;                       // a Wi-Fi reader
  w.type = T_WIFI;
  w.target = "WiFi_OBDII";        // network name
  w.secret = "";                  // Wi-Fi password, "" if the network is open
  w.host = DEF_HOST;              // reader's IP address ("192.168.0.10")
  w.port = DEF_PORT;              // reader's TCP port (35000)
  readers.push_back(w);

  Reader b;                       // a Bluetooth Classic reader
  b.type = T_BT;
  b.target = "AA:BB:CC:DD:EE:FF"; // MAC address from --bt-scan, or the device name
  b.secret = "auto";              // PIN, or "auto" to try 1234, 0000, 6789
  readers.push_back(b);

  mode = "auto";
}
```

This list is only loaded on a board with no saved settings. On a board that has run this
firmware before, also send `#RESET` once after flashing (it restores the built-in list and
keeps the logs). If you publish your fork, remember these values end up in your repository.

## Scanning a vehicle from the Mac

```bash
.venv/bin/python tools/diagnose.py --list                     # vehicle profiles
.venv/bin/python tools/diagnose.py ranger --esp32             # through the ESP32 (any listed reader)
.venv/bin/python tools/diagnose.py avalanche --wifi           # Wi-Fi reader (join its network first)
.venv/bin/python tools/diagnose.py ranger --ble               # Bluetooth LE reader
.venv/bin/python tools/diagnose.py ranger --port /dev/cu.OBDII  # paired Bluetooth Classic / USB
.venv/bin/python tools/diagnose.py ranger --esp32 --clear-codes # asks before clearing
.venv/bin/python tools/log_live.py ranger --esp32 --seconds 300 # live log -> logs/live/
.venv/bin/python tools/probe.py 192.168.0.10:35000            # identify a Wi-Fi adapter
```

Add `--debug` to `diagnose.py` to see raw adapter traffic.

The vehicle name picks a profile from `tools/vehicles.py`: the protocol to try first and
notes on common faults. The profiles included are examples; add your own, or use `unknown`
for any car (protocol auto-detect).

| Key | Vehicle | Protocol |
|---|---|---|
| `avalanche` | 2003 Chevrolet Avalanche 1500 4x4 | J1850 VPW |
| `ranger` | 2000 Ford Ranger | J1850 PWM |
| `camaro` | Chevrolet Camaro (year to confirm from VIN) | J1850 VPW |
| `ram_van` | 2003 Dodge Ram 3500 Van | auto |
| `ram_2500` | 2003 Dodge Ram 2500 4x4 | auto |
| `unknown` | any other car | auto-detect |

## Display (optional)

A 0.91" 128x32 I2C OLED (SSD1306, white) shows what the board is doing. Wire it to the
ESP32's **3.3 V** pin, not 5 V, so the I2C lines stay at 3.3 V:

| OLED | ESP32 |
|------|-------|
| GND  | GND |
| VCC  | 3V3 |
| SCL  | GPIO 22 |
| SDA  | GPIO 21 |

The board looks for it at power-up (I2C address 0x3C or 0x3D); `--status` reports
`display=yes`. Without a display everything works as before. Screens:

- **Signal bars** (top right, 0-4 like a phone): live strength for a Wi-Fi reader; for a
  Bluetooth reader, the strength measured when it was found. No bars while searching.
- **Searching:** the search round and the current step (scanning, joining, PIN tried).
- **Connected:** the logger's state on the top line (starting, waiting for the engine,
  recording, or "Mac using reader"), then log use and uptime.
- **Recording:** large RPM plus coolant temperature (°F), speed (mph), and battery voltage, or
  engine load on cars without the voltage value (most 2000-2003 cars).
- **Trouble codes:** stored (mode 03) and pending (mode 07) codes, one at a time in large
  text, taking turns with the live values. The board reads them when it connects and again
  every 5 minutes while recording, so a code that sets mid-drive shows up. "no codes" on
  the idle screen means the car was asked and reported none. Stored codes are also saved
  in each log file's header; pending codes are display-only.

**Testing the screens without hardware** uses the [Wokwi](https://wokwi.com) ESP32
simulator (`firmware/wokwi/`). A demo build (`-DOLED_DEMO`) skips the reader and Bluetooth
(Wokwi has no Bluetooth) and plays a one-minute script through every screen with fake data.
Wokwi's SSD1306 is the 128x64 model, so the 128x32 picture appears in its top half.

1. Install `wokwi-cli`: see the
   [installation guide](https://docs.wokwi.com/wokwi-ci/cli-installation).
2. Create a free token at <https://wokwi.com/dashboard/ci> and add
   `export WOKWI_CLI_TOKEN=...` to your shell profile.
3. Run it. Screenshots of each screen land in `firmware/wokwi/shots/`:

```bash
firmware/wokwi/run.sh            # demo build + a screenshot of every screen
firmware/wokwi/run.sh 18000      # one screenshot at 18 s of simulated time
```

To watch it live instead, install the Wokwi extension in VS Code, run "Wokwi: Request a
new license", build once with `run.sh`, then open `firmware/wokwi/diagram.json`.

## Data logging

When a reader is connected and no Mac program is using it, the board logs standard OBD2
data from any car. It asks the car which of 32 common values it supports and logs those:
rpm, speed, load, airflow, fuel trims, O2 sensors (including wideband), temperatures, fuel
level, battery voltage, rail pressure, torque. Fast values are read every cycle and slow
ones every fifth cycle. Each session also stores the protocol, the supported-value
fingerprint, the stored trouble codes and a hashed VIN.

- **Storage:** 2 MB of flash in 64 KB files. The oldest file is deleted first, so it keeps
  roughly 20-40 hours of engine-on time (about 40 h on a 2000 Ford Ranger).
- **Power loss:** logs survive power cycles; a sudden power cut loses at most the last ~15 s.
- **No clock:** the board only knows the real time while the Mac is connected, so logs made
  on battery or power-bank power show minutes into the session.
- **Sharing the reader:** the Mac helper sends `#HOLD` / `#RELEASE`, so the logger pauses
  while a Mac program uses the reader.

```bash
.venv/bin/python tools/esp32_logs.py list        # files on the board
.venv/bin/python tools/esp32_logs.py download    # -> logs/esp32/ (checksummed, ~8 KB/s)
.venv/bin/python tools/esp32_logs.py decode      # one CSV per session, readable units
.venv/bin/python tools/esp32_logs.py clear       # delete logs on the board (asks first)
```

Only one program can hold the ESP32's USB port at a time, so stop a running bridge before
downloading. In Claude Code, the **esp32-log-analysis** skill does download, decode,
analysis, vehicle matching and the comparison with earlier sessions in one step. Just ask it
to "check the ESP32 logs".

Only one program can hold the ESP32's USB port at a time, so stop a running bridge before
downloading. In [Claude Code](https://claude.com/claude-code), the **esp32-log-analysis**
skill does download, decode, analysis, vehicle matching and the comparison with earlier
sessions in one step. Just ask it to "check the ESP32 logs".

## Build and flash

Needs `arduino-cli` (`brew install arduino-cli`) and the `esp32:esp32` core 4.0.0
(board-manager URL `https://espressif.github.io/arduino-esp32/package_esp32_index.json`).
The sketch's `partitions.csv` sets the flash layout; the `PartitionScheme` flag is still
required.

```bash
cd firmware/esp32_obd_bridge
arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=huge_app --build-property upload.maximum_size=1966080 --output-dir build .
arduino-cli upload  --fqbn esp32:esp32:esp32:PartitionScheme=huge_app -p /dev/cu.usbserial-0001 --input-dir build .
```

Replace `/dev/cu.usbserial-0001` with your board's port (`arduino-cli board list`). The
Mac tools open the port with DTR/RTS off; with them on, the tested board floods the line
with garbage.

## Testing without a car

```bash
.venv/bin/elm -n 35001 -s car        # emulator (a modern CAN car), in another terminal
.venv/bin/python tools/diagnose.py unknown --url socket://127.0.0.1:35001
```

## Known limitations

**ESP32**
- Original ESP32 only (it needs Bluetooth Classic), with 4 MB flash. Tested on one board
  type; other boards may use a different LED pin or USB-serial chip.
- The firmware fills about 90% of its 1.875 MB partition, so there's little room to add code.
- Bluetooth LE readers aren't supported by the ESP32. On the Mac, `tools/ble_bridge.py`
  handles them.
- No clock: logs recorded away from the Mac are timed in minutes into the session.
- A sudden power cut loses up to the last ~15 s of a log. The 2 MB log keeps roughly 20-40
  hours and then overwrites the oldest data.
- Reader settings (Wi-Fi password, Bluetooth PIN) are stored unencrypted in the board's
  flash, and `#LIST` prints Bluetooth PINs.
- Commands can take up to ~40 s to answer while the board is in a search step.
- The Wokwi simulator has no Bluetooth, so it only tests the display screens.

**Readers**
- Only tested with three ELM327 clones. Clones vary a lot: many don't support J1850 PWM
  (Ford) at all, some send replies one command late, and the tested Wi-Fi reader drops an
  idle connection after ~2 s (the firmware reconnects on demand).
- The tested clones can't reach GM's non-engine modules (body, ABS, transmission), and
  `ATMA` (bus monitoring) doesn't work on them. `gm_probe.py` needs a genuine adapter such
  as an OBDLink (STN chip) or vLinker.

**Vehicles and data**
- Generic OBD2 covers engine and emissions data only: no ABS, airbag, or most transmission
  data.
- Developed on pre-CAN vehicles from 2000-2003. CAN cars (2008+) have only been tested
  against the ELM327 emulator, not a real car. The Camaro and Dodge Ram profiles are
  untested.
- Heavy-duty trucks (over 8,500 lb) and the 2003 Cummins diesel report only part of the
  generic data.
- Sampling is slow on older cars: one full set of values every ~2-4 s. That's fine for
  trends such as fuel trims and temperatures, not for O2 switching speed or brief misfires.
- The analysis thresholds are fixed rules of thumb, not a substitute for a service manual.

**Computer side**
- Written for and tested on macOS only. Serial port names and Bluetooth handling will need
  changes on Linux or Windows.
- A Mac has one Wi-Fi interface, so joining a Wi-Fi reader's network directly takes the
  Mac offline. Going through the ESP32 avoids that.

**Privacy**
- VINs are never stored in plain text, only a truncated SHA-256. That hash can still
  identify a car to someone who can guess the make, model and year, so treat `logs/` and
  `reports/` as private. They're git-ignored for that reason.

## License

[MIT](LICENSE)
