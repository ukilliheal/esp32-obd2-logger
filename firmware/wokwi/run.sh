#!/bin/sh
# Build the display demo firmware and run it in the Wokwi simulator, saving screenshots
# of the OLED at times that land on each demo screen (see demoTick() in the sketch).
#   firmware/wokwi/run.sh            build + all screenshots -> firmware/wokwi/shots/
#   firmware/wokwi/run.sh 18000      build + one screenshot at 18 s of simulated time
# Needs arduino-cli, wokwi-cli, and WOKWI_CLI_TOKEN (https://wokwi.com/dashboard/ci).
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
SKETCH="$HERE/../esp32_obd_bridge"
WOKWI=$(command -v wokwi-cli || echo "$HOME/.wokwi/bin/wokwi-cli")

arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=huge_app \
  --build-property upload.maximum_size=1966080 \
  --build-property compiler.cpp.extra_flags=-DOLED_DEMO \
  --output-dir "$SKETCH/build-demo" "$SKETCH" | tail -2

mkdir -p "$HERE/shots"
TIMES=${1:-"6000 10500 18000 30000 34500 44000 52000"}
for t in $TIMES; do
  "$WOKWI" "$HERE" --quiet --timeout $((t + 500)) --timeout-exit-code 0 \
    --screenshot-part oled --screenshot-time "$t" --screenshot-file "shots/oled_${t}ms.png" \
    --serial-log-file "shots/serial_${t}ms.log"
  echo "saved shots/oled_${t}ms.png"
done
