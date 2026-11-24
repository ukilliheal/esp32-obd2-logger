"""Bridge a Bluetooth LE ELM327 adapter to a local TCP port.

python-OBD speaks serial/TCP, not BLE. This script connects to the adapter over
BLE and exposes it at 127.0.0.1:<port>, so diagnose.py can treat it exactly like
a Wi-Fi adapter.

    .venv/bin/python tools/ble_bridge.py --scan          # list nearby BLE devices
    .venv/bin/python tools/ble_bridge.py [--name vLinker] [--port 35000]

On first run macOS will ask to allow Bluetooth for your terminal app - say yes.
"""
import argparse
import asyncio
import sys

from bleak import BleakClient, BleakScanner

# Name fragments used by common OBD2 BLE adapters.
KNOWN_NAMES = ["obd", "elm", "vlink", "v-link", "vlinker", "obdlink", "veepeak",
               "icar", "vgate", "konnwei", "carista", "ios-vlink", "android-vlink"]


async def scan(timeout=8.0):
    found = await BleakScanner.discover(timeout=timeout, return_adv=True)
    return [(dev, adv) for dev, adv in found.values()]


async def find_adapter(name_filter):
    for dev, adv in await scan():
        name = (dev.name or adv.local_name or "").lower()
        if name_filter:
            if name_filter.lower() in name:
                return dev
        elif any(k in name for k in KNOWN_NAMES):
            return dev
    return None


def pick_characteristics(client):
    """Find a notify characteristic (adapter -> us) and a write one (us -> adapter),
    preferring a pair inside the same service."""
    best = None
    for service in client.services:
        notify = write = None
        for ch in service.characteristics:
            props = ch.properties
            if notify is None and ("notify" in props or "indicate" in props):
                notify = ch
            if write is None and ("write" in props or "write-without-response" in props):
                write = ch
        if notify and write:
            # Skip standard GATT services (Generic Access/Attribute, Device Info, Battery).
            if service.uuid.lower()[4:8] in ("1800", "1801", "180a", "180f"):
                continue
            best = (notify, write)
            break
    return best


async def run_bridge(name_filter, port):
    print("Scanning for OBD2 BLE adapter...", flush=True)
    dev = await find_adapter(name_filter)
    if dev is None:
        print("ERROR: no adapter found. Is it plugged in, ignition ON, and not connected "
              "to a phone? Try --scan to see device names.", flush=True)
        sys.exit(1)
    print(f"Connecting to {dev.name} ({dev.address})...", flush=True)

    async with BleakClient(dev) as client:
        pair = pick_characteristics(client)
        if pair is None:
            print("ERROR: could not find read/write characteristics on this device.", flush=True)
            sys.exit(1)
        notify_ch, write_ch = pair
        use_response = "write" in write_ch.properties and \
            "write-without-response" not in write_ch.properties
        current_writer = {"w": None}

        def on_notify(_sender, data: bytearray):
            w = current_writer["w"]
            if w is not None:
                w.write(bytes(data))

        await client.start_notify(notify_ch, on_notify)

        async def handle(reader, writer):
            current_writer["w"] = writer
            try:
                while True:
                    data = await reader.read(256)
                    if not data:
                        break
                    for i in range(0, len(data), 20):  # BLE default MTU payload
                        await client.write_gatt_char(write_ch, data[i:i + 20],
                                                     response=use_response)
            finally:
                current_writer["w"] = None
                writer.close()

        server = await asyncio.start_server(handle, "127.0.0.1", port)
        print(f"READY socket://127.0.0.1:{port}", flush=True)
        async with server:
            while client.is_connected:
                await asyncio.sleep(0.5)
        print("Adapter disconnected.", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scan", action="store_true", help="list BLE devices and exit")
    ap.add_argument("--name", help="part of the adapter's BLE name to match")
    ap.add_argument("--port", type=int, default=35000)
    args = ap.parse_args()

    if args.scan:
        for dev, adv in asyncio.run(scan()):
            print(f"{adv.rssi:>5} dBm  {dev.address}  {dev.name or adv.local_name or '?'}")
        return
    asyncio.run(run_bridge(args.name, args.port))


if __name__ == "__main__":
    main()
