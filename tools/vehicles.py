"""Vehicle profiles for the cars in this garage.

`protocol` is the python-OBD / ELM327 protocol id to try first:
    "1" = SAE J1850 PWM (41.6 kbaud)   - Ford
    "2" = SAE J1850 VPW (10.4 kbaud)   - GM, most 2000s Chrysler
    "3" = ISO 9141-2                   - some Chrysler / imports
    "6".."9" = CAN                     - 2008+ (not used by these cars)
    None = let the adapter auto-detect (slower, but always a valid fallback)

If the first protocol fails, diagnose.py automatically retries with auto-detect.
"""

VEHICLES = {
    "unknown": {
        "name": "Unidentified vehicle",
        "protocol": None,
        "notes": ["Generic scan with protocol auto-detect. Rename the report once you "
                  "know which car it was."],
    },
    "avalanche": {
        "name": "2003 Chevrolet Avalanche 1500 4x4",
        "protocol": "2",
        "notes": [
            "GM Class 2 / J1850 VPW. OBD port is under the dash, left of the steering column.",
            "5.3L Vortec: common codes are P0300-P0308 (misfire), P0171/P0174 (lean - intake "
            "gaskets / MAF), P0440-P0455 (EVAP, often the gas cap or vent solenoid).",
        ],
    },
    "ranger": {
        "name": "2000 Ford Ranger",
        "protocol": "1",
        "notes": [
            "Ford uses J1850 PWM. MANY cheap ELM327 clones do NOT support PWM - "
            "if it won't connect, the adapter is the usual suspect (OBDLink / vLinker are fine).",
            "Common codes: P0171/P0174 (lean), P1131/P1151 (upstream O2 lack of switching), "
            "P0401 (EGR flow, 3.0L/4.0L), P1000 (monitors not complete - not a fault).",
        ],
    },
    "camaro": {
        # NOTE: Chevrolet built no 2005 Camaro (production ended with the 2002 model
        # year and resumed for 2010). Confirm the year from the VIN (10th character:
        # 2=2002, 1=2001, Y=2000 ...). All 1996-2002 Camaros use J1850 VPW.
        "name": "Chevrolet Camaro (listed as 2005 - confirm year from VIN)",
        "protocol": "2",
        "notes": [
            "No 2005 Camaro exists; 1998-2002 Camaros are J1850 VPW. If the VIN says it's a "
            "different car (e.g. 2005 Cobalt/Malibu also VPW or CAN), auto-detect will handle it.",
            "LS1 V8: P0300 misfire, P0171/P0174 lean, P0420/P0430 catalyst efficiency.",
        ],
    },
    "ram_van": {
        "name": "2003 Dodge Ram 3500 Van",
        "protocol": None,  # 2003 Chrysler is mostly J1850 VPW, a few used ISO 9141-2
        "notes": [
            "Chrysler 2003: usually J1850 VPW, sometimes ISO 9141-2 - auto-detect is used.",
            "3500-series vans can be over 8,500 lb GVWR. Heavy-duty vehicles were not fully "
            "OBD2-required in 2003, so some PIDs or monitors may be missing - that is normal.",
            "Common codes: P0300-series misfire, P0440-P0456 EVAP, P0700 (go read the "
            "transmission module - generic OBD2 can't see the 46RE/47RE codes).",
        ],
    },
    "ram_2500": {
        "name": "2003 Dodge Ram 2500 4x4",
        "protocol": None,
        "notes": [
            "Gas (5.7 Hemi / 8.0 V10): J1850 VPW, full generic OBD2.",
            "5.9 Cummins diesel: the engine ECM is not generic-OBD2 compliant in 2003 - expect "
            "very limited live data. Full engine data needs a J1939-capable tool.",
            "Over 8,500 lb GVWR, so some monitors may be missing - that is normal.",
        ],
    },
}
