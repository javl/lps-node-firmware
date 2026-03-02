# lib/libdw3000

DW3000 driver integration layer for the LPS node firmware (ESP32-S3 port).

## Directory structure

```
lib/libdw3000/
├── README.md            ← this file
├── decadriver/          ← Qorvo DW3000 driver (must be populated, see below)
│   ├── deca_device.c
│   ├── deca_device_api.h
│   ├── deca_regs.h
│   ├── deca_vals.h
│   └── deca_version.h
├── inc/
│   ├── libdw3000_shim.h ← DW1000-compatible API shim (algorithm files #include this)
│   ├── libdw1000.h      ← Redirect: #include "libdw3000_shim.h"
│   └── deca_spi.h       ← ESP32-S3 platform SPI function declarations
└── src/
    └── libdw3000_shim.c ← Shim implementation (maps dw*() → dwt_*())
```

The ESP32-S3 platform SPI functions (`writetospi`, `readfromspi`, `deca_sleep`,
`wakeup_device_with_io`, `decamutexon`, `decamutexoff`) are implemented in
`src/dwOps.c` alongside the SPI bus initialisation.

---

## Step 1 — Obtain the Qorvo DW3000 decadriver

The `decadriver/` subdirectory must contain the five files listed above.
There are two equivalent sources:

### Option A — foldedtoad/dwm3000 (Zephyr port, easiest)

```bash
git clone --depth 1 https://github.com/foldedtoad/dwm3000.git /tmp/dwm3000
cp /tmp/dwm3000/decadriver/deca_device.c      lib/libdw3000/decadriver/
cp /tmp/dwm3000/decadriver/deca_device_api.h  lib/libdw3000/decadriver/
cp /tmp/dwm3000/decadriver/deca_regs.h        lib/libdw3000/decadriver/
cp /tmp/dwm3000/decadriver/deca_vals.h        lib/libdw3000/decadriver/
cp /tmp/dwm3000/decadriver/deca_version.h     lib/libdw3000/decadriver/
```

### Option B — Qorvo SDK (official)

Download the DWM3000EVB SDK from
<https://www.qorvo.com/products/p/DWM3000#documents> and copy the same five
files from its `decadriver/` folder.

---

## Step 2 — Build

After adding the decadriver files the project builds normally:

```bash
pio run -e anchor
```

`lib/libdw1000` is excluded from the build via `lib_ignore = libdw1000` in
`platformio.ini`; only `lib/libdw3000` is compiled.

---

## Architecture

```
Algorithm files                  shim                  DW3000 driver
(uwb_twr_anchor.c, …)          (libdw3000_shim.c)     (deca_device.c)
        │                            │                       │
   dwNewReceive(dev)  ──────►  dwt_forcetrxoff()            │
   dwStartReceive(dev) ──────► dwt_rxenable()               │
   dwSetData(dev, …)  ──────►  dwt_writetxdata()            │
                               dwt_writetxfctrl()           │
   dwStartTransmit()  ──────►  dwt_starttx()                │
   dwHandleInterrupt() ─────►  dwt_isr() ──────────────────►│
                                    ◄── _cb_txok/_cb_rxok ──┘
   dev->handleSent(dev)  ◄─────────┘
```

The DW3000 driver uses **global internal state** (`pdw3000local` in
`deca_device.c`) rather than a device handle. `dwDevice_t` in this shim is
kept purely as an application-level struct that stores callbacks and TX/RX
flags; it is not passed to any `dwt_*` function.

---

## Key differences from DW1000

| Feature              | DW1000                         | DW3000 (this shim)             |
|----------------------|--------------------------------|--------------------------------|
| Supported channels   | 1, 2, 3, 4, 5, 7               | **5 and 9 only**               |
| Supported data rates | 110K, 850K, 6.8M               | **850K and 6.8M only**         |
| PRF                  | 16 MHz or 64 MHz               | 64 MHz only (internal)         |
| Max SPI              | 20 MHz                         | 38 MHz (use ≤ 20 MHz in CRC mode)|
| Preamble codes Ch 5  | 3, 4, 11, 12, 19, 20           | **9–12**                       |
| Antenna delay reg    | 16-bit, single value           | Separate dwt_setrxantennadelay / dwt_settxantennadelay |
| Smart TX power       | Chip feature (auto reduce)     | Not present; configure via dwt_configuretxrf |
| Device ID            | 0xDECA0130                     | 0xDECA0302 / 0xDECA0312        |

---

## Antenna delay calibration

The default antenna delay in `src/uwb.c` is `16436` (both RX and TX), which is
the Qorvo reference value for the DWM3000 module at 64 MHz PRF on Channel 5.

To calibrate:
1. Place two nodes at a precisely known distance (e.g. 1.000 m).
2. Measure the reported range.
3. Adjust `DW3000_ANT_DLY` in `src/uwb.c` until the reported distance matches.
   Increasing the delay value increases the reported range.
