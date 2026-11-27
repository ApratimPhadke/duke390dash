# KTM Duke 390 — ESP32 CAN Bus Logger & Diagnostic Dashboard

A self-contained **ESP32** firmware that taps the CAN bus of a **KTM Duke 390 (Gen-1, 2013–2016, Bosch Motronic ME17)**, logs every frame, talks OBD-II / UDS / KWP2000 to the on-board modules, and serves a live web dashboard over its own Wi-Fi hotspot — no app, no cloud, no external libraries.

Everything the firmware transmits is **read-only diagnostics**. It never commands an actuator, never performs security access, never writes or clears anything.

![platform](https://img.shields.io/badge/platform-ESP32-000?logo=espressif)
![framework](https://img.shields.io/badge/framework-Arduino%20(arduino--esp32%203.x)-00979D?logo=arduino)
![bus](https://img.shields.io/badge/CAN-500%20kbit%2Fs%2C%2011--bit-orange)
![license](https://img.shields.io/badge/license-MIT-green)

> ⚠️ **Experimental, hobbyist project.** Connecting to a vehicle CAN bus carries risk. Read the [Safety](#safety--legal) section before wiring anything. Not affiliated with or endorsed by KTM, Bosch, or any company named below.

---

## Table of contents
- [What it does](#what-it-does)
- [Hardware / bill of materials](#hardware--bill-of-materials)
- [Wiring & pinout](#wiring--pinout)
- [Build & flash](#build--flash)
- [Using the dashboard](#using-the-dashboard)
- [Reverse-engineering the broadcast signals](#reverse-engineering-the-broadcast-signals)
- [ABS diagnostics (read-only)](#abs-diagnostics-read-only)
- [Known limitations & unverified data](#known-limitations--unverified-data)
- [Safety & legal](#safety--legal)
- [Acknowledgements & references](#acknowledgements--references)
- [License](#license)

---

## What it does

- **Auto-baud, listen-only sniffer.** Finds the bus bitrate (500k / 250k / 1M / 125k) without ever transmitting while it searches. On this bike the bus is confirmed **11-bit, 500 kbit/s**.
- **Full frame logger.** Per-ID table (rate, count, per-byte change highlighting and automatic classification: constant / counter / flag-bits / value / checksum), a 3000-frame ring buffer, and a **`candump`-format download** that opens in SavvyCAN and `can-utils` `canplayer`.
- **Module discovery.** Lists every module that answers OBD-II and sweeps the diagnostic address range, then asks each one read-only identification (UDS `22 F1xx`, KWP `1A xx`) and fault-memory questions.
- **Sensor inventory.** Polls every OBD-II PID the ECU advertises and shows the sensor behind it with a plausibility check, plus a run/trip view and an ECU odometer read (`01 A6`, if supported).
- **ECU data-block sweep.** Reads KWP2000 "read data by local identifier" (`21 01`…`21 FE`) blocks — where the raw internal sensor values often live.
- **Signal finder.** In the browser: correlates every broadcast byte / 16-bit word against the live OBD values, plus an **action detector** (operate the brake / clutch / side-stand and it finds the bit that changed) so you can name the bike's undocumented signals yourself. Saved on the phone.
- **Health check + key-on self-test.** 14 checks (bus link, transceiver loop-back, charging, coolant, idle quality, fuel trims…), with a report frozen a few seconds after key-on.
- **Read-only ABS fault reader.** Requests the ABS module's stored/pending fault memory and decodes each code with its standardized ISO-14229 status bits.

---

## Hardware / bill of materials

| Part | Notes |
|---|---|
| ESP32 dev board | Any ESP32 with TWAI (classic ESP32, S3, C3…) |
| CAN transceiver | **CJMCU-2551** (Microchip MCP2551, 5 V) as used here. A 3.3 V part (TI **SN65HVD230** or NXP **TJA1051T/3**) is cleaner and skips the level-shift resistors. |
| Buck converter | 12 V → 5 V, ≥ 1 A, to power the ESP32 from the bike |
| Resistors | 1 kΩ + 2 kΩ for the CRX level divider (only with a 5 V transceiver) |
| Capacitor | 470–1000 µF across the ESP32 5 V rail (prevents Wi-Fi brownout resets) |

---

## Wiring & pinout

### Bike CAN tap (4-wire connector observed at the rear of the Duke 390's module)

| Bike wire | Signal | Goes to |
|---|---|---|
| +12.5 V | Battery / ignition supply | Buck converter **IN+** |
| Ground | Chassis ground | Buck **IN−** and common ground |
| **Yellow** | CAN-H | Transceiver **CANH** |
| **Blue** | CAN-L | Transceiver **CANL** |

> Colours are as observed on this bike, not a guaranteed standard. Confirm with the 60 Ω termination check below before trusting them.

### ESP32 ↔ CJMCU-2551 (MCP2551)

| ESP32 | CJMCU-2551 | Notes |
|---|---|---|
| GPIO5 | CTX (TXD) | 3.3 V drive is enough for the MCP2551 input |
| GPIO4 | CRX (RXD) **via divider** | CRX is 5 V → **1 kΩ in series, then 2 kΩ to GND** before GPIO4 |
| 5V / VIN | VCC | 5 V from the buck converter |
| GND | GND | **Common ground with the bike is mandatory** |
| — | Rs (pin 8) → GND | Must sit near 0 V = high-speed mode (> 0.75·VCC = standby = dead) |

```
 Bike +12.5V ──► Buck IN+        Buck 5V ─┬─► ESP32 5V/VIN
 Bike GND  ───► Buck IN− ─┐               └─► CJMCU VCC
                          └── common GND ──► ESP32 GND ─── CJMCU GND
 Yellow (CAN-H) ───────────────────────────► CANH
 Blue   (CAN-L) ───────────────────────────► CANL
 ESP32 GPIO5 ───────────────────────────────► CTX
 CJMCU CRX ──[1k]──┬── GPIO4
                   └──[2k]── GND
```

**Termination check (ohmmeter, everything off):**
- Across the transceiver board's CANH/CANL: ~120 Ω means it has a terminating resistor — **remove it**, the bike bus is already terminated.
- Across the bike's yellow & blue, module unplugged: **~60 Ω = healthy bus**, ~120 Ω = one end missing, no reading = broken wire.

---

## Build & flash

1. **Arduino IDE** with the **arduino-esp32 core 3.x** (Boards Manager → "esp32 by Espressif"). No external libraries are needed.
2. Open `duke390_can_logger/duke390_can_logger.ino`.
3. **Tools → Board:** *ESP32 Dev Module* (or your board).
4. **Tools → Partition Scheme:** *Huge APP (3MB No OTA)* if you hit "sketch too big".
5. **Upload.** Open **Serial Monitor at 115200** to see the bitrate lock, the Wi-Fi IP, and a `[boot] reset reason` line (useful for diagnosing power brownouts).

Then join Wi-Fi **`DUKE390-CAN`** / password **`duke390can`** and open **`http://192.168.4.1`**.

---

## Using the dashboard

| Tab | What it shows |
|---|---|
| **Overview** | Live engine data cards, mileage, every decoded signal |
| **Health check** | 14 live checks + the frozen key-on self-test |
| **Sensors & modules** | Discovered modules, every OBD sensor, raw ECU data blocks |
| **Signal finder** | Auto-correlation + action detector to name unknown signals |
| **Bus IDs** | Every CAN ID with byte classification |
| **Raw log** | Live frames, record, and `candump` download |
| **Diagnostics** | Fault codes, OBD status, CAN controller counters |
| **ABS service** | Read-only ABS fault-code reader + pump-dead checklist |

First run with the **bike on its stand, ignition ON**. Once the bitrate locks, the firmware switches to normal mode and begins read-only OBD requests, like any scan tool.

---

## Reverse-engineering the broadcast signals

The KTM broadcast IDs (RPM, gear, wheel speed, lean…) are **not publicly documented for the 390**. The decode shipped here is borrowed from the 690/1290 maps and is marked *unverified*. The **Signal finder** tab lets you verify/derive them on your own bike:

1. **Auto-match:** engine running, blip the throttle over ~30 s, warm it up, roll the bike. Every broadcast byte is correlated against the ECU's own RPM / throttle / coolant / speed. A match near `|r| = 1` is the same signal, with the scale factor shown. Save it with one tap.
2. **Action detector:** press *Start*, hold still 5 s, then operate **one** control repeatedly for 6 s (front brake, clutch, side-stand, indicator…). The bits that changed **only** during the action are listed to name.

Your named signals are stored in the browser and exportable as JSON — please consider contributing verified 390 maps back.

---

## ABS diagnostics (read-only)

The **Bosch ABS 9M Base (9MB)** sits on the same CAN bus (responder `0x7E9`). The **ABS service** tab reads its stored fault memory (ISO 14229 service `19`, KWP `18`) and decodes each code with its standardized status bits (`confirmed`, `pending`, `failedSinceClear`…).

> **No pump or valve is ever commanded.** Actuator bleeding and any pump run must be done with the KTM dealer tool or a scanner that implements the 9MB bleed routine with its safety interlocks. Confirm brake function before riding. The exact meaning of each `C`-code is KTM/Bosch-specific — cross-reference the code number in service data.

The tab also carries a multimeter checklist for a dead pump: fault codes → the separate ABS pump-motor **fuse** → motor supply → ground → motor winding resistance.

---

## Known limitations & unverified data

- KTM broadcast decodes are **guesses from other KTM models** until you verify them with the Signal finder.
- Some standard OBD PIDs are known to be absent on this ECU; RPM may require the community `21 21` request. The firmware probes and falls back automatically.
- The ECU odometer (`01 A6`) is only present on newer ECUs; `01 31` is **distance-since-DTC-clear**, *not* total mileage, and is labelled as such.
- Switches wired straight to the ECU/dash (not onto CAN) will never appear in the sniffer.

---

## Safety & legal

- Get the wiring right **before** connecting to the bike. A wrong connection or a transceiver stuck in standby can disturb the bus. Do the first session stationary.
- This is a **read-only diagnostic** tool. It is **not** a substitute for a dealer diagnostic system, and it does not and must not control the braking system.
- Use at your own risk. The author and contributors accept no liability. Not affiliated with, authorised by, or endorsed by KTM, Bosch, Espressif, Microchip, or anyone else named here. All trademarks belong to their owners.

---

## Acknowledgements & references

**Vehicle & component makers (trademarks of their owners, named for identification only):**
- **KTM AG** — Duke 390 motorcycle
- **Robert Bosch GmbH** — Motronic ME17 engine ECU and ABS 9M Base (9MB) — [bosch-mobility.com](https://www.bosch-mobility.com/en/solutions/braking-systems/abs-module/)
- **Espressif Systems** — ESP32 and the arduino-esp32 core
- **Microchip Technology** — MCP2551 CAN transceiver (on the CJMCU-2551 board)
- **Texas Instruments** / **NXP Semiconductors** — SN65HVD230 / TJA1051 (alternative transceivers)
- **Arduino** — IDE & toolchain
- **TE Connectivity (Tyco)** — automotive connector series referenced in the 9MB spec

**Community references that made the CAN decoding possible:**
- [`blalor/ktm-can`](https://github.com/blalor/ktm-can) — KTM broadcast-frame decoding (690 Enduro R / 1290 Super Duke); the broadcast map adapted here
- [`sruon/ktm-1390-can`](https://github.com/sruon/ktm-1390-can) — further KTM CAN reference
- [RC390 Forum — OBD2 connector thread](https://www.rc390-forum.com/) — the non-standard PIDs and the community `21 21` RPM request
- [Klavkarr compatibility list](https://www.klavkarr.com/) — confirmed CAN 11-bit / 500 kbit/s on the platform

**Tooling the logs are compatible with:**
- [SavvyCAN](https://www.savvycan.com/) (Collin Kidder) — opens the exported `candump` logs
- [`linux-can/can-utils`](https://github.com/linux-can/can-utils) — `candump` / `canplayer` format

If you verify Duke 390 signal maps with the Signal finder, a PR back here is very welcome.

---

## License

MIT — see [`LICENSE`](LICENSE). Built by [Apratim Phadke](https://github.com/ApratimPhadke).
