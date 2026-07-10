# Peloton BLE Bridge

Reads cadence and power from a Peloton Bike (original) via its serial port and re-broadcasts as a standard BLE Cycling Power Service (CPS) device. Works with Garmin Vivoactive 5, Zwift, and any BLE cycling power meter receiver.

## Hardware

### Parts
- ESP32 dev board (~$10)
- MAX3232 breakout module (~$2) — converts RS-232 voltage to 3.3V TTL
- 3.5mm TRRS cable (or TRRS splitter for non-destructive tap)
- Jumper wires

### Wiring

```
Peloton handlepost cable (TRRS)
  Tip    → HU TX (ignore — we don't need to send anything*)
  Ring1  → Bike TX  ──→  MAX3232 R1IN  →  MAX3232 R1OUT  →  ESP32 GPIO 16
  Ring2  → GND      ──→  ESP32 GND
  Sleeve → GND      ──→  ESP32 GND

* ESP32 TX (GPIO 17) → MAX3232 T1IN → MAX3232 T1OUT → Tip (TRRS)
  Required for the head unit startup sequence (ESP32 acts as the tablet)
```

> **Note:** The tablet must be disconnected. The ESP32 replaces it as the "head unit" that polls the bike.

### Tap method (non-destructive)
Use a TRRS Y-splitter on the handlepost cable. One output goes to the tablet as normal; the other goes to the MAX3232/ESP32. In this mode, the tablet handles startup and you passively read Ring1 — remove the TX wiring and adjust the firmware to passive-listen mode (see comments in `src/main.cpp`).

## Software

Built with [PlatformIO](https://platformio.org/).

```bash
# Install PlatformIO CLI
pip install platformio

# Build and flash
pio run --target upload

# Monitor serial output
pio device monitor
```

## BLE output

The device advertises as **"Peloton Power"** and exposes:
- **Cycling Power Service (0x1818)** — instantaneous power (watts) + crank cadence (RPM)

Pair it on your Vivoactive 5 under 
Settings → Sensors → Add Sensor → Power Meter.

## Protocol reference

- [PeloMon Part I — Decoding Peloton](https://ihaque.org/posts/2020/10/15/pelomon-part-i-decoding-peloton/)
- [PeloMon Part II — Emulating Peloton](https://ihaque.org/posts/2020/12/26/pelomon-part-ii-emulating-peloton/)
- [Gymnasticon — Peloton PR](https://github.com/ptx2/gymnasticon/pull/12)
