# Peloton BLE Bridge

Reads cadence and power from a Peloton Bike (original) via its serial port and re-broadcasts as a standard BLE Cycling Power Service (CPS) device. Works with the Garmin Edge 530, Zwift, and any BLE cycling power meter receiver.

Inspired by [ihaque's excellent PeloMon reverse engineering](https://ihaque.org/posts/2020/10/15/pelomon-part-i-decoding-peloton/) — the protocol decoding wouldn't exist without that work.

## Features

### Auto-detecting polling mode

The bridge starts in **headless mode** (listen-only) and auto-detects whether a tablet is present:

1. If the bike's TX line goes silent for >1 s, it switches to **active polling** (sends its own requests).
2. If polling gets no response either, it enters a brief **probe** window.
3. If bytes appear during probe, a tablet was found — switches back to headless. Otherwise stays polling.

No configuration needed. If you run without a tablet, the bridge takes over automatically.

### BLE Cycling Power Service

Advertises as **"Peloton Power"** with the standard Cycling Power Service (0x1818):
- Instantaneous power (watts)
- Crank cadence (RPM) via cumulative crank revolution data

Correctly implements the CPS crank timestamp spec so cadence reads accurately on strict clients. Confirmed working as a power meter and cadence source on the **Garmin Edge 530**.

### Multicast debug broadcast

All multicast is sent to 239.255.42.99. Listen from any machine on the LAN:

| Port  | Content | Always on? |
|-------|---------|------------|
| 41234 | Heartbeat JSON every 5 s: `{"timestamp":…,"cadence":…,"power":…,"resistance":…,"rxBytes":…,"calib":…}` | Yes (when WiFi connected) |
| 41235 | Free-text debug log lines | No — set `debugLogMulticast = true` in firmware |
| 41236 | Raw CPS Measurement bytes (same cadence as BLE notify) | Yes (when WiFi connected) |

```bash
# Heartbeat JSON
socat UDP4-RECVFROM:41234,ip-add-membership=239.255.42.99:0.0.0.0,reuseaddr -

# Debug log stream (requires debugLogMulticast = true)
socat UDP4-RECVFROM:41235,ip-add-membership=239.255.42.99:0.0.0.0,reuseaddr -
```

### OTA firmware updates

Connects to WiFi on boot and registers with ArduinoOTA as `peloton-bridge`. Push new firmware without a USB cable:

```bash
pio run --target upload --upload-port peloton-bridge.local
```

WiFi credentials and OTA password live in `include/secrets.h` (not checked in). OTA failure is non-fatal — the bridge continues working with no WiFi.

### Resistance tracking

Decodes the bike's resistance calibration table (31 points, sent at bike boot) and logs resistance as a percentage alongside cadence and power. Not transmitted over BLE (CPS has no resistance field), but visible in the multicast heartbeat JSON.

## Hardware

### Parts
- ESP32 dev board (~$10)
- MAX3232 breakout module (~$2) — converts RS-232 voltage to 3.3V TTL
- 3.5mm TRRS cable (or TRRS splitter for non-destructive tap)
- Jumper wires

### Wiring

```
Peloton handlepost cable (TRRS)
  Ring1  → Bike TX (RS-232)  ──→  MAX3232 R1IN → MAX3232 R1OUT (RXD)  ──→  ESP32 GPIO 12 (UART2 RX)
  Ring2  → GND               ──→  ESP32 GND
  Sleeve → GND               ──→  ESP32 GND
  Tip    → Bike RX (RS-232)  ←──  MAX3232 T1OUT ← MAX3232 T1IN (TXD)  ←──  ESP32 GPIO 13 (UART2 TX)*

* GPIO 13 / TXD only needed in head-unit mode (ESP32 actively polls the bike).
  In headless mode, omit this wire — the tablet drives Tip instead.
```

**Summary:** MAX3232 breakout **RXD** pin → GPIO 12 (inbound from bike), **TXD** pin → GPIO 13 (outbound to bike).

> **Note:** In active polling mode the tablet must be disconnected — both driving the Tip line simultaneously causes bus contention.

### Tap method (non-destructive)

Use a TRRS Y-splitter on the handlepost cable. One output goes to the tablet; the other goes to the MAX3232/ESP32. In this mode the tablet handles startup; run in headless mode and omit the TX wiring.

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

Copy `include/secrets.h.example` to `include/secrets.h` and fill in your WiFi SSID, password, and OTA password before building.

## Protocol reference

- [PeloMon Part I — Decoding Peloton](https://ihaque.org/posts/2020/10/15/pelomon-part-i-decoding-peloton/)
- [PeloMon Part II — Emulating Peloton](https://ihaque.org/posts/2020/12/26/pelomon-part-ii-emulating-peloton/)
- [Gymnasticon — Peloton PR](https://github.com/ptx2/gymnasticon/pull/12)
