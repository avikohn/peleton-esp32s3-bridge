/*
 * Peloton BLE Bridge
 *
 * Reads cadence + power off the Peloton bike's serial line and rebroadcasts
 * them as a standard BLE Cycling Power Service (CPS) device.
 *
 * Two polling modes, runtime-switchable (see `headUnitMode` below, default
 * false) via control multicast — no reflash needed (control listener is
 * currently disabled pending a stability investigation, see setup()):
 *   headless (false) — tablet stays connected and polls the bike; ESP32
 *       only listens on the bike's TX (response) line and never transmits.
 *   head unit (true) — ESP32 sends its own requests; tablet must be
 *       disconnected, since both driving the Tip line at once causes bus
 *       contention that breaks communication for both.
 * Either way, incoming responses are parsed the same way, since each one
 * is self-describing (echoes the command it's answering).
 *
 * Hardware:
 *   - Peloton TRRS cable → MAX3232 breakout → ESP32 UART2
 *   - TRRS Ring1 (Bike TX) → MAX3232 R1IN → MAX3232 R1OUT → GPIO 12 (RX)
 *   - TRRS Tip (Bike RX)   → MAX3232 T1OUT → MAX3232 T1IN → GPIO 13 (TX)
 *   - TRRS Ring2/Sleeve (GND) → ESP32 GND
 *
 * Protocol (from ihaque.org/posts/2020/10/15/pelomon-part-i-decoding-peloton):
 *   Request:  F5 [cmd] [checksum] F6  (checksum = sum of preceding bytes % 256)
 *   Response: F1 [echo_cmd] [len] [ascii_digits...] [checksum] F6
 *   Cadence (0x41): payload = ASCII RPM
 *   Power   (0x44): payload = ASCII deciwatts (divide by 10 for watts)
 *
 * Remote control: send a JSON object with any subset of these fields as a
 * UDP packet to the control multicast port (same group as everything else,
 * CONTROL_PORT):
 *   {"mode": "head" | "headless", "multicast": "a.b.c.d"}
 * "mode" toggles active polling; "multicast" redirects where the heartbeat
 * JSON is broadcast.
 * ponytail: the multicast control listener has no auth at all — anyone on
 * the LAN can send it commands. Fine for a home network, not for anything
 * more exposed.
 */

#include <Arduino.h>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "secrets.h"

#define WIFI_CONNECT_TIMEOUT_MS 10000  // don't block bike/BLE startup forever if WiFi's unavailable
#define TIME_SYNC_TIMEOUT_MS    10000  // don't block startup forever if NTP is unreachable
#define NTP_SERVER              "pool.ntp.org"

// --- Multicast ---
// 239.255.x.x is administratively-scoped (private-use) multicast — safe to
// pick an arbitrary address/port here without colliding with anything routed.
#define MULTICAST_IP       IPAddress(239, 255, 42, 99)
#define MULTICAST_PORT     41234  // structured heartbeat JSON (target is runtime-configurable)
#define DEBUG_LOG_PORT     41235  // free-text debug log lines, same group, always fixed
#define CONTROL_PORT       41236  // control messages in, same group, always fixed
#define BLE_MIRROR_PORT    41237  // raw CPS Measurement bytes, same rate as BLE notify — lets you
                                   // diff what's actually going out over BLE without a BLE client

// --- Pin config ---
#define PELOTON_RX_PIN  12
#define PELOTON_TX_PIN  13
#define PELOTON_BAUD    19200

// Physical mode jumper: short GPIO5 to GPIO4 for head-unit (active) mode;
// leave open for headless (safe default — no TX, no bus-contention risk).
// GPIO4 is just driven low as a local "ground" so no actual GND wire is needed.
#define MODE_JUMPER_GND_PIN   4
#define MODE_JUMPER_SENSE_PIN 5

// --- Protocol constants ---
#define PKT_RESPONSE_HEADER 0xF1
#define PKT_RESPONSE_FOOTER 0xF6

#define CMD_CADENCE      0x41
#define CMD_POWER        0x44
#define CMD_RESISTANCE   0x4A  // raw sensor value; needs CALIBRATION_TABLE to convert to %
#define CMD_CALIBRATION  0xF7  // one of 31 evenly-spaced (raw value, %) points, sent at bike boot
#define CALIBRATION_POINTS 31

#define PKT_REQUEST_HEADER 0xF5
#define PKT_REQUEST_FOOTER 0xF6
#define POLL_INTERVAL_MS   100  // matches stock head unit's own poll rate

#define HEARTBEAT_INTERVAL_MS 5000
#define NOTIFY_INTERVAL_MS    500  // BLE notify rate — decoupled from serial poll rate so
                                    // clients don't see repeated/stale crank data on every packet

// Status LED (onboard RGB) is only informative during startup — after the bike
// link is either established or clearly not, continuous flashing is just noise.
#define LED_STATUS_WINDOW_MS 60000
#define LED_FLASH_MS 50

// --- BLE UUIDs ---
// 16-bit SIG-assigned UUIDs. Use these directly (not their expanded 128-bit
// string forms) when constructing the GATT database — three reference CPS
// implementations all do this, and it's the last remaining structural
// difference from our layout. Some strict clients (e.g. Garmin Edge sensor
// search) reportedly key off the 16-bit form specifically in service
// discovery, not just advertising.
#define CPS_SERVICE_UUID       ((uint16_t)0x1818)
#define CPS_MEASUREMENT_UUID   ((uint16_t)0x2A63)
#define CPS_FEATURE_UUID       ((uint16_t)0x2A65)
#define CPS_LOCATION_UUID      ((uint16_t)0x2A5D)
#define CPS_CONTROL_POINT_UUID ((uint16_t)0x2A66)

// Cycling Power Control Point op codes we care about (org.bluetooth.characteristic.cycling_power_control_point)
#define CP_OPCODE_START_OFFSET_COMPENSATION 0x0C
#define CP_OPCODE_RESPONSE_CODE             0x20
#define CP_RESPONSE_SUCCESS                 0x01
#define CP_RESPONSE_OPCODE_NOT_SUPPORTED    0x02

// Device Information Service — some Garmin sensor-search validation paths
// expect this alongside the Cycling Power Service.
#define DIS_SERVICE_UUID      ((uint16_t)0x180A)
#define DIS_MANUFACTURER_UUID ((uint16_t)0x2A29)
#define DIS_MODEL_UUID        ((uint16_t)0x2A24)

// CPS Measurement flags
#define CPS_FLAGS_CRANK_DATA     0x0020  // bit 5: Crank Revolution Data Present
// Bits 0+1: Pedal Power Balance Present + Reference=Left. We repurpose this
// field to carry resistance % (0-100 → raw 0-200 at 0.5% resolution) so a
// CIQ data field can display it without any extra BLE connection.
#define CPS_FLAGS_PEDAL_BALANCE  0x0003

// --- State ---
static NimBLECharacteristic* cpsMeasurement = nullptr;
static int16_t  currentPower      = 0;   // watts
static uint16_t currentCadence    = 0;   // RPM
static uint8_t  currentResistance = 0;   // 0-100% (requires calibration table)
static uint8_t  calibCount        = 0;   // 0-31; 31 = table full, resistance valid
static uint16_t crankRevs         = 0;   // cumulative crank revolutions
static uint16_t lastCrankTime     = 0;   // in 1/1024 sec units

static uint32_t lastHeartbeatMs = 0;
static uint32_t lastNotifyMs    = 0;
static uint32_t lastCadenceMs   = 0;
static uint32_t rxByteCount     = 0;  // total bytes seen on the bike's TX line

static uint32_t lastValidMsgMs  = 0;  // millis() of the last fully-valid (checksum+footer+digits ok) frame on PELOTON_RX_PIN, 0 = none yet
static uint32_t lastRxStatusMs  = 0;
#define RX_STATUS_INTERVAL_MS 1000
#define RX_VALID_WINDOW_MS    2000

// Runtime-controllable via multicast control messages — see header comment.
static bool      headUnitMode          = false;  // safe default; the mode jumper takes over on first loop()
static IPAddress broadcastMulticastIp  = MULTICAST_IP;  // where the heartbeat JSON goes

// Single shared socket for all multicast traffic (send and receive) — three
// separate WiFiUDP objects (two send-only, one joined-for-receive) may have
// been contending for the ESP32 WiFi radio's limited multicast filter table;
// consolidating is a cheap way to test that theory.
static WiFiUDP      sharedUdp;

// ponytail: disabled — chatty over UDP during active polling. Flip true to remote-tail logs.
static bool debugLogMulticast = false;

// Prints locally (same as before) and, if enabled and WiFi's up, also
// broadcasts the same line as a UDP packet to DEBUG_LOG_PORT — lets you tail
// logs without a USB cable. Used for the hex dump too, so expect real traffic
// to be chatty.
static void debugLog(const char* fmt, ...) {
    char buf[192];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    // `if (Serial)` checks whether a USB host actually has the CDC port open
    // (DTR asserted) — skips the write instead of blocking/dropping into a
    // full TX buffer when nothing's attached to read it.
    if (Serial) Serial.println(buf);

    if (debugLogMulticast && WiFi.status() == WL_CONNECTED) {
        sharedUdp.beginPacket(MULTICAST_IP, DEBUG_LOG_PORT);
        sharedUdp.write((const uint8_t*)buf, strlen(buf));
        sharedUdp.write('\n');  // so raw listeners (socat, nc) show one line per packet
        sharedUdp.endPacket();
    }
}

// ---------------------------------------------------------------------------
// Remote control (control multicast)
// ---------------------------------------------------------------------------

// Applies any subset of {"mode", "multicast"} present in the JSON.
static void applyControlMessage(const char* json, size_t len) {
    StaticJsonDocument<128> doc;
    DeserializationError err = deserializeJson(doc, json, len);
    if (err) {
        debugLog("[control] bad JSON: %s", err.c_str());
        return;
    }

    if (doc.containsKey("mode")) {
        const char* mode = doc["mode"];
        if (strcmp(mode, "head") == 0) {
            headUnitMode = true;
            debugLog("[control] mode -> head");
        } else if (strcmp(mode, "headless") == 0) {
            headUnitMode = false;
            debugLog("[control] mode -> headless");
        } else {
            debugLog("[control] unknown mode: %s", mode);
        }
    }

    if (doc.containsKey("multicast")) {
        IPAddress newIp;
        const char* addr = doc["multicast"];
        if (newIp.fromString(addr)) {
            broadcastMulticastIp = newIp;
            debugLog("[control] multicast -> %s", addr);
        } else {
            debugLog("[control] bad multicast address: %s", addr);
        }
    }

}

// Non-blocking: only checks for one waiting packet per call, same pattern as
// pollBikeTraffic — call every loop().
static void checkControlMulticast() {
    int packetSize = sharedUdp.parsePacket();
    if (packetSize <= 0) return;

    char buf[128];
    int n = sharedUdp.read(buf, sizeof(buf) - 1);
    if (n <= 0) return;
    buf[n] = '\0';
    applyControlMessage(buf, n);
}

// ---------------------------------------------------------------------------
// Physical mode jumper
// ---------------------------------------------------------------------------

static void setupModeJumper() {
    pinMode(MODE_JUMPER_GND_PIN, OUTPUT);
    digitalWrite(MODE_JUMPER_GND_PIN, LOW);
    pinMode(MODE_JUMPER_SENSE_PIN, INPUT_PULLUP);
}

// Checked every loop() so the jumper works live — no reflash or reboot needed
// to switch modes, just add/remove the wire.
static void checkModeJumper() {
    bool jumpered = digitalRead(MODE_JUMPER_SENSE_PIN) == LOW;
    if (jumpered != headUnitMode) {
        headUnitMode = jumpered;
        debugLog("[jumper] mode -> %s", headUnitMode ? "head" : "headless");
    }
}

// ---------------------------------------------------------------------------
// Status LED
// ---------------------------------------------------------------------------

static uint32_t ledOffAtMs = 0;  // 0 = LED currently off

static void ledFlash(uint8_t r, uint8_t g, uint8_t b) {
    neopixelWrite(RGB_BUILTIN, r, g, b);
    ledOffAtMs = millis() + LED_FLASH_MS;
}

// Non-blocking: turns the flash off once its duration elapses. Call every loop().
static void ledUpdate() {
    if (ledOffAtMs != 0 && millis() >= ledOffAtMs) {
        neopixelWrite(RGB_BUILTIN, 0, 0, 0);
        ledOffAtMs = 0;
    }
}

// ---------------------------------------------------------------------------
// BLE setup
// ---------------------------------------------------------------------------

// Resumes advertising after a client disconnects, so a second device can find us
// — NimBLE stops advertising once connected and won't restart on its own.
static uint32_t connectedAtMs = 0;

// NimBLEServerCallbacks::onDisconnect doesn't expose the actual HCI disconnect
// reason, only ble_gap_event does — hook the raw GAP event stream just to log it.
static int gapEventHandler(ble_gap_event* event, void* arg) {
    if (event->type == BLE_GAP_EVENT_DISCONNECT) {
        debugLog("[ble] disconnect reason=%d", event->disconnect.reason);
    }
    return 0;  // let NimBLE's own handling continue as normal
}

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
        connectedAtMs = millis();
        debugLog("[ble] connected, conn_handle=%u", desc->conn_handle);
        // ponytail: tried requesting conn param update here (12/24/0/400) —
        // made drops faster (450-690ms vs 1.1-1.9s) and the Edge gave up
        // retrying sooner. Reverted; NimBLE's default negotiation isn't the
        // proven cause, don't re-add without new evidence.
    }
    void onDisconnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
        debugLog("[ble] disconnected after %lu ms", (unsigned long)(millis() - connectedAtMs));
        NimBLEDevice::getAdvertising()->start();
    }
    void onMTUChange(uint16_t mtu, ble_gap_conn_desc* desc) override {
        debugLog("[ble] MTU changed to %u", mtu);
    }
};

class MeasurementCallbacks : public NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic* chr, ble_gap_conn_desc* desc, uint16_t subValue) override {
        debugLog("[measurement] subscribe subValue=%u", subValue);
    }
};

// Head units (e.g. Garmin Edge) write calibration/config requests here and
// expect an indicated response even when there's nothing for us to actually
// calibrate — our power reading comes pre-computed from the bike, not a raw
// strain gauge. Answering "not supported" for everything we don't implement
// beats leaving the client hanging with no response at all.
class ControlPointCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* chr) override {
        std::string req = chr->getValue();
        uint8_t opcode = req.empty() ? 0 : (uint8_t)req[0];
        debugLog("[cp] write len=%u opcode=0x%02X raw=%02X %02X %02X %02X",
                 (unsigned)req.size(), opcode,
                 req.size() > 0 ? (uint8_t)req[0] : 0,
                 req.size() > 1 ? (uint8_t)req[1] : 0,
                 req.size() > 2 ? (uint8_t)req[2] : 0,
                 req.size() > 3 ? (uint8_t)req[3] : 0);

        uint8_t resp[3] = {CP_OPCODE_RESPONSE_CODE, opcode, CP_RESPONSE_OPCODE_NOT_SUPPORTED};
        if (opcode == CP_OPCODE_START_OFFSET_COMPENSATION) {
            resp[2] = CP_RESPONSE_SUCCESS;
        }
        chr->setValue(resp, sizeof(resp));
        chr->indicate();
        debugLog("[cp] responded value=0x%02X, indicate() called", resp[2]);
    }

    void onStatus(NimBLECharacteristic* chr, Status s, int code) override {
        debugLog("[cp] indicate status=%d code=%d", (int)s, code);
    }

    void onSubscribe(NimBLECharacteristic* chr, ble_gap_conn_desc* desc, uint16_t subValue) override {
        debugLog("[cp] subscribe subValue=%u", subValue);
    }
};

static void setupBLE() {
    NimBLEDevice::init("Peloton Power");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    NimBLEDevice::setCustomGapHandler(gapEventHandler);

    // ponytail: no security/bonding config — three independent reference CPS
    // implementations (incl. Yesoul_BLE, a same-purpose bike->Garmin Edge
    // bridge) advertise with zero pairing setup. We tried Just Works bonding
    // hoping it'd fix the Edge's ~1.2s deliberate disconnect; it didn't change
    // the timing at all, so it's unproven complexity — removed.
    NimBLEServer* server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());
    NimBLEService* cpsService = server->createService(NimBLEUUID(CPS_SERVICE_UUID));

    // Cycling Power Measurement — notify
    cpsMeasurement = cpsService->createCharacteristic(
        NimBLEUUID(CPS_MEASUREMENT_UUID),
        NIMBLE_PROPERTY::NOTIFY
    );
    cpsMeasurement->setCallbacks(new MeasurementCallbacks());

    // Cycling Power Feature — read
    NimBLECharacteristic* cpsFeature = cpsService->createCharacteristic(
        NimBLEUUID(CPS_FEATURE_UUID),
        NIMBLE_PROPERTY::READ
    );
    // Feature flags (CPS Feature characteristic, 0x2A65):
    //   bit 0 = Pedal Power Balance Supported (we repurpose for resistance %)
    //   bit 3 = Crank Revolution Data Supported
    uint32_t featureFlags = 0x00000009;
    cpsFeature->setValue(featureFlags);

    // Sensor Location — read (6 = Left Crank)
    NimBLECharacteristic* cpsLocation = cpsService->createCharacteristic(
        NimBLEUUID(CPS_LOCATION_UUID),
        NIMBLE_PROPERTY::READ
    );
    uint8_t location = 6;
    cpsLocation->setValue(&location, 1);

    cpsService->start();

    // Everything fits in the 31-byte legacy primary packet (flags 3 + UUID 4 +
    // appearance 4 + name 15 = 26 bytes) — put it all there and drop the scan
    // response entirely, in case the Garmin Edge passive-scans and never reads
    // scan response data the way generic phone/desktop scanners do.
    // Must add as a 16-bit UUID, not the full 128-bit string — passing the
    // 128-bit form bloats the AD field to 18 bytes (vs 4) and, more importantly,
    // sensor-search filters that look for the SIG 16-bit Service UUID AD type
    // (0x02/0x03) never find a service only advertised in the 128-bit type
    // (0x06/0x07).
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(NimBLEUUID(CPS_SERVICE_UUID));
    adv->setAppearance(0x0484);  // GAP category: Cycling: Power Sensor (0x0480 is generic "Cycling", not Power) — Garmin Edge sensor search filters on this
    adv->setName("Peloton Power");
    adv->setScanResponse(false);
    adv->setMinInterval(80);   // 80 * 0.625ms = 50ms
    adv->setMaxInterval(160);  // 160 * 0.625ms = 100ms
    adv->start();

    debugLog("BLE advertising as 'Peloton Power'");
}

// Syncs UTC time via NTP (bounded wait — timestamps just stay at epoch-ish
// values if this fails, nothing else depends on it).
static void syncTime() {
    configTime(0, 0, NTP_SERVER);  // 0, 0 offsets: keep everything in UTC

    Serial.print("Syncing time");
    uint32_t start = millis();
    time_t now = time(nullptr);
    while (now < 1000000000 && millis() - start < TIME_SYNC_TIMEOUT_MS) {  // ~2001-09-09; anything before means not synced
        delay(250);
        Serial.print(".");
        now = time(nullptr);
    }
    Serial.println();

    if (now >= 1000000000) {
        debugLog("Time synced: %lu (UTC epoch)", (unsigned long)now);
    } else {
        debugLog("Time sync failed — timestamps will be wrong until it succeeds later");
    }
}

// Connects to WiFi (bounded wait — bike/BLE must work even if WiFi's unavailable)
// and starts ArduinoOTA so firmware can be pushed without a USB cable.
static void setupOTA() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
        neopixelWrite(RGB_BUILTIN, 0, 0, 20);  // blue blink: connecting
        delay(125);
        neopixelWrite(RGB_BUILTIN, 0, 0, 0);
        delay(125);
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected — OTA unavailable this session");
        for (uint8_t i = 0; i < 5; i++) {  // red blink: failure
            neopixelWrite(RGB_BUILTIN, 20, 0, 0);
            delay(150);
            neopixelWrite(RGB_BUILTIN, 0, 0, 0);
            delay(150);
        }
        return;
    }

    debugLog("WiFi connected: %s", WiFi.localIP().toString().c_str());
    neopixelWrite(RGB_BUILTIN, 0, 20, 0);  // solid green: connected
    delay(500);
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);

    syncTime();

    ArduinoOTA.setHostname("peloton-bridge");
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.begin();
    debugLog("OTA ready");
}

// ---------------------------------------------------------------------------
// Multicast
// ---------------------------------------------------------------------------

static void broadcastHeartbeatMulticast() {
    if (WiFi.status() != WL_CONNECTED) return;
    char payload[160];
    int len = snprintf(payload, sizeof(payload),
                        "{\"timestamp\":%lu,\"cadence\":%u,\"power\":%d,\"resistance\":%u,\"rxBytes\":%lu,\"calib\":%u}",
                        (unsigned long)time(nullptr), currentCadence, currentPower, currentResistance, rxByteCount, calibCount);

    sharedUdp.beginPacket(broadcastMulticastIp, MULTICAST_PORT);
    sharedUdp.write((const uint8_t*)payload, len);
    sharedUdp.write('\n');  // so raw listeners (socat, nc) show one line per packet
    if (!sharedUdp.endPacket()) {
        debugLog("Multicast heartbeat send failed");
    }
}

// Joins the fixed control multicast group/port — this address never changes
// at runtime (only broadcastMulticastIp, the *send* target, does), so there's
// no chicken-and-egg problem of losing the ability to receive new commands.
// Using the same shared socket for sends too, per the single-socket theory.
static void setupControlListener() {
    sharedUdp.beginMulticast(MULTICAST_IP, CONTROL_PORT);
}

// ---------------------------------------------------------------------------
// BLE notification
// ---------------------------------------------------------------------------

// Mirrors the exact bytes just sent over BLE notify to a fixed multicast port,
// so you can diff what actually went out over the air without a BLE client.
static void broadcastBleMirror(const uint8_t* pkt, size_t len) {
    if (WiFi.status() != WL_CONNECTED) return;
    sharedUdp.beginPacket(MULTICAST_IP, BLE_MIRROR_PORT);
    sharedUdp.write(pkt, len);
    sharedUdp.endPacket();
}

static void notifyBLE() {
    // CPS Measurement packet (9 bytes):
    //   uint16 flags
    //   int16  instantaneous power (watts)
    //   uint8  pedal power balance (repurposed: resistance % × 2, per CPS 0.5% resolution)
    //   uint16 cumulative crank revolutions   (flags bit 5)
    //   uint16 last crank event time (1/1024s) (flags bit 5)
    uint8_t pkt[9];
    uint16_t flags = CPS_FLAGS_CRANK_DATA | CPS_FLAGS_PEDAL_BALANCE;
    memcpy(pkt + 0, &flags,          2);
    memcpy(pkt + 2, &currentPower,   2);
    pkt[4] = (uint8_t)(currentResistance * 2);  // 0-100% → 0-200 raw
    memcpy(pkt + 5, &crankRevs,      2);
    memcpy(pkt + 7, &lastCrankTime,  2);

    cpsMeasurement->setValue(pkt, sizeof(pkt));
    cpsMeasurement->notify();
    broadcastBleMirror(pkt, sizeof(pkt));
    debugLog("[notify] power=%d crankRevs=%u crankTime=%u", currentPower, crankRevs, lastCrankTime);
}

// Updates cumulative crank revolution counters from current cadence + elapsed time.
// Sub-revolution remainder carries over between calls — at real cadences and this
// bike's poll rate, a single interval is well under one revolution, so truncating
// per-call (instead of accumulating) would round every update down to zero.
static float pendingRevs = 0.0f;

static void updateCrankCounters(uint32_t elapsedMs) {
    if (currentCadence == 0) return;
    pendingRevs += (currentCadence / 60.0f) * (elapsedMs / 1000.0f);
    // Per CPS spec, Last Crank Event Time is the timestamp of the last
    // actually-completed crank revolution — not just "time has passed".
    // Advancing it by elapsedMs every call (as we used to) meant the Edge
    // saw ΔcrankTime > 0 with Δrevs = 0 in the notify windows between whole
    // revolutions, which it interpreted as "0 RPM", alternating with a real
    // value on windows where a revolution did complete. Only advance it
    // once per completed rev, by exactly one rev period at the reported
    // cadence — 60/C sec = 61440/C in 1/1024 sec units.
    while (pendingRevs >= 1.0f) {
        crankRevs++;
        pendingRevs -= 1.0f;
        lastCrankTime += (uint16_t)(61440UL / currentCadence);
    }
}

// ---------------------------------------------------------------------------
// Peloton serial protocol
// ---------------------------------------------------------------------------

static uint8_t pollCmds[]  = {CMD_CADENCE, CMD_POWER, CMD_RESISTANCE};
static uint8_t pollIndex   = 0;
static uint32_t lastPollMs = 0;

static uint32_t txByteCount = 0;  // total bytes we've written to Serial2 in head-unit mode

static void sendPollRequest() {
    uint8_t cmd = pollCmds[pollIndex];
    pollIndex = (pollIndex + 1) % 3;

    uint16_t sum = PKT_REQUEST_HEADER + cmd;
    uint8_t pkt[4] = {PKT_REQUEST_HEADER, cmd, (uint8_t)(sum & 0xFF), PKT_REQUEST_FOOTER};
    txByteCount += Serial2.write(pkt, sizeof(pkt));

    // ponytail: log every 31st TX (~3 s) — 31 is not divisible by 3 so all cmds get logged
    static uint8_t txLogCounter = 0;
    if (++txLogCounter >= 31) {
        txLogCounter = 0;
        debugLog("[tx] cmd=0x%02X pkt=%02X %02X %02X %02X  (total tx=%lu)", cmd, pkt[0], pkt[1], pkt[2], pkt[3], txByteCount);
    }
}

// Resistance calibration: the bike answers 31 (raw sensor value) points evenly
// spaced across 0-100%, but only once, at its own boot — we don't know which of
// the 31 we're getting since that index is on the request line we don't see in
// passive mode. Raw values are monotonically increasing with %, so keeping the
// table sorted by raw value recovers the mapping without needing the index.
// Table is persisted to NVS so a bike reboot isn't required after first capture.
static int32_t calibRaw[CALIBRATION_POINTS];

static void saveCalibration() {
    Preferences prefs;
    prefs.begin("calib", false);
    prefs.putBytes("raw", calibRaw, sizeof(calibRaw));
    prefs.end();
    debugLog("[calib] saved %u points to NVS", calibCount);
}

static void loadCalibration() {
    Preferences prefs;
    prefs.begin("calib", true);
    size_t loaded = prefs.getBytes("raw", calibRaw, sizeof(calibRaw));
    prefs.end();
    if (loaded == sizeof(calibRaw)) {
        calibCount = CALIBRATION_POINTS;
        debugLog("[calib] loaded %u points from NVS", calibCount);
    } else {
        debugLog("[calib] no saved table — waiting for bike boot");
    }
}

static void recordCalibrationPoint(int32_t raw) {
    if (calibCount >= CALIBRATION_POINTS) return;
    uint8_t i = calibCount;
    while (i > 0 && calibRaw[i - 1] > raw) {
        calibRaw[i] = calibRaw[i - 1];
        i--;
    }
    if (i > 0 && calibRaw[i - 1] == raw) return;  // duplicate point, discard
    calibRaw[i] = raw;
    calibCount++;
    if (calibCount == CALIBRATION_POINTS) saveCalibration();
}

static uint8_t resistancePercent(int32_t raw) {
    if (calibCount < CALIBRATION_POINTS) return 0;  // table not ready yet
    if (raw <= calibRaw[0]) return 0;
    if (raw >= calibRaw[CALIBRATION_POINTS - 1]) return 100;
    for (uint8_t i = 0; i < CALIBRATION_POINTS - 1; i++) {
        if (raw <= calibRaw[i + 1]) {
            float frac = (float)(raw - calibRaw[i]) / (calibRaw[i + 1] - calibRaw[i]);
            return (uint8_t)((i + frac) * 100.0f / (CALIBRATION_POINTS - 1));
        }
    }
    return 100;
}

static void applyMeasurement(uint8_t cmd, int32_t value) {
    if (cmd == CMD_CADENCE) {
        uint32_t now = millis();
        uint32_t elapsed = now - lastCadenceMs;
        if (lastCadenceMs != 0) updateCrankCounters(elapsed);
        lastCadenceMs = now;
        currentCadence = (uint16_t)value;
        debugLog("Cadence: %d RPM  (crankRevs=%u crankTime=%u)", currentCadence, crankRevs, lastCrankTime);
    } else if (cmd == CMD_POWER) {
        currentPower = (int16_t)(value / 10);  // deciwatts → watts
        debugLog("Power:   %d W", currentPower);
    } else if (cmd == CMD_RESISTANCE) {
        currentResistance = resistancePercent(value);
        debugLog("Resist:  %d%%", currentResistance);
    } else if (cmd == CMD_CALIBRATION) {
        recordCalibrationPoint(value);
        return;  // not a live measurement — nothing to notify over BLE
    } else {
        return;  // response to a command we don't care about
    }
}

// Consumes whatever bytes the bike has sent to the tablet so far, and applies
// any complete, valid response packet it finds. Non-blocking — call every loop().
static uint32_t badFrameCount = 0;

static void dumpFrame(const char* why, const uint8_t* frame, uint8_t len) {
    if (++badFrameCount > 8) return;  // avoid flooding the log
    char buf[128];
    int n = snprintf(buf, sizeof(buf), "%s:", why);
    for (uint8_t i = 0; i < len && n < (int)sizeof(buf) - 3; i++) {
        n += snprintf(buf + n, sizeof(buf) - n, " %02X", frame[i]);
    }
    debugLog("%s", buf);
}

// Classic 16-bytes-per-row hexdump -C style: offset, hex bytes, ASCII gutter.
// Buffered into one line and sent via debugLog so it's visible over the
// multicast log too, not just local Serial.
static void hexDumpByte(uint8_t b) {
    static uint8_t line[16];
    static uint8_t lineLen = 0;
    static uint32_t offset = 0;

    line[lineLen++] = b;
    if (lineLen < 16) return;

    char buf[96];
    int n = snprintf(buf, sizeof(buf), "%08lX  ", offset);
    for (uint8_t i = 0; i < 16; i++) {
        n += snprintf(buf + n, sizeof(buf) - n, "%02X ", line[i]);
        if (i == 7) n += snprintf(buf + n, sizeof(buf) - n, " ");
    }
    n += snprintf(buf + n, sizeof(buf) - n, " |");
    for (uint8_t i = 0; i < 16; i++) {
        char c = (char)line[i];
        buf[n++] = (c >= 0x20 && c < 0x7F) ? c : '.';
    }
    buf[n++] = '|';
    buf[n] = '\0';
    debugLog("%s", buf);

    offset += 16;
    lineLen = 0;
}

static void pollBikeTraffic() {
    static uint8_t buf[32];
    static uint8_t pos = 0;

    while (Serial2.available()) {
        uint8_t b = Serial2.read();
        rxByteCount++;
        hexDumpByte(b);

        if (pos == 0 && b != PKT_RESPONSE_HEADER) continue;
        buf[pos++] = b;
        if (pos < 3) continue;

        uint8_t echoCmd    = buf[1];
        uint8_t payloadLen = buf[2];

        // Bound payloadLen before computing packetLen, so "3 + payloadLen + 2"
        // can't wrap a uint8_t and slip past the size check below.
        if (payloadLen > sizeof(buf) - 5) {
            dumpFrame("Oversized payloadLen", buf, pos);
            pos = 0;
            continue;
        }

        uint8_t packetLen = 3 + payloadLen + 2;  // header+cmd+len + checksum+footer
        if (pos < packetLen) continue;           // wait for more bytes

        bool footerOk = (buf[packetLen - 1] == PKT_RESPONSE_FOOTER);
        if (!footerOk) {
            dumpFrame("Bad footer", buf, packetLen);
            pos = 0;
            continue;
        }

        // Checksum = sum of header+cmd+len+payload bytes, mod 256 — catches the
        // occasional serial-noise bit-flip that would otherwise decode as a
        // wildly wrong (but still framing-valid) value.
        uint8_t checksum = 0;
        for (uint8_t i = 0; i < packetLen - 2; i++) checksum += buf[i];
        if (checksum != buf[packetLen - 2]) {
            dumpFrame("Bad checksum", buf, packetLen);
            pos = 0;
            continue;
        }
        pos = 0;  // reset for next packet

        // Peloton sends ASCII digits little-endian (least-significant digit
        // first), per https://ihaque.org/posts/2020/10/15/pelomon-part-i-decoding-peloton/
        int32_t value = 0;
        int32_t place = 1;
        bool digitsOk = true;
        for (uint8_t i = 0; i < payloadLen; i++) {
            uint8_t digit = buf[3 + i];
            if (digit < 0x30 || digit > 0x39) { digitsOk = false; break; }
            value += (digit - 0x30) * place;
            place *= 10;
        }
        if (digitsOk) {
            lastValidMsgMs = millis();
            if (millis() < LED_STATUS_WINDOW_MS) ledFlash(0, 10, 10);  // cyan: bike data received
            applyMeasurement(echoCmd, value);
        } else {
            dumpFrame("Bad digits", buf, packetLen);
        }
    }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    delay(1500);  // let native USB CDC re-attach after reset before printing
    Serial.println("Peloton BLE Bridge starting...");

    // One-time diagnostic: with an internal pull-up enabled, a floating/
    // disconnected line reads HIGH. If it still reads LOW, something external
    // is actively holding it down (a short, or a driven-low output).
    pinMode(PELOTON_RX_PIN, INPUT_PULLUP);
    delay(10);
    Serial.printf("GPIO%d with internal pull-up: %s\n", PELOTON_RX_PIN,
                  digitalRead(PELOTON_RX_PIN) ? "HIGH (floating/disconnected)" : "LOW (actively held down)");

    Serial2.begin(PELOTON_BAUD, SERIAL_8N1, PELOTON_RX_PIN, PELOTON_TX_PIN);
    loadCalibration();

    setupBLE();
    setupOTA();
    // setupControlListener();  // ponytail: disabled again — reliably causes instability, needs USB-console root cause
    setupModeJumper();

    Serial.println("Ready — listening for bike traffic");
}

void loop() {
    uint32_t now = millis();

    ArduinoOTA.handle();
    ledUpdate();

    if (now - lastRxStatusMs >= RX_STATUS_INTERVAL_MS) {
        lastRxStatusMs = now;
        if (lastValidMsgMs != 0 && now - lastValidMsgMs <= RX_VALID_WINDOW_MS) {
            ledFlash(20, 0, 0);  // red: bike link alive (valid frame within last 2s)
        }
    }

    if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeatMs = now;
        badFrameCount = 0;  // reset so bad-frame dumps don't go permanently silent
        debugLog("[heartbeat] Cadence=%d RPM  Power=%d W  Resist=%d%%  TXbytes=%lu  RXbytes=%lu  GPIO%d=%s  Mode=%s  Jumper=%s  Built=%s",
                 currentCadence, currentPower, currentResistance, txByteCount, rxByteCount, PELOTON_RX_PIN,
                 digitalRead(PELOTON_RX_PIN) ? "HIGH" : "LOW",
                 headUnitMode ? "head" : "headless",
                 digitalRead(MODE_JUMPER_SENSE_PIN) ? "open" : "shorted",
                 __DATE__ " " __TIME__);
        broadcastHeartbeatMulticast();
    }

    if (now - lastNotifyMs >= NOTIFY_INTERVAL_MS) {
        lastNotifyMs = now;
        notifyBLE();
    }

    if (headUnitMode && now - lastPollMs >= POLL_INTERVAL_MS) {
        lastPollMs = now;
        sendPollRequest();
    }

    // checkControlMulticast();  // ponytail: disabled again — reliably causes instability, needs USB-console root cause
    checkModeJumper();

    pollBikeTraffic();
}
