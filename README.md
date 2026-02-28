# X32 Mute Monitor

An ESP32-S3 firmware for the **M5Stack AtomS3R** that watches a Behringer X32 input channel over Wi-Fi via OSC and flashes the screen red when the channel has been silent for too long — a discreet prompt for a guitarist (or any wireless-pack user) who forgot to unmute.

---

## Hardware

| Component | Detail |
|-----------|--------|
| Device | M5Stack AtomS3R (C126) |
| MCU | ESP32-S3FN8 — 8 MB flash, 8 MB OPI PSRAM |
| Display | 0.85″ GC9107, 128 × 128 px colour IPS |
| Network | 802.11 b/g/n Wi-Fi |
| Mixer | Behringer X32 (any variant with OSC support) |

---

## Prerequisites

- [PlatformIO](https://platformio.org/) (CLI or VS Code extension)
- A Wi-Fi network that both the AtomS3R and the X32 are on
- The X32 must have network connectivity enabled (Setup → Network on the console)
- The X32's IP address — assign it a static lease on your router for reliability

---

## Configuration

Edit **`src/config.h`** before building. At minimum fill in the Wi-Fi credentials and X32 IP:

```cpp
// Wi-Fi
#define WIFI_SSID        "YourNetworkSSID"
#define WIFI_PASSWORD    "YourNetworkPassword"

// X32
#define X32_IP           "192.168.1.100"   // set a static DHCP lease
#define X32_PORT         10023             // default X32 OSC port — don't change
#define LOCAL_UDP_PORT   10024             // port this device listens on

// What to watch
#define MONITOR_CHANNEL  8                 // input channel 1–32

// Alert threshold
#define NOISE_FLOOR_DB   -50.0f            // dBFS below which = "silent"
#define MUTE_TIMEOUT_MS  10000             // ms of silence before alert fires
```

### All parameters

| Define | Default | Description |
|--------|---------|-------------|
| `WIFI_SSID` | — | Wi-Fi network name |
| `WIFI_PASSWORD` | — | Wi-Fi password |
| `X32_IP` | — | X32 IP address on your LAN |
| `X32_PORT` | `10023` | X32 OSC UDP port |
| `LOCAL_UDP_PORT` | `10024` | Port the device listens on |
| `MONITOR_CHANNEL` | `8` | Input channel to monitor (1–32) |
| `NOISE_FLOOR_DB` | `-50.0` | Silence threshold in dBFS |
| `MUTE_TIMEOUT_MS` | `10000` | Silence duration before alert (ms) |
| `SUBSCRIBE_PACKETS` | `60` | Meter packets per subscription burst (~3 s) |
| `SUBSCRIBE_RENEW_MS` | `2500` | How often to re-subscribe (ms) |
| `XREMOTE_RENEW_MS` | `8000` | How often to renew `/xremote` registration (ms) |
| `FLASH_INTERVAL_MS` | `400` | Muted-alert flash period (ms) |
| `DEBUG_SERIAL` | `1` | Print dB + state to Serial at 2 Hz (set `0` to disable) |

---

## Build & Flash

```bash
# Install dependencies and build
pio run

# Build and upload (USB-C, device in normal mode)
pio run --target upload

# Serial monitor (115200 baud)
pio device monitor
```

If the upload fails, hold the **Reset** button for ~2 seconds until the green LED lights, then retry.

---

## Display

### Boot sequence

```
X32 Mute Monitor     →    Wi-Fi connected!     →    Registering X32...
AtomS3R  v1.0              MyNetwork                  192.168.1.100
                            192.168.1.42
```

### Normal operation

```
┌──────────────────────────┐
│       CHANNEL 8          │  ← header
│  ┌────────────────────┐  │
│  │       ██████       │  │  ← VU bar (−70 to 0 dBFS)
│  │       ██████       │  | Green → Yellow → Orange → Red
│  │  ─ ─ ─ ──────────  │  │  ← white tick = noise floor (−50 dB default)
│  │                    │  │
│  └────────────────────┘  │
│  −23.4 dB           ●    │  ← dB value | status dot
└──────────────────────────┘
```

**Status dot / footer indicator:**

| Indicator | Meaning |
|-----------|---------|
| Green dot | Signal present, level above threshold |
| Yellow "9s" countdown | Level below threshold, N seconds until alert |
| Orange "NODATA" | Connected to X32 but no meter packets received yet |
| Orange "NOCONN" | Wi-Fi is down |

### Muted alert

The screen alternates between solid red with white **MUTED / Ch 8** text and black at the `FLASH_INTERVAL_MS` rate (default 400 ms = 1.25 Hz). The flash clears the instant signal returns.

---

## OSC Protocol Details

The X32 uses OSC over UDP. This firmware sends two message types:

| Message | Rate | Purpose |
|---------|------|---------|
| `/xremote` | every 8 s | Registers this device; X32 learns where to push data |
| `/meters/1 ,i 60` | every 2.5 s | Requests 60 pre-fader meter packets at ~20 Hz |

Incoming `/meters/1` packets carry a blob of **32 × IEEE-754 float32** values (big-endian), one per input channel, in linear amplitude (0.0 = silence, 1.0 ≈ 0 dBFS). Channel 8 is at index 7 (0-based).

The parser handles both the 128-byte (bare floats) and 132-byte (4-byte count prefix + floats) blob variants found across X32 firmware versions.

---

## Calibration

With `DEBUG_SERIAL 1` and a serial monitor open at 115200 baud, you'll see output like:

```
[X32] Ch8  -27.3 dB  muted=0  wifi=1  data=1
[X32] Ch8  -25.8 dB  muted=0  wifi=1  data=1
[X32] Ch8  -82.1 dB  muted=0  wifi=1  data=1   ← pack muted
```

Play the instrument and note the typical active level; then mute the pack and note the floor. Set `NOISE_FLOOR_DB` halfway between the two. For most wireless guitar setups, the default of −50 dBFS works out of the box.

---

## Troubleshooting

| Symptom | Check |
|---------|-------|
| Stuck on "Connecting Wi-Fi" | SSID/password in `config.h`; 2.4 GHz band only |
| "NODATA" after connecting | X32 IP correct? Network port 10023 not firewalled? |
| Alert fires even when playing | `NOISE_FLOOR_DB` too high — lower it (e.g. `-40.0`) |
| Alert never fires | `NOISE_FLOOR_DB` too low — raise it (e.g. `-60.0`) |
| Upload fails | Hold Reset button until green LED lights, retry upload |
| Garbled display | Ensure M5Unified version ≥ 0.2.0 in `platformio.ini` |

---

## Project Structure

```
x32/
├── platformio.ini      Board config, library deps, build flags
└── src/
    ├── config.h        All user-tunable parameters
    └── main.cpp        Application (Wi-Fi, OSC, state machine, display)
```

No external OSC library is used — the protocol encoding and blob parsing are implemented directly for determinism and minimal dependencies.
