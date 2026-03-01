// =============================================================================
//  X32 Mute Monitor – ATOMS3R
//
//  Connects to a Behringer X32 over Wi-Fi, subscribes to pre-fader input
//  channel meters via OSC (/meters/1), and monitors the configured channel
//  for sustained silence.  After MUTE_TIMEOUT_MS of continuous signal below
//  NOISE_FLOOR_DB the screen starts flashing red to alert the musician that
//  their wireless pack is muted.
//
//  OSC implementation is hand-rolled (no external library) for determinism.
// =============================================================================

#include <Arduino.h>
#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include "config.h"

// =============================================================================
//  Display geometry (AtomS3R: 128 × 128 px)
// =============================================================================
static constexpr int SCREEN_W  = 128;
static constexpr int SCREEN_H  = 128;

// Vertical VU bar
static constexpr int BAR_X     = 14;
static constexpr int BAR_W     = 100;
static constexpr int BAR_Y     = 20;   // top of bar
static constexpr int BAR_H     = 86;   // height in pixels

// dB range mapped to the bar
static constexpr float DB_MIN  = -70.0f;
static constexpr float DB_MAX  =   0.0f;

// Dark-grey bar background colour (RGB565 ≈ #202020)
static constexpr uint16_t COL_BAR_BG = 0x1084;

// Draw on change
static float g_lastDrawnDb = -9999.0f;
// =============================================================================
//  Globals
// =============================================================================
static WiFiUDP   g_udp;
static IPAddress g_x32Addr;

// Signal state
static float    g_levelDb       = -140.0f;  // last measured level
static uint32_t g_lastSignalMs  = 0;        // millis() when level last exceeded noise floor
static bool     g_muted         = false;
static bool     g_dataReceived  = false;    // true once first meter packet arrives

// Connection state
static bool     g_wifiOk        = false;

// Mix mute state (MIX_MUTE_PATH – e.g. main L/R bus)
static bool     g_mixMuted      = false;   // true = bus is muted
static bool     g_mixMuteKnown  = false;   // true once we've received a value

// Display state
static bool     g_flashOn       = false;

// Change-detection for normal screen redraws
static bool     g_lastMixMuted  = false;
static bool     g_lastMixKnown  = false;

// Timers
static uint32_t g_lastXremoteMs   = 0;
static uint32_t g_lastSubMs       = 0;
static uint32_t g_lastFlashMs     = 0;
static uint32_t g_lastDisplayMs   = 0;
static uint32_t g_lastWifiCheckMs = 0;
static uint32_t g_lastDebugMs     = 0;

// Scratch buffers
static uint8_t g_txBuf[64];
static uint8_t g_rxBuf[512];

// =============================================================================
//  Low-level helpers
// =============================================================================

// Read one big-endian IEEE-754 float32 from 4 bytes.
static float readBeFloat(const uint8_t* p)
{
    uint32_t u = ((uint32_t)p[0] << 24) |
                 ((uint32_t)p[1] << 16) |
                 ((uint32_t)p[2] <<  8) |
                  (uint32_t)p[3];
    float f;
    memcpy(&f, &u, 4);
    return f;
}

static float linearToDb(float linear)
{
    if (linear <= 1e-7f) return -140.0f;
    return 20.0f * log10f(linear);
}

// =============================================================================
//  OSC encoding
//  Strings are null-terminated and zero-padded to the next 4-byte boundary.
// =============================================================================
static int oscWriteStr(uint8_t* buf, int pos, const char* str)
{
    int len = (int)strlen(str) + 1;
    memcpy(buf + pos, str, len);
    pos += len;
    while (pos % 4 != 0) buf[pos++] = 0;
    return pos;
}

// Build:  /xremote  (no arguments) → 16 bytes
static int buildXremote(uint8_t* buf)
{
    int pos = 0;
    pos = oscWriteStr(buf, pos, "/xremote");
    pos = oscWriteStr(buf, pos, ",");   // type tag, no arguments
    return pos;
}

// Build:  /meters/1  ,i  n → 20 bytes
static int buildMetersRequest(uint8_t* buf, int32_t n)
{
    int pos = 0;
    pos = oscWriteStr(buf, pos, "/meters/1");
    pos = oscWriteStr(buf, pos, ",i");
    // int32 big-endian
    buf[pos++] = (uint8_t)(n >> 24);
    buf[pos++] = (uint8_t)(n >> 16);
    buf[pos++] = (uint8_t)(n >>  8);
    buf[pos++] = (uint8_t)(n);
    return pos;
}

// Build:  /addr  ,   (empty args – query/get current value)
static int buildOscGet(uint8_t* buf, const char* addr)
{
    int pos = 0;
    pos = oscWriteStr(buf, pos, addr);
    pos = oscWriteStr(buf, pos, ",");
    return pos;
}

// Build:  /addr  ,i  value
static int buildOscSetInt(uint8_t* buf, const char* addr, int32_t value)
{
    int pos = 0;
    pos = oscWriteStr(buf, pos, addr);
    pos = oscWriteStr(buf, pos, ",i");
    buf[pos++] = (uint8_t)(value >> 24);
    buf[pos++] = (uint8_t)(value >> 16);
    buf[pos++] = (uint8_t)(value >>  8);
    buf[pos++] = (uint8_t)(value);
    return pos;
}

// =============================================================================
//  OSC blob parsing
// =============================================================================

// Extract the linear level for channel (1-indexed) from the meter blob.
//
// X32 blob formats for /meters/1:
//   128 bytes  – 32 × float32 big-endian, no count prefix
//   132 bytes  – 4-byte count (uint32, any endian, value=32) + 32 × float32
//
// Returns 0.0f on any size/index error.
static float extractChannelLevel(const uint8_t* blob, int blobLen, int ch1)
{
    const int idx    = ch1 - 1;     // 0-based
    int       offset = 0;

    if      (blobLen == 128) { offset = 0; }
    else if (blobLen == 132) { offset = 4; }
    else {
#if DEBUG_SERIAL
        Serial.printf("[OSC] Unexpected blob length: %d bytes\n", blobLen);
#endif
        return 0.0f;
    }

    const int bytePos = offset + idx * 4;
    if (idx < 0 || bytePos + 4 > blobLen) return 0.0f;

    return readBeFloat(blob + bytePos);
}

// Parse an incoming UDP payload, look for /meters/1 with a blob argument,
// and return the linear level for ch1 (1-indexed).
// Returns -1.0f on parse failure or unrecognised packet.
static float parseMetersPacket(const uint8_t* buf, int bufLen, int ch1)
{
    const char* wantAddr = "/meters/1";
    const int   addrLen  = (int)strlen(wantAddr);

    if (bufLen < addrLen + 1) return -1.0f;
    if (memcmp(buf, wantAddr, addrLen) != 0) return -1.0f;

    // Skip address (string + null + padding to 4-byte boundary)
    int pos = addrLen + 1;
    while (pos % 4 != 0) pos++;

    // Type tag must start with ",b"
    if (pos + 2 > bufLen)                    return -1.0f;
    if (buf[pos] != ',' || buf[pos+1] != 'b') return -1.0f;

    // Skip type tag string
    while (pos < bufLen && buf[pos] != '\0') pos++;
    pos++;
    while (pos % 4 != 0) pos++;

    // Blob size prefix (4 bytes, big-endian)
    if (pos + 4 > bufLen) return -1.0f;
    const uint32_t blobSize = ((uint32_t)buf[pos+0] << 24) |
                              ((uint32_t)buf[pos+1] << 16) |
                              ((uint32_t)buf[pos+2] <<  8) |
                              ((uint32_t)buf[pos+3]);
    pos += 4;

    if (pos + (int)blobSize > bufLen) return -1.0f;

    return extractChannelLevel(buf + pos, (int)blobSize, ch1);
}

// Parse an OSC message with a single int32 argument at the given address.
// Returns the integer value on success, -1 on parse failure or address mismatch.
static int parseOscInt(const uint8_t* buf, int bufLen, const char* addr)
{
    const int addrLen = (int)strlen(addr);
    if (bufLen < addrLen + 1) return -1;
    if (memcmp(buf, addr, addrLen) != 0) return -1;

    // Skip address (null-terminated, padded to 4-byte boundary)
    int pos = addrLen + 1;
    while (pos % 4 != 0) pos++;

    // Expect type tag ",i\0\0" (4 bytes)
    if (pos + 4 > bufLen)                     return -1;
    if (buf[pos] != ',' || buf[pos+1] != 'i') return -1;
    pos += 4;

    // Read int32 big-endian
    if (pos + 4 > bufLen) return -1;
    return (int)(((int32_t)buf[pos+0] << 24) |
                 ((int32_t)buf[pos+1] << 16) |
                 ((int32_t)buf[pos+2] <<  8) |
                 ((int32_t)buf[pos+3]));
}

// =============================================================================
//  Display helpers
// =============================================================================

// Small boot/status text screen (up to 3 lines).
static void displayStatus(const char* l1, const char* l2 = nullptr, const char* l3 = nullptr)
{
    M5.Display.startWrite();
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextDatum(TL_DATUM);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setTextSize(1);
    int y = 8;
    if (l1) { M5.Display.drawString(l1, 4, y); y += 20; }
    if (l2) { M5.Display.drawString(l2, 4, y); y += 20; }
    if (l3) { M5.Display.drawString(l3, 4, y); }
    M5.Display.endWrite();
}

// Normal operating display: channel header + VU bar + footer info.
static void drawNormalScreen()
{
    const uint32_t now = millis();

    M5.Display.startWrite();
    M5.Display.fillScreen(TFT_BLACK);

    // ── Header ───────────────────────────────────────────────────────────────
    M5.Display.setTextSize(1);
    M5.Display.setTextDatum(TC_DATUM);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.drawString("CHANNEL " + String(MONITOR_CHANNEL), SCREEN_W / 2, 5);

    // ── L/R (mix) mute indicator – small coloured box, top-right corner ───────
    {
        const uint16_t col = !g_mixMuteKnown ? (uint16_t)0x4208   // dark grey = unknown
                           : g_mixMuted      ? TFT_RED
                           :                   TFT_GREEN;
        M5.Display.fillRect(SCREEN_W - 15, 1, 14, 14, col);
        M5.Display.setTextDatum(MC_DATUM);
        M5.Display.setTextColor(g_mixMuted ? TFT_WHITE : TFT_BLACK, col);
        M5.Display.drawString("LR", SCREEN_W - 8, 8);
    }

    // ── Bar background ────────────────────────────────────────────────────────
    M5.Display.fillRect(BAR_X, BAR_Y, BAR_W, BAR_H, COL_BAR_BG);

    // ── Noise floor tick mark (white horizontal line) ─────────────────────────
    // Marks where the alert threshold is on the bar.
    {
        const float floorFrac = (NOISE_FLOOR_DB - DB_MIN) / (DB_MAX - DB_MIN);
        const int   tickY     = BAR_Y + BAR_H - (int)(floorFrac * (float)BAR_H);
        M5.Display.drawFastHLine(BAR_X, tickY, BAR_W, TFT_WHITE);
    }

    // ── Filled level bar ──────────────────────────────────────────────────────
    {
        const float clamped = max(DB_MIN, min(DB_MAX, g_levelDb));
        const int   fillH   = max(0, min(BAR_H,
                              (int)((clamped - DB_MIN) / (DB_MAX - DB_MIN) * (float)BAR_H)));
        if (fillH > 0) {
            const float pct = (float)fillH / (float)BAR_H;
            uint16_t colour;
            if      (pct < 0.70f) colour = TFT_GREEN;
            else if (pct < 0.85f) colour = TFT_YELLOW;
            else if (pct < 0.95f) colour = TFT_ORANGE;
            else                  colour = TFT_RED;

            M5.Display.fillRect(BAR_X,
                                BAR_Y + BAR_H - fillH,
                                BAR_W, fillH, colour);
        }
    }

    // ── Footer: dB value (left) + status indicator (right) ────────────────────
    {
        char dbStr[14];
        if (g_levelDb <= -139.0f)
            snprintf(dbStr, sizeof(dbStr), " -inf dB");
        else
            snprintf(dbStr, sizeof(dbStr), "%+6.1f dB", g_levelDb);

        M5.Display.setTextSize(1);
        M5.Display.setTextDatum(ML_DATUM);
        M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
        M5.Display.drawString(dbStr, 2, 118);

        // Right-hand status indicator
        if (!g_wifiOk) {
            M5.Display.setTextDatum(MR_DATUM);
            M5.Display.setTextColor(TFT_ORANGE, TFT_BLACK);
            M5.Display.drawString("NOCONN", SCREEN_W - 2, 118);

        } else if (!g_dataReceived) {
            M5.Display.setTextDatum(MR_DATUM);
            M5.Display.setTextColor(TFT_ORANGE, TFT_BLACK);
            M5.Display.drawString("NODATA", SCREEN_W - 2, 118);

        } else if (g_levelDb > NOISE_FLOOR_DB) {
            // Signal present → green circle
            M5.Display.fillCircle(SCREEN_W - 8, 118, 5, TFT_GREEN);

        } else {
            // Silence timing → yellow countdown (e.g. "9s", "10s")
            const uint32_t elapsed   = now - g_lastSignalMs;
            const int      remaining = (elapsed >= MUTE_TIMEOUT_MS) ? 0 :
                                       (int)((MUTE_TIMEOUT_MS - elapsed + 999UL) / 1000UL);
            char cntStr[6];
            snprintf(cntStr, sizeof(cntStr), "%ds", remaining);
            M5.Display.setTextDatum(MR_DATUM);
            M5.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
            M5.Display.drawString(cntStr, SCREEN_W - 2, 118);
        }
    }

    M5.Display.endWrite();
}

// Muted alert – red screen with "MUTED" text.
static void drawMutedOn()
{
    M5.Display.startWrite();
    M5.Display.fillScreen(TFT_RED);
    M5.Display.setTextColor(TFT_WHITE, TFT_RED);
    M5.Display.setTextDatum(MC_DATUM);
    M5.Display.setTextSize(3);
    M5.Display.drawString("MUTED", SCREEN_W / 2, SCREEN_H / 2 - 14);
    M5.Display.setTextSize(1);
    M5.Display.drawString("Ch " + String(MONITOR_CHANNEL), SCREEN_W / 2, SCREEN_H / 2 + 18);

    // ── L/R mix mute indicator (top-right, dark-red when muted so it shows on red bg)
    {
        const uint16_t col = !g_mixMuteKnown ? (uint16_t)0x4208   // dark grey
                           : g_mixMuted      ? (uint16_t)0x5800   // dark red on red bg
                           :                   TFT_GREEN;
        M5.Display.fillRect(SCREEN_W - 15, 1, 14, 14, col);
        M5.Display.setTextDatum(MC_DATUM);
        M5.Display.setTextColor(TFT_WHITE, col);
        M5.Display.drawString("LR", SCREEN_W - 8, 8);
    }
    M5.Display.endWrite();
}

static void drawMutedOff()
{
    M5.Display.startWrite();
    M5.Display.fillScreen(TFT_BLACK);

    // ── L/R mix mute indicator (keep visible even on the "off" flash frame)
    {
        const uint16_t col = !g_mixMuteKnown ? (uint16_t)0x4208
                           : g_mixMuted      ? TFT_RED
                           :                   TFT_GREEN;
        M5.Display.fillRect(SCREEN_W - 15, 1, 14, 14, col);
        M5.Display.setTextDatum(MC_DATUM);
        M5.Display.setTextColor(g_mixMuted ? TFT_WHITE : TFT_BLACK, col);
        M5.Display.drawString("LR", SCREEN_W - 8, 8);
    }
    M5.Display.endWrite();
}

// =============================================================================
//  Wi-Fi
// =============================================================================

// Blocking initial connection shown on-screen.  Restarts if it times out.
static void connectWiFi()
{
    displayStatus("Connecting Wi-Fi", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        if (++attempts > 60) {           // 30-second timeout
            displayStatus("Wi-Fi timeout.", "Check config.h", "Rebooting...");
            delay(3000);
            ESP.restart();
        }
    }

    const String ip = WiFi.localIP().toString();
    displayStatus("Wi-Fi connected!", WIFI_SSID, ip.c_str());
    delay(1500);
    g_wifiOk = true;
}

// Non-blocking reconnect watchdog called every 5 s from the main loop.
static void checkWiFi()
{
    if (WiFi.status() == WL_CONNECTED) {
        if (!g_wifiOk) {
            // Just reconnected – restart UDP and force immediate subscription.
            g_wifiOk = true;
            g_udp.begin(LOCAL_UDP_PORT);
            g_lastXremoteMs = 0;
            g_lastSubMs     = 0;
            queryMixMute();   // refresh mix mute state after reconnect
        }
        return;
    }
    // Not connected
    g_wifiOk       = false;
    g_mixMuteKnown = false;   // lose knowledge of mix state; indicator goes grey
    WiFi.reconnect();
}

// =============================================================================
//  OSC send helpers
// =============================================================================
static void sendXremote()
{
    if (!g_wifiOk) return;
    const int sz = buildXremote(g_txBuf);
    g_udp.beginPacket(g_x32Addr, X32_PORT);
    g_udp.write(g_txBuf, sz);
    g_udp.endPacket();
}

static void sendMetersSubscription()
{
    if (!g_wifiOk) return;
    const int sz = buildMetersRequest(g_txBuf, SUBSCRIBE_PACKETS);
    g_udp.beginPacket(g_x32Addr, X32_PORT);
    g_udp.write(g_txBuf, sz);
    g_udp.endPacket();
}

// Query the current value of MIX_MUTE_PATH from the X32.
static void queryMixMute()
{
    if (!g_wifiOk) return;
    const int sz = buildOscGet(g_txBuf, MIX_MUTE_PATH);
    g_udp.beginPacket(g_x32Addr, X32_PORT);
    g_udp.write(g_txBuf, sz);
    g_udp.endPacket();
}

// Set MIX_MUTE_PATH on the X32.  X32 convention: 1 = on/unmuted, 0 = muted.
static void sendMixMuteSet(bool muted)
{
    if (!g_wifiOk) return;
    const int sz = buildOscSetInt(g_txBuf, MIX_MUTE_PATH, muted ? 0 : 1);
    g_udp.beginPacket(g_x32Addr, X32_PORT);
    g_udp.write(g_txBuf, sz);
    g_udp.endPacket();
}

// =============================================================================
//  OSC receive
// =============================================================================
static void handleIncomingOSC()
{
    const int pktSize = g_udp.parsePacket();
    if (pktSize <= 0) return;

    if (pktSize > (int)sizeof(g_rxBuf)) {
        g_udp.flush();
        return;
    }

    const int len = g_udp.read(g_rxBuf, sizeof(g_rxBuf));
    if (len <= 0) return;

    // ── Meter packet ─────────────────────────────────────────────────────────
    const float linear = parseMetersPacket(g_rxBuf, len, MONITOR_CHANNEL);
    if (linear >= 0.0f) {
        // On first valid packet, reset the silence timer so we get a clean
        // window from the moment monitoring actually starts.
        if (!g_dataReceived) {
            g_lastSignalMs = millis();
            g_dataReceived = true;
        }
        g_levelDb = linearToDb(linear);
        return;
    }

    // ── Mix mute state (e.g. main L/R) ───────────────────────────────────────
    const int mixOn = parseOscInt(g_rxBuf, len, MIX_MUTE_PATH);
    if (mixOn >= 0) {
        g_mixMuted     = (mixOn == 0);   // X32: 1=on(unmuted), 0=off(muted)
        g_mixMuteKnown = true;
    }
}

// =============================================================================
//  Arduino entry points
// =============================================================================
void setup()
{
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Display.setRotation(0);
    Serial.begin(115200);
    delay(300);

    displayStatus("X32 Mute Monitor", "AtomS3R  v1.0");
    delay(1000);

    connectWiFi();

    g_udp.begin(LOCAL_UDP_PORT);
    g_x32Addr.fromString(X32_IP);

    displayStatus("Registering X32...", X32_IP);

    // Initial /xremote registration + meter subscription + mix mute query
    sendXremote();
    sendMetersSubscription();
    queryMixMute();

    const uint32_t now = millis();
    g_lastXremoteMs = now;
    g_lastSubMs     = now;
    g_lastSignalMs  = now;    // grace period before first alert

    delay(500);
    drawNormalScreen();
}

void loop()
{
    M5.update();
    const uint32_t now = millis();

    // ── Button: toggle L/R (mix) mute ────────────────────────────────────────
    if (M5.BtnA.wasPressed() && g_mixMuteKnown && g_wifiOk) {
        g_mixMuted = !g_mixMuted;
        sendMixMuteSet(g_mixMuted);
    }

    // ── Wi-Fi watchdog (every 5 s) ────────────────────────────────────────────
    if (now - g_lastWifiCheckMs >= 5000UL) {
        checkWiFi();
        g_lastWifiCheckMs = now;
    }

    // ── OSC subscription renewals ─────────────────────────────────────────────
    if (g_wifiOk) {
        if (now - g_lastXremoteMs >= XREMOTE_RENEW_MS) {
            sendXremote();
            g_lastXremoteMs = now;
        }
        if (now - g_lastSubMs >= SUBSCRIBE_RENEW_MS) {
            sendMetersSubscription();
            g_lastSubMs = now;
        }
    }

    // ── Receive and parse incoming OSC ────────────────────────────────────────
    handleIncomingOSC();

    // ── Mute state machine ────────────────────────────────────────────────────
    // Only start counting once we have real data from the X32.
    if (g_dataReceived) {
        if (g_levelDb > NOISE_FLOOR_DB) {
            g_lastSignalMs = now;
            g_muted = false;
        } else if (!g_muted && (now - g_lastSignalMs) >= MUTE_TIMEOUT_MS) {
            g_muted = true;
        }
    }

    // ── Display refresh ─────────────────────────────────────────────
    if (g_muted) {
        if (now - g_lastFlashMs >= FLASH_INTERVAL_MS) {
            g_flashOn     = !g_flashOn;
            g_lastFlashMs = now;
            if (g_flashOn) drawMutedOn();
            else           drawMutedOff();
        }
    } else {
        if (g_dataReceived && (fabs(g_levelDb - g_lastDrawnDb) > 0.5f ||
                               g_mixMuted    != g_lastMixMuted ||
                               g_mixMuteKnown != g_lastMixKnown)) {
            drawNormalScreen();
            g_lastDrawnDb  = g_levelDb;
            g_lastMixMuted = g_mixMuted;
            g_lastMixKnown = g_mixMuteKnown;
        }
    }

    // ── Serial debug (2 Hz) ───────────────────────────────────────────────────
#if DEBUG_SERIAL
    if (now - g_lastDebugMs >= 500UL) {
        g_lastDebugMs = now;
        Serial.printf("[X32] Ch%d  %+.1f dB  chMuted=%d  lrMuted=%d(%s)  wifi=%d  data=%d\n",
                      MONITOR_CHANNEL, g_levelDb,
                      (int)g_muted, (int)g_mixMuted,
                      g_mixMuteKnown ? "known" : "?",
                      (int)g_wifiOk, (int)g_dataReceived);
    }
#endif

    delay(5);
}
