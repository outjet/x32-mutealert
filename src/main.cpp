// =============================================================================
//  X32 Guitarist Mute Alert – ATOMS3R (Discreet Sentinel Edition)
//  Purpose: Monitors Pre-Mute Signal on Ch 8. Screen stays dark until muted.
// =============================================================================

#include <Arduino.h>
#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>

// ── Configuration ────────────────────────────────────────────────────────────
#define WIFI_SSID       "SJB Production"
#define WIFI_PASSWORD   "crestron!!"
#define X32_IP          "192.168.0.2" 
#define MONITOR_CHANNEL 8               
#define NOISE_FLOOR_DB  -75.0f          // Practical noise floor 
#define WINDOW_SEC      15              
#define SAMPLE_RATE_MS  200             

static constexpr int BUFFER_SIZE = (WINDOW_SEC * 1000) / SAMPLE_RATE_MS;
static constexpr int X32_PORT    = 10023; // [cite: 10]
static constexpr int LOCAL_PORT  = 10023; // [cite: 171]

// ── Globals ──────────────────────────────────────────────────────────────────
static WiFiUDP   g_udp;
static IPAddress g_x32Addr;
static float     g_instantDb    = -144.0f;
static float     g_averageDb    = -144.0f;
static uint32_t  g_lastRenewMs  = 0;
static uint32_t  g_lastSampleMs = 0;
static bool      g_isMutedAlert = false;
static float     g_history[BUFFER_SIZE]; 
static int       g_histIdx = 0;

// X32 meter values are Little-Endian Float32[cite: 63, 65].
static float readLeFloat(const uint8_t* p) {
    union { uint32_t u; float f; } converter;
    converter.u = ((uint32_t)p[3] << 24) | ((uint32_t)p[2] << 16) | 
                  ((uint32_t)p[1] << 8)  | (uint32_t)p[0];
    return (isnan(converter.f) || isinf(converter.f)) ? 0.0f : converter.f;
}

static int oscWriteStr(uint8_t* buf, int pos, const char* str) {
    int len = strlen(str) + 1;
    memcpy(buf + pos, str, len);
    pos += len;
    while (pos % 4 != 0) buf[pos++] = 0;
    return pos;
}

static void sendHeartbeat() {
    uint8_t buf[64];
    int pos = oscWriteStr(buf, pos, "/xremote");
    pos = oscWriteStr(buf, pos, ",");
    g_udp.beginPacket(g_x32Addr, X32_PORT);
    g_udp.write(buf, pos);
    g_udp.endPacket();

    pos = 0;
    pos = oscWriteStr(buf, pos, "/meters");
    pos = oscWriteStr(buf, pos, ",si");
    pos = oscWriteStr(buf, pos, "/meters/1"); 
    buf[pos++] = 0; buf[pos++] = 0; buf[pos++] = 0; buf[pos++] = 4; // ~200ms [cite: 165]
    g_udp.beginPacket(g_x32Addr, X32_PORT);
    g_udp.write(buf, pos);
    g_udp.endPacket();
}

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Display.setRotation(0);
    M5.Display.fillScreen(TFT_BLACK);
    Serial.begin(115200);

    // Initialize history with -60dB (0.001f) to prevent immediate boot-alert 
    for(int i=0; i<BUFFER_SIZE; i++) g_history[i] = 0.001f;

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) { delay(500); }

    g_udp.begin(LOCAL_PORT); 
    g_x32Addr.fromString(X32_IP);
    sendHeartbeat();
}

void loop() {
    const uint32_t now = millis();

    if (now - g_lastRenewMs >= 7000) {
        sendHeartbeat();
        g_lastRenewMs = now;
    }

    int pktSize = g_udp.parsePacket();
    if (pktSize >= 140) {
        uint8_t rx[512];
        int len = g_udp.read(rx, sizeof(rx));
        
        int blobStart = -1;
        for (int i = 4; i < 40; i++) {
            if (rx[i] == ',' && rx[i+1] == 'b') {
                blobStart = i + 8; // Skip tag, padding, and length [cite: 58, 62]
                break;
            }
        }

        if (blobStart != -1) {
            int channelOffset = blobStart + 4 + ((MONITOR_CHANNEL - 1) * 4);
            float linear = readLeFloat(rx + channelOffset); 
            if (linear > 1.0f) linear = 1.0f; // [cite: 78]

            if (now - g_lastSampleMs >= SAMPLE_RATE_MS) {
                g_instantDb = (linear <= 1e-7f) ? -144.0f : 20.0f * log10f(linear);
                
                g_history[g_histIdx] = linear;
                g_histIdx = (g_histIdx + 1) % BUFFER_SIZE;
                
                float sum = 0;
                for(int i=0; i<BUFFER_SIZE; i++) sum += g_history[i];
                float avgLin = sum / BUFFER_SIZE;
                g_averageDb = (avgLin <= 1e-7f) ? -144.0f : 20.0f * log10f(avgLin);

                g_lastSampleMs = now;
                Serial.printf("Instant: %+.1f dB | 15s Avg: %+.1f dB | Mute: %s\n", 
                              g_instantDb, g_averageDb, g_isMutedAlert ? "YES" : "NO");
            }
        }
    }

    g_isMutedAlert = (g_averageDb < NOISE_FLOOR_DB);
    
    if (g_isMutedAlert) {
        static uint32_t lastFlash = 0;
        static bool flashOn = false;
        if (now - lastFlash >= 500) {
            flashOn = !flashOn;
            M5.Display.fillScreen(flashOn ? TFT_RED : TFT_BLACK);
            if (flashOn) {
                M5.Display.setTextSize(2);
                M5.Display.setTextDatum(MC_DATUM);
                M5.Display.setTextColor(TFT_WHITE);
                M5.Display.drawString("MUTED", 64, 64); // Centered [64, 64]
            }
            lastFlash = now;
        }
    } else {
        static float lastDrawn = -999;
        if (fabs(g_averageDb - lastDrawn) > 1.0f) {
            M5.Display.startWrite();
            M5.Display.fillRect(0, 0, 128, 110, TFT_BLACK); // Clear alert area
            M5.Display.fillRect(0, 110, 128, 18, TFT_BLACK);
            M5.Display.setTextColor(0x4208); 
            M5.Display.setTextSize(1);
            M5.Display.setTextDatum(BC_DATUM);
            char buf[32];
            snprintf(buf, sizeof(buf), "Avg: %.1f dB", g_averageDb);
            M5.Display.drawString(buf, 64, 126);
            M5.Display.endWrite();
            lastDrawn = g_averageDb;
        }
    }
    delay(10);
}