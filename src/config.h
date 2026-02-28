#pragma once

// =============================================================================
//  X32 Mute Monitor – User Configuration
//  Edit the values below to match your environment.
// =============================================================================

// ── Wi-Fi ─────────────────────────────────────────────────────────────────────
#define WIFI_SSID           "YourNetworkSSID"
#define WIFI_PASSWORD       "YourNetworkPassword"

// ── Behringer X32 ─────────────────────────────────────────────────────────────
// IP address of the X32 on your LAN (set a static lease on your router).
#define X32_IP              "192.168.1.100"
// OSC UDP port – 10023 is the X32 default. Do not change unless you have
// a specific reason to.
#define X32_PORT            10023
// Local UDP port this device listens on for meter data pushed by the X32.
#define LOCAL_UDP_PORT      10024

// ── Channel to monitor ────────────────────────────────────────────────────────
// Input channel 1–32 (pre-fader meter, /meters/1 blob index 0-based).
#define MONITOR_CHANNEL     8

// ── Signal detection ──────────────────────────────────────────────────────────
// Level below which we consider the channel "silent" (dBFS).
// A guitar through a wireless pack typically reads −30 to −10 dBFS when playing.
// A quiet preamp with nothing plugged in is typically below −80 dBFS.
// −50 dBFS is a safe default for most live-sound environments.
#define NOISE_FLOOR_DB      -50.0f

// Duration of continuous silence before the MUTED alert fires (milliseconds).
#define MUTE_TIMEOUT_MS     10000UL

// ── OSC subscription timing ───────────────────────────────────────────────────
// The X32 sends one meter packet per ~50 ms after a /meters/1 subscription.
// SUBSCRIBE_PACKETS × 50 ms must be longer than SUBSCRIBE_RENEW_MS.
#define SUBSCRIBE_PACKETS   60          // Request 60 packets (~3 s of data)
#define SUBSCRIBE_RENEW_MS  2500UL      // Re-subscribe every 2.5 s

// The X32 drops a client registration (/xremote) after 10 s of inactivity.
// Renew well within that window.
#define XREMOTE_RENEW_MS    8000UL

// ── Display ───────────────────────────────────────────────────────────────────
// Muted-alert flash period in milliseconds (on + off each = this value).
#define FLASH_INTERVAL_MS   400UL

// ── Debug ─────────────────────────────────────────────────────────────────────
// Set to 1 to print channel dB level and state to Serial at 2 Hz.
// Set to 0 for production to save a small amount of CPU.
#define DEBUG_SERIAL        1
