// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2026 TheRepairForge
// hostsim shim: definitions behind Arduino.h / SPI.h, plus a OneWire that never answers
// (OneWire2.cpp itself is not compiled: it drives ESP32 GPIO registers directly).
// Guarded whole: PlatformIO builds every .cpp under src_dir = . (D14), this one included.
#ifdef POCKETOBI_HOSTSIM
#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"
#include "esp_timer.h"
#include "shim_gpio.h"
#include "../../../OneWire2.h"

HardwareSerial Serial;
SPIClass SPI;
TwoWire Wire;
void (*hostsimTouchHook)() = nullptr;

static unsigned long g_us = 0;          // simulated clock: advances only when code waits

// One-shot esp_timers (esp_timer.h): fired, in due order, while the clock advances.
struct shim_timer { esp_timer_cb_t cb; void *arg; bool armed; unsigned long due; };
static shim_timer g_timers[8];
static int g_nTimers = 0;
esp_err_t esp_timer_create(const esp_timer_create_args_t *a, esp_timer_handle_t *out) {
  if (g_nTimers >= 8) return -1;
  g_timers[g_nTimers] = { a->callback, a->arg, false, 0 };
  *out = &g_timers[g_nTimers++];
  return ESP_OK;
}
esp_err_t esp_timer_start_once(esp_timer_handle_t t, uint64_t us) {
  if (t->armed) return -1;                            // like ESP-IDF: stop it first
  t->armed = true; t->due = g_us + (unsigned long)us;
  return ESP_OK;
}
esp_err_t esp_timer_stop(esp_timer_handle_t t) { t->armed = false; return ESP_OK; }
static void advance(unsigned long us) {
  unsigned long end = g_us + us;
  for (;;) {
    shim_timer *next = nullptr;
    for (int i = 0; i < g_nTimers; i++)
      if (g_timers[i].armed && g_timers[i].due <= end && (!next || g_timers[i].due < next->due)) next = &g_timers[i];
    if (!next) break;
    if (next->due > g_us) g_us = next->due;
    next->armed = false;
    next->cb(next->arg);
  }
  g_us = end;
}
unsigned long millis() { return g_us / 1000; }
unsigned long micros() { return g_us; }
void delay(unsigned long ms) { advance(ms * 1000UL); }
void delayMicroseconds(unsigned int us) { advance(us); }

// GPIO: mode and level recorded against the clock (shim_gpio.h).
struct ShimPin { bool out; uint8_t level; unsigned long lowSince; uint32_t maxLow; int releases; };
static ShimPin g_pins[64];
static bool pinLow(const ShimPin &p) { return p.out && p.level == LOW; }
static void pinChange(uint8_t pin, bool out, uint8_t level) {
  if (pin >= 64) return;
  ShimPin &p = g_pins[pin];
  bool was = pinLow(p);
  if (p.out && !out) p.releases++;
  p.out = out; p.level = level;
  bool now = pinLow(p);
  if (was && !now && g_us - p.lowSince > p.maxLow) p.maxLow = g_us - p.lowSince;
  if (!was && now) p.lowSince = g_us;
}
void pinMode(uint8_t pin, uint8_t mode) { if (pin < 64) pinChange(pin, mode == OUTPUT, g_pins[pin].level); }
void digitalWrite(uint8_t pin, uint8_t val) { if (pin < 64) pinChange(pin, g_pins[pin].out, val ? HIGH : LOW); }
void shimPinReset() {
  for (auto &p : g_pins) { p.maxLow = 0; p.lowSince = g_us; p.releases = 0; }
}
bool shimPinDriven(uint8_t pin) { return pin < 64 && g_pins[pin].out; }
int shimPinReleases(uint8_t pin) { return pin < 64 ? g_pins[pin].releases : 0; }
uint32_t shimPinMaxLowUs(uint8_t pin) {
  if (pin >= 64) return 0;
  const ShimPin &p = g_pins[pin];
  uint32_t cur = pinLow(p) ? g_us - p.lowSince : 0;
  return cur > p.maxLow ? cur : p.maxLow;
}
int  digitalRead(uint8_t) { return HIGH; }   // buttons idle (active-low)
long random(long max) { return max > 0 ? rand() % max : 0; }
long random(long lo, long hi) { return hi > lo ? lo + rand() % (hi - lo) : lo; }

// No pack on the bus: reset() reports no presence, reads float high. The decode tests
// (POCKETOBI_HOSTTEST) bring their own OneWire, a scripted pack: fakepack.cpp.
#ifndef POCKETOBI_HOSTTEST
void OneWire::begin(uint8_t) {}
uint8_t OneWire::reset(void) { return 0; }
void OneWire::select(const uint8_t*) {}
void OneWire::skip(void) {}
void OneWire::write(uint8_t, uint8_t) {}
void OneWire::write_bytes(const uint8_t*, uint16_t, bool) {}
uint8_t OneWire::read(void) { return 0xFF; }
void OneWire::read_bytes(uint8_t* buf, uint16_t n) { memset(buf, 0xFF, n); }
void OneWire::write_bit(uint8_t) {}
uint8_t OneWire::read_bit(void) { return 1; }
void OneWire::depower(void) {}
#if ONEWIRE_SEARCH
void OneWire::reset_search() {}
void OneWire::target_search(uint8_t) {}
bool OneWire::search(uint8_t*, bool) { return false; }
#endif
#if ONEWIRE_CRC
uint8_t OneWire::crc8(const uint8_t*, uint8_t) { return 0; }
#if ONEWIRE_CRC16
bool OneWire::check_crc16(const uint8_t*, uint16_t, const uint8_t*, uint16_t) { return true; }
uint16_t OneWire::crc16(const uint8_t*, uint16_t, uint16_t) { return 0; }
#endif
#endif
#endif // !POCKETOBI_HOSTTEST
#endif // POCKETOBI_HOSTSIM
