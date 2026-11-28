// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2026 TheRepairForge
// fakepack: the OneWire class for the host decode tests (see fakepack.h).
// Guarded whole: PlatformIO builds every .cpp under src_dir = . (D14), this one included.
#ifdef POCKETOBI_HOSTTEST
#include "fakepack.h"
#include <map>
#include <cstdio>
#include <cstring>
#include "../../OneWire2.h"

static std::map<std::string, std::vector<Bytes>> g_script;
static std::map<std::string, size_t> g_next;              // next answer index per key
static std::vector<std::string> g_log;
static Bytes g_writes;                                     // this transaction's writes so far
static std::string g_readKey;                              // key the current answer belongs to
static Bytes g_answer;
static size_t g_pos = 0;
static bool g_open = false;

static std::string keyOf(const Bytes &w) {
  std::string k;
  char h[4];
  for (size_t i = 0; i < w.size(); i++) { snprintf(h, sizeof(h), i ? " %02X" : "%02X", w[i]); k += h; }
  return k;
}
static void closeTxn() {
  if (g_open && !g_writes.empty()) g_log.push_back(keyOf(g_writes));
  g_writes.clear(); g_readKey.clear(); g_answer.clear(); g_pos = 0; g_open = false;
}

void fakepackClear() { closeTxn(); g_script.clear(); g_next.clear(); g_log.clear(); }
void fakepackSet(const char *key, std::vector<Bytes> answers) { g_script[key] = answers; g_next[key] = 0; }
int fakepackCount(const char *key) {
  closeTxn();
  int n = 0;
  for (auto &k : g_log) if (k == key) n++;
  return n;
}
const std::vector<std::string> &fakepackLog() { closeTxn(); return g_log; }

Bytes le16b(uint16_t v) { return Bytes{ (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) }; }
Bytes ascii(const char *s, size_t len) { Bytes b(len, 0); memcpy(b.data(), s, strnlen(s, len)); return b; }
Bytes silent(size_t len) { return Bytes(len, 0xFF); }

// ---- OneWire, as the firmware sees it ----
void OneWire::begin(uint8_t) {}
uint8_t OneWire::reset(void) { closeTxn(); g_open = true; return 0; }   // no presence pulse, like the BMS
void OneWire::select(const uint8_t*) {}
void OneWire::skip(void) {}
void OneWire::write(uint8_t b, uint8_t) { g_writes.push_back(b); }
void OneWire::write_bytes(const uint8_t *b, uint16_t n, bool) { for (uint16_t i = 0; i < n; i++) write(b[i], 0); }
uint8_t OneWire::read(void) {
  std::string k = keyOf(g_writes);
  if (k != g_readKey) {                 // first read after a write: look the key up
    g_readKey = k; g_pos = 0; g_answer.clear();
    auto it = g_script.find(k);
    if (it != g_script.end() && !it->second.empty()) {
      size_t &i = g_next[k];
      g_answer = it->second[i < it->second.size() ? i : it->second.size() - 1];
      if (i < it->second.size()) i++;
    }
  }
  return g_pos < g_answer.size() ? g_answer[g_pos++] : 0xFF;
}
void OneWire::read_bytes(uint8_t *buf, uint16_t n) { for (uint16_t i = 0; i < n; i++) buf[i] = read(); }
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
#endif // POCKETOBI_HOSTTEST
