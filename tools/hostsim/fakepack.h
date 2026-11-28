// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2026 TheRepairForge
// fakepack: a scriptable Makita pack on the OneWire bus, for the host decode tests.
//
// It sits UNDER the firmware's own protocol code (sendCommand, readF0513Raw, ...): the
// OneWire class is replaced, nothing above it is. Each makita.reset() opens a transaction;
// the bytes the firmware writes form a key ("CC D7 00 00 FF", "33", "33 AA 00", "31"), and
// the first read after a write looks that key up. A key with no script answers 0xFF, like
// an empty bus. A key can hold several answers: each lookup takes the next one and the last
// one repeats, so "silent once, then answers" or "00 00, then 3.6 V" are one line each.
#pragma once
#include <stdint.h>
#include <string>
#include <vector>

typedef std::vector<uint8_t> Bytes;

void fakepackClear();                                     // no pack: every key silent
void fakepackSet(const char *key, std::vector<Bytes> answers);
int  fakepackCount(const char *key);                      // transactions whose writes were exactly `key`
const std::vector<std::string> &fakepackLog();            // every transaction's writes, in order

// Little helpers for building answers.
Bytes le16b(uint16_t v);                                  // 2 bytes, little-endian
Bytes ascii(const char *s, size_t len);                   // padded with 0x00 to len
Bytes silent(size_t len);                                 // len x 0xFF
