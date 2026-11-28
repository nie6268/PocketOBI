// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2026 TheRepairForge
#pragma once
#include "pocketobi.h"

void mkWrite(uint8_t b);
uint8_t mkRead();
uint8_t nibbleSwap(uint8_t b);
uint16_t le16(const uint8_t *buf, int idx);
void enableOn();
// ENABLE idle release (#41, protocol.cpp): after the last command of an operation on the device,
// and after each command from the PC bridge, which holds the low through a gap of up to 120 ms,
// above busPowerCycle()'s 100 ms (bridge.cpp, #88).
#define ENABLE_IDLE_MS         35
#ifndef ENABLE_BRIDGE_IDLE_MS                  // -D override: the host case shows 35 ms failing
#define ENABLE_BRIDGE_IDLE_MS 120
#endif
void enableOff(bool release = true, uint16_t idleMs = ENABLE_IDLE_MS);  // release=false: held low (busPowerCycle)
void enableIdleTick();
void enableBoot();                      // setup(): drive ENABLE low, then let the idle timer release it
bool sendCommand(const uint8_t *c, uint8_t *outPayload, uint8_t *romIdOut = nullptr);
bool isPrintableAscii(const uint8_t *b, int n);
bool sendCommandRetry(const uint8_t *c, uint8_t *outPayload, uint8_t *romIdOut, uint8_t tries);
bool readF0513Raw(uint8_t cmdByte, uint8_t *byte1, uint8_t *byte2);
void busPowerCycle();
void resetErrors();
void ledsSet(bool on);
void ledsOn();
void ledsOff();
uint8_t d6ReadByte(uint16_t addr);
uint8_t d4ReadByte(uint16_t addr);
void d4ReadBlock(uint16_t addr, uint8_t *buf, uint8_t n);
