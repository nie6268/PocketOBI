// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2026 TheRepairForge
// hostsim shim: what the host tests can ask about a GPIO (pinMode / digitalWrite are recorded
// against the simulated clock). "Driven low" = output mode and level LOW.
#pragma once
#include <stdint.h>

void shimPinReset();                       // forget the history; the current state stays
bool shimPinDriven(uint8_t pin);           // in output mode now
uint32_t shimPinMaxLowUs(uint8_t pin);     // longest stretch driven low since the reset, the current one included
int shimPinReleases(uint8_t pin);          // output -> input switches since the reset
