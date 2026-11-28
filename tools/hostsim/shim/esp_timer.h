// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2026 TheRepairForge
// hostsim shim: the subset of ESP-IDF esp_timer the firmware uses. A one-shot timer fires
// while the simulated clock advances (delay / delayMicroseconds), never on its own.
#pragma once
#include <stdint.h>

typedef int esp_err_t;
#ifndef ESP_OK
#define ESP_OK 0
#endif
typedef void (*esp_timer_cb_t)(void *arg);
typedef enum { ESP_TIMER_TASK } esp_timer_dispatch_t;
typedef struct {
  esp_timer_cb_t callback;
  void *arg;
  esp_timer_dispatch_t dispatch_method;
  const char *name;
  bool skip_unhandled_events;
} esp_timer_create_args_t;
typedef struct shim_timer *esp_timer_handle_t;

esp_err_t esp_timer_create(const esp_timer_create_args_t *args, esp_timer_handle_t *out);
esp_err_t esp_timer_start_once(esp_timer_handle_t t, uint64_t timeout_us);
esp_err_t esp_timer_stop(esp_timer_handle_t t);
