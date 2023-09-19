#ifndef __ESP32_COMMON_H__
#define __ESP32_COMMON_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

TickType_t get_tickcnt(void);
bool is_tickcnt_elapsed(TickType_t tickcnt, uint32_t tickcnt_ms);

#endif

