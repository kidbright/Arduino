#include "esp32_common.h"

TickType_t get_tickcnt(void) {
	return xTaskGetTickCount();
}

bool is_tickcnt_elapsed(TickType_t tickcnt, uint32_t tickcnt_ms) {
	TickType_t curr_tickcnt = xTaskGetTickCount();

	if ((curr_tickcnt - tickcnt) >= (tickcnt_ms / portTICK_RATE_MS)) {
		tickcnt = curr_tickcnt;
		return true;
	}

	return false;
}

