/*
 * logger.c
 *
 *  Created on: Nov 14, 2022
 *      Author: hmcga
 */
#include "logger.h"
#include <stdio.h>
#include "stm32f4xx_hal.h"

#define NUM_LOGS 100
static LogItem logs[NUM_LOGS];

// Current index to log to.
static uint16_t log_idx = 0;

bool log_item(LogSource source, uint32_t timestamp, uint16_t val1, uint16_t val2) {
	if(log_idx >= NUM_LOGS) {
		return false;
	}

	logs[log_idx].source = source;
	logs[log_idx].timestamp = timestamp;
	logs[log_idx].data[0] = val1;
	logs[log_idx].data[1] = val2;
	log_idx++;
	return true;
}

void log_output(void) {
	for(int i = 0; i < log_idx; i++) {
		printf("%d, %lu, %d, %d,\r\n", logs[i].source, logs[i].timestamp, logs[i].data[0], logs[i].data[1]);

		// For each log, need to wait for the other nucleo to printf it over USB after sending
		HAL_Delay(10);
	}
}

