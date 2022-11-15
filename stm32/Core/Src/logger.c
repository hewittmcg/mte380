/*
 * logger.c
 *
 *  Created on: Nov 14, 2022
 *      Author: hmcga
 */
#include "logger.h"
#include <stdio.h>

#define NUM_LOGS 1000
static LogItem logs[NUM_LOGS];

// Current index to log to.
static uint16_t log_idx = 0;

bool log_item(LogSource source, uint16_t timestamp, uint16_t val1, uint16_t val2) {
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
		printf("%d, %d, %d, %d,\r\n", logs[i].source, logs[i].timestamp, logs[i].data[0], logs[i].data[1]);
	}
}

