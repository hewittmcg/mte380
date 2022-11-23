/*
 * logger.h
 *
 *  Created on: Nov 14, 2022
 *      Author: hmcga
 */

#ifndef INC_LOGGER_H_
#define INC_LOGGER_H_

#include <stdbool.h>
#include <stdint.h>

// Sources a log can come from.
typedef enum {
	LOG_SOURCE_FORWARD_TOF = 0,
	LOG_SOURCE_SIDE_TOFS,
	LOG_SOURCE_IMU,
	LOG_SOURCE_GENERAL,
	LOG_SOURCE_PIT_DETECT,
	LOG_SOURCE_IMU_TURN,
	LOG_SOURCE_TURN_STARTING,
	NUM_LOG_SOURCES,
} LogSource;

#pragma pack(1)
typedef struct {
	uint8_t source; // Source of the log
	uint32_t timestamp; // Log timestamp
	uint16_t data[2]; // Each log type has around two data values
} LogItem;

// Log an item.
bool log_item(LogSource source, uint32_t timestamp, uint16_t val1, uint16_t val2);

// Output all the recorded logs.
void log_output(void);

#endif /* INC_LOGGER_H_ */
