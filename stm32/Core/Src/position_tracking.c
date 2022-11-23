/*
 * position_tracking.c
 *
 *  Created on: Nov 23, 2022
 *      Author: erfan
 */

/*
 * imu_tracking.c
 *
 *  Created on: Nov 18, 2022
 *      Author: hmcga
 */

#include <inttypes.h>
#include "stm32f4xx_hal.h"
#include "constants.h"

 // IMU reading definition
typedef struct {
	float reading;
	uint32_t ticks;
} TofReading;

#define NUM_TOF_READINGS 10
// We only want to consider readings for the last second
#define TOF_RECENT_READING_MAX_TIME 1000
#define TOF_MIN_READING_INTERVAL 50 // TODO: Tuning

static TofReading tof_reading_storage[NUM_TOF_READINGS] = { 0 };
static int cur_tof_reading_idx = 0;
static uint32_t prev_reading_time = 0;

// Take a position reading for controlled stopping.
void add_front_tof_reading(int front) {
	// Don't read if we haven't passed the minimum reading time yet
	if(prev_reading_time != 0 && HAL_GetTick() < prev_reading_time + TOF_MIN_READING_INTERVAL) {
		return;
	}

	if (front > 1000) {
		front = 1000;
	}

	prev_reading_time = HAL_GetTick();

	tof_reading_storage[cur_tof_reading_idx].reading = front;
	tof_reading_storage[cur_tof_reading_idx].ticks = HAL_GetTick();
	cur_tof_reading_idx++;

	// Wrap around back to the start and overwrite with the new reading
	if(cur_tof_reading_idx >= NUM_TOF_READINGS) {
		cur_tof_reading_idx = 0;
	}
}

// Get the change in position across the recently taken position readings by
// numerical integration.
float get_front_tof_recent_diff(void) {
	// Numerically integrate
	float total_position = 0;

	uint32_t tick_threshold = HAL_GetTick() - TOF_RECENT_READING_MAX_TIME;
	// TODO: check if the bounds are correct here
    for(int i = 0; i < NUM_TOF_READINGS - 2; i++) {
		// Account for how the buffer of readings works
		// (i.e. we want to start at cur_tof_reading_idx, since it points to the least recent reading)
		int cur_idx = i + cur_tof_reading_idx;
		int next_idx = cur_idx + 1;

		// Wrap both back around to 0 if needed
		cur_idx = cur_idx % NUM_TOF_READINGS;
		next_idx = next_idx % NUM_TOF_READINGS;

		// Ignore readings after threshold
		if(tof_reading_storage[cur_idx].ticks < tick_threshold) {
			continue;
		}
		// Don't divide by zero or have a negative reading
		if(tof_reading_storage[next_idx].ticks <= tof_reading_storage[cur_idx].ticks) {
			continue;
		}
		float increment = ((tof_reading_storage[cur_idx].reading + tof_reading_storage[next_idx].reading)/2)
				* (tof_reading_storage[next_idx].ticks - tof_reading_storage[cur_idx].ticks) / MS_PER_SEC;
		total_position += increment;
	}

	return total_position;
}

// Reset position array
void reset_position_tracking() {
	memset(tof_reading_storage, 0, sizeof(tof_reading_storage));
}
