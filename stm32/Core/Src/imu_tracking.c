/*
 * imu_tracking.c
 *
 *  Created on: Nov 18, 2022
 *      Author: hmcga
 */

 #include <inttypes.h>
 #include "ICM20948.h"
 #include "stm32f4xx_hal.h"
 #include "constants.h"
#include "logger.h"
#include <string.h>

 // IMU reading definition
typedef struct {
	float reading;
	uint32_t ticks;
} ImuReading;

#define NUM_IMU_READINGS 250
// We only want to consider readings for the last second 
#define IMU_RECENT_ANGLE_MAX_TIME 5000
#define IMU_MIN_READING_INTERVAL 25

static ImuReading imu_reading_storage[NUM_IMU_READINGS] = { 0 };
static int cur_imu_reading_idx = 0;
static uint32_t prev_reading_time = 0;

void add_gyro_x_reading(void) {
	// Don't read if we haven't passed the minimum reading time yet
	if(prev_reading_time != 0 && HAL_GetTick() < prev_reading_time + IMU_MIN_READING_INTERVAL) {
		return;
	}
	prev_reading_time = HAL_GetTick();
	// Take a reading
	axises data;
	icm20948_gyro_read_dps(&data);
	imu_reading_storage[cur_imu_reading_idx].reading = data.x;
	imu_reading_storage[cur_imu_reading_idx].ticks = HAL_GetTick();
	cur_imu_reading_idx++;

	// Wrap around back to the start and overwrite with the new reading
	if(cur_imu_reading_idx >= NUM_IMU_READINGS) {
		cur_imu_reading_idx = 0;
	}
	axises accel;
	icm20948_accel_read(&accel);

	// log_item(LOG_SOURCE_IMU, HAL_GetTick(), data.x, accel.z);
}

// Calculate the recent x angle across the stored IMU readings.
float get_gyro_recent_x_diff(void) {
	// Numerically integrate
	float total_angle = 0;

	uint32_t tick_threshold = HAL_GetTick() - IMU_RECENT_ANGLE_MAX_TIME;
	// TODO: check if the bounds are correct here
    for(int i = 0; i < NUM_IMU_READINGS - 2; i++) {
		// Account for how the buffer of readings works
		// (i.e. we want to start at cur_imu_reading_idx, since it points to the least recent reading)
		int cur_idx = i + cur_imu_reading_idx;
		int next_idx = cur_idx + 1;

		// Wrap both back around to 0 if needed
		cur_idx = cur_idx % NUM_IMU_READINGS;
		next_idx = next_idx % NUM_IMU_READINGS;

		// Ignore readings after threshold
		if(imu_reading_storage[cur_idx].ticks < tick_threshold) {
			continue;
		}
		// Don't divide by zero or have a negative reading
		if(imu_reading_storage[next_idx].ticks <= imu_reading_storage[cur_idx].ticks) {
			continue;
		}
		float increment = ((imu_reading_storage[cur_idx].reading + imu_reading_storage[next_idx].reading)/2)
				* (imu_reading_storage[next_idx].ticks - imu_reading_storage[cur_idx].ticks) / MS_PER_SEC;
		total_angle += increment;
	}


	return total_angle;
}

void reset_imu_tracking(void) {
	memset(imu_reading_storage, 0, sizeof(imu_reading_storage)); 
}


