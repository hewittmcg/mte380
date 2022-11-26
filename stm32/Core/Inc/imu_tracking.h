/*
 * imu_tracking.h
 *
 *  Created on: Nov 18, 2022
 *      Author: hmcga
 */

#ifndef INC_IMU_TRACKING_H_
#define INC_IMU_TRACKING_H_

// Take an IMU x reading for pit detection.
void add_gyro_x_reading(void);

// Get the change in angle across the recently taken IMU x readings by
// numerical integration.
float get_gyro_recent_x_diff(void);

void reset_imu_tracking(void);

#endif /* INC_IMU_TRACKING_H_ */
