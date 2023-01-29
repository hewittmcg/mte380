/*
 * position_tracking.h
 *
 *  Created on: Nov 23, 2022
 *      Author: erfan
 */

#ifndef INC_POSITION_TRACKING_H_
#define INC_POSITION_TRACKING_H_

// Take a position reading for controlled stopping.
void add_front_tof_reading(int front);

// Get the change in position across the recently taken position readings by
// numerical integration.
float get_front_tof_recent_diff(void);

// Reset position array
void reset_position_tracking();

#endif /* INC_POSITION_TRACKING_H_ */
