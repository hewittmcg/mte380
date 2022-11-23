/*
 * constants.h
 *
 *  Created on: Nov. 7, 2022
 *      Author: hmcga
 */

#ifndef INC_CONSTANTS_H_
#define INC_CONSTANTS_H_

#define MS_PER_SEC 1000.0

// ToF calibration info.
#define STOPPING_DISTANCE_MM 240
#define BOARD_SQUARE_SIZE_MM 300 // original - 300
#define TOF_BASE_SIDE_DIST_MM 100 // distance from the side TOFs to the edge of the board. Need to revisit.

// Motor speed values.
#define BASE_MOTOR_SPEED 100
#define TURNING_MOTOR_SPEED 100

#define MOTOR_BRAKE_SPEED 30
#define MOTOR_BRAKE_DELAY 175

#define RIGHT_TURN_DELAY 350
#define LEFT_TURN_DELAY 350

// IMU turning constants
#define IMU_TURN_ERROR_THRESH 5
#define IMU_TURN_MIN_TIME 750
#define IMU_TURN_CORRECTION_SPEED (TURNING_MOTOR_SPEED / 2)
#define IMU_TURN_MIN_FULL_SPEED_ERROR 30

// Course correction values
#define CORRECTION_FACTOR 10

// Photoresistor bounds
#define PHOTORESISTOR_ABS_UPPER_BOUND 4200
#define PHOTORESISTOR_THRESHOLD 2300

// Turning Correction Values
#define ANGLE_CORRECTION_THRESHOLD 15
#define RADIONS_TO_DEGREES 57.2958

#endif /* INC_CONSTANTS_H_ */
