/*
 * constants.h
 *
 *  Created on: Nov. 7, 2022
 *      Author: hmcga
 */

#ifndef INC_CONSTANTS_H_
#define INC_CONSTANTS_H_

#define MS_PER_SEC 1000.0
#define RADIONS_TO_DEGREES 57.2958

#define SIDE_TOF_SEPARATION_MM 165 // Distance between the two side ToF sensors. TODO revise this

// ToF calibration info.
#define STOPPING_DISTANCE_MM 200 // original - 240 TODO: Tuning
#define BOARD_SQUARE_SIZE_MM 305 // original - 300
#define TOF_BASE_SIDE_DIST_MM 90 // distance from the side TOFs to the edge of the board. Need to revisit.
#define TOF_STOPPING_DISTANCE_OFFSET_MIDDLE -15
#define TOF_STOPPING_DISTANCE_OFFSET_CENTRE 15

// Motor speed values.
#define BASE_MOTOR_SPEED 100
#define TURNING_MOTOR_SPEED 100
#define MIN_MOTOR_SPEED 30

#define MOTOR_BRAKE_SPEED 30
#define MOTOR_BRAKE_DELAY 175

#define RIGHT_TURN_DELAY 350
#define LEFT_TURN_DELAY 350

// IMU turning constants
#define IMU_TURN_ERROR_THRESH 5
#define IMU_TURN_MIN_TIME 750
#define IMU_TURN_MIN_FULL_SPEED_ERROR 30
#define IMU_TURN_CORRECTION_SPEED 80 //(TURNING_MOTOR_SPEED / 2)

// Course correction values
#define CORRECTION_FACTOR 10
#define CORRECTION_RANGE_MIN -15
#define CORRECTION_RANGE_MAX 15

// Pit Detection values
#define PIT_DETECT_THRESH -4.0f

// Photoresistor bounds
#define PHOTORESISTOR_ABS_UPPER_BOUND 4200
#define PHOTORESISTOR_THRESHOLD 900

// Turning Correction Values
#define ANGLE_CORRECTION_ADJUSTMENT_THRESHOLD 5
#define TURN_CORRECTION_MOTOR_SPEED (TURNING_MOTOR_SPEED / 2)
#define TOF_ANGLE_CORRECTION_THRESHOLD 8

// Controlled Stopping constants
#define CC_KP 0.003f // Proportional coefficient (unused)
#define CC_KI 0.005f // Derivative coefficient (unused)
#define CONTROLLED_STOP_RANGE 25 // TODO: Tuning
#define CONTROLLED_STOP_DISTANCE_CORRECTION 130

#endif /* INC_CONSTANTS_H_ */
