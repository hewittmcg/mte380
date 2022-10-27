/*
 * l298n_motor_controller.h
 *
 *  Created on: Oct 26, 2022
 *      Author: hmcga
 */

#ifndef INC_L298N_MOTOR_CONTROLLER_H_
#define INC_L298N_MOTOR_CONTROLLER_H_

typedef enum {
	FRONT_LEFT_MOTOR = 0,
	FRONT_RIGHT_MOTOR,
	REAR_LEFT_MOTOR,
	REAR_RIGHT_MOTOR,
	NUM_MOTORS,
} Motor;

typedef enum {
	MOTOR_DIR_FORWARD = 0,
	MOTOR_DIR_BACKWARD,
	MOTOR_DIR_OFF,
	NUM_MOTOR_DIRS,
} MotorDir;

// Set the direction of the motor.
void set_motor_direction(Motor motor, MotorDir dir);

#endif /* INC_L298N_MOTOR_CONTROLLER_H_ */
