/*
 * l298n_motor_controller.h
 *
 *  Created on: Oct 26, 2022
 *      Author: hmcga
 */

#ifndef INC_L298N_MOTOR_CONTROLLER_H_
#define INC_L298N_MOTOR_CONTROLLER_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

typedef enum {
	MOTOR_DIR_FORWARD = 0,
	MOTOR_DIR_BACKWARD,
	MOTOR_DIR_OFF,
	NUM_MOTOR_DIRS,
} MotorDir;

// General IN pin for use with motor controllers.
typedef struct {
	uint32_t pin;
	GPIO_TypeDef *port;
} MotorPin;

// Required PWM info for motor controller EN pins.
typedef struct {
	TIM_HandleTypeDef *tim_handle;
	uint32_t tim_channel;
	__IO uint32_t *ccr_ptr; // Pointer to the CCR register
} MotorPWMPin;

// Invidual output of the motor controller to a single DC motor. 
// There will be two instances of this struct per physical motor controller.
typedef struct {
	MotorPin in1_pin;
	MotorPin in2_pin;
	MotorPWMPin en_pin;
	MotorDir dir;
	int speed;
} MotorController;

// Set the direction of the motor. Returns true if successful, false otherwise.
bool set_motor_direction(MotorController *mc, MotorDir dir);

// Set the speed of the motor using PWM. Returns true if successful, false otherwise.
bool set_motor_speed(MotorController *mc, int8_t speed_percent);

// Initialize a motor controller. Returns true if successful, false otherwise.
bool motor_init(MotorController *mc);

#endif /* INC_L298N_MOTOR_CONTROLLER_H_ */
