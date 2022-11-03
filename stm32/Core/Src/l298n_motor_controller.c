/*
 * l298n_motor_controller.c
 *
 *  Created on: Oct 26, 2022
 *      Author: hmcga
 */


#include "l298n_motor_controller.h"

// CCR register goes from 0 to 0xffff
#define SPEED_PERCENT_TO_CCR 0xffff

#define PERCENT_TO_DEC 100

bool set_motor_direction(MotorController *mc, MotorDir dir) {
	if(dir == MOTOR_DIR_FORWARD) {
		HAL_GPIO_WritePin(mc->in1_pin.port, mc->in1_pin.pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mc->in2_pin.port, mc->in2_pin.pin, GPIO_PIN_SET);
	} else if(dir == MOTOR_DIR_BACKWARD) {
		HAL_GPIO_WritePin(mc->in1_pin.port, mc->in1_pin.pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mc->in2_pin.port, mc->in2_pin.pin, GPIO_PIN_RESET);
	} else if(dir == MOTOR_DIR_OFF) {
		// TODO: Not sure if this is correct
		HAL_GPIO_WritePin(mc->in1_pin.port, mc->in1_pin.pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mc->in2_pin.port, mc->in2_pin.pin, GPIO_PIN_RESET);
	}
	return true;
}

bool set_motor_speed(MotorController *mc, uint8_t speed_percent) {
	if(speed_percent > 100) {
		speed_percent = 100;
	}
	// Get value to set CCR to
	uint16_t ccr_val = (uint16_t)((float)speed_percent / PERCENT_TO_DEC * SPEED_PERCENT_TO_CCR);
	*(mc->en_pin.ccr_ptr) = ccr_val;
	
	return true;
}

bool motor_init(MotorController *mc) {
	set_motor_direction(mc, MOTOR_DIR_OFF);

	HAL_TIM_PWM_Start(mc->en_pin.tim_handle, mc->en_pin.tim_channel);
	return true;
}
