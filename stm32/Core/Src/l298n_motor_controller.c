/*
 * l298n_motor_controller.c
 *
 *  Created on: Oct 26, 2022
 *      Author: hmcga
 */


#include "l298n_motor_controller.h"
#include "stm32f4xx_hal.h"

/* TODO
typedef struct {
	uint32t
};
typedef struct {
	uint32_t in1_pin;
	uint32_t in2_pin;
	GPIO_TypeDef *port;
} MotorController;

static MotorController[NUM_MOTORS] controllers = {
		[FRONT_LEFT_MOTOR] = {0, 0},
		[FRONT_RIGHT_MOTOR] = {
};
*/
// Pin definitions
void set_motor_direction(Motor motor, MotorDir dir) {
	// Just one for testing
	if(motor == FRONT_LEFT_MOTOR) {
		if(dir == MOTOR_DIR_FORWARD) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
		}
	}
}
