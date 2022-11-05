#include "l298n_motor_controller.h"
#include <stdio.h>
#include "stm32f4xx_hal_gpio.h"

// Motors used in murphy.
typedef enum {
	FRONT_LEFT_MOTOR = 0,
	FRONT_RIGHT_MOTOR,
	REAR_LEFT_MOTOR,
	REAR_RIGHT_MOTOR,
	NUM_MOTORS,
} Motor;

// initialize motor controllers
void movement_init(MotorController *controllersInit[], int turning_speed, int base_speed);

// move forward function
void move_forward(int speed);

// move backwards function
void move_forward(int speed);

// turn right
void turn_right();

// turn left 
void turn_left();

// stop vehicle
void stop();
