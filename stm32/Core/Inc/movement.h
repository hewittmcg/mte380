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

// Initialize the motor controllers with the given turning speed and base speed.
void movement_init(MotorController *mcs);

// Move the robot forward, starting from 0 and slowly accelerating to the speed %.
void move_forward(int speed);

// Move the robot backwards, starting from 0 and slowly accelerating to the speed %.
void move_backward(int speed);

// Turn the robot 90 degrees to the right.
void turn_right();

// Turn the robot 90 degrees to the left.
void turn_left();

// Stop all the motors, without engine braking.
void stop();
