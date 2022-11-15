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

// Sets motor speed given the motor
void setMotorSpeed(Motor motor, int speed);

// Turn the robot 90 degrees to the right.
void turn_right();

// Turn to degrees using the IMU for positioning.
void turn_right_imu(uint16_t degrees);

// Turn the robot 90 degrees to the left.
void turn_left();

// Stop all the motors, without engine braking.
void stop();
