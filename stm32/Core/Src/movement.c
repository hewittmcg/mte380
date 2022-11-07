#include "movement.h"

// Motors used in murphy.
static MotorController *controllers;

// Motor Speeds
static int TURNING_MOTOR_SPEED;
static int BASE_MOTOR_SPEED;

void movement_init(MotorController *mcs, int turn_speed, int base_speed) {
  controllers = mcs;
  TURNING_MOTOR_SPEED = turn_speed;
  BASE_MOTOR_SPEED = base_speed;

  motor_init(&controllers[FRONT_LEFT_MOTOR]);
	motor_init(&controllers[FRONT_RIGHT_MOTOR]);
	motor_init(&controllers[REAR_LEFT_MOTOR]);
	motor_init(&controllers[REAR_RIGHT_MOTOR]);

	set_motor_direction(&controllers[FRONT_RIGHT_MOTOR], MOTOR_DIR_OFF);
	set_motor_direction(&controllers[FRONT_LEFT_MOTOR], MOTOR_DIR_OFF);
	set_motor_direction(&controllers[REAR_RIGHT_MOTOR], MOTOR_DIR_OFF);
	set_motor_direction(&controllers[REAR_LEFT_MOTOR], MOTOR_DIR_OFF);
}

// Stop robot movement.
void stop() {
  set_motor_direction(&controllers[FRONT_RIGHT_MOTOR], MOTOR_DIR_OFF);
  set_motor_direction(&controllers[FRONT_LEFT_MOTOR], MOTOR_DIR_OFF);
  set_motor_direction(&controllers[REAR_RIGHT_MOTOR], MOTOR_DIR_OFF);
  set_motor_direction(&controllers[REAR_LEFT_MOTOR], MOTOR_DIR_OFF);

  set_motor_speed(&controllers[FRONT_RIGHT_MOTOR], 0);
  set_motor_speed(&controllers[REAR_RIGHT_MOTOR], 0);
  set_motor_speed(&controllers[FRONT_LEFT_MOTOR], 0);
  set_motor_speed(&controllers[REAR_LEFT_MOTOR], 0);
}

void move_forward(int speed) {
  // The left motors are wired up backwards.
	set_motor_direction(&controllers[FRONT_RIGHT_MOTOR], MOTOR_DIR_FORWARD);
	set_motor_direction(&controllers[FRONT_LEFT_MOTOR], MOTOR_DIR_BACKWARD);
	set_motor_direction(&controllers[REAR_RIGHT_MOTOR], MOTOR_DIR_FORWARD);
	set_motor_direction(&controllers[REAR_LEFT_MOTOR], MOTOR_DIR_BACKWARD);

  // Slowly increase speed.
	for (int i = 0; i <= speed; i+=1) {
		set_motor_speed(&controllers[FRONT_RIGHT_MOTOR], i);
		set_motor_speed(&controllers[FRONT_LEFT_MOTOR], i);
		set_motor_speed(&controllers[REAR_RIGHT_MOTOR], i);
		set_motor_speed(&controllers[REAR_LEFT_MOTOR], i);
		HAL_Delay(25);
	}
}

void move_backward(int speed) {
  // The left motors are wired up backwards.
	set_motor_direction(&controllers[FRONT_RIGHT_MOTOR], MOTOR_DIR_BACKWARD);
	set_motor_direction(&controllers[FRONT_LEFT_MOTOR], MOTOR_DIR_FORWARD);
	set_motor_direction(&controllers[REAR_RIGHT_MOTOR], MOTOR_DIR_BACKWARD);
	set_motor_direction(&controllers[REAR_LEFT_MOTOR], MOTOR_DIR_FORWARD);

  // Slowly increase speed.
	for (int i = 0; i <= speed; i+=1) {
		set_motor_speed(&controllers[FRONT_RIGHT_MOTOR], i);
		set_motor_speed(&controllers[FRONT_LEFT_MOTOR], i);
		set_motor_speed(&controllers[REAR_RIGHT_MOTOR], i);
		set_motor_speed(&controllers[REAR_LEFT_MOTOR], i);
		HAL_Delay(25);
	}
}

// Turn 90 degrees to the right.
void turn_right() {
  set_motor_direction(&controllers[FRONT_RIGHT_MOTOR], MOTOR_DIR_BACKWARD);
  set_motor_direction(&controllers[FRONT_LEFT_MOTOR], MOTOR_DIR_BACKWARD);
  set_motor_direction(&controllers[REAR_RIGHT_MOTOR], MOTOR_DIR_BACKWARD);
  set_motor_direction(&controllers[REAR_LEFT_MOTOR], MOTOR_DIR_BACKWARD);

  set_motor_speed(&controllers[FRONT_RIGHT_MOTOR], TURNING_MOTOR_SPEED);
  set_motor_speed(&controllers[REAR_RIGHT_MOTOR], TURNING_MOTOR_SPEED);

  set_motor_speed(&controllers[FRONT_LEFT_MOTOR], TURNING_MOTOR_SPEED);
  set_motor_speed(&controllers[REAR_LEFT_MOTOR], TURNING_MOTOR_SPEED);

  HAL_Delay(400);

  stop();
}

// Turn 90 degrees to the left.
void turn_left() {
  set_motor_direction(&controllers[FRONT_RIGHT_MOTOR], MOTOR_DIR_FORWARD);
  set_motor_direction(&controllers[FRONT_LEFT_MOTOR], MOTOR_DIR_FORWARD);
  set_motor_direction(&controllers[REAR_RIGHT_MOTOR], MOTOR_DIR_FORWARD);
  set_motor_direction(&controllers[REAR_LEFT_MOTOR], MOTOR_DIR_FORWARD);

  set_motor_speed(&controllers[FRONT_RIGHT_MOTOR], TURNING_MOTOR_SPEED);
  set_motor_speed(&controllers[REAR_RIGHT_MOTOR], TURNING_MOTOR_SPEED);

  set_motor_speed(&controllers[FRONT_LEFT_MOTOR], TURNING_MOTOR_SPEED);
  set_motor_speed(&controllers[REAR_LEFT_MOTOR], TURNING_MOTOR_SPEED);

  HAL_Delay(400);

  stop();
}
