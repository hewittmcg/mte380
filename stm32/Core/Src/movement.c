#include "movement.h"
#include "constants.h"

// Motors used in murphy.
static MotorController *controllers;


void movement_init(MotorController *mcs) {
  controllers = mcs;

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
  set_motor_speed(&controllers[FRONT_RIGHT_MOTOR], (-1)*MOTOR_BRAKE_SPEED);
  set_motor_speed(&controllers[REAR_RIGHT_MOTOR], (-1)*MOTOR_BRAKE_SPEED);
  set_motor_speed(&controllers[FRONT_LEFT_MOTOR], (-1)*MOTOR_BRAKE_SPEED);
  set_motor_speed(&controllers[REAR_LEFT_MOTOR], (-1)*MOTOR_BRAKE_SPEED);

  HAL_Delay(MOTOR_BRAKE_DELAY);

  set_motor_speed(&controllers[FRONT_RIGHT_MOTOR], 0);
  set_motor_speed(&controllers[REAR_RIGHT_MOTOR], 0);
  set_motor_speed(&controllers[FRONT_LEFT_MOTOR], 0);
  set_motor_speed(&controllers[REAR_LEFT_MOTOR], 0);

}

void move_forward(int speed) {
  // Slowly increase speed.
    for (int i = 0; i <= speed; i+=10) {
        set_motor_speed(&controllers[FRONT_RIGHT_MOTOR], i);
        set_motor_speed(&controllers[FRONT_LEFT_MOTOR], i);
        set_motor_speed(&controllers[REAR_RIGHT_MOTOR], i);
        set_motor_speed(&controllers[REAR_LEFT_MOTOR], i);
        HAL_Delay(25);
    }
}

void move_backward(int speed) {
  // Slowly increase speed.
	for (int i = 0; i >= speed; i-=10) {
		set_motor_speed(&controllers[FRONT_RIGHT_MOTOR], i);
		set_motor_speed(&controllers[FRONT_LEFT_MOTOR], i);
		set_motor_speed(&controllers[REAR_RIGHT_MOTOR], i);
		set_motor_speed(&controllers[REAR_LEFT_MOTOR], i);
		HAL_Delay(25);
	}
}

// Turn 90 degrees to the right.
void turn_right() {
  set_motor_speed(&controllers[FRONT_RIGHT_MOTOR], (-1)*TURNING_MOTOR_SPEED);
  set_motor_speed(&controllers[REAR_RIGHT_MOTOR], (-1)*TURNING_MOTOR_SPEED);

  set_motor_speed(&controllers[FRONT_LEFT_MOTOR], TURNING_MOTOR_SPEED);
  set_motor_speed(&controllers[REAR_LEFT_MOTOR], TURNING_MOTOR_SPEED);

  HAL_Delay(475);

  stop();
}

// Turn 90 degrees to the left.
void turn_left() {
  set_motor_speed(&controllers[FRONT_RIGHT_MOTOR], TURNING_MOTOR_SPEED);
  set_motor_speed(&controllers[REAR_RIGHT_MOTOR], TURNING_MOTOR_SPEED);

  set_motor_speed(&controllers[FRONT_LEFT_MOTOR], (-1)*TURNING_MOTOR_SPEED);
  set_motor_speed(&controllers[REAR_LEFT_MOTOR], (-1)*TURNING_MOTOR_SPEED);

  HAL_Delay(400);

  stop();
}
