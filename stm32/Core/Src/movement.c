#include "movement.h"
#include "constants.h"
#include "ICM20948.h"
#include <stdlib.h>

#define ACCELERATION_INCREMENT 10

// Motors used in murphy.
static MotorController *controllers;


void movement_init(MotorController *mcs) {
  controllers = mcs;

  motor_init(mcs);
  motor_init(mcs);
  motor_init(mcs);
  motor_init(mcs);

  set_motor_direction(mcs, MOTOR_DIR_OFF);
  set_motor_direction(mcs, MOTOR_DIR_OFF);
  set_motor_direction(mcs, MOTOR_DIR_OFF);
  set_motor_direction(mcs, MOTOR_DIR_OFF);
}

// Stop robot movement.
void stop() {
  set_motor_id_speed(FRONT_RIGHT_MOTOR, (-1)*MOTOR_BRAKE_SPEED);
  set_motor_id_speed(REAR_RIGHT_MOTOR, (-1)*MOTOR_BRAKE_SPEED);
  set_motor_id_speed(FRONT_LEFT_MOTOR, (-1)*MOTOR_BRAKE_SPEED);
  set_motor_id_speed(REAR_LEFT_MOTOR, (-1)*MOTOR_BRAKE_SPEED);

  HAL_Delay(MOTOR_BRAKE_DELAY);

  set_motor_id_speed(FRONT_RIGHT_MOTOR, 0);
  set_motor_id_speed(REAR_RIGHT_MOTOR, 0);
  set_motor_id_speed(FRONT_LEFT_MOTOR, 0);
  set_motor_id_speed(REAR_LEFT_MOTOR, 0);

}

void move_forward(int speed) {
  // Slowly increase speed.
    for (int i = 0; i <= speed; i += ACCELERATION_INCREMENT) {
        set_motor_id_speed(FRONT_RIGHT_MOTOR, i);
        set_motor_id_speed(FRONT_LEFT_MOTOR, i);
        set_motor_id_speed(REAR_RIGHT_MOTOR, i);
        set_motor_id_speed(REAR_LEFT_MOTOR, i);
        HAL_Delay(25);
    }
}

void move_backward(int speed) {
  // Slowly increase speed.
	for (int i = 0; i >= speed; i -= ACCELERATION_INCREMENT) {
		set_motor_id_speed(FRONT_RIGHT_MOTOR, i);
		set_motor_id_speed(FRONT_LEFT_MOTOR, i);
		set_motor_id_speed(REAR_RIGHT_MOTOR, i);
		set_motor_id_speed(REAR_LEFT_MOTOR, i);
		HAL_Delay(25);
	}
}

void set_motor_id_speed(Motor motor, int speed) {
  set_motor_speed(&controllers[motor], speed);
}

// Turn 90 degrees to the right.
void turn_right() {
  set_motor_id_speed(FRONT_RIGHT_MOTOR, (-1)*TURNING_MOTOR_SPEED);
  set_motor_id_speed(REAR_RIGHT_MOTOR, (-1)*TURNING_MOTOR_SPEED);

  set_motor_id_speed(FRONT_LEFT_MOTOR, TURNING_MOTOR_SPEED);
  set_motor_id_speed(REAR_LEFT_MOTOR, TURNING_MOTOR_SPEED);

  HAL_Delay(RIGHT_TURN_DELAY);

  stop();
}

void turn_right_imu(uint16_t degrees) {
	// Read from the IMU and numerically integrate to get the number of degrees
  axises gyro_reading;
  float degrees_turned = 0;

  icm20948_gyro_read_dps(&gyro_reading);
  float prev_reading = gyro_reading.z;

  float cur_time = HAL_GetTick();
  float prev_time = HAL_GetTick();
  float error = degrees - degrees_turned;

  // Initial turning setting
  set_motor_id_speed(FRONT_RIGHT_MOTOR, (-1)*TURNING_MOTOR_SPEED);
  set_motor_id_speed(REAR_RIGHT_MOTOR, (-1)*TURNING_MOTOR_SPEED);

  set_motor_id_speed(FRONT_LEFT_MOTOR, TURNING_MOTOR_SPEED);
  set_motor_id_speed(REAR_LEFT_MOTOR, TURNING_MOTOR_SPEED);

  // Turn until the error is minimized and we have been turning for at least IMU_TURN_MIN_TIME
  // This is slow, but should be quite accurate for the time being.
  // N: should be based on error and change in error (error and gyro.z)
  while(abs(error) > IMU_TURN_ERROR_THRESH || abs(gyro_reading.z) > IMU_TURN_OMEGA_THRESH ) {
	  // TODO: we only need to read the z value here, not all three.
	  icm20948_gyro_read_dps(&gyro_reading);
	  cur_time = HAL_GetTick();
	  if(cur_time == prev_time) continue; // Avoid divide by zero errors

	  // Numerically integrate
	  float cur_reading = gyro_reading.z;
	  float cur_degrees = ((prev_reading + cur_reading)/2) * (cur_time - prev_time) / MS_PER_SEC;

	  // Right turn seems to be positive IMU reading in the z-axis
	  degrees_turned += cur_degrees;
	  error = degrees - degrees_turned;

	  float x = 0;
	  int turn_back = error/cur_reading ? 1 : -1; // N: I rly shouldnt need this, but im stupid and want test
	  float Kp = 0.9;
	  float Kd = 0.2;
	  if(error > IMU_TURN_MIN_FULL_SPEED_ERROR) {
		  x = TURNING_MOTOR_SPEED;
	  } else {
		  // If we've overshot the target, turn back to it
		  turn_back = (error > 90 ? 1 : -1);
		  x = IMU_TURN_CORRECTION_SPEED * (Kp * error + Kd * gyro_reading.z * turn_back);
	  }

	  // Scale motors as we reach the reading
	  set_motor_id_speed(FRONT_RIGHT_MOTOR, (-1)*x);
	  set_motor_id_speed(REAR_RIGHT_MOTOR, (-1)*x);

	  set_motor_id_speed(FRONT_LEFT_MOTOR, x);
	  set_motor_id_speed(REAR_LEFT_MOTOR, x);

	  //N: set these after to use in controller section
	  prev_reading = cur_reading;
	  prev_time = cur_time;
  }


  set_motor_id_speed(FRONT_RIGHT_MOTOR, 0);
  set_motor_id_speed(REAR_RIGHT_MOTOR, 0);
  set_motor_id_speed(FRONT_LEFT_MOTOR, 0);
  set_motor_id_speed(REAR_LEFT_MOTOR, 0);
}

// Turn 90 degrees to the left.
void turn_left() {
  set_motor_id_speed(FRONT_RIGHT_MOTOR, TURNING_MOTOR_SPEED);
  set_motor_id_speed(REAR_RIGHT_MOTOR, TURNING_MOTOR_SPEED);

  set_motor_id_speed(FRONT_LEFT_MOTOR, (-1)*TURNING_MOTOR_SPEED);
  set_motor_id_speed(REAR_LEFT_MOTOR, (-1)*TURNING_MOTOR_SPEED);

  HAL_Delay(LEFT_TURN_DELAY);

  stop();
}
