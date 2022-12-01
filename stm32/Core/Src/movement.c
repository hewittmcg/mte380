#include "movement.h"
#include "constants.h"
#include "ICM20948.h"
#include "logger.h"
#include <stdlib.h>

#define ACCELERATION_INCREMENT 10

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

void set_motor_id_speed(Motor motor, int speed) {
  set_motor_speed(&controllers[motor], speed);
}

void set_motors_to_stop() {
  set_motor_id_speed(FRONT_RIGHT_MOTOR, 0);
  set_motor_id_speed(REAR_RIGHT_MOTOR, 0);
  set_motor_id_speed(FRONT_LEFT_MOTOR, 0);
  set_motor_id_speed(REAR_LEFT_MOTOR, 0);
}

// Stop robot movement.
void stop() {

  // determine direction of movement
  int cur_motor_right = get_motor_speed(&controllers[FRONT_RIGHT_MOTOR]);
  int cur_motor_left = get_motor_speed(&controllers[FRONT_LEFT_MOTOR]);
  int motor_brake_dir = -1;
  if (cur_motor_left < 0 && cur_motor_right < 0) {
    motor_brake_dir = 1;
  }

  set_motor_id_speed(FRONT_RIGHT_MOTOR, motor_brake_dir*MOTOR_BRAKE_SPEED);
  set_motor_id_speed(REAR_RIGHT_MOTOR, motor_brake_dir*MOTOR_BRAKE_SPEED);
  set_motor_id_speed(FRONT_LEFT_MOTOR, motor_brake_dir*MOTOR_BRAKE_SPEED);
  set_motor_id_speed(REAR_LEFT_MOTOR, motor_brake_dir*MOTOR_BRAKE_SPEED);

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
        HAL_Delay(10);
    }
}

void move_backward(int speed) {
  // Slowly increase speed.
	for (int i = 0; i >= speed; i -= ACCELERATION_INCREMENT) {
		set_motor_id_speed(FRONT_RIGHT_MOTOR, i);
		set_motor_id_speed(FRONT_LEFT_MOTOR, i);
		set_motor_id_speed(REAR_RIGHT_MOTOR, i);
		set_motor_id_speed(REAR_LEFT_MOTOR, i);
		HAL_Delay(10);
	}
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
  log_item(LOG_SOURCE_TURN_STARTING, HAL_GetTick(), 0, 0);
  axises gyro_reading;
  float degrees_turned = 0;

  icm20948_gyro_read_dps(&gyro_reading);
  float prev_reading = gyro_reading.z;

  float prev_time = HAL_GetTick();
  float error = degrees - degrees_turned;
  uint32_t start_time = HAL_GetTick();

  // Initial turning setting
  set_motor_id_speed(FRONT_RIGHT_MOTOR, (-1)*TURNING_MOTOR_SPEED);
  set_motor_id_speed(REAR_RIGHT_MOTOR, (-1)*TURNING_MOTOR_SPEED);

  set_motor_id_speed(FRONT_LEFT_MOTOR, TURNING_MOTOR_SPEED);
  set_motor_id_speed(REAR_LEFT_MOTOR, TURNING_MOTOR_SPEED);
  uint32_t prev_log_time = 0;

  // Turn until the error is minimized and we have been turning for at least IMU_TURN_MIN_TIME
  // This is slow, but should be quite accurate for the time being.
  while(abs(error) > IMU_TURN_ERROR_THRESH || prev_time - start_time < IMU_TURN_MIN_TIME) {
    
	  // TODO: we only need to read the z value here, not all three.
	  icm20948_gyro_read_dps(&gyro_reading);
	  float cur_time = HAL_GetTick();
	  if(cur_time == prev_time) continue; // Avoid divide by zero errors

	  // Numerically integrate
	  float cur_degrees = ((prev_reading + gyro_reading.z)/2) * (cur_time - prev_time) / MS_PER_SEC;

	  // Right turn seems to be positive IMU reading in the z-axis
	  degrees_turned += cur_degrees;
	  prev_reading = gyro_reading.z;
	  prev_time = cur_time;
	  error = degrees - degrees_turned;

	  float turning_speed = 0;
	  if(error > IMU_TURN_MIN_FULL_SPEED_ERROR) {
		  turning_speed = TURNING_MOTOR_SPEED;
	  } else {
		  // If we've overshot the target, turn back to it
		  turning_speed = IMU_TURN_CORRECTION_SPEED * (error > 0 ? 1 : -1);
	  }
    if(cur_time > prev_log_time + 100) {
      log_item(LOG_SOURCE_IMU_TURN, HAL_GetTick(), cur_degrees, degrees_turned);
      prev_log_time = cur_time;
    }

	  // Scale motors as we reach the reading
	  set_motor_id_speed(FRONT_RIGHT_MOTOR, (-1)*turning_speed);
	  set_motor_id_speed(REAR_RIGHT_MOTOR, (-1)*turning_speed);

	  set_motor_id_speed(FRONT_LEFT_MOTOR, turning_speed);
	  set_motor_id_speed(REAR_LEFT_MOTOR, turning_speed);
  }

  set_motors_to_stop();
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
