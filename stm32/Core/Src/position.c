#include "position.h"
#include "movement.h"
#include "vl53l0x_api.h"
#include "main.h"
#include "helpers.h"

static int BASE_MOTOR_SPEED = 50;
static int TOF_CALIBRATION_DIST = 43000;
static int STOPPING_DISTANCE = 250;

// Storage for status of whether ToF sensor data ready
static TofStatus tof_status;

void position_init(int base_speed, int tof_calibration, int stopping_dist) {
	BASE_MOTOR_SPEED = base_speed;
	TOF_CALIBRATION_DIST = tof_calibration;
	STOPPING_DISTANCE = stopping_dist;
}

// Read data from the given ToF sensor and set the pointer passed in to the range, returning any errors.
// To be called when using continuous ranging with interrupts.
// cannot be called until interrupt fires, i.e, check status before calling
VL53L0X_Error get_tof_rangedata_cts(VL53L0X_DEV dev, uint16_t *range) {
	VL53L0X_RangingMeasurementData_t tof_rangedata = { 0 };
	VL53L0X_Error err = VL53L0X_GetRangingMeasurementData(dev, &tof_rangedata);
	if(err) {
		return err;
	}

	// Needs to be a critical section to avoid an edge case where the interrupt mask is cleared and the interrupt fires
	// before tof_status.data_ready[sensor] is set to 0, meaning that it will get set to 0 and never back to 1.
	bool enabled = critical_section_start();
	err = VL53L0X_ClearInterruptMask(dev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

	if(err) {
		// This should likely be a hard fault condition, as the data ready state is now unclear.
		critical_section_end(enabled);
		return err;
	}

	tof_status.data_ready[FORWARD_TOF] = 0;

	critical_section_end(enabled);

	*range = tof_rangedata.RangeMilliMeter;
	return VL53L0X_ERROR_NONE;
}

// Check if the forward-facing ToF sensor detects a wall and turn 90 degrees to the right if so.
// This is a blocking call.
void detect_wall_and_turn(VL53L0X_DEV front_tof) {

	uint16_t range = 0;
	VL53L0X_Error err = get_tof_rangedata_cts(front_tof, &range);

	if(err) {
		stop();
		while(1);
	}

	if(range < STOPPING_DISTANCE) {
		// Execute right turn and continue
		stop();

		HAL_Delay(25);

		turn_right();

		HAL_Delay(250);

		move_forward(BASE_MOTOR_SPEED);
	}
}

void course_correction(MotorController controllers[], VL53L0X_DEV FL_Tof, VL53L0X_DEV RL_Tof) {

  // leaving the below code in to compare reading speeds.
	/*
  // Currently, these measurements take around 70 ms to complete.
  static VL53L0X_RangingMeasurementData_t tof_fl_rangedata;
	static VL53L0X_RangingMeasurementData_t tof_rl_rangedata;
	VL53L0X_Error err1 = VL53L0X_PerformSingleRangingMeasurement(FL_I2C1, &tof_fl_rangedata);
	VL53L0X_Error err2 = VL53L0X_PerformSingleRangingMeasurement(RL_I2C2, &tof_rl_rangedata);
	float front = tof_fl_rangedata.RangeMilliMeter;
	float rear = tof_rl_rangedata.RangeMilliMeter;
	*/

	// Get data from the side ToF sensors
	uint16_t front = 0;
	uint16_t rear = 0;
	VL53L0X_Error err1 = get_tof_rangedata_cts(FL_Tof, &front);
	VL53L0X_Error err2 = get_tof_rangedata_cts(RL_Tof, &rear);

	if(err1 != VL53L0X_ERROR_NONE || err2 != VL53L0X_ERROR_NONE) {
	  // I2C might be disconnected, so stop to indicate we're having issues.
	  stop();
	  while(1);
	}

	if (front > rear) {
		float x = (float)(front - rear)/(float)front;

		set_motor_speed(&controllers[FRONT_RIGHT_MOTOR], (int)((1+x) * BASE_MOTOR_SPEED));
		set_motor_speed(&controllers[REAR_RIGHT_MOTOR], (int)((1+x) * BASE_MOTOR_SPEED));

		set_motor_speed(&controllers[FRONT_LEFT_MOTOR], 0);
		set_motor_speed(&controllers[REAR_LEFT_MOTOR], 0);
	}

	if (rear > front) {
		float x = (float)(rear - front)/(float)rear;

		set_motor_speed(&controllers[FRONT_RIGHT_MOTOR], 0);
		set_motor_speed(&controllers[REAR_RIGHT_MOTOR], 0);

		set_motor_speed(&controllers[FRONT_LEFT_MOTOR], (int)((1+x) * BASE_MOTOR_SPEED));
		set_motor_speed(&controllers[REAR_LEFT_MOTOR], (int)((1+x) * BASE_MOTOR_SPEED));
	}

}

int getTofStatus(TofSensor sensor) {
	return tof_status.data_ready[sensor];
}

void setTofStatus(TofSensor sensor, int value) {
	tof_status.data_ready[sensor] = value;
}

