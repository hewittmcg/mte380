#include "position.h"
#include "movement.h"
#include "vl53l0x_api.h"
#include "main.h"
#include "helpers.h"

#define TOF_CALIBRATION_DIST 43000
#define STOPPING_DISTANCE 250

// ToF sensor device mappings
static const VL53L0X_DEV *GET_TOF_DEV_PTR;
static const TOF_Calibration *TOFs;
static int BASE_MOTOR_SPEED;

// Storage for status of whether ToF sensor data ready
static TofStatus tof_status;

// Handle external interrupt from ToF sensors
void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin) {
	switch(gpio_pin) {
	case FR_TOF_EXTI_Pin:
		tof_status.data_ready[FORWARD_TOF] = 1;
	case FL_TOF_EXTI_Pin:
		tof_status.data_ready[FRONT_SIDE_TOF] = 1;
	case RL_TOF_EXTI_Pin:
		tof_status.data_ready[REAR_SIDE_TOF] = 1;
	}
}

void TOF_Init(VL53L0X_DEV *dev[], struct TOF_Calibration *tofs[], int speed){
	GET_TOF_DEV_PTR = dev;
	TOFs = tofs;
	BASE_MOTOR_SPEED = speed;

	for(int i = 0; i < sizeof(TOFs) / sizeof(struct TOF_Calibration); i++)
	{

		// VL53L0X init for Single Measurement
		VL53L0X_WaitDeviceBooted(GET_TOF_DEV_PTR[i]);
		VL53L0X_DataInit(GET_TOF_DEV_PTR[i]);
		VL53L0X_StaticInit(GET_TOF_DEV_PTR[i]);
		VL53L0X_PerformRefCalibration(GET_TOF_DEV_PTR[i], &TOFs[i].VhvSettings, &TOFs[i].PhaseCal);
		VL53L0X_PerformRefSpadManagement(GET_TOF_DEV_PTR[i], &TOFs[i].refSpadCount, &TOFs[i].isApertureSpads);
		VL53L0X_SetOffsetCalibrationDataMicroMeter(GET_TOF_DEV_PTR[i], TOF_CALIBRATION_DIST);
		VL53L0X_SetDeviceMode(GET_TOF_DEV_PTR[i], VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

		// Enable/Disable Sigma and Signal check
		VL53L0X_SetLimitCheckEnable(GET_TOF_DEV_PTR[i], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
		VL53L0X_SetLimitCheckEnable(GET_TOF_DEV_PTR[i], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
		VL53L0X_SetLimitCheckValue(GET_TOF_DEV_PTR[i], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
		VL53L0X_SetLimitCheckValue(GET_TOF_DEV_PTR[i], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
		VL53L0X_SetMeasurementTimingBudgetMicroSeconds(GET_TOF_DEV_PTR[i], 33000);
		VL53L0X_SetVcselPulsePeriod(GET_TOF_DEV_PTR[i], VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
		VL53L0X_SetVcselPulsePeriod(GET_TOF_DEV_PTR[i], VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

		// Set up GPIO for interrupts
		VL53L0X_SetGpioConfig(GET_TOF_DEV_PTR[i], 0, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
				VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY,
				VL53L0X_INTERRUPTPOLARITY_LOW);
		VL53L0X_StartMeasurement(GET_TOF_DEV_PTR[i]);
		VL53L0X_ClearInterruptMask(GET_TOF_DEV_PTR[i], VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
	}
}

// Read data from the given ToF sensor and set the pointer passed in to the range, returning any errors.
// To be called when using continuous ranging with interrupts.
VL53L0X_Error get_tof_rangedata_cts(TofSensor sensor, uint16_t *range) {
	VL53L0X_RangingMeasurementData_t tof_rangedata = { 0 };
	VL53L0X_DEV dev = &GET_TOF_DEV_PTR[sensor];
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

	tof_status.data_ready[sensor] = 0;

	critical_section_end(enabled);

	*range = tof_rangedata.RangeMilliMeter;
	return VL53L0X_ERROR_NONE;
}

// Check if the forward-facing ToF sensor detects a wall and turn 90 degrees to the right if so.
// This is a blocking call.
void detect_wall_and_turn() {

	uint16_t range = 0;
	VL53L0X_Error err = get_tof_rangedata_cts(FORWARD_TOF, &range);

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

void course_correction(MotorController controllers[]) {

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
	VL53L0X_Error err1 = get_tof_rangedata_cts(FRONT_SIDE_TOF, &front);
	VL53L0X_Error err2 = get_tof_rangedata_cts(REAR_SIDE_TOF, &rear);

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
