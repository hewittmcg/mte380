#include "position.h"
#include "movement.h"
#include "vl53l0x_api.h"
#include "main.h"
#include "helpers.h"
#include "constants.h"
#include <math.h>
#include <stdlib.h>
#include "stm32f4xx_hal_gpio.h"
#include "imu_tracking.h"
#include "ICM20948.h"

// General course correction constants
#define SIDE_TOF_SEPARATION_MM 177 // Distance between the two side ToF sensors. TODO revise this
#define CC_KP 0.01f // Proportional coefficient (unused)
#define CC_KD 0.001f // Derivative coefficient (unused)

// Info for a straight-line section of the course.
typedef struct {
	uint16_t front_stop_dist_mm;
	uint16_t side_dist_mm;
	// Percentage to scale speed by (i.e. in later sections, where accuracy is more important)
	// float speed_scaling_percent; 
	uint32_t ticks_before_stop; // Number of ticks before we should start checking the front ToF sensor
} CourseSec;

// Only test the first 4 sections for the time being
#define COURSE_NUM_SECTIONS 11

#define EST_MAX_SPEED 0.8f // Estimated max speed in m/s
#define COURSE_SEC_LEN_M 0.3f // Length of one course section

// For D control, we want to track the last few errors in a ring buffer to compute the running average.
#define CC_NUM_TRACKED_MEASUREMENTS 10

// Full course map.
static const CourseSec COURSE_SECTIONS[COURSE_NUM_SECTIONS] = {
	{
		.front_stop_dist_mm = STOPPING_DISTANCE_MM,
		.side_dist_mm = TOF_BASE_SIDE_DIST_MM,
		.ticks_before_stop = (4 * COURSE_SEC_LEN_M) / EST_MAX_SPEED * MS_PER_SEC,
	},
	{
		.front_stop_dist_mm = STOPPING_DISTANCE_MM,
		.side_dist_mm = TOF_BASE_SIDE_DIST_MM,
		.ticks_before_stop = (4 * COURSE_SEC_LEN_M) / EST_MAX_SPEED * MS_PER_SEC,
	},
	{
		.front_stop_dist_mm = STOPPING_DISTANCE_MM,
		.side_dist_mm = TOF_BASE_SIDE_DIST_MM,
		.ticks_before_stop = (4 * COURSE_SEC_LEN_M) / EST_MAX_SPEED * MS_PER_SEC,
	},
	{
		.front_stop_dist_mm = STOPPING_DISTANCE_MM + BOARD_SQUARE_SIZE_MM,
		.side_dist_mm = TOF_BASE_SIDE_DIST_MM,
		.ticks_before_stop = (3 * COURSE_SEC_LEN_M) / EST_MAX_SPEED * MS_PER_SEC,
	},
	{
		.front_stop_dist_mm = STOPPING_DISTANCE_MM + BOARD_SQUARE_SIZE_MM,
		.side_dist_mm = TOF_BASE_SIDE_DIST_MM + BOARD_SQUARE_SIZE_MM,
		.ticks_before_stop = (3 * COURSE_SEC_LEN_M) / EST_MAX_SPEED * MS_PER_SEC,
		
	},
	{
		.front_stop_dist_mm = STOPPING_DISTANCE_MM + BOARD_SQUARE_SIZE_MM,
		.side_dist_mm = TOF_BASE_SIDE_DIST_MM + BOARD_SQUARE_SIZE_MM,
		.ticks_before_stop = (2 * COURSE_SEC_LEN_M) / EST_MAX_SPEED * MS_PER_SEC,
	},
	{
		.front_stop_dist_mm = STOPPING_DISTANCE_MM + BOARD_SQUARE_SIZE_MM,
		.side_dist_mm = TOF_BASE_SIDE_DIST_MM + BOARD_SQUARE_SIZE_MM,
		.ticks_before_stop = (2 * COURSE_SEC_LEN_M) / EST_MAX_SPEED * MS_PER_SEC,
	},
	{
		.front_stop_dist_mm = STOPPING_DISTANCE_MM + 2*BOARD_SQUARE_SIZE_MM,
		.side_dist_mm = TOF_BASE_SIDE_DIST_MM + BOARD_SQUARE_SIZE_MM,
		.ticks_before_stop = (1 * COURSE_SEC_LEN_M) / EST_MAX_SPEED * MS_PER_SEC,
	},
	{
		.front_stop_dist_mm = STOPPING_DISTANCE_MM + 2*BOARD_SQUARE_SIZE_MM,
		.side_dist_mm = TOF_BASE_SIDE_DIST_MM + 2*BOARD_SQUARE_SIZE_MM,
		.ticks_before_stop = (1 * COURSE_SEC_LEN_M) / EST_MAX_SPEED * MS_PER_SEC,
	},
	{
		.front_stop_dist_mm = STOPPING_DISTANCE_MM + 2*BOARD_SQUARE_SIZE_MM,
		.side_dist_mm = TOF_BASE_SIDE_DIST_MM + 2*BOARD_SQUARE_SIZE_MM,
		.ticks_before_stop = (0 * COURSE_SEC_LEN_M) / EST_MAX_SPEED * MS_PER_SEC,
	},
	{
		.front_stop_dist_mm = STOPPING_DISTANCE_MM + 2*BOARD_SQUARE_SIZE_MM,
		.side_dist_mm = TOF_BASE_SIDE_DIST_MM + 2*BOARD_SQUARE_SIZE_MM,
		.ticks_before_stop = (0 * COURSE_SEC_LEN_M) / EST_MAX_SPEED * MS_PER_SEC,
	}
};

// Section of the course currently being traversed.
static uint8_t cur_course_sec = 0;

// Number of ticks elapsed at the start of the current section.
static uint32_t ticks_at_start_of_sec = 0;

// Storage for status of whether ToF sensor data ready
static TofStatus tof_status;

// ToF sensor device mappings
static VL53L0X_Dev_t TOF_I2C[NUM_TOFS] = {0};
static int TOF_CALIBRATION_DIST[NUM_TOFS] = {70000, 60000, 45000};
static struct TOF_Calibration TOFs[NUM_TOFS] = {0};

// Handle external interrupt from ToF sensors
void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin) {
	switch(gpio_pin) {
	case FR_TOF_EXTI_Pin:
		tof_status.data_ready[FORWARD_TOF] = 1;
		break;
	case FL_TOF_EXTI_Pin:
		tof_status.data_ready[FRONT_SIDE_TOF] = 1;
		break;
	case RL_TOF_EXTI_Pin:
		tof_status.data_ready[REAR_SIDE_TOF] = 1;
		break;
	}
}

void TOF_Init(I2C_HandleTypeDef *hi2c, TofSensor sensor){
	TOF_I2C[sensor].I2cHandle = hi2c;
	TOF_I2C[sensor].I2cDevAddr = 0x52;

	// VL53L0X init for Single Measurement
	VL53L0X_WaitDeviceBooted(&TOF_I2C[sensor]);
	VL53L0X_DataInit(&TOF_I2C[sensor]);
	VL53L0X_StaticInit(&TOF_I2C[sensor]);
	VL53L0X_PerformRefCalibration(&TOF_I2C[sensor], &TOFs[sensor].VhvSettings, &TOFs[sensor].PhaseCal);
	VL53L0X_PerformRefSpadManagement(&TOF_I2C[sensor], &TOFs[sensor].refSpadCount, &TOFs[sensor].isApertureSpads);
	VL53L0X_SetOffsetCalibrationDataMicroMeter(&TOF_I2C[sensor], TOF_CALIBRATION_DIST[sensor]);
	VL53L0X_SetDeviceMode(&TOF_I2C[sensor], VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

	// Enable/Disable Sigma and Signal check
	VL53L0X_SetLimitCheckEnable(&TOF_I2C[sensor], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckEnable(&TOF_I2C[sensor], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckValue(&TOF_I2C[sensor], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
	VL53L0X_SetLimitCheckValue(&TOF_I2C[sensor], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
	VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&TOF_I2C[sensor], 33000);
	VL53L0X_SetVcselPulsePeriod(&TOF_I2C[sensor], VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
	VL53L0X_SetVcselPulsePeriod(&TOF_I2C[sensor], VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

	// Set up GPIO for interrupts
	VL53L0X_SetGpioConfig(&TOF_I2C[sensor], 0, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
	        VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY,
	        VL53L0X_INTERRUPTPOLARITY_LOW);
	VL53L0X_StartMeasurement(&TOF_I2C[sensor]);
	VL53L0X_ClearInterruptMask(&TOF_I2C[sensor], VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
}


// Read data from the given ToF sensor and set the pointer passed in to the range, returning any errors.
// To be called when using continuous ranging with interrupts.
// cannot be called until interrupt fires, i.e, check status before calling
VL53L0X_Error get_tof_rangedata_cts(TofSensor sensor, uint16_t *range) {
	VL53L0X_RangingMeasurementData_t tof_rangedata = { 0 };
	VL53L0X_Error err = VL53L0X_GetRangingMeasurementData(&TOF_I2C[sensor], &tof_rangedata);
	if(err) {
		return err;
	}

	// Needs to be a critical section to avoid an edge case where the interrupt mask is cleared and the interrupt fires
	// before tof_status.data_ready[sensor] is set to 0, meaning that it will get set to 0 and never back to 1.
	bool enabled = critical_section_start();
	err = VL53L0X_ClearInterruptMask(&TOF_I2C[sensor], VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

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

static inline void get_side_tof_readings(uint16_t *front, uint16_t *rear) {
	// Get data from the side ToF sensors
	VL53L0X_Error err1 = get_tof_rangedata_cts(FRONT_SIDE_TOF, front);
	VL53L0X_Error err2 = get_tof_rangedata_cts(REAR_SIDE_TOF, rear);

	if(err1 != VL53L0X_ERROR_NONE || err2 != VL53L0X_ERROR_NONE) {
	  // I2C might be disconnected, so stop to indicate we're having issues.
	  stop();
	  while(1);
	}
}

// Check if the forward-facing ToF sensor detects a wall and turn 90 degrees to the right if so.
// This is a blocking call.
void detect_wall_and_turn() {
	// Set ticks_at_start_of_sec at the first call
	if(ticks_at_start_of_sec == 0) {
		ticks_at_start_of_sec = HAL_GetTick();
	}
	
	// Don't do anything if it's before we want to start checking
	if(HAL_GetTick() - ticks_at_start_of_sec < COURSE_SECTIONS[cur_course_sec].ticks_before_stop) {
		return;
	}

	printf("detect wall and turn\r\n");
	uint16_t range = 0;
	VL53L0X_Error err = get_tof_rangedata_cts(FORWARD_TOF, &range);

	if(err) {
		stop();
		while(1);
	}

	if(range < COURSE_SECTIONS[cur_course_sec].front_stop_dist_mm) {
		// Check whether we are in a pit and ignore the reading if so
		float angle = get_gyro_recent_x_diff();
		if(angle <= -10.0f) {
			printf("In pit with angle = %d.%d\r\n", (int)(angle), (int)((angle - (int)angle * 1000)));
			stop();
			HAL_Delay(2000);
			move_forward(BASE_MOTOR_SPEED);
			return;
		} else {
			printf("At wall with angle = %d.%d\r\n", (int)(angle), (int)((angle - (int)angle * 1000)));
		}

		// Execute right turn and continue on the next course section
		cur_course_sec++;
		if(cur_course_sec >= COURSE_NUM_SECTIONS) {
			cur_course_sec = 0;
			stop();
			while(1);
		}

		stop();

		// Adjust the amount we turn by the side ToF reading
		uint16_t front = 0;
		uint16_t rear = 0;
		get_side_tof_readings(&front, &rear);
		// Calculate angle robot is turned at (this should maybe be moved into its own function):
		float cur_angle = atan2((front - rear), SIDE_TOF_SEPARATION_MM);

		turn_right_imu(90);
		// Test this: I don't think this is needed anymore
		//HAL_Delay(100);

		// Reset ticks at start
		ticks_at_start_of_sec = HAL_GetTick();
		move_forward(BASE_MOTOR_SPEED);
	}

}

// Calculates the right-angle distance from the centre of the robot to the wall
// based on readings from the two side ToF sensors.
static inline float calc_centre_dist(float dist_front, float dist_rear) {
	// Calculate angle robot is turned at:
	float theta = atan2((dist_front - dist_rear), SIDE_TOF_SEPARATION_MM);

	// Calculate distance from the bottom ToF and then relate that to the centre
	return dist_rear*cos(theta) + SIDE_TOF_SEPARATION_MM/2 * sin(theta);
}

void course_correction() {
	uint16_t front = 0;
	uint16_t rear = 0;
	get_side_tof_readings(&front, &rear);

	if (front > rear) {
	        if(rear - COURSE_SECTIONS[cur_course_sec].side_dist_mm < (-15)){}
	        else if(rear - COURSE_SECTIONS[cur_course_sec].side_dist_mm > (15)){
	            float x = 0.5;
	            set_motor_id_speed(FRONT_RIGHT_MOTOR, (int)((1+x) * BASE_MOTOR_SPEED));
	            set_motor_id_speed(REAR_RIGHT_MOTOR, (int)((1+x)* BASE_MOTOR_SPEED));

	            set_motor_id_speed(FRONT_LEFT_MOTOR, (int)((1-x) * BASE_MOTOR_SPEED));
	            set_motor_id_speed(REAR_LEFT_MOTOR, (int)((1-x) * BASE_MOTOR_SPEED));
	        }
	        else{
	            float x = (float)(front - rear)/(float)front * CORRECTION_FACTOR;
	            if(1 - x < 0) {
	                x = 1;
	            }

	            set_motor_id_speed(FRONT_RIGHT_MOTOR, (int)((1+x) * BASE_MOTOR_SPEED));
	            set_motor_id_speed(REAR_RIGHT_MOTOR, (int)((1+x) * BASE_MOTOR_SPEED));

	            set_motor_id_speed(FRONT_LEFT_MOTOR, (int)((1-x) * BASE_MOTOR_SPEED));
	            set_motor_id_speed(REAR_LEFT_MOTOR, (int)((1-x) * BASE_MOTOR_SPEED));
	        }
	    }

	if (rear > front) {
	        if(front - COURSE_SECTIONS[cur_course_sec].side_dist_mm > 15){}
	        else if(front - COURSE_SECTIONS[cur_course_sec].side_dist_mm < - 15){
	            float x = 0.5;
	            set_motor_id_speed(FRONT_RIGHT_MOTOR, (int)((1-x) * BASE_MOTOR_SPEED));
	            set_motor_id_speed(REAR_RIGHT_MOTOR, (int)((1-x)* BASE_MOTOR_SPEED));

	            set_motor_id_speed(FRONT_LEFT_MOTOR, (int)((1+x) * BASE_MOTOR_SPEED));
	            set_motor_id_speed(REAR_LEFT_MOTOR, (int)((1+x) * BASE_MOTOR_SPEED));
	        }
	        else{
	            float x = (float)(rear - front)/(float)rear * CORRECTION_FACTOR;
	            if(1 - x < 0) {
	                x = 1;
	            }

	            set_motor_id_speed(FRONT_RIGHT_MOTOR, (int)((1-x) * BASE_MOTOR_SPEED));
	            set_motor_id_speed(REAR_RIGHT_MOTOR, (int)((1-x)* BASE_MOTOR_SPEED));

	            set_motor_id_speed(FRONT_LEFT_MOTOR, (int)((1+x) * BASE_MOTOR_SPEED));
	            set_motor_id_speed(REAR_LEFT_MOTOR, (int)((1+x) * BASE_MOTOR_SPEED));
	        }
	    }

	}

int get_tof_status(TofSensor sensor) {
	return tof_status.data_ready[sensor];
}
