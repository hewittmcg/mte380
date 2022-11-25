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
#include "photoresistor.h"
#include "position_tracking.h"
#include "logger.h"

// Info for a straight-line section of the course.
typedef struct {
	uint16_t front_stop_dist_mm;
	uint16_t side_dist_mm;
	// Percentage to scale speed by (i.e. in later sections, where accuracy is more important)
	float speed_scaling_percent;
	uint32_t ticks_before_stop; // Number of ticks before we should start checking the front ToF sensor
} CourseSec;

#define COURSE_NUM_SECTIONS 11

#define EST_MAX_SPEED 1.0f // Estimated max speed in m/s
#define COURSE_SEC_LEN_M 0.3f // Length of one course section

// For D control, we want to track the last few errors in a ring buffer to compute the running average.
#define CC_NUM_TRACKED_MEASUREMENTS 10

// Full course map.
static const CourseSec COURSE_SECTIONS[COURSE_NUM_SECTIONS] = {
	{
		.front_stop_dist_mm = STOPPING_DISTANCE_MM,
		.side_dist_mm = TOF_BASE_SIDE_DIST_MM,
		.ticks_before_stop = (3 * COURSE_SEC_LEN_M) / EST_MAX_SPEED * MS_PER_SEC,
		.speed_scaling_percent = 1.0,
	},
	{
		.front_stop_dist_mm = STOPPING_DISTANCE_MM,
		.side_dist_mm = TOF_BASE_SIDE_DIST_MM,
		.ticks_before_stop = (4 * COURSE_SEC_LEN_M) / EST_MAX_SPEED * MS_PER_SEC,
		.speed_scaling_percent = 1.0,
	},
	{
		.front_stop_dist_mm = STOPPING_DISTANCE_MM,
		.side_dist_mm = TOF_BASE_SIDE_DIST_MM,
		.ticks_before_stop = (4 * COURSE_SEC_LEN_M) / EST_MAX_SPEED * MS_PER_SEC,
		.speed_scaling_percent = 1.0,
	},
	{
		.front_stop_dist_mm = STOPPING_DISTANCE_MM + BOARD_SQUARE_SIZE_MM,
		.side_dist_mm = TOF_BASE_SIDE_DIST_MM,
		.ticks_before_stop = (3 * COURSE_SEC_LEN_M) / EST_MAX_SPEED * MS_PER_SEC,
		.speed_scaling_percent = 1.0,
	},
	{
		.front_stop_dist_mm = STOPPING_DISTANCE_MM + BOARD_SQUARE_SIZE_MM + TOF_STOPPING_DISTANCE_OFFSET_MIDDLE,
		.side_dist_mm = TOF_BASE_SIDE_DIST_MM + BOARD_SQUARE_SIZE_MM - 15,
		.ticks_before_stop = (3 * COURSE_SEC_LEN_M) / EST_MAX_SPEED * MS_PER_SEC,
		.speed_scaling_percent = 1.00,
		
	},
	{
		.front_stop_dist_mm = STOPPING_DISTANCE_MM + BOARD_SQUARE_SIZE_MM + TOF_STOPPING_DISTANCE_OFFSET_MIDDLE,
		.side_dist_mm = TOF_BASE_SIDE_DIST_MM + BOARD_SQUARE_SIZE_MM - 15,
		.ticks_before_stop = (2 * COURSE_SEC_LEN_M) / EST_MAX_SPEED * MS_PER_SEC,
		.speed_scaling_percent = 1.00,
	},
	{
		.front_stop_dist_mm = STOPPING_DISTANCE_MM + BOARD_SQUARE_SIZE_MM + TOF_STOPPING_DISTANCE_OFFSET_MIDDLE,
		.side_dist_mm = TOF_BASE_SIDE_DIST_MM + BOARD_SQUARE_SIZE_MM - 15,
		.ticks_before_stop = (2 * COURSE_SEC_LEN_M) / EST_MAX_SPEED * MS_PER_SEC,
		.speed_scaling_percent = 1.00,
	},
	{
		.front_stop_dist_mm = STOPPING_DISTANCE_MM + 2*BOARD_SQUARE_SIZE_MM + TOF_STOPPING_DISTANCE_OFFSET_MIDDLE,
		.side_dist_mm = TOF_BASE_SIDE_DIST_MM + BOARD_SQUARE_SIZE_MM - 15,
		.ticks_before_stop = (1 * COURSE_SEC_LEN_M) / EST_MAX_SPEED * MS_PER_SEC,
		.speed_scaling_percent = 1.00,
	},
	{
		.front_stop_dist_mm = STOPPING_DISTANCE_MM + 2*BOARD_SQUARE_SIZE_MM + TOF_STOPPING_DISTANCE_OFFSET_CENTRE,
		.side_dist_mm = TOF_BASE_SIDE_DIST_MM + 2*BOARD_SQUARE_SIZE_MM - 15,
		.ticks_before_stop = (1 * COURSE_SEC_LEN_M) / EST_MAX_SPEED * MS_PER_SEC,
		.speed_scaling_percent = 0.8,
	},
	{
		.front_stop_dist_mm = STOPPING_DISTANCE_MM + 2*BOARD_SQUARE_SIZE_MM + TOF_STOPPING_DISTANCE_OFFSET_CENTRE,
		.side_dist_mm = TOF_BASE_SIDE_DIST_MM + 2*BOARD_SQUARE_SIZE_MM - 10,
		.ticks_before_stop = (0 * COURSE_SEC_LEN_M) / EST_MAX_SPEED * MS_PER_SEC,
		.speed_scaling_percent = 0.8,
	},
	{
		.front_stop_dist_mm = STOPPING_DISTANCE_MM + 2*BOARD_SQUARE_SIZE_MM + TOF_STOPPING_DISTANCE_OFFSET_CENTRE,
		.side_dist_mm = TOF_BASE_SIDE_DIST_MM + 2*BOARD_SQUARE_SIZE_MM - 10,
		.ticks_before_stop = (0 * COURSE_SEC_LEN_M) / EST_MAX_SPEED * MS_PER_SEC,
		.speed_scaling_percent = 0.8,
	}
};

// Section of the course currently being traversed.
static uint8_t cur_course_sec = 0;

// Number of ticks elapsed at the start of the current section.
static uint32_t ticks_at_start_of_sec = 0;

// Storage for status of whether ToF sensor data ready
static TofStatus tof_status;

static bool in_pit_section = false;

// ToF sensor device mappings
static VL53L0X_Dev_t TOF_I2C[NUM_TOFS] = {0};
static int TOF_CALIBRATION_DIST[NUM_TOFS] = {44000, 39000, 27000};
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

void calibrate_tof(int32_t *forward_offset, int32_t *fs_offset, int32_t *rs_offset) {
	FixPoint1616_t cal_dist = 300 << 16;
	VL53L0X_PerformOffsetCalibration(&TOF_I2C[FORWARD_TOF], cal_dist, forward_offset);
	VL53L0X_PerformOffsetCalibration(&TOF_I2C[FRONT_SIDE_TOF], cal_dist, fs_offset);
	VL53L0X_PerformOffsetCalibration(&TOF_I2C[REAR_SIDE_TOF], cal_dist, rs_offset);
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

	uint16_t range = 0;
	VL53L0X_Error err = get_tof_rangedata_cts(FORWARD_TOF, &range);

	if(err) {
		stop();
		while(1);
	}

	// uint16_t volt = 0;
	// bool sand;
	// sand = in_sand(&volt);

	// log_item(LOG_SOURCE_FORWARD_TOF, HAL_GetTick(), range, 0);
	// log_item(LOG_SOURCE_PIT_DETECT, HAL_GetTick(), get_gyro_recent_x_diff(), volt);


	if(range < COURSE_SECTIONS[cur_course_sec].front_stop_dist_mm) {
		// If in sand, ignore reading until we're out
		uint16_t volt = 0;
		if(in_sand(&volt)) {
			HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
			return;
		} else {
			HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
		}
		// Check whether we are in a pit and ignore the reading if so
		float angle = get_gyro_recent_x_diff();

		// log_item(LOG_SOURCE_PIT_DETECT, HAL_GetTick(), angle, volt);
		if(angle <= PIT_DETECT_THRESH) {
			printf("In pit with angle = %d.%d\r\n", (int)(angle), (int)((angle - (int)angle * 1000)));
			return;
		} else {
			printf("At wall with angle = %d.%d\r\n", (int)(angle), (int)((angle - (int)angle * 1000)));
		}

		stop();
		// Just testing: check if we're in sand after 100 ms and continue
		if(in_sand(&volt)) {
			move_forward(BASE_MOTOR_SPEED);
			return;
		}

		cur_course_sec++;
		// If we're in the section with the pit, we behave differently
		if (cur_course_sec == 3) {
			adjust_turn_tof();
			in_pit_section = true;
		} else {
			in_pit_section = false;
		}
		
		if(cur_course_sec >= COURSE_NUM_SECTIONS) {
			cur_course_sec = 0;
			stop();
			while(1);
		}

		// Adjust the amount we turn by using the ToF angle
		// < 5 so we don't turn in the middle turns
		turn_right_imu(90);

		// Reset IMU readings when we turn
		reset_imu_tracking();
		
		// Don't adjust in centre, inaccurate
		//if(cur_course_sec > 6) {
			#if 0
			uint16_t front_side, rear_side;
			get_side_tof_readings(&front_side, &rear_side);
			float theta = get_angle_with_wall(front_side, rear_side);
			turn_right_imu(theta);
			#endif
			/*
			if (theta > TOF_ANGLE_CORRECTION_THRESHOLD) {
				adjust_turn_tof();
			}
			*/
		//}

		// Execute right turn and continue on the next course section
		

		// Reset ticks at start
		ticks_at_start_of_sec = HAL_GetTick();
		move_forward(BASE_MOTOR_SPEED * COURSE_SECTIONS[cur_course_sec].speed_scaling_percent);
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

	// log_item(LOG_SOURCE_SIDE_TOFS, HAL_GetTick(), front, rear);
	float x = 0.0;
	if (front > rear) {
		if(calc_centre_dist(front, rear) - COURSE_SECTIONS[cur_course_sec].side_dist_mm < (CORRECTION_RANGE_MIN)){
			x = 0.5;
		}
		else if(calc_centre_dist(front, rear) - COURSE_SECTIONS[cur_course_sec].side_dist_mm > (CORRECTION_RANGE_MAX)){
			x = -0.9;		
		}else{
			x = (float)(front - rear)/(float)front * CORRECTION_FACTOR * -1;
			if(x < -0.5) {
				x = -0.5;
			}
		}
	}

	else if (rear > front) {
	        if(calc_centre_dist(front, rear) - COURSE_SECTIONS[cur_course_sec].side_dist_mm > (CORRECTION_RANGE_MAX)){
				x = -0.5;			
			} else if(calc_centre_dist(front, rear) - COURSE_SECTIONS[cur_course_sec].side_dist_mm <(CORRECTION_RANGE_MIN)){
	            x = 0.9;	        
			} else{
	            x = (float)(rear - front)/(float)rear * CORRECTION_FACTOR;
	            if(0.5 - x < 0) {
	                x = 0.5;
	            }
	        }
	    }
	// Handle pits better by not course correcting that much
	if(in_pit_section) {
		if(x < 0) {
			x = MAX(-0.15, x);
		} else {
			x = MIN(0.15, x);
		}
	}
	set_motor_id_speed(FRONT_RIGHT_MOTOR, (int)((1-x) * BASE_MOTOR_SPEED * COURSE_SECTIONS[cur_course_sec].speed_scaling_percent));
	set_motor_id_speed(REAR_RIGHT_MOTOR, (int)((1-x)* BASE_MOTOR_SPEED * COURSE_SECTIONS[cur_course_sec].speed_scaling_percent));
	set_motor_id_speed(FRONT_LEFT_MOTOR, (int)((1+x) * BASE_MOTOR_SPEED * COURSE_SECTIONS[cur_course_sec].speed_scaling_percent));
	set_motor_id_speed(REAR_LEFT_MOTOR, (int)((1+x) * BASE_MOTOR_SPEED * COURSE_SECTIONS[cur_course_sec].speed_scaling_percent));

	uint16_t dummy;
	if(in_sand(&dummy)) {
		set_motor_id_speed(FRONT_RIGHT_MOTOR, 100);
		set_motor_id_speed(FRONT_LEFT_MOTOR, 100);
		set_motor_id_speed(REAR_RIGHT_MOTOR, 100);
		set_motor_id_speed(REAR_LEFT_MOTOR, 100);
	}

}

// Gets the angle between the left side of the vehicle and the wall
float get_angle_with_wall(uint16_t front_side, uint16_t rear_side) {
	return atan2((front_side - rear_side), SIDE_TOF_SEPARATION_MM) * RADIONS_TO_DEGREES;
}


void adjust_turn_tof() { 
	uint16_t front_side = 0;
	uint16_t rear_side = 0;

	get_side_tof_readings(&front_side, &rear_side);
	float theta = get_angle_with_wall(front_side, rear_side);

	while (fabs(theta) > ANGLE_CORRECTION_ADJUSTMENT_THRESHOLD) { // only correct if off by ANGLE_CORRECTION_ADJUSTMENT_THRESHOLD degrees, TODO: Tuning
		int turning_speed = TURNING_MOTOR_SPEED;
		if (theta < TOF_ANGLE_CORRECTION_THRESHOLD) {
			// Go slower if angle to wall is small
			turning_speed = TURN_CORRECTION_MOTOR_SPEED;
		}
	
		if (front_side > rear_side) {
			// turn to the left
			set_motor_id_speed(FRONT_RIGHT_MOTOR, turning_speed);
			set_motor_id_speed(REAR_RIGHT_MOTOR, turning_speed);
			set_motor_id_speed(FRONT_LEFT_MOTOR, turning_speed * -1);
			set_motor_id_speed(REAR_LEFT_MOTOR, turning_speed * -1);
		} else {
			// turn to the right
			set_motor_id_speed(FRONT_RIGHT_MOTOR, turning_speed * -1);
			set_motor_id_speed(REAR_RIGHT_MOTOR, turning_speed * -1);
			set_motor_id_speed(FRONT_LEFT_MOTOR, turning_speed);
			set_motor_id_speed(REAR_LEFT_MOTOR, turning_speed);
		}

		get_side_tof_readings(&front_side, &rear_side);
		theta = get_angle_with_wall(front_side, rear_side);

		if (fabs(theta ) < ANGLE_CORRECTION_ADJUSTMENT_THRESHOLD) {
			set_motors_to_stop();
			// double check theta in case of drifting
			get_side_tof_readings(&front_side, &rear_side);
			theta = get_angle_with_wall(front_side, rear_side);
		}
	}

	set_motors_to_stop();
	return;
}

void controlled_stop() {
	uint16_t range = 0;
	VL53L0X_Error err = get_tof_rangedata_cts(FORWARD_TOF, &range);

	if (err) {
		stop();
		while(1);
	}

	int position_difference = range - (COURSE_SECTIONS[cur_course_sec].front_stop_dist_mm - CONTROLLED_STOP_DISTANCE_CORRECTION);
	float total_position_error = get_front_tof_recent_diff();

	// instantiated pre while loop for debugging
	float scaling_factor = 0;
	int motor_speed = 0;
	while (abs(position_difference) > CONTROLLED_STOP_RANGE) {
		scaling_factor = ( (float) position_difference) * CC_KP + total_position_error * CC_KI;
		motor_speed = (int) (BASE_MOTOR_SPEED * scaling_factor);
		
		// Go at max half base speed when correcting
		if (abs(motor_speed) > (BASE_MOTOR_SPEED / 2)) {
			motor_speed = (BASE_MOTOR_SPEED / 2) * (scaling_factor < 0 ? -1 : 1);
		}

		// Dont go below minimum motor speed
		if (abs(motor_speed) < MIN_MOTOR_SPEED) {
			motor_speed = MIN_MOTOR_SPEED * (scaling_factor < 0 ? -1 : 1);
		}

		set_motor_id_speed(FRONT_LEFT_MOTOR, motor_speed);
		set_motor_id_speed(FRONT_RIGHT_MOTOR, motor_speed);
		set_motor_id_speed(REAR_RIGHT_MOTOR, motor_speed);
		set_motor_id_speed(REAR_LEFT_MOTOR, motor_speed);

		get_tof_rangedata_cts(FORWARD_TOF, &range);
		position_difference = range - (COURSE_SECTIONS[cur_course_sec].front_stop_dist_mm - CONTROLLED_STOP_DISTANCE_CORRECTION);

		add_front_tof_reading(position_difference);
		total_position_error = get_front_tof_recent_diff();
	}

	stop();
	reset_position_tracking();
	//HAL_Delay(300);
	return;
}

int get_tof_status(TofSensor sensor) {
	return tof_status.data_ready[sensor];
}
