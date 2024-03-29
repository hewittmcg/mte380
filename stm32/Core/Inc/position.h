#include <stdio.h>

#include "l298n_motor_controller.h"
#include "stm32f4xx_hal.h"
#include "vl53l0x_api.h"

typedef struct TOF_Calibration{
  uint32_t refSpadCount;
  uint8_t isApertureSpads;
  uint8_t VhvSettings;
  uint8_t PhaseCal;
} TOF_Calibration;

// ToF sensor locations.
typedef enum {
  FORWARD_TOF = 0,
  FRONT_SIDE_TOF,
  REAR_SIDE_TOF,
  NUM_TOFS,
} TofSensor;

// Status of ToF sensors.
typedef struct {
	volatile int data_ready[NUM_TOFS]; // Set by ISR handling EXTI from ToF sensor
} TofStatus;

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin);

void TOF_Init(I2C_HandleTypeDef *hi2c, TofSensor sensor);

void calibrate_tof(int32_t *forward_offset, int32_t *fs_offset, int32_t *rs_offset);

VL53L0X_Error get_tof_rangedata_cts(TofSensor sensor, uint16_t *range);

void detect_wall_and_turn();

void course_correction();

float get_angle_with_wall(uint16_t front_side, uint16_t rear_side);

void adjust_turn_tof();

void controlled_stop();

int get_tof_status(TofSensor sensor);
