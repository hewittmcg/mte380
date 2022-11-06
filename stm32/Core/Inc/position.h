#include <stdio.h>

#include "l298n_motor_controller.h"
#include "stm32f4xx_hal.h"
#include "vl53l0x_api.h"

typedef struct TOF_Calibration{
  uint32_t refSpadCount;
  uint8_t isApertureSpads;
  uint8_t VhvSettings;
  uint8_t PhaseCal;
  VL53L0X_RangingMeasurementData_t RangingData;
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

void position_init(int base_speed, int tof_calibration, int stopping_dist);

VL53L0X_Error get_tof_rangedata_cts(VL53L0X_DEV dev, uint16_t *range);

void detect_wall_and_turn(VL53L0X_DEV F_Tof);

void course_correction(MotorController controllers[], VL53L0X_DEV FL_Tof, VL53L0X_DEV RL_Tof);

int getTofStatus(TofSensor sensor);

void setTofStatus(TofSensor sensor, int value);
