/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "l298n_motor_controller.h"
#include "vl53l0x_api.h"
#include <stdio.h>
#include "stm32f4xx_hal_gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_RATIO_MIN 1.0f
#define MOTOR_RATIO_MAX 3.0f
#define TOF_CALIBRATION_DIST 43000
#define STOPPING_DISTANCE 250

// Motor speeds
#define BASE_MOTOR_SPEED 50
#define TURNING_MOTOR_SPEED 100

// Scaling factor to compensate for right motor being weaker.
#define RMOTOR_SCALING_FACTOR 2

// Motors used in murphy.
typedef enum {
	FRONT_LEFT_MOTOR = 0,
	FRONT_RIGHT_MOTOR,
	REAR_LEFT_MOTOR,
	REAR_RIGHT_MOTOR,
	NUM_MOTORS,
} Motor;

struct TOF_Calibration{
  uint32_t refSpadCount;
  uint8_t isApertureSpads;
  uint8_t VhvSettings;
  uint8_t PhaseCal;
  VL53L0X_RangingMeasurementData_t RangingData;
};

// Status of ToF sensors.
typedef struct {
	volatile int front_left_tof_ready;
	volatile int front_right_tof_ready;
	volatile int rear_left_tof_ready;
} TofStatus;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FMPI2C_HandleTypeDef hfmpi2c1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// TODO: move this somewhere better
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

// ToF sensor comms
VL53L0X_Dev_t  vl53l0x_1; // front left
VL53L0X_DEV    FL_I2C1 = &vl53l0x_1;
VL53L0X_Dev_t  vl53l0x_2; // rear left
VL53L0X_DEV    RL_I2C2 = &vl53l0x_2;
VL53L0X_Dev_t  vl53l0x_3; // rear right
VL53L0X_DEV    FR_I2C3 = &vl53l0x_3;

// Motor controller pin mappings
static MotorController controllers[NUM_MOTORS] = {
		[FRONT_LEFT_MOTOR] = {
			.in1_pin = {GPIO_PIN_14, GPIOB},
			.in2_pin = {GPIO_PIN_15, GPIOB},
			.en_pin = {&htim1, TIM_CHANNEL_2, &TIM1->CCR2},
		},
		[FRONT_RIGHT_MOTOR] = {
			.in1_pin = {GPIO_PIN_11, GPIOC},
			.in2_pin = {GPIO_PIN_10, GPIOC},
			.en_pin = {&htim1, TIM_CHANNEL_3, &TIM1->CCR3},
		},
		[REAR_LEFT_MOTOR] = {
			.in1_pin = {GPIO_PIN_15, GPIOA},
			.in2_pin = {GPIO_PIN_1, GPIOA},
			.en_pin = {&htim1, TIM_CHANNEL_4, &TIM1->CCR4},
		},
		[REAR_RIGHT_MOTOR] = {
			.in1_pin = {GPIO_PIN_4, GPIOB},
			.in2_pin = {GPIO_PIN_5, GPIOB},
			.en_pin = {&htim8, TIM_CHANNEL_3, &TIM8->CCR3},
		},
};

// Storage for status of whether ToF sensor data ready
static TofStatus tof_status;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FMPI2C1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
float get_tof_rangedata(VL53L0X_DEV dev);
void stop();
void detect_wall_and_turn(void);

// TODO: move this somewhere else, just here for the time being
// Handle external interrupt from ToF sensors
void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin) {
	switch(gpio_pin) {
	case FR_TOF_EXTI_Pin:
		tof_status.front_right_tof_ready = 1;
	case FL_TOF_EXTI_Pin:
		tof_status.front_left_tof_ready = 1;
	case RL_TOF_EXTI_Pin:
		tof_status.rear_left_tof_ready = 1;
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TOF_Init(VL53L0X_DEV dev, struct TOF_Calibration tof){
	// VL53L0X init for Single Measurement
	VL53L0X_WaitDeviceBooted(dev);
	VL53L0X_DataInit(dev);
	VL53L0X_StaticInit(dev);
	VL53L0X_PerformRefCalibration(dev, &tof.VhvSettings, &tof.PhaseCal);
	VL53L0X_PerformRefSpadManagement(dev, &tof.refSpadCount, &tof.isApertureSpads);
    VL53L0X_SetOffsetCalibrationDataMicroMeter(dev, TOF_CALIBRATION_DIST);
    VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

	// Enable/Disable Sigma and Signal check
	VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckValue(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
	VL53L0X_SetLimitCheckValue(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
	VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev, 33000);
	VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
	VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

	// Set up GPIO for interrupts
	VL53L0X_SetGpioConfig(dev, 0, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
	        VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY,
	        VL53L0X_INTERRUPTPOLARITY_LOW);
	VL53L0X_ClearInterruptMask(dev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
	VL53L0X_StartMeasurement(dev);
}

// P control loop.
void course_correction() {
	static VL53L0X_RangingMeasurementData_t tof_fl_rangedata;
	static VL53L0X_RangingMeasurementData_t tof_rl_rangedata;

	/*
  // Currently, these measurements take around 70 ms to complete.
	VL53L0X_Error err1 = VL53L0X_PerformSingleRangingMeasurement(FL_I2C1, &tof_fl_rangedata);
	VL53L0X_Error err2 = VL53L0X_PerformSingleRangingMeasurement(RL_I2C2, &tof_rl_rangedata);
	float front = tof_fl_rangedata.RangeMilliMeter;
	float rear = tof_rl_rangedata.RangeMilliMeter;
	*/

	while(!tof_status.front_left_tof_ready || !tof_status.rear_left_tof_ready);

	VL53L0X_Error err1 = VL53L0X_GetRangingMeasurementData(FL_I2C1, &tof_fl_rangedata);

	VL53L0X_ClearInterruptMask(FL_I2C1, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
	tof_status.front_left_tof_ready = 0;

	VL53L0X_Error err2 = VL53L0X_GetRangingMeasurementData(RL_I2C2, &tof_rl_rangedata);

	VL53L0X_ClearInterruptMask(RL_I2C2, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
	tof_status.rear_left_tof_ready = 0;

	float front = tof_fl_rangedata.RangeMilliMeter;
	float rear = tof_rl_rangedata.RangeMilliMeter;

	if(err1 != VL53L0X_ERROR_NONE || err2 != VL53L0X_ERROR_NONE) {
	  // I2C might be disconnected, so stop to indicate we're having issues.
	  stop();
	  while(1);
	}

	if (front > rear) {
		float x = (front - rear)/front;

		set_motor_speed(&controllers[FRONT_RIGHT_MOTOR], (int)((1+x) * RMOTOR_SCALING_FACTOR * BASE_MOTOR_SPEED));
		set_motor_speed(&controllers[REAR_RIGHT_MOTOR], (int)((1+x) * RMOTOR_SCALING_FACTOR * BASE_MOTOR_SPEED));

		set_motor_speed(&controllers[FRONT_LEFT_MOTOR], 0);
		set_motor_speed(&controllers[REAR_LEFT_MOTOR], 0);
	}

	if (rear > front) {
		float x = (rear - front)/rear;

		set_motor_speed(&controllers[FRONT_RIGHT_MOTOR], 0);
		set_motor_speed(&controllers[REAR_RIGHT_MOTOR], 0);

		set_motor_speed(&controllers[FRONT_LEFT_MOTOR], (int)((1+x) * BASE_MOTOR_SPEED));
		set_motor_speed(&controllers[REAR_LEFT_MOTOR], (int)((1+x) * BASE_MOTOR_SPEED));
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

// Read data from the given ToF sensor and return the range.
float get_tof_rangedata(VL53L0X_DEV dev) {
  static VL53L0X_RangingMeasurementData_t tof_rangedata;
  VL53L0X_PerformSingleRangingMeasurement(dev, &tof_rangedata);
  return tof_rangedata.RangeMilliMeter;
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
		set_motor_speed(&controllers[FRONT_RIGHT_MOTOR], RMOTOR_SCALING_FACTOR * i);
		set_motor_speed(&controllers[FRONT_LEFT_MOTOR], i);
		set_motor_speed(&controllers[REAR_RIGHT_MOTOR], RMOTOR_SCALING_FACTOR * i);
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
		set_motor_speed(&controllers[FRONT_RIGHT_MOTOR], RMOTOR_SCALING_FACTOR * i);
		set_motor_speed(&controllers[FRONT_LEFT_MOTOR], i);
		set_motor_speed(&controllers[REAR_RIGHT_MOTOR], RMOTOR_SCALING_FACTOR * i);
		set_motor_speed(&controllers[REAR_LEFT_MOTOR], i);
		detect_wall_and_turn();
		HAL_Delay(25);
	}
}

// Check if the forward-facing ToF sensor detects a wall and turn 90 degrees to the right if so.
// This is a blocking call.
void detect_wall_and_turn(void) {
	static VL53L0X_RangingMeasurementData_t tof_fr_rangedata;
	//VL53L0X_Error err = VL53L0X_PerformSingleRangingMeasurement(FR_I2C3, &tof_fr_rangedata);
	// Wait for interrupt to fire
	while(!tof_status.front_right_tof_ready);

	VL53L0X_Error err = VL53L0X_GetRangingMeasurementData(FR_I2C3, &tof_fr_rangedata);

	err = VL53L0X_ClearInterruptMask(FR_I2C3, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
	tof_status.front_right_tof_ready = 0;

	if(err) {
		stop();
		while(1);
	}

	if(tof_fr_rangedata.RangeMilliMeter < STOPPING_DISTANCE) {
		// Execute right turn and continue
		stop();

		HAL_Delay(25);

		turn_right();

		HAL_Delay(250);

		move_forward(BASE_MOTOR_SPEED);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	struct TOF_Calibration TOF_FL;
	struct TOF_Calibration TOF_RL;
	struct TOF_Calibration TOF_RR;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_FMPI2C1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  FL_I2C1->I2cHandle = &hi2c1;
  FL_I2C1->I2cDevAddr = 0x52;
  RL_I2C2->I2cHandle = &hi2c2;
  RL_I2C2->I2cDevAddr = 0x52;
  FR_I2C3->I2cHandle = &hi2c3;
  FR_I2C3->I2cDevAddr = 0x52;

  TOF_Init(FL_I2C1, TOF_FL);
  TOF_Init(RL_I2C2, TOF_RL);
  TOF_Init(FR_I2C3, TOF_RR);

  motor_init(&controllers[FRONT_LEFT_MOTOR]);
  motor_init(&controllers[FRONT_RIGHT_MOTOR]);
  motor_init(&controllers[REAR_LEFT_MOTOR]);
  motor_init(&controllers[REAR_RIGHT_MOTOR]);

  set_motor_direction(&controllers[FRONT_RIGHT_MOTOR], MOTOR_DIR_OFF);
  set_motor_direction(&controllers[FRONT_LEFT_MOTOR], MOTOR_DIR_OFF);
  set_motor_direction(&controllers[REAR_RIGHT_MOTOR], MOTOR_DIR_OFF);
  set_motor_direction(&controllers[REAR_LEFT_MOTOR], MOTOR_DIR_OFF);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Testing the ToF sensor
  /*
  static VL53L0X_RangingMeasurementData_t tof_fr_rangedata;
  uint16_t values_read[10] = { 0 };
  HAL_Delay(500);
  for(int i = 0; i < 10; i++) {
	  while(!tof_status.front_right_tof_ready);

	  VL53L0X_Error err = VL53L0X_GetRangingMeasurementData(FR_I2C3, &tof_fr_rangedata);

	  err = VL53L0X_ClearInterruptMask(FR_I2C3, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
	  tof_status.front_right_tof_ready = 0;

	  values_read[i] = tof_fr_rangedata.RangeMilliMeter;
  }
  while(1);
  */

  while (1)
  {

  // Wait for button press before starting to move.
	while(HAL_GPIO_ReadPin(Pushbutton_GPIO_Port, Pushbutton_Pin) == 1);
	
    HAL_Delay(1000);
	move_forward(BASE_MOTOR_SPEED);

  // Course correct and detect walls until button is pressed again.
	while(HAL_GPIO_ReadPin(Pushbutton_GPIO_Port, Pushbutton_Pin) == 1) {
		course_correction();
		detect_wall_and_turn();
	}

	stop();
	HAL_Delay(1000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FMPI2C1;
  PeriphClkInitStruct.Fmpi2c1ClockSelection = RCC_FMPI2C1CLKSOURCE_APB;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FMPI2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FMPI2C1_Init(void)
{

  /* USER CODE BEGIN FMPI2C1_Init 0 */

  /* USER CODE END FMPI2C1_Init 0 */

  /* USER CODE BEGIN FMPI2C1_Init 1 */

  /* USER CODE END FMPI2C1_Init 1 */
  hfmpi2c1.Instance = FMPI2C1;
  hfmpi2c1.Init.Timing = 0x00A0A3F7;
  hfmpi2c1.Init.OwnAddress1 = 0;
  hfmpi2c1.Init.AddressingMode = FMPI2C_ADDRESSINGMODE_7BIT;
  hfmpi2c1.Init.DualAddressMode = FMPI2C_DUALADDRESS_DISABLE;
  hfmpi2c1.Init.OwnAddress2 = 0;
  hfmpi2c1.Init.OwnAddress2Masks = FMPI2C_OA2_NOMASK;
  hfmpi2c1.Init.GeneralCallMode = FMPI2C_GENERALCALL_DISABLE;
  hfmpi2c1.Init.NoStretchMode = FMPI2C_NOSTRETCH_DISABLE;
  if (HAL_FMPI2C_Init(&hfmpi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_FMPI2CEx_ConfigAnalogFilter(&hfmpi2c1, FMPI2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FMPI2C1_Init 2 */

  /* USER CODE END FMPI2C1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|LD2_Pin|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : Pushbutton_Pin */
  GPIO_InitStruct.Pin = Pushbutton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Pushbutton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 LD2_Pin PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LD2_Pin|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : FR_TOF_EXTI_Pin */
  GPIO_InitStruct.Pin = FR_TOF_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FR_TOF_EXTI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RL_TOF_EXTI_Pin FL_TOF_EXTI_Pin */
  GPIO_InitStruct.Pin = RL_TOF_EXTI_Pin|FL_TOF_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
