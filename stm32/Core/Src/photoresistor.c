/*
 * photoresistor.c
 *
 *  Created on: Nov. 20, 2022
 *      Author: alaex
 */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "main.h"
#include "constants.h"
#include "logger.h"

bool in_sand(void){
	//reading the sensor to get color value
	HAL_ADC_Start(&hadc1);
	HAL_StatusTypeDef Err_ADC = HAL_ADC_PollForConversion(&hadc1, 1);
	uint16_t voltage = HAL_ADC_GetValue(&hadc1);
	log_item(LOG_SOURCE_PHOTORESISTOR, HAL_GetTick(), voltage, 0);

	if (Err_ADC != HAL_OK || voltage < PHOTORESISTOR_THRESHOLD ||
			voltage > PHOTORESISTOR_ABS_UPPER_BOUND) {
		return false;
	}

	else return true;

}

