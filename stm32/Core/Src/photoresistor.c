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

bool in_pit(void){
	uint16_t voltage;
	bool in_pit = true;
	HAL_StatusTypeDef Err_ADC = HAL_OK;

	//reading the sensor to get color value
	HAL_ADC_Start(&hadc1);
	Err_ADC = HAL_ADC_PollForConversion(&hadc1, 1);
	voltage = HAL_ADC_GetValue(&hadc1);

	if (Err_ADC != HAL_OK || voltage < PHOTORESISTOR_LOWER_BOUND ||
			voltage > PHOTORESISTOR_UPPER_BOUND) {
		in_pit = false;
	}

	return in_pit;

}

