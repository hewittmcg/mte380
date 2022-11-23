#include "l298n_motor_controller.h"
#include <stdio.h>
#include "stm32f4xx_hal_gpio.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

// Indicates the start of a critical section. Disables interrupts and returns whether interrupts were previously enabled.
bool critical_section_start(void);

// Indicates the end of a critical section. Enables interrupts only if they were enabled prior to the start of the critical section.
void critical_section_end(bool enabled);

// Gets the angle between the left side of the vehicle and the wall 
float get_angle_with_wall(uint16_t front_side, uint16_t rear_side);
