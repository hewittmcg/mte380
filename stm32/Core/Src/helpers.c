#include "helpers.h"
#include "constants.h"

// Indicates the start of a critical section. Disables interrupts and returns whether interrupts were previously enabled.
bool critical_section_start(void) {
	bool enabled = (__get_PRIMASK() == 0);
	__disable_irq();
	return enabled;
}

// Indicates the end of a critical section. Enables interrupts only if they were enabled prior to the start of the critical section.
void critical_section_end(bool enabled) {
	if(enabled) {
		__enable_irq();
	}
}

// Gets the angle between the left side of the vehicle and the wall 
float get_angle_with_wall(uint16_t front_side, uint16_t rear_side) {
	return atan2((front_side - rear_side), SIDE_TOF_SEPARATION_MM) * RADIONS_TO_DEGREES;
}
