#include "helpers.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

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
