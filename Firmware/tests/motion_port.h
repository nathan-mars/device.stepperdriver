/* Host-side version of motion_port.h: the HAL is implemented by sim_hal.c. */
#ifndef MOTION_PORT_H
#define MOTION_PORT_H
#include <stdint.h>
/* The Harp core library predates <stdbool.h>; keep its convention so the
 * prototypes it exports see the same `bool` everywhere. */
#ifndef bool
	#define bool uint8_t
#endif
#ifndef true
	#define true 1
	#define false 0
#endif
#include <limits.h>
#define MOTION_ATOMIC() for (int once_ = 1; once_; once_ = 0)
void motion_hal_start(uint8_t i, uint16_t period_ticks, bool dir_positive);
void motion_hal_set_period(uint8_t i, uint16_t period_ticks);
bool motion_hal_timer_running(uint8_t i);
void motion_hal_led(uint8_t i, bool on);
#endif
