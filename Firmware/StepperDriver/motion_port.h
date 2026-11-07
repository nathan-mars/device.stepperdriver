/*
 * motion_port.h - hardware abstraction used by motion.c (AVR XMEGA version).
 * The host tests provide their own copy of this header in ../tests.
 */
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
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

/* Short critical sections for 16/32-bit values shared with the step ISR and
 * for 16-bit timer register writes (XMEGA TEMP register hazard). */
#define MOTION_ATOMIC() ATOMIC_BLOCK(ATOMIC_RESTORESTATE)

/* Set the DIR pin and start the pulse timer with the given period. */
void motion_hal_start(uint8_t i, uint16_t period_ticks, bool dir_positive);
/* Hand a new period to the timer; takes effect at the next pulse. */
void motion_hal_set_period(uint8_t i, uint16_t period_ticks);
/* True while the timer is running (the step ISR has not stopped it yet). */
bool motion_hal_timer_running(uint8_t i);
/* Visual indicator */
void motion_hal_led(uint8_t i, bool on);

#endif
