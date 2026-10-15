/*
 * quick_movement.h - compatibility layer for the Quick* registers (130-140).
 *
 * The quick-movement registers describe a move in physical units (um per
 * pulse, mm/s, m/s^2, mm). Since firmware 1.0 every move is a trapezoid run
 * by the motion engine, so these registers are only converted into the
 * engine's units (pulses, pulses/s, pulses/s^2) and the move is queued like
 * any relative move. Motors 1 and 2 only, as before.
 */
#ifndef _QUICK_MOVEMENT_H_
#define _QUICK_MOVEMENT_H_
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

/* Convert the Quick* registers of motor 1 or 2 into an engine profile and a
 * signed pulse count. Returns false if the registers are unusable. */
bool quick_movement_prepare(uint8_t motor_index, int32_t *pulses);

#endif /* _QUICK_MOVEMENT_H_ */
