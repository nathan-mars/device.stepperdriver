#ifndef SIM_HAL_H
#define SIM_HAL_H
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
#include "../StepperDriver/motion.h"

typedef struct {
	bool     running;
	bool     dir_positive;
	uint16_t per;        /* active period (ticks) */
	uint16_t perbuf;     /* pending period, latched at the next pulse */
	bool     perbuf_valid;
	double   next_pulse_t;   /* seconds */
	uint32_t pulses;         /* pulses emitted since reset */
	int32_t  hw_position;    /* independent pulse counter (what an encoder would see) */
	uint16_t min_per, max_per;
} sim_timer_t;

extern sim_timer_t sim[MOTION_MOTORS];
extern double sim_time;

/* Optional per-pulse observer (for plotting). */
extern void (*sim_on_pulse)(uint8_t i, double t, uint16_t per);

void sim_reset(void);
/* Advance the simulation by dt seconds, emitting pulses and running the per-pulse
 * ISR body exactly as the firmware would. */
void sim_advance(double dt);
/* Convenience: run n planner milliseconds (advance 1 ms, then tick). */
void sim_run_ms(uint32_t n);
#endif
