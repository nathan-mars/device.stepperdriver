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
#include "../StepperDriver/homing.h"

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

/* Simulated endstop on a digital input, watching one motor's hardware position.
 * The sensor is active while the position is at or beyond `edge` on the side
 * given by `at_negative_end`; it releases `hysteresis` pulses further out. */
typedef struct {
	bool     used;
	uint8_t  motor;
	int32_t  edge;
	int32_t  hysteresis;
	bool     at_negative_end;
	bool     active_low;     /* the line goes low when the sensor is active */
	bool     active;
} sim_endstop_t;
extern sim_endstop_t sim_endstop[4];
void sim_set_endstop(uint8_t input, uint8_t motor, int32_t edge, bool at_negative_end, bool active_low);

/* Speed profile restored by homing_hal_restore_profile (stands in for the registers). */
extern float sim_profile[MOTION_MOTORS][3];   /* v0, vmax, a */
void sim_set_profile(uint8_t i, float v0, float vmax, float a);

void sim_reset(void);
/* Advance the simulation by dt seconds, emitting pulses and running the per-pulse
 * ISR body exactly as the firmware would. */
void sim_advance(double dt);
/* Convenience: run n planner milliseconds (advance 1 ms, then tick). */
void sim_run_ms(uint32_t n);   /* homing tick, then motion tick, each ms */
#endif
