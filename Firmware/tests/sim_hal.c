#include "sim_hal.h"
#include <stdio.h>

sim_timer_t sim[MOTION_MOTORS];
double sim_time = 0.0;
void (*sim_on_pulse)(uint8_t i, double t, uint16_t per) = 0;

#define TICK_S (1.0 / (double)MOTION_F_TICK_HZ)

void sim_reset(void)
{
	sim_time = 0.0;
	for (int i = 0; i < MOTION_MOTORS; i++) {
		sim[i] = (sim_timer_t){0};
		sim[i].min_per = 0xFFFF;
	}
}

void motion_hal_start(uint8_t i, uint16_t period_ticks, bool dir_positive)
{
	sim[i].running = true;
	sim[i].dir_positive = dir_positive;
	sim[i].per = period_ticks;
	sim[i].perbuf_valid = false;
	sim[i].next_pulse_t = sim_time + period_ticks * TICK_S;
}

void motion_hal_set_period(uint8_t i, uint16_t period_ticks)
{
	sim[i].perbuf = period_ticks;
	sim[i].perbuf_valid = true;
}

bool motion_hal_timer_running(uint8_t i) { return sim[i].running; }
void motion_hal_led(uint8_t i, bool on) { (void)i; (void)on; }

static void hw_stop(uint8_t i) { sim[i].running = false; }

static void pulse(uint8_t i)
{
	sim_timer_t *s = &sim[i];
	s->pulses++;
	s->hw_position += s->dir_positive ? 1 : -1;
	if (s->per < s->min_per) s->min_per = s->per;
	if (s->per > s->max_per) s->max_per = s->per;
	if (sim_on_pulse) sim_on_pulse(i, sim_time, s->per);
	/* The firmware's per-pulse interrupt. */
	MOTION_ISR_BODY(motion[i], hw_stop(i));
	/* Period buffer latches at the pulse boundary (XMEGA UPDATE at BOTTOM). */
	if (s->perbuf_valid) { s->per = s->perbuf; s->perbuf_valid = false; }
	s->next_pulse_t += s->per * TICK_S;
}

void sim_advance(double dt)
{
	double end = sim_time + dt;
	for (;;) {
		int next = -1;
		for (int i = 0; i < MOTION_MOTORS; i++)
			if (sim[i].running && (next < 0 || sim[i].next_pulse_t < sim[next].next_pulse_t)) next = i;
		if (next < 0 || sim[next].next_pulse_t > end) break;
		sim_time = sim[next].next_pulse_t;
		pulse((uint8_t)next);
	}
	sim_time = end;
}

void sim_run_ms(uint32_t n)
{
	for (uint32_t k = 0; k < n; k++) { sim_advance(0.001); motion_tick_1ms(); }
}
