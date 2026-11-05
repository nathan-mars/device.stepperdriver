#include "sim_hal.h"
#include <stdio.h>

sim_timer_t sim[MOTION_MOTORS];
double sim_time = 0.0;
void (*sim_on_pulse)(uint8_t i, double t, uint16_t per) = 0;
sim_endstop_t sim_endstop[4];
float sim_profile[MOTION_MOTORS][3];

#define TICK_S (1.0 / (double)MOTION_F_TICK_HZ)

void sim_reset(void)
{
	sim_time = 0.0;
	for (int i = 0; i < MOTION_MOTORS; i++) {
		sim[i] = (sim_timer_t){0};
		sim[i].min_per = 0xFFFF;
		sim_profile[i][0] = 500.0f; sim_profile[i][1] = 4000.0f; sim_profile[i][2] = 1.0e5f;
	}
	for (int k = 0; k < 4; k++) sim_endstop[k] = (sim_endstop_t){0};
}

static bool endstop_line_high(const sim_endstop_t *e) { return e->active != e->active_low; }

/* Recompute the sensor from the motor position; on a change of the line,
 * run the firmware's input interrupt. */
static void endstop_update(uint8_t k)
{
	sim_endstop_t *e = &sim_endstop[k];
	if (!e->used) return;
	int32_t p = sim[e->motor].hw_position;
	int32_t beyond = e->at_negative_end ? e->edge - p : p - e->edge;   /* >= 0: inside the sensor */
	bool now = e->active ? (beyond >= -e->hysteresis) : (beyond >= 0);
	if (now != e->active) {
		e->active = now;
		homing_input_edge(k, endstop_line_high(e));
	}
}

void sim_set_endstop(uint8_t input, uint8_t motor, int32_t edge, bool at_negative_end, bool active_low)
{
	sim_endstop_t *e = &sim_endstop[input];
	*e = (sim_endstop_t){ .used = true, .motor = motor, .edge = edge, .at_negative_end = at_negative_end, .active_low = active_low };
	int32_t p = sim[motor].hw_position;
	e->active = at_negative_end ? p <= edge : p >= edge;
}

void sim_set_profile(uint8_t i, float v0, float vmax, float a)
{
	sim_profile[i][0] = v0; sim_profile[i][1] = vmax; sim_profile[i][2] = a;
	motion_set_profile(i, v0, vmax, a);
}

bool homing_hal_input_high(uint8_t input)
{
	const sim_endstop_t *e = &sim_endstop[input];
	return e->used ? endstop_line_high(e) : true;   /* pulled up when nothing is wired */
}

void homing_hal_restore_profile(uint8_t i)
{
	motion_set_profile(i, sim_profile[i][0], sim_profile[i][1], sim_profile[i][2]);
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
	/* Endstops see the new position (input interrupt after the step interrupt). */
	for (uint8_t k = 0; k < 4; k++)
		if (sim_endstop[k].used && sim_endstop[k].motor == i) endstop_update(k);
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
	for (uint32_t k = 0; k < n; k++) { sim_advance(0.001); homing_tick_1ms(); motion_tick_1ms(); }
}
