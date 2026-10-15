/*
 * motion.c - trapezoidal motion engine (see motion.h for the architecture).
 * No AVR headers here: compiled unchanged by the host-side tests.
 */
#include "motion.h"
#include <motion_port.h>   /* found via -I: AVR version in this dir, host version in ../tests */
#include <math.h>

motion_t motion[MOTION_MOTORS];

static int32_t *pos_reg;
static int32_t  max_lim[MOTION_MOTORS];
static int32_t  min_lim[MOTION_MOTORS];
static uint8_t        stopped_mask;

/* ------------------------------------------------------------------------ */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------ */
static uint16_t period_from_speed(float v)
{
	float p = (float)MOTION_F_TICK_HZ / v;
	if (p < (float)MOTION_MIN_PERIOD_TICKS) return MOTION_MIN_PERIOD_TICKS;
	if (p > (float)MOTION_MAX_PERIOD_TICKS) return MOTION_MAX_PERIOD_TICKS;
	return (uint16_t)(p + 0.5f);
}

static uint16_t cost_of(float vmax)
{
	/* permille of CPU: vmax [pulses/s] * cost [ns] / 1e6 */
	float c = vmax * (float)MOTION_ISR_COST_NS * 1e-6f;
	if (c > 1000.0f) c = 1000.0f;
	return (uint16_t)(c + 0.5f);
}

static uint32_t read_remaining(uint8_t i)
{
	uint32_t r;
	MOTION_ATOMIC() { r = motion[i].remaining; }
	return r;
}

static int32_t position_now(uint8_t i, uint32_t remaining)
{
	motion_t *m = &motion[i];
	int32_t done = (int32_t)(m->total - remaining);
	return m->dir_positive ? m->start_pos + done : m->start_pos - done;
}

static void write_pos(uint8_t i, int32_t p)
{
	MOTION_ATOMIC() { pos_reg[i] = p; }
}

/* Pulses available before hitting the travel limit in the given direction
 * (reaching the limit exactly is allowed). INT32_MAX when the limit is off. */
static int32_t room_to_limit(uint8_t i, int32_t from_pos, bool positive)
{
	int32_t lim = positive ? max_lim[i] : min_lim[i];
	if (lim == 0) return INT32_MAX;
	int32_t room = positive ? lim - from_pos : from_pos - lim;
	return room < 0 ? 0 : room;
}

/* Pulses needed to decelerate from v to v0 at a, plus one planner tick of travel. */
static float stop_distance(const motion_t *m)
{
	return (m->v * m->v - m->v0 * m->v0) / (2.0f * m->a) + m->v * MOTION_DT_S;
}

/* ------------------------------------------------------------------------ */
/* Configuration                                                             */
/* ------------------------------------------------------------------------ */
void motion_init(int32_t *pos)
{
	pos_reg = pos;
	stopped_mask = 0;
	for (uint8_t i = 0; i < MOTION_MOTORS; i++) {
		motion_t *m = &motion[i];
		max_lim[i] = 0; min_lim[i] = 0;
		m->remaining = 0; m->stop_req = 0; m->stopped = 0;
		m->running = 0; m->dir_positive = 1; m->decel = 0;
		m->total = 0; m->start_pos = 0;
		m->jog = 0; m->jog_pending = 0; m->jog_dir_positive = 1; m->jog_v = 0.0f;
		motion_set_profile_us(i, 250, 2000, motion_legacy_accel(250, 2000, 10));
	}
}

void motion_set_limits(uint8_t i, int32_t minl, int32_t maxl)
{
	min_lim[i] = minl; max_lim[i] = maxl;
}

void motion_set_profile(uint8_t i, float v0, float vmax, float a)
{
	motion_t *m = &motion[i];
	if (v0 < 1.0f) v0 = 1.0f;
	if (vmax < v0) vmax = v0;
	if (a < 1.0f) a = 1.0f;
	m->v0 = v0; m->vmax = vmax; m->a = a;
	m->cost_permille = cost_of(vmax);
}

void motion_set_profile_us(uint8_t i, uint16_t nominal_us, uint16_t start_us, uint32_t accel)
{
	if (nominal_us == 0) nominal_us = 1;
	if (start_us == 0) start_us = 1;
	motion_set_profile(i, 1e6f / (float)start_us, 1e6f / (float)nominal_us, (float)accel);
}

uint32_t motion_legacy_accel(uint16_t nominal_us, uint16_t start_us, uint16_t dp_us)
{
	/* Old ramp: interval decreases by dp_us per pulse from start_us to nominal_us.
	 * n pulses take T = sum of intervals = n * (start + nominal) / 2 (arithmetic series).
	 * Equivalent constant acceleration: (vmax - v0) / T. */
	if (dp_us == 0 || start_us <= nominal_us || nominal_us == 0) return 1;
	float n = (float)(start_us - nominal_us) / (float)dp_us;
	if (n < 1.0f) n = 1.0f;
	float T = n * ((float)start_us + (float)nominal_us) * 0.5f * 1e-6f;
	float a = (1e6f / (float)nominal_us - 1e6f / (float)start_us) / T;
	if (a < 1.0f) a = 1.0f;
	if (a > 4e9f) a = 4e9f;
	return (uint32_t)a;
}

/* ------------------------------------------------------------------------ */
/* Admission control                                                         */
/* ------------------------------------------------------------------------ */
static uint16_t running_cost_except(uint8_t i)
{
	uint16_t sum = 0;
	for (uint8_t k = 0; k < MOTION_MOTORS; k++) {
		if (k == i || !motion[k].running) continue;
		sum += motion[k].jog ? cost_of(motion[k].jog_v) : motion[k].cost_permille;
	}
	return sum;
}

bool motion_admit(uint8_t i)
{
	if (motion[i].jog || motion[i].jog_pending) return false;   /* positioning moves wait for the jog to end */
	return (uint16_t)(running_cost_except(i) + motion[i].cost_permille) <= MOTION_BUDGET_PERMILLE;
}

/* ------------------------------------------------------------------------ */
/* Start / stop                                                              */
/* ------------------------------------------------------------------------ */
static void launch(uint8_t i, uint32_t pulses, bool positive, bool jog)
{
	motion_t *m = &motion[i];
	int32_t p = pos_reg[i];
	int32_t room = room_to_limit(i, p, positive);
	if ((int32_t)pulses > room) pulses = (uint32_t)room;
	if (pulses == 0) {
		stopped_mask |= (uint8_t)(1u << i);   /* nothing to do: report "stopped" */
		return;
	}
	m->dir_positive = positive;
	m->start_pos = p;
	m->total = pulses;
	m->jog = jog;
	m->v = jog ? m->jog_v : m->v0;
	m->decel = 0;
	m->stopped = 0;
	m->stop_req = 0;
	m->period_ticks = period_from_speed(m->v);
	MOTION_ATOMIC() { m->remaining = pulses; }
	m->running = 1;
	motion_hal_start(i, m->period_ticks, positive);
	motion_hal_led(i, true);
}

void motion_request_stop(uint8_t i)
{
	motion_t *m = &motion[i];
	if (motion_hal_timer_running(i)) {
		m->stop_req = 1;   /* the step ISR completes the stop at the next pulse */
	}
	m->running = 0;
}

bool motion_jog(uint8_t i, float v, bool positive)
{
	motion_t *m = &motion[i];
	if (v < 1.0f) v = 1.0f;
	if ((uint16_t)(running_cost_except(i) + cost_of(v)) > MOTION_BUDGET_PERMILLE) return false;
	m->jog_v = v;
	m->jog_dir_positive = positive;
	m->jog_pending = 1;
	return true;
}

void motion_jog_stop(uint8_t i)
{
	motion[i].jog_pending = 0;
	if (motion[i].jog) motion_request_stop(i);
}

bool motion_is_jogging(uint8_t i) { return motion[i].jog != 0 || motion[i].jog_pending != 0; }

bool motion_is_running(uint8_t i) { return motion[i].running != 0; }

bool motion_is_idle(uint8_t i)
{
	return !motion[i].running && !motion_hal_timer_running(i) && !motion[i].stopped;
}

void motion_set_position(uint8_t i, int32_t p)
{
	motion_t *m = &motion[i];
	MOTION_ATOMIC() {
		if (m->running) {
			int32_t done = (int32_t)(m->total - m->remaining);
			m->start_pos = m->dir_positive ? p - done : p + done;
		}
		pos_reg[i] = p;
	}
}

int32_t motion_request(uint8_t i, int32_t steps)
{
	motion_t *m = &motion[i];
	if (steps == 0) return 0;
	if (m->jog || m->jog_pending) return steps;   /* wait for the jog to end */

	if (!m->running) {
		if (!motion_hal_timer_running(i)) {
			launch(i, steps > 0 ? (uint32_t)steps : (uint32_t)(-steps), steps > 0, false);
			return 0;
		}
		return steps;   /* a stop is still completing; try again next tick */
	}

	bool same_dir = (steps > 0) == (m->dir_positive != 0);
	uint32_t n = steps > 0 ? (uint32_t)steps : (uint32_t)(-steps);
	uint32_t r = read_remaining(i);

	if (same_dir) {
		int32_t room = room_to_limit(i, position_now(i, r), m->dir_positive);
		uint32_t add = n;
		if ((int32_t)r + (int32_t)add > room) add = (uint32_t)(room - (int32_t)r);
		if (add == 0) return 0;
		MOTION_ATOMIC() {
			m->remaining += add;
		}
		m->total += add;
		m->decel = 0;      /* re-evaluate deceleration against the new distance */
		return 0;
	}

	/* Opposite direction: shorten the current move, never below the stopping distance. */
	uint32_t d_stop = (uint32_t)(stop_distance(m) + 1.0f);
	if (r <= d_stop) return steps;                 /* already stopping: re-issue later */
	uint32_t can_cut = r - d_stop;
	uint32_t cut = n < can_cut ? n : can_cut;
	MOTION_ATOMIC() {
		m->remaining -= cut;
	}
	m->total -= cut;
	if (cut == n) return 0;
	return steps > 0 ? steps - (int32_t)cut : steps + (int32_t)cut;
}

/* ------------------------------------------------------------------------ */
/* Planner                                                                   */
/* ------------------------------------------------------------------------ */
static void finish(uint8_t i)
{
	motion_t *m = &motion[i];
	uint32_t r;
	MOTION_ATOMIC() { r = m->remaining; m->stopped = 0; m->stop_req = 0; }
	write_pos(i, position_now(i, r));
	m->running = 0;
	m->jog = 0;
	motion_hal_led(i, false);
	stopped_mask |= (uint8_t)(1u << i);
}

void motion_tick_1ms(void)
{
	for (uint8_t i = 0; i < MOTION_MOTORS; i++) {
		motion_t *m = &motion[i];

		if (m->stopped) { finish(i); continue; }

		if (m->jog_pending) {
			if (!m->running) {
				if (!motion_hal_timer_running(i)) {
					m->jog_pending = 0;
					launch(i, (uint32_t)INT32_MAX, m->jog_dir_positive != 0, true);
				}
			} else if (m->jog && (m->dir_positive != 0) == (m->jog_dir_positive != 0)) {
				m->jog_pending = 0;            /* same direction: just change speed */
			} else {
				motion_request_stop(i);        /* reverse or interrupt a positioning move */
				continue;
			}
		}
		if (!m->running) continue;

		uint32_t r = read_remaining(i);
		int32_t p = position_now(i, r);
		write_pos(i, p);

		/* Travel limits can change while moving: keep the target inside. */
		int32_t room = room_to_limit(i, p, m->dir_positive);
		if ((int32_t)r > room) {
			uint32_t cut = r - (uint32_t)room;
			MOTION_ATOMIC() { m->remaining -= cut; }
			m->total -= cut;
			r = (uint32_t)room;
			if (r == 0) { motion_request_stop(i); continue; }
		}

		if (m->jog) {
			m->v = m->jog_v;
		} else if (!m->decel) {
			if ((float)r <= stop_distance(m)) {
				m->decel = 1;
			} else {
				m->v += m->a * MOTION_DT_S;
				if (m->v > m->vmax) m->v = m->vmax;
			}
		}
		if (!m->jog && m->decel) {
			/* Speed v to hold for this tick such that, after travelling v*dt,
			 * the distance left equals the stopping distance from v:
			 *   r - v*dt = (v^2 - v0^2) / (2a)
			 * Solving the quadratic gives a smooth ramp that lands on v0. */
			float adt = m->a * MOTION_DT_S;
			float v = sqrtf(adt * adt + m->v0 * m->v0 + 2.0f * m->a * (float)r) - adt;
			m->v = v < m->v0 ? m->v0 : v;
		}

		uint16_t per = period_from_speed(m->v);
		if (per != m->period_ticks) {
			m->period_ticks = per;
			motion_hal_set_period(i, per);
		}
	}
}

uint8_t motion_take_stopped_mask(void)
{
	uint8_t mask;
	MOTION_ATOMIC() { mask = stopped_mask; stopped_mask = 0; }
	return mask;
}
