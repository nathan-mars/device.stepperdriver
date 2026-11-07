/* Host-side tests of the motion engine against a simulated pulse timer. */
#include "sim_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static int failures = 0, checks = 0;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { failures++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static int32_t pos[MOTION_MOTORS];

static void reset_all(void)
{
	memset(pos, 0, sizeof pos);
	sim_reset();
	motion_init(pos);
	homing_init();
	motion_take_stopped_mask();
}

/* Run until motor i is idle; returns elapsed ms (or -1 on timeout). */
static int run_until_idle(uint8_t i, int max_ms)
{
	for (int ms = 0; ms < max_ms; ms++) {
		sim_run_ms(1);
		if (motion_is_idle(i)) return ms + 1;
	}
	return -1;
}

static uint16_t period_of(float v) { float p = (float)MOTION_F_TICK_HZ / v; return (uint16_t)(p + 0.5f); }

/* --- per-pulse recorder for profile checks --- */
#define MAXREC 200000
static double rec_t[MAXREC]; static uint16_t rec_per[MAXREC]; static int rec_n;
static void recorder(uint8_t i, double t, uint16_t per) { if (i == 0 && rec_n < MAXREC) { rec_t[rec_n] = t; rec_per[rec_n] = per; rec_n++; } }

static void test_exact_counts(void)
{
	printf("exact pulse counts\n");
	const int32_t cases[] = {1, -1, 2, 10, -10, 1000, 100000, -4321};
	for (size_t c = 0; c < sizeof cases / sizeof cases[0]; c++) {
		reset_all();
		motion_set_profile(0, 800.0f, 80000.0f, 2.0e6f);     /* 1 -> 100 mm/s @ 1.25 um, 2.5 m/s^2 */
		int32_t left = motion_request(0, cases[c]);
		CHECK(left == 0, "request %d returned leftover %d", cases[c], left);
		int ms = run_until_idle(0, 20000);
		CHECK(ms > 0, "move %d did not finish", cases[c]);
		CHECK(sim[0].hw_position == cases[c], "move %d: hardware emitted %d pulses", cases[c], sim[0].hw_position);
		CHECK(pos[0] == cases[c], "move %d: position register %d", cases[c], pos[0]);
		CHECK(motion_take_stopped_mask() == 1, "move %d: stopped mask", cases[c]);
		CHECK(sim[0].min_per >= MOTION_MIN_PERIOD_TICKS, "period below floor");
	}
}

static void test_trapezoid_shape(void)
{
	printf("trapezoid and triangle profiles\n");
	reset_all();
	float v0 = 800.0f, vmax = 80000.0f, a = 2.0e6f;
	motion_set_profile(0, v0, vmax, a);
	rec_n = 0; sim_on_pulse = recorder;
	motion_request(0, 12000);                                  /* 15 mm */
	int ms = run_until_idle(0, 5000);
	sim_on_pulse = 0;
	CHECK(ms > 0, "long move did not finish");
	CHECK(sim[0].min_per == period_of(vmax), "did not reach vmax: min period %u vs %u", sim[0].min_per, period_of(vmax));
	/* first pulse at v0, last pulses no faster than v0 + one planner quantum */
	CHECK(rec_per[0] == period_of(v0), "first period %u (v0 period %u)", rec_per[0], period_of(v0));
	CHECK(rec_per[rec_n - 1] >= period_of(v0 + a * MOTION_DT_S), "terminal period %u too short (limit %u)", rec_per[rec_n - 1], period_of(v0 + a * MOTION_DT_S));
	/* monotonic: periods never increase before the minimum, never decrease after it */
	int imin = 0; for (int k = 0; k < rec_n; k++) if (rec_per[k] < rec_per[imin]) imin = k;
	int bad = 0;
	for (int k = 1; k < imin; k++) if (rec_per[k] > rec_per[k-1]) bad++;
	for (int k = imin + 1; k < rec_n; k++) if (rec_per[k] < rec_per[k-1]) bad++;
	CHECK(bad == 0, "%d non-monotonic period changes", bad);
	/* ideal trapezoid time: t = (vmax-v0)/a * 2 + (d - 2*d_ramp)/vmax */
	double d_ramp = (vmax*vmax - v0*v0) / (2*a);
	double t_ideal = 2*(vmax - v0)/a + (12000 - 2*d_ramp)/vmax;
	double t_sim = rec_t[rec_n-1] - rec_t[0];
	CHECK(fabs(t_sim - t_ideal) < 0.012, "move time %.1f ms vs ideal %.1f ms", t_sim*1e3, t_ideal*1e3);
	/* write a profile CSV for plotting */
	FILE *f = fopen("build/profile.csv", "w");
	if (f) { fprintf(f, "t_s,period_ticks,speed_pulses_s\n"); for (int k = 0; k < rec_n; k++) fprintf(f, "%.7f,%u,%.1f\n", rec_t[k], rec_per[k], (double)MOTION_F_TICK_HZ / rec_per[k]); fclose(f); }

	/* short move: triangle, must not reach vmax but must still land exactly */
	reset_all();
	motion_set_profile(0, v0, vmax, a);
	motion_request(0, 800);                                    /* 1 mm */
	ms = run_until_idle(0, 5000);
	CHECK(ms > 0 && sim[0].hw_position == 800 && pos[0] == 800, "short move landed at %d", sim[0].hw_position);
	CHECK(sim[0].min_per > period_of(vmax), "short move should not reach vmax (min per %u)", sim[0].min_per);
}

static void test_limits(void)
{
	printf("travel limits\n");
	reset_all();
	motion_set_profile(0, 800.0f, 80000.0f, 2.0e6f);
	motion_set_limits(0, 0, 5000);
	motion_request(0, 20000);
	int ms = run_until_idle(0, 5000);
	CHECK(ms > 0 && pos[0] == 5000 && sim[0].hw_position == 5000, "max limit: ended at %d", pos[0]);
	/* min limit going negative from 5000 */
	motion_set_limits(0, -100, 5000);
	motion_request(0, -20000);
	ms = run_until_idle(0, 5000);
	CHECK(ms > 0 && pos[0] == -100 && sim[0].hw_position == -100, "min limit: ended at %d", pos[0]);
	/* limit tightened mid-move: must stop at or before the new limit */
	reset_all();
	motion_set_profile(0, 800.0f, 20000.0f, 2.0e5f);
	motion_request(0, 20000);
	sim_run_ms(50);
	int32_t p_mid = pos[0];
	motion_set_limits(0, 0, p_mid + 2000);
	ms = run_until_idle(0, 5000);
	CHECK(ms > 0 && pos[0] <= p_mid + 2000 && sim[0].hw_position == pos[0], "tightened limit: ended at %d (limit %d)", pos[0], p_mid + 2000);
	/* already at the limit: request is a no-op but still reports stopped */
	reset_all(); motion_set_limits(0, 0, 10); pos[0] = 10;
	CHECK(motion_request(0, 5) == 0, "at-limit request leftover");
	sim_run_ms(2);
	CHECK(motion_take_stopped_mask() == 1 && sim[0].pulses == 0, "at-limit request emitted %u pulses", sim[0].pulses);
}

static void test_admission(void)
{
	printf("admission control\n");
	reset_all();
	motion_set_profile_us(0, 10, 2000, 2000000);     /* 100k pulses/s: cost 250 */
	motion_set_profile_us(1, 20, 2000, 2000000);     /* 50k: cost 125 */
	motion_set_profile_us(2, 100, 2000, 50000);      /* 10k: cost 25 */
	motion_set_profile_us(3, 50, 2000, 50000);       /* 20k: cost 50 -> would exceed 300 */
	CHECK(motion[0].cost_permille == 250, "cost0 %u", motion[0].cost_permille);
	CHECK(motion_admit(0), "fast axis alone must be admitted");
	motion_request(0, 50000);
	CHECK(!motion_admit(1), "second fast axis must be refused while axis 0 runs");
	CHECK(motion_admit(2), "slow axis must be admitted next to the fast one");
	motion_request(2, 1000);
	CHECK(!motion_admit(3), "third axis exceeding the budget must be refused");
	run_until_idle(0, 5000);
	CHECK(motion_admit(1), "fast axis admitted once the other has stopped");
	/* speeds that together exceed the budget are refused, slower ones accepted */
	reset_all();
	for (int i = 0; i < 4; i++) motion_set_profile_us(i, 34, 2000, 1000000);   /* 29.4k: cost 74 each -> 296 total OK */
	for (int i = 0; i < 4; i++) { CHECK(motion_admit(i), "4 x 34 us must fit"); motion_request(i, 1000); }
	reset_all();
	for (int i = 0; i < 4; i++) motion_set_profile_us(i, 30, 2000, 1000000);   /* 33k: cost 83 -> 4th refused */
	for (int i = 0; i < 3; i++) { CHECK(motion_admit(i), "3 x 30 us must fit"); motion_request(i, 1000); }
	CHECK(!motion_admit(3), "4 x 30 us must be refused");
}

static void test_retarget_and_stop(void)
{
	printf("retargeting and stop requests\n");
	reset_all();
	motion_set_profile(0, 800.0f, 40000.0f, 1.0e6f);
	motion_request(0, 3000);
	sim_run_ms(20);
	CHECK(motion_request(0, 2000) == 0, "extension leftover");
	int ms = run_until_idle(0, 5000);
	CHECK(ms > 0 && pos[0] == 5000 && sim[0].hw_position == 5000, "extended move ended at %d", pos[0]);

	/* opposite request shortens the move (net position semantics) */
	reset_all();
	motion_set_profile(0, 800.0f, 40000.0f, 1.0e6f);
	motion_request(0, 10000);
	sim_run_ms(20);
	int32_t left = motion_request(0, -3000);
	CHECK(left == 0, "shortening leftover %d", left);
	ms = run_until_idle(0, 5000);
	CHECK(ms > 0 && pos[0] == 7000 && sim[0].hw_position == 7000, "shortened move ended at %d", pos[0]);

	/* reverse beyond the stopping distance: leftover returned, re-issued after stop */
	reset_all();
	motion_set_profile(0, 800.0f, 40000.0f, 1.0e6f);
	motion_request(0, 10000);
	sim_run_ms(40);
	int32_t pending = motion_request(0, -20000);
	CHECK(pending < 0 && pending > -20000, "reverse leftover %d", pending);
	int32_t stop_pos = 0;
	for (int k = 0; k < 5000 && pending != 0; k++) {
		sim_run_ms(1);
		if (motion_is_idle(0)) { stop_pos = pos[0]; pending = motion_request(0, pending); }
	}
	ms = run_until_idle(0, 5000);
	CHECK(ms > 0 && pos[0] == -10000 && sim[0].hw_position == -10000, "reverse ended at %d (stopped at %d)", pos[0], stop_pos);

	/* stop request: stops within one pulse period, position consistent */
	reset_all();
	motion_set_profile(0, 800.0f, 80000.0f, 2.0e6f);
	motion_request(0, 100000);
	sim_run_ms(30);
	uint32_t before = sim[0].pulses;
	motion_request_stop(0);
	CHECK(!motion_is_running(0), "running flag cleared immediately");
	sim_advance(0.0001);                                          /* 100 us: well past one 12.5 us period */
	CHECK(!sim[0].running, "timer still running 100 us after stop request");
	CHECK(sim[0].pulses - before <= 1, "%u pulses after stop request", sim[0].pulses - before);
	motion_tick_1ms();
	CHECK(pos[0] == sim[0].hw_position, "position %d vs hardware %d after stop", pos[0], sim[0].hw_position);
	CHECK(motion_take_stopped_mask() == 1, "stopped mask after stop request");
	motion_request_stop(0);                                       /* stop when idle: harmless */
	sim_run_ms(2);
	CHECK(motion_take_stopped_mask() == 0, "spurious stopped event");
}

static void test_legacy_accel(void)
{
	printf("legacy acceleration conversion\n");
	uint32_t a = motion_legacy_accel(250, 2000, 10);
	/* old default ramp: 175 pulses, T = 175*(2000+250)/2 us = 196.9 ms, dv = 3500 pulses/s */
	CHECK(a > 17000 && a < 18500, "legacy accel %u", a);
	reset_all();
	motion_set_profile_us(0, 250, 2000, a);
	rec_n = 0; sim_on_pulse = recorder;
	motion_request(0, 4000);
	run_until_idle(0, 5000);
	sim_on_pulse = 0;
	/* time to reach vmax should be close to the old ramp's 197 ms */
	int k = 0; while (k < rec_n && rec_per[k] > period_of(4000.0f)) k++;
	double t_ramp = rec_t[k] - rec_t[0];
	CHECK(fabs(t_ramp - 0.197) < 0.02, "ramp to vmax took %.1f ms", t_ramp * 1e3);
	CHECK(motion_legacy_accel(250, 250, 10) == 1, "degenerate ramp");
	CHECK(motion_legacy_accel(250, 2000, 0) == 1, "zero step");
}

static void test_set_position(void)
{
	printf("set position while moving\n");
	reset_all();
	motion_set_profile(0, 800.0f, 20000.0f, 2.0e5f);
	motion_request(0, 5000);
	sim_run_ms(30);
	motion_set_position(0, 100000);
	int32_t hw_at_set = sim[0].hw_position;
	run_until_idle(0, 5000);
	CHECK(pos[0] == 100000 + (5000 - hw_at_set), "position after rebase %d", pos[0]);
}

static void test_jog(void)
{
	printf("jog (ImmediateSteps) mode\n");
	reset_all();
	motion_set_profile(0, 800.0f, 80000.0f, 2.0e6f);
	CHECK(motion_jog(0, 10000.0f, true), "jog admitted");
	sim_run_ms(51);                                           /* launch on the first tick, then 50 ms */
	CHECK(sim[0].running, "jog running");
	CHECK(sim[0].hw_position >= 490 && sim[0].hw_position <= 510, "jog travelled %d pulses in 50 ms", sim[0].hw_position);
	CHECK(sim[0].min_per == period_of(10000.0f) && sim[0].max_per == period_of(10000.0f), "jog speed not constant (%u..%u)", sim[0].min_per, sim[0].max_per);
	CHECK(!motion_admit(0), "positioning moves refused while jogging");
	CHECK(motion_request(0, 100) == 100, "relative request deferred while jogging");
	/* speed change in the same direction: no stop */
	uint32_t pulses_before = sim[0].pulses;
	CHECK(motion_jog(0, 20000.0f, true), "jog speed change admitted");
	sim_run_ms(20);
	CHECK(sim[0].running && sim[0].pulses > pulses_before, "jog continued through speed change");
	CHECK(sim[0].per == period_of(20000.0f), "jog period after speed change %u", sim[0].per);
	CHECK(pos[0] == sim[0].hw_position, "jog position %d vs hw %d", pos[0], sim[0].hw_position);
	/* reverse: stop first, then run the other way */
	int32_t p_rev = pos[0];
	CHECK(motion_jog(0, 10000.0f, false), "reverse jog admitted");
	sim_run_ms(30);
	CHECK(sim[0].running && !sim[0].dir_positive, "reverse jog running");
	CHECK(pos[0] < p_rev && pos[0] == sim[0].hw_position, "reverse jog position %d", pos[0]);
	/* stop */
	motion_jog_stop(0);
	sim_run_ms(2);
	CHECK(!sim[0].running && motion_is_idle(0), "jog stopped");
	CHECK(pos[0] == sim[0].hw_position, "position after jog stop %d vs %d", pos[0], sim[0].hw_position);
	CHECK(motion_take_stopped_mask() & 1, "stopped mask after jog");
	/* jog into a travel limit stops at the limit */
	reset_all();
	motion_set_limits(0, 0, 300);
	motion_jog(0, 10000.0f, true);
	sim_run_ms(100);
	CHECK(!sim[0].running && pos[0] == 300 && sim[0].hw_position == 300, "jog hit limit at %d", pos[0]);
	/* admission: a jog that exceeds the budget next to a fast motor is refused */
	reset_all();
	motion_set_profile(1, 800.0f, 100000.0f, 2.0e6f);     /* cost 250 */
	motion_request(1, 100000);
	CHECK(!motion_jog(0, 40000.0f, true), "jog of cost 100 refused next to cost 250");
	CHECK(motion_jog(0, 10000.0f, true), "jog of cost 25 admitted next to cost 250");
}

/* ------------------------------------------------------------------------ */
/* Homing                                                                    */
/* ------------------------------------------------------------------------ */
static homing_params_t home_params(uint8_t input)
{
	/* 100 us fast (12.5 mm/s at 1.25 um), 500 us slow, 30 mm search, 1 mm backoff */
	homing_params_t p = { .config = input, .fast_us = 100, .slow_us = 500, .search = 26000, .backoff = 800 };
	return p;
}

/* Put motor i at hardware position hw (and position register hw). */
static void place(uint8_t i, int32_t hw) { sim[i].hw_position = hw; pos[i] = hw; }

static int run_until_homing_done(uint8_t mask, int max_ms)
{
	for (int ms = 0; ms < max_ms; ms++) {
		sim_run_ms(1);
		if (!(homing_active_mask() & mask)) return ms + 1;
	}
	return -1;
}

static void test_homing_basic(void)
{
	printf("homing: basic, repeatability, profile restored\n");
	reset_all();
	sim_set_profile(0, 500.0f, 4000.0f, 1.0e5f);
	place(0, 5000);
	sim_set_endstop(0, 0, 0, true, false);         /* active-high sensor, active at hw <= 0 */
	homing_params_t p = home_params(0);
	homing_take_changed();
	rec_n = 0;
	sim_on_pulse = recorder;
	homing_start(0, &p);
	CHECK(homing_take_changed() && homing_state() == 0x10, "state after start 0x%02x", homing_state());
	int ms = run_until_homing_done(1, 20000);
	sim_on_pulse = 0;
	CHECK(ms > 0, "homing did not finish");
	CHECK(homing_homed_mask() == 1 && homing_state() == 0x01, "homed state 0x%02x", homing_state());
	CHECK(homing_take_changed(), "change flag after homing");
	sim_run_ms(5);
	CHECK(pos[0] == 0, "position after homing %d", pos[0]);
	CHECK(sim[0].hw_position == -1, "stopped at hw %d (edge 0, one pulse past)", sim[0].hw_position);
	CHECK(motion_is_idle(0) && !sim[0].running, "motor idle after homing");
	CHECK(motion[0].vmax == 4000.0f && motion[0].v0 == 500.0f, "profile not restored (v0 %.0f vmax %.0f)", motion[0].v0, motion[0].vmax);
	/* 5001 search (to hw -1) + 800 backoff + 800 slow pulses; the last leg ran at 500 us, the search at 100 us */
	CHECK(rec_n == 5001 + 800 + 800, "homing emitted %d pulses", rec_n);
	uint16_t slow_min = 0xFFFF, fast_min = 0xFFFF;
	for (int k = rec_n - 800; k < rec_n; k++) if (rec_per[k] < slow_min) slow_min = rec_per[k];
	for (int k = 0; k < 5001; k++) if (rec_per[k] < fast_min) fast_min = rec_per[k];
	CHECK(slow_min == 2000, "slow leg min period %u ticks (500 us = 2000)", slow_min);
	CHECK(fast_min == 400, "search min period %u ticks (100 us = 400)", fast_min);
	/* home again from another place: same hardware point */
	motion_request(0, 3000);
	run_until_idle(0, 5000);
	homing_start(0, &p);
	CHECK(homing_homed_mask() == 0, "homed bit cleared at start");
	run_until_homing_done(1, 20000);
	sim_run_ms(5);
	CHECK(homing_homed_mask() == 1 && pos[0] == 0 && sim[0].hw_position == -1, "second home: pos %d hw %d", pos[0], sim[0].hw_position);
	CHECK((motion_take_stopped_mask() & 1) != 0, "motion reported stops");
}

static void test_homing_starts_on_sensor(void)
{
	printf("homing: starting inside the sensor\n");
	reset_all();
	place(0, -300);
	sim_set_endstop(0, 0, 0, true, false);
	sim_endstop[0].hysteresis = 20;
	CHECK(sim_endstop[0].active, "sensor active at start");
	homing_params_t p = home_params(0);
	homing_start(0, &p);
	run_until_homing_done(1, 20000);
	sim_run_ms(5);
	CHECK(homing_homed_mask() == 1 && pos[0] == 0 && sim[0].hw_position == -1, "pos %d hw %d", pos[0], sim[0].hw_position);
}

static void test_homing_failures(void)
{
	printf("homing: failures\n");
	/* sensor beyond the search distance */
	reset_all();
	place(0, 5000);
	sim_set_endstop(0, 0, -30000, true, false);
	homing_params_t p = home_params(0);
	homing_start(0, &p);
	run_until_homing_done(1, 30000);
	sim_run_ms(5);
	CHECK(homing_state() == 0, "not found: state 0x%02x", homing_state());
	CHECK(sim[0].pulses == 26000, "not found: travelled %u pulses", sim[0].pulses);
	/* wrong polarity: configured active-low, sensor is active-high and not triggered */
	reset_all();
	place(0, 5000);
	sim_set_endstop(0, 0, 0, true, false);
	p = home_params(0);
	p.config |= HOMING_CFG_ACTIVE_LOW;
	homing_start(0, &p);
	run_until_homing_done(1, 30000);
	sim_run_ms(5);
	CHECK(homing_state() == 0, "wrong polarity: state 0x%02x", homing_state());
	CHECK(sim[0].pulses == HOMING_CLEAR_BACKOFFS * 800 && sim[0].hw_position > 5000, "wrong polarity: travelled %u pulses to %d", sim[0].pulses, sim[0].hw_position);
	/* backoff shorter than the sensor's hysteresis */
	reset_all();
	place(0, 5000);
	sim_set_endstop(0, 0, 0, true, false);
	sim_endstop[0].hysteresis = 100;
	p = home_params(0);
	p.backoff = 50;
	homing_start(0, &p);
	run_until_homing_done(1, 30000);
	CHECK(homing_state() == 0, "short backoff: state 0x%02x", homing_state());
	/* cancel in the middle of the search */
	reset_all();
	sim_set_profile(0, 500.0f, 4000.0f, 1.0e5f);
	place(0, 5000);
	sim_set_endstop(0, 0, 0, true, false);
	p = home_params(0);
	homing_start(0, &p);
	sim_run_ms(100);
	CHECK(sim[0].running, "searching");
	homing_cancel(1);
	sim_run_ms(3);
	CHECK(homing_state() == 0 && !sim[0].running && motion_is_idle(0), "cancel: state 0x%02x", homing_state());
	CHECK(pos[0] == sim[0].hw_position, "cancel: position %d hw %d", pos[0], sim[0].hw_position);
	CHECK(motion[0].vmax == 4000.0f, "cancel: profile not restored");
	homing_cancel(1);                                  /* no effect when idle */
	CHECK(homing_state() == 0, "cancel when idle");
	/* lost: driver disabled after homing */
	homing_start(0, &p);
	run_until_homing_done(1, 20000);
	CHECK(homing_homed_mask() == 1, "homed before lost");
	homing_take_changed();
	homing_lost(1);
	CHECK(homing_homed_mask() == 0 && homing_take_changed(), "lost clears homed");
}

static void test_homing_limits_and_direction(void)
{
	printf("homing: travel limits ignored, positive direction\n");
	reset_all();
	place(0, 5000);
	motion_set_limits(0, 1000, 20000);                 /* stale limits would stop the search */
	sim_set_endstop(0, 0, 0, true, false);
	homing_params_t p = home_params(0);
	homing_start(0, &p);
	run_until_homing_done(1, 20000);
	sim_run_ms(5);
	CHECK(homing_homed_mask() == 1 && sim[0].hw_position == -1, "homed through a stale limit: hw %d", sim[0].hw_position);
	/* limits are active again afterwards: at 0 with MinPosition 1000, a negative move cannot start */
	motion_take_stopped_mask();
	motion_request(0, -10);
	sim_run_ms(5);
	CHECK(sim[0].hw_position == -1, "limits restored after homing");
	/* endstop at the positive end, active-low, on input 3 driving motor 1 */
	reset_all();
	place(1, -2000);
	sim_set_endstop(3, 1, 7000, false, true);
	p = home_params(3);
	p.config |= HOMING_CFG_POSITIVE | HOMING_CFG_ACTIVE_LOW;
	homing_start(1, &p);
	CHECK(homing_input_mask() == 0x08, "input mask 0x%02x", homing_input_mask());
	run_until_homing_done(2, 20000);
	sim_run_ms(5);
	CHECK(homing_homed_mask() == 2 && pos[1] == 0 && sim[1].hw_position == 7001, "positive: pos %d hw %d", pos[1], sim[1].hw_position);
}

static void test_homing_concurrent(void)
{
	printf("homing: several axes, late start, busy motor\n");
	reset_all();
	for (uint8_t i = 0; i < 4; i++) { place(i, 4000 + 1000 * i); sim_set_endstop(i, i, 0, true, false); }
	homing_params_t p[4];
	for (uint8_t i = 0; i < 4; i++) p[i] = home_params(i);
	CHECK(homing_admit(0x05, p), "two axes at 100 us admitted");
	homing_start(0, &p[0]);
	homing_start(2, &p[2]);
	sim_run_ms(50);
	CHECK(homing_state() == 0x50, "0 and 2 homing: 0x%02x", homing_state());
	/* motor 1 requested while 0 and 2 are searching */
	CHECK(homing_admit(0x02, p), "third axis admitted while two are homing");
	homing_start(1, &p[1]);
	CHECK(homing_state() == 0x70, "0, 1, 2 homing: 0x%02x", homing_state());
	/* a repeated request for an axis already homing is ignored (no restart) */
	int32_t hw0 = sim[0].hw_position;
	homing_start(0, &p[0]);
	sim_run_ms(10);
	CHECK(sim[0].hw_position < hw0, "axis 0 kept searching");
	/* motor 3 is busy with a move: homing stops it first */
	motion_request(3, 20000);
	sim_run_ms(20);
	CHECK(sim[3].running && sim[3].dir_positive, "motor 3 moving away");
	homing_start(3, &p[3]);
	int ms = run_until_homing_done(0x0F, 40000);
	sim_run_ms(5);
	CHECK(ms > 0 && homing_state() == 0x0F, "all homed: 0x%02x", homing_state());
	for (uint8_t i = 0; i < 4; i++)
		CHECK(pos[i] == 0 && sim[i].hw_position == -1, "axis %u: pos %d hw %d", i, pos[i], sim[i].hw_position);
	/* admission: two axes at 10 us (cost 250 each) do not fit */
	for (uint8_t i = 0; i < 4; i++) p[i].fast_us = 10;
	CHECK(!homing_admit(0x03, p), "two fast axes refused");
	CHECK(homing_admit(0x01, p), "one fast axis admitted");
	/* ...and an axis between two legs keeps its share */
	p[0].fast_us = 10;
	homing_start(0, &p[0]);
	CHECK(!homing_admit(0x02, p), "budget reserved for an axis between legs");
}

int main(void)
{
	test_jog();
	test_exact_counts();
	test_trapezoid_shape();
	test_limits();
	test_admission();
	test_retarget_and_stop();
	test_legacy_accel();
	test_set_position();
	test_homing_basic();
	test_homing_starts_on_sensor();
	test_homing_failures();
	test_homing_limits_and_direction();
	test_homing_concurrent();
	printf("%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
