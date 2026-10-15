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
	printf("%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
