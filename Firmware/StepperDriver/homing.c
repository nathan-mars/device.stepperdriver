/*
 * homing.c - per-axis homing state machine (see homing.h).
 * No AVR headers here: compiled unchanged by the host-side tests.
 */
#include "homing.h"
#include <motion_port.h>

enum {
	PH_IDLE = 0,
	PH_START,
	PH_CLEAR,
	PH_FAST,
	PH_BACKOFF,
	PH_SLOW
};

typedef struct {
	homing_params_t p;
	volatile uint8_t phase;
	volatile uint8_t cancel_req;
	volatile uint8_t hit;          /* set by the input interrupt: the leg's edge was seen */
	uint8_t leg;                   /* a leg's move has been launched and not evaluated yet */
	float   v0;                    /* the motor's start speed and acceleration when homing began */
	float   a;
} homing_t;

static homing_t h[MOTION_MOTORS];
static volatile uint8_t homed_mask;
static volatile uint8_t changed;

/* ------------------------------------------------------------------------ */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------ */
static uint8_t input_of(uint8_t i) { return h[i].p.config & HOMING_CFG_INPUT_MASK; }
static bool    toward_positive(uint8_t i) { return (h[i].p.config & HOMING_CFG_POSITIVE) != 0; }

static bool sensor_active(uint8_t i)
{
	bool high = homing_hal_input_high(input_of(i));
	return (h[i].p.config & HOMING_CFG_ACTIVE_LOW) ? !high : high;
}

static float speed_of(uint16_t us) { return 1.0e6f / (float)(us ? us : 1); }

static void set_speed(uint8_t i, uint16_t us)
{
	float v = speed_of(us);
	float v0 = h[i].v0 < v ? h[i].v0 : v;   /* never start faster than the leg's speed */
	motion_set_profile(i, v0, v, h[i].a);
}

/* Launch one leg: `toward` the sensor or away from it. */
static void leg(uint8_t i, uint8_t phase, uint16_t us, uint32_t pulses, bool toward)
{
	homing_t *s = &h[i];
	bool positive = toward ? toward_positive(i) : !toward_positive(i);
	if (pulses > (uint32_t)INT32_MAX) pulses = (uint32_t)INT32_MAX;
	if (pulses == 0) pulses = 1;
	set_speed(i, us);
	s->hit = 0;
	s->phase = phase;
	s->leg = 1;
	motion_request(i, positive ? (int32_t)pulses : -(int32_t)pulses);
}

static void finish(uint8_t i, bool success)
{
	homing_t *s = &h[i];
	if (motion_is_running(i)) motion_request_stop(i);
	s->leg = 0;
	motion_ignore_limits(i, false);
	homing_hal_restore_profile(i);
	/* Register writes (higher interrupt level) may cancel or restart this axis
	 * at any time: decide and go idle in one step, idle last. */
	MOTION_ATOMIC() {
		if (success && !s->cancel_req) homed_mask |= (uint8_t)(1u << i);
		else                           homed_mask &= (uint8_t)~(1u << i);
		s->cancel_req = 0;
		changed = 1;
		s->phase = PH_IDLE;
	}
}

/* ------------------------------------------------------------------------ */
/* API                                                                       */
/* ------------------------------------------------------------------------ */
void homing_init(void)
{
	for (uint8_t i = 0; i < MOTION_MOTORS; i++) {
		h[i].phase = PH_IDLE; h[i].cancel_req = 0; h[i].hit = 0; h[i].leg = 0;
	}
	homed_mask = 0;
	changed = 0;
}

bool homing_admit(uint8_t mask, const homing_params_t *p)
{
	uint16_t sum = motion_running_cost_permille(mask);
	for (uint8_t i = 0; i < MOTION_MOTORS; i++) {
		uint8_t bit = (uint8_t)(1u << i);
		if (mask & bit) {
			sum += motion_cost_permille(speed_of(p[i].fast_us));
		} else if (h[i].phase != PH_IDLE && !motion_is_running(i)) {
			/* homing between two legs: reserve its fast speed */
			sum += motion_cost_permille(speed_of(h[i].p.fast_us));
		}
	}
	return sum <= MOTION_BUDGET_PERMILLE;
}

void homing_start(uint8_t i, const homing_params_t *p)
{
	homing_t *s = &h[i];
	if (s->phase != PH_IDLE) return;
	s->p = *p;
	s->cancel_req = 0;
	s->hit = 0;
	s->leg = 0;
	homed_mask &= (uint8_t)~(1u << i);
	changed = 1;
	s->phase = PH_START;           /* last: the tick only looks at axes with a phase */
}

void homing_cancel(uint8_t mask)
{
	for (uint8_t i = 0; i < MOTION_MOTORS; i++) {
		if (!(mask & (1u << i)) || h[i].phase == PH_IDLE) continue;
		h[i].cancel_req = 1;
		motion_request_stop(i);
	}
}

void homing_lost(uint8_t mask)
{
	MOTION_ATOMIC() {
		homing_cancel(mask);
		if (homed_mask & mask) {
			homed_mask &= (uint8_t)~mask;
			changed = 1;
		}
	}
}

void homing_input_edge(uint8_t input, bool line_high)
{
	for (uint8_t i = 0; i < MOTION_MOTORS; i++) {
		homing_t *s = &h[i];
		if (!s->leg || input_of(i) != input) continue;
		uint8_t ph = s->phase;
		if (ph != PH_CLEAR && ph != PH_FAST && ph != PH_SLOW) continue;
		bool active = (s->p.config & HOMING_CFG_ACTIVE_LOW) ? !line_high : line_high;
		bool wanted = (ph == PH_CLEAR) ? !active : active;   /* CLEAR waits for the release */
		if (wanted) {
			s->hit = 1;
			motion_request_stop(i);
		}
	}
}

void homing_tick_1ms(void)
{
	for (uint8_t i = 0; i < MOTION_MOTORS; i++) {
		homing_t *s = &h[i];
		uint8_t ph = s->phase;
		if (ph == PH_IDLE) continue;

		if (s->cancel_req) { finish(i, false); continue; }

		if (!motion_is_idle(i)) {
			if (ph == PH_START) {          /* stop the move or jog that was running */
				motion_jog_stop(i);
				if (motion_is_running(i)) motion_request_stop(i);
			}
			continue;                      /* a leg (or the stop) is still in progress */
		}

		/* The motor is idle: evaluate the leg that just ended and start the next one. */
		bool hit = s->hit != 0;
		s->leg = 0;
		const homing_params_t *p = &s->p;
		uint32_t backoff = p->backoff ? p->backoff : 1;

		switch (ph) {
		case PH_START:
			s->v0 = motion[i].v0;
			s->a = motion[i].a;
			motion_ignore_limits(i, true);
			if (sensor_active(i)) leg(i, PH_CLEAR, p->fast_us, HOMING_CLEAR_BACKOFFS * backoff, false);
			else                  leg(i, PH_FAST, p->fast_us, p->search, true);
			break;

		case PH_CLEAR:
			if (!hit || sensor_active(i)) { finish(i, false); break; }   /* never released */
			leg(i, PH_FAST, p->fast_us, p->search, true);
			break;

		case PH_FAST:
			if (!hit) { finish(i, false); break; }                       /* sensor not found */
			leg(i, PH_BACKOFF, p->fast_us, backoff, false);
			break;

		case PH_BACKOFF:
			if (sensor_active(i)) { finish(i, false); break; }           /* backoff too short */
			leg(i, PH_SLOW, p->slow_us, HOMING_SLOW_BACKOFFS * backoff, true);
			break;

		case PH_SLOW:
			if (!hit) { finish(i, false); break; }
			motion_set_position(i, 0);
			finish(i, true);
			break;

		default:
			finish(i, false);
			break;
		}
	}
}

bool homing_is_active(uint8_t i) { return i < MOTION_MOTORS && h[i].phase != PH_IDLE; }

uint8_t homing_active_mask(void)
{
	uint8_t m = 0;
	for (uint8_t i = 0; i < MOTION_MOTORS; i++)
		if (h[i].phase != PH_IDLE) m |= (uint8_t)(1u << i);
	return m;
}

uint8_t homing_homed_mask(void) { return homed_mask; }

uint8_t homing_input_mask(void)
{
	uint8_t m = 0;
	for (uint8_t i = 0; i < MOTION_MOTORS; i++)
		if (h[i].phase != PH_IDLE) m |= (uint8_t)(1u << input_of(i));
	return m;
}

uint8_t homing_state(void) { return (uint8_t)((homing_active_mask() << 4) | (homed_mask & 0x0F)); }

bool homing_take_changed(void)
{
	bool c;
	MOTION_ATOMIC() { c = changed; changed = 0; }
	return c;
}
