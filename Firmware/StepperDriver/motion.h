/*
 * motion.h - trapezoidal motion engine for the Harp StepperDriver.
 *
 * Platform independent: this file and motion.c contain no AVR headers, so the
 * host-side unit tests (../tests) compile the exact code that runs on the board.
 * Hardware access goes through the small HAL declared in motion_port.h.
 *
 * Architecture (see Assets/trapezoidal-motion-explainer.html):
 *   - The STEP pulse train is produced by a hardware timer in PWM mode.
 *   - One interrupt per pulse (MOTION_ISR_BODY) only decrements `remaining`
 *     and stops the timer when the move is complete or a stop was requested.
 *     It touches nothing else, so it costs ~2 us on a 32 MHz XMEGA.
 *   - motion_tick_1ms(), called from the Harp core's 1 ms callback, does all
 *     the arithmetic: accelerate, decide when to decelerate from the remaining
 *     distance, convert speed to a timer period, track position, enforce the
 *     travel limits and report completed moves.
 */
#ifndef MOTION_H
#define MOTION_H

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

#define MOTION_MOTORS           4

/* Timer: 32 MHz / 8 -> 4 MHz, 0.25 us per tick, 16-bit period. */
#define MOTION_F_TICK_HZ        4000000UL
#define MOTION_PULSE_TICKS      8          /* 2 us STEP pulse (TMC2210 needs >= ~0.1 us) */
#define MOTION_MIN_PERIOD_TICKS 40         /* 10 us between pulses (100k pulses/s) */
#define MOTION_MAX_PERIOD_TICKS 65535      /* 16.38 ms between pulses */
#define MOTION_DT_S             0.001f     /* planner tick */

/* Admission control: each pulse costs about MOTION_ISR_COST of CPU time in the
 * step interrupt. The sum over moving motors of (cost / period) must stay
 * below MOTION_BUDGET_PERMILLE / 1000 of the CPU, or the move is refused. */
#define MOTION_ISR_COST_NS      2500
#define MOTION_BUDGET_PERMILLE  300

typedef struct {
	/* --- shared with the step interrupt --- */
	volatile uint32_t remaining;     /* pulses still to emit in this move */
	volatile uint8_t  stop_req;      /* 1: stop at the next pulse */
	volatile uint8_t  stopped;       /* set by the ISR when the timer has been stopped */
	/* --- planner state (1 ms context only) --- */
	uint8_t  running;                /* a move is in progress (cleared as soon as a stop is requested) */
	uint8_t  dir_positive;
	uint8_t  decel;
	uint8_t  reserved;
	uint32_t total;                  /* pulses in this move */
	int32_t  start_pos;              /* position when the move started */
	float    v;                      /* current speed, pulses/s */
	float    v0;                     /* start/stop speed, pulses/s */
	float    vmax;                   /* cruise speed, pulses/s */
	float    a;                      /* acceleration, pulses/s^2 */
	uint16_t period_ticks;           /* last period handed to the timer */
	uint16_t cost_permille;          /* CPU cost of this motor at vmax, for admission control */
	/* --- jog (constant speed, ImmediateSteps registers) --- */
	uint8_t  jog;                    /* current move is a jog: no ramp, no end */
	uint8_t  jog_pending;            /* a jog request waits to be applied by the tick */
	uint8_t  jog_dir_positive;
	float    jog_v;                  /* requested jog speed, pulses/s */
} motion_t;

extern motion_t motion[MOTION_MOTORS];

/* pos: array of 4 int32 position registers the engine keeps updated. */
void motion_init(int32_t *pos);

/* Travel limits in position units; 0 disables a limit (as the registers define). */
void motion_set_limits(uint8_t i, int32_t min_lim, int32_t max_lim);

/* Profile in register units (microseconds between pulses) or directly in pulses/s. */
void motion_set_profile_us(uint8_t i, uint16_t nominal_us, uint16_t start_us, uint32_t accel_pulses_s2);
void motion_set_profile(uint8_t i, float v0, float vmax, float a);

/* Equivalent constant acceleration for the firmware <= 0.7 ramp that removed
 * `dp_us` from the pulse interval on every pulse (kept for backward compatibility). */
uint32_t motion_legacy_accel(uint16_t nominal_us, uint16_t start_us, uint16_t dp_us);

/* True if starting motor i at its configured speed fits the CPU budget next to
 * the motors already moving. Cheap integer arithmetic, safe from any context. */
bool motion_admit(uint8_t i);

/* Relative move request, same contract as the old user_sent_request():
 * starts a move, or extends/shortens the current one. Returns the part of the
 * request that could not be applied now (opposite direction), to be re-issued
 * once the motor has stopped. Call from the 1 ms context. */
int32_t motion_request(uint8_t i, int32_t steps);

/* Continuous rotation at a constant speed (ImmediateSteps semantics: no ramp).
 * Speed changes take effect within 1 ms; a direction change stops the motor
 * first and restarts it. Returns false if refused by admission control.
 * Safe from any context; the launch itself happens in motion_tick_1ms(). */
bool motion_jog(uint8_t i, float v, bool positive);
void motion_jog_stop(uint8_t i);
bool motion_is_jogging(uint8_t i);

/* Ask the motor to stop at the next pulse. Safe from any interrupt context. */
void motion_request_stop(uint8_t i);
bool motion_is_running(uint8_t i);
bool motion_is_idle(uint8_t i);   /* not running and the timer has fully stopped */

/* Set the position register while keeping a running move consistent. */
void motion_set_position(uint8_t i, int32_t pos);

/* Planner. Call once per millisecond from a low-priority context. */
void motion_tick_1ms(void);

/* Bit mask of motors whose move ended (or was stopped) since the last call. */
uint8_t motion_take_stopped_mask(void);

/* Body of the per-pulse interrupt, shared between the AVR ISR macros and the
 * host simulator. `hw_stop` must stop the timer and the pulse output. */
#define MOTION_ISR_BODY(m, hw_stop)                                   \
	do {                                                              \
		uint32_t r_ = (m).remaining - 1;                              \
		(m).remaining = r_;                                           \
		if (r_ == 0 || (m).stop_req) {                                \
			hw_stop;                                                  \
			(m).stopped = 1;                                          \
		}                                                             \
	} while (0)

#endif /* MOTION_H */
