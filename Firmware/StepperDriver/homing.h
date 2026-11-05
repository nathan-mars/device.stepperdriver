/*
 * homing.h - per-axis homing against an endstop on a digital input.
 *
 * Platform independent like motion.c (compiled by the host tests too). The
 * platform supplies the two homing_hal_* functions declared at the end.
 *
 * Each axis runs its own state machine, advanced by homing_tick_1ms():
 *
 *   START    stop whatever the motor is doing and wait until it is idle
 *   CLEAR    only if the sensor is already active: move away from it, at the
 *            fast speed, until it releases (at most 4 x backoff pulses)
 *   FAST     move toward the sensor at the fast speed until it triggers
 *            (at most `search` pulses)
 *   BACKOFF  move away by `backoff` pulses at the fast speed; the sensor must
 *            have released by then
 *   SLOW     move toward the sensor at the slow speed until it triggers
 *            (at most 2 x backoff pulses); that point becomes position 0
 *
 * The sensor is caught in the input interrupt (homing_input_edge), which stops
 * the motor at the next pulse, so the speed does not limit repeatability. Any
 * leg that ends without the expected edge fails the homing of that axis.
 * Travel limits are ignored while homing (they refer to the old position);
 * the limit registers themselves are not changed.
 */
#ifndef HOMING_H
#define HOMING_H

#include "motion.h"

/* HomingConfig register bits */
#define HOMING_CFG_INPUT_MASK   0x03   /* digital input wired to the endstop */
#define HOMING_CFG_POSITIVE     0x10   /* the endstop is in the positive direction */
#define HOMING_CFG_ACTIVE_LOW   0x20   /* the sensor pulls the line low when triggered */
#define HOMING_CFG_VALID_BITS   (HOMING_CFG_INPUT_MASK | HOMING_CFG_POSITIVE | HOMING_CFG_ACTIVE_LOW)

#define HOMING_CLEAR_BACKOFFS   4      /* CLEAR leg: at most this many backoffs */
#define HOMING_SLOW_BACKOFFS    2      /* SLOW leg: at most this many backoffs */

typedef struct {
	uint8_t  config;      /* HOMING_CFG_* */
	uint16_t fast_us;     /* us between pulses, search and backoff */
	uint16_t slow_us;     /* us between pulses, final approach */
	uint32_t search;      /* maximum pulses travelled looking for the sensor */
	uint16_t backoff;     /* pulses to back off between the two approaches */
} homing_params_t;

void homing_init(void);

/* True if homing the axes in `mask` at their fast speeds fits the CPU budget,
 * counting the motors outside `mask` that are moving or homing. */
bool homing_admit(uint8_t mask, const homing_params_t *p);

/* Start homing axis i. Ignored if the axis is already homing. Safe from any
 * interrupt context: the work happens in homing_tick_1ms(). */
void homing_start(uint8_t i, const homing_params_t *p);

/* Abort homing in progress on the axes in `mask`, stopping them (an axis that
 * is not homing is left alone). A homing start already clears the homed bit,
 * so an aborted axis ends up not homed. Safe from any interrupt context. */
void homing_cancel(uint8_t mask);
/* Forget that the axes in `mask` were homed (driver disabled, error, emergency);
 * also aborts homing in progress on them. */
void homing_lost(uint8_t mask);

/* Call from the digital input interrupt with the new line level. */
void homing_input_edge(uint8_t input, bool line_high);

/* Call once per millisecond, before motion_tick_1ms(). */
void homing_tick_1ms(void);

bool    homing_is_active(uint8_t i);
uint8_t homing_active_mask(void);
uint8_t homing_homed_mask(void);
/* Digital inputs watched by the axes that are homing (bit k = input k). */
uint8_t homing_input_mask(void);
/* HomingState register value: homed axes in bits 0-3, axes homing in bits 4-7. */
uint8_t homing_state(void);
/* True once after each change of homing_state(). */
bool    homing_take_changed(void);

/* --- provided by the platform --- */
bool homing_hal_input_high(uint8_t input);   /* level of the input line now */
void homing_hal_restore_profile(uint8_t i);  /* reload the motor's speed profile from its registers
                                                (called while the axis still counts as homing) */

#endif /* HOMING_H */
