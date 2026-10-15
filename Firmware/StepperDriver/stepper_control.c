/*
 * stepper_control.c - AVR XMEGA glue for the motion engine.
 *
 * Pulse generation: TCx0 in single-slope PWM, prescaler DIV8 (0.25 us/tick),
 * CCA = 2 us pulse. One MED-level overflow interrupt per pulse runs
 * MOTION_ISR_BODY: a 32-bit decrement and a stop check, nothing else. All
 * speed arithmetic lives in motion_tick_1ms() (LOW level, 1 ms callback).
 *
 * 16-bit timer registers are written only from the planner, inside
 * MOTION_ATOMIC(), because the XMEGA shares one TEMP byte per timer between
 * all 16-bit accesses; the ISR touches only 8-bit registers.
 */
#include "stepper_control.h"
#include "motion_port.h"
#include "app_ios_and_regs.h"
#include "hwbp_core.h"

extern AppRegs app_regs;

/************************************************************************/
/* Hardware map                                                         */
/************************************************************************/
static TC0_t * const motor_timer[MOTION_MOTORS] = { &TCC0, &TCD0, &TCE0, &TCF0 };
static PORT_t * const dir_port[MOTION_MOTORS]   = { &PORTC, &PORTD, &PORTE, &PORTF };
static const uint8_t dir_pin[MOTION_MOTORS]     = { 1, 1, 1, 1 };
static PORT_t * const led_port[MOTION_MOTORS]   = { &PORTH, &PORTH, &PORTJ, &PORTQ };
static const uint8_t led_pin[MOTION_MOTORS]     = { 3, 4, 0, 1 };

/************************************************************************/
/* motion_port.h implementation                                         */
/************************************************************************/
void motion_hal_start(uint8_t i, uint16_t period_ticks, bool dir_positive)
{
	TC0_t *t = motor_timer[i];

	if (dir_positive) dir_port[i]->OUTSET = (1 << dir_pin[i]);
	else              dir_port[i]->OUTCLR = (1 << dir_pin[i]);
	/* DIR-to-STEP setup time is 20 ns; the register writes below take far longer. */

	MOTION_ATOMIC() {
		t->CTRLA    = TC_CLKSEL_OFF_gc;
		t->CTRLFSET = TC_CMD_RESET_gc;            /* all timer registers to defaults */
		t->PER      = period_ticks;
		t->CCA      = MOTION_PULSE_TICKS;
		t->CTRLB    = TC_WGMODE_SS_gc | TC0_CCAEN_bm;
		t->INTCTRLB = 0;                           /* no compare interrupts */
		t->INTFLAGS = 0xFF;                        /* clear stale flags */
		t->INTCTRLA = TC_OVFINTLVL_MED_gc;
		t->CTRLA    = TC_CLKSEL_DIV8_gc;
	}
}

void motion_hal_set_period(uint8_t i, uint16_t period_ticks)
{
	MOTION_ATOMIC() { motor_timer[i]->PERBUF = period_ticks; }
}

bool motion_hal_timer_running(uint8_t i)
{
	return motor_timer[i]->CTRLA != 0;
}

void motion_hal_led(uint8_t i, bool on)
{
	if (on) {
		if (core_bool_is_visual_enabled()) led_port[i]->OUTSET = (1 << led_pin[i]);
	} else {
		led_port[i]->OUTCLR = (1 << led_pin[i]);
	}
}

/************************************************************************/
/* Per-pulse interrupts: fixed addresses, 8-bit registers only           */
/************************************************************************/
#define STOP_TIMER(T) ((T).CTRLB &= (uint8_t)~TC0_CCAEN_bm, (T).CTRLA = 0, (T).INTCTRLA = 0)

ISR(TCC0_OVF_vect) { MOTION_ISR_BODY(motion[0], STOP_TIMER(TCC0)); }
ISR(TCD0_OVF_vect) { MOTION_ISR_BODY(motion[1], STOP_TIMER(TCD0)); }
ISR(TCE0_OVF_vect) { MOTION_ISR_BODY(motion[2], STOP_TIMER(TCE0)); }
ISR(TCF0_OVF_vect) { MOTION_ISR_BODY(motion[3], STOP_TIMER(TCF0)); }

/************************************************************************/
/* Register <-> engine                                                  */
/************************************************************************/
static uint16_t reg_u16(const uint16_t *base, uint8_t i) { return base[i]; }

void refresh_motor_profile(uint8_t i)
{
	if (i >= MOTION_MOTORS) return;
	uint16_t nominal = reg_u16(&app_regs.REG_MOTOR0_NOMINAL_STEP_INTERVAL, i);
	uint16_t start   = reg_u16(&app_regs.REG_MOTOR0_MAXIMUM_STEP_INTERVAL, i);
	uint32_t accel   = (&app_regs.REG_MOTOR0_ACCELERATION)[i];
	motion_set_profile_us(i, nominal, start, accel);
}

void refresh_motor_limits(uint8_t i)
{
	if (i >= MOTION_MOTORS) return;
	motion_set_limits(i, (&app_regs.REG_MOTOR0_MIN_STEPS_INTEGRATION)[i],
	                     (&app_regs.REG_MOTOR0_MAX_STEPS_INTEGRATION)[i]);
}

bool initialize_motors(void)
{
	motion_init(app_regs.REG_ACCUMULATED_STEPS);
	for (uint8_t i = 0; i < MOTION_MOTORS; i++) {
		refresh_motor_profile(i);
		refresh_motor_limits(i);
	}
	return true;
}

/************************************************************************/
/* Start & stop                                                         */
/************************************************************************/
void stop_rotation(uint8_t i)
{
	if (i >= MOTION_MOTORS) return;
	motion_request_stop(i);
	led_port[i]->OUTCLR = (1 << led_pin[i]);
}

bool if_moving_stop_rotation(uint8_t i)
{
	if (i >= MOTION_MOTORS) return false;
	if (motion_is_running(i)) {
		stop_rotation(i);
		return true;
	}
	return false;
}

bool motor_is_moving(uint8_t i)
{
	return i < MOTION_MOTORS && motion_is_running(i);
}

void send_motors_stopped_event(uint8_t motor_stop_bit_mask)
{
	app_regs.REG_MOTORS_STOPPED = (uint8_t)(motor_stop_bit_mask << 4);
	for (uint8_t i = 0; i < MOTION_MOTORS; i++) {
		if (!motion_is_running(i)) app_regs.REG_MOTORS_STOPPED |= (uint8_t)(1 << i);
	}
	core_func_send_event(ADD_REG_MOTORS_STOPPED, true);
}

int32_t user_sent_request(int32_t requested_steps, uint8_t i)
{
	if (i >= MOTION_MOTORS) return 0;
	return motion_request(i, requested_steps);
}
