#include "cpu.h"
#include "hwbp_core_types.h"
#include "app_ios_and_regs.h"
#include "app_funcs.h"
#include "hwbp_core.h"

#include "stepper_control.h"
#include "homing.h"

/************************************************************************/
/* Declare application registers                                        */
/************************************************************************/
extern AppRegs app_regs;

/************************************************************************/
/* Interrupts from Timers                                               */
/************************************************************************/
// ISR(TCC0_OVF_vect, ISR_NAKED)
// ISR(TCD0_OVF_vect, ISR_NAKED)
// ISR(TCE0_OVF_vect, ISR_NAKED)
// ISR(TCF0_OVF_vect, ISR_NAKED)
// 
// ISR(TCC0_CCA_vect, ISR_NAKED)
// ISR(TCD0_CCA_vect, ISR_NAKED)
// ISR(TCE0_CCA_vect, ISR_NAKED)
// ISR(TCF0_CCA_vect, ISR_NAKED)
// 
// ISR(TCD1_OVF_vect, ISR_NAKED)
// 
// ISR(TCD1_CCA_vect, ISR_NAKED)


uint8_t inputs_previous_read = 0;

/* InputNOpMode stops leave a homing motor alone: homing watches its own edges */
static void stop_unless_homing(uint8_t motor)
{
	if (!homing_is_active(motor)) if_moving_stop_rotation(motor);
}

/************************************************************************/ 
/* INPUT0                                                               */
/************************************************************************/
ISR(PORTK_INT0_vect, ISR_NAKED)
{
	uint8_t inputs_current_read = inputs_previous_read;
	bool line_high = read_INPUT0 ? true : false;
	
	/* Homing first: it stops its motor at the next pulse on the endstop edge */
	homing_input_edge(0, line_high);
	
	if (line_high)
	{
		inputs_current_read &= ~B_INPUT0;
		
		if (app_regs.REG_INPUT0_OPERATION_MODE & 0x20)	// Means it's configured to stop when falling
		{
			stop_unless_homing(app_regs.REG_INPUT0_OPERATION_MODE & 0x03);
		}
	}
	else
	{
		inputs_current_read |= B_INPUT0;
		
		if (app_regs.REG_INPUT0_OPERATION_MODE & 0x10)	// Means it's configured to stop when rising
		{
			stop_unless_homing(app_regs.REG_INPUT0_OPERATION_MODE & 0x03);
		}
	}
	
	if (inputs_current_read != inputs_previous_read)
	{
		app_regs.REG_DIGITAL_INPUTS_STATE = ((inputs_previous_read ^ inputs_current_read) << 4) | (inputs_current_read & 0x0F);
		core_func_send_event(ADD_REG_DIGITAL_INPUTS_STATE, true);
		
		inputs_previous_read = inputs_current_read;
	}
	
	/* MotorStopped is sent by the 1 ms callback once the motor has stopped */
	
	reti();
}

/************************************************************************/ 
/* INPUT1                                                               */
/************************************************************************/
ISR(PORTQ_INT0_vect, ISR_NAKED)
{
	uint8_t inputs_current_read = inputs_previous_read;
	bool line_high = read_INPUT1 ? true : false;
	
	/* Homing first: it stops its motor at the next pulse on the endstop edge */
	homing_input_edge(1, line_high);
	
	if (line_high)
	{
		inputs_current_read &= ~B_INPUT1;
		
		if (app_regs.REG_INPUT1_OPERATION_MODE & 0x20)	// Means it's configured to stop when falling
		{
			stop_unless_homing(app_regs.REG_INPUT1_OPERATION_MODE & 0x03);
		}
	}
	else
	{
		inputs_current_read |= B_INPUT1;
		
		if (app_regs.REG_INPUT1_OPERATION_MODE & 0x10)	// Means it's configured to stop when rising
		{
			stop_unless_homing(app_regs.REG_INPUT1_OPERATION_MODE & 0x03);
		}
	}
	
	if (inputs_current_read != inputs_previous_read)
	{
		app_regs.REG_DIGITAL_INPUTS_STATE = ((inputs_previous_read ^ inputs_current_read) << 4) | (inputs_current_read & 0x0F);
		core_func_send_event(ADD_REG_DIGITAL_INPUTS_STATE, true);
		
		inputs_previous_read = inputs_current_read;
	}
	
	/* MotorStopped is sent by the 1 ms callback once the motor has stopped */
	
	reti();
}

/************************************************************************/ 
/* INPUT2                                                               */
/************************************************************************/
ISR(PORTC_INT0_vect, ISR_NAKED)
{
	uint8_t inputs_current_read = inputs_previous_read;
	bool line_high = read_INPUT2 ? true : false;
	
	/* Homing first: it stops its motor at the next pulse on the endstop edge */
	homing_input_edge(2, line_high);
	
	if (line_high)
	{
		inputs_current_read &= ~B_INPUT2;
		
		if (app_regs.REG_INPUT2_OPERATION_MODE & 0x20)	// Means it's configured to stop when falling
		{
			stop_unless_homing(app_regs.REG_INPUT2_OPERATION_MODE & 0x03);
		}
	}
	else
	{
		inputs_current_read |= B_INPUT2;
		
		if (app_regs.REG_INPUT2_OPERATION_MODE & 0x10)	// Means it's configured to stop when rising
		{
			stop_unless_homing(app_regs.REG_INPUT2_OPERATION_MODE & 0x03);
		}
	}
	
	if (inputs_current_read != inputs_previous_read)
	{
		app_regs.REG_DIGITAL_INPUTS_STATE = ((inputs_previous_read ^ inputs_current_read) << 4) | (inputs_current_read & 0x0F);
		core_func_send_event(ADD_REG_DIGITAL_INPUTS_STATE, true);
		
		inputs_previous_read = inputs_current_read;
	}
	
	/* MotorStopped is sent by the 1 ms callback once the motor has stopped */
	
	reti();
}

/************************************************************************/ 
/* INPUT3                                                               */
/************************************************************************/
ISR(PORTH_INT0_vect, ISR_NAKED)
{
	uint8_t inputs_current_read = inputs_previous_read;
	bool line_high = read_INPUT3 ? true : false;
	
	/* Homing first: it stops its motor at the next pulse on the endstop edge */
	homing_input_edge(3, line_high);
	
	if (line_high)
	{
		inputs_current_read &= ~B_INPUT3;
		
		if (app_regs.REG_INPUT3_OPERATION_MODE & 0x20)	// Means it's configured to stop when falling
		{
			stop_unless_homing(app_regs.REG_INPUT3_OPERATION_MODE & 0x03);
		}
	}
	else
	{
		inputs_current_read |= B_INPUT3;
		
		if (app_regs.REG_INPUT3_OPERATION_MODE & 0x10)	// Means it's configured to stop when rising
		{
			stop_unless_homing(app_regs.REG_INPUT3_OPERATION_MODE & 0x03);
		}
	}
	
	if (inputs_current_read != inputs_previous_read)
	{
		app_regs.REG_DIGITAL_INPUTS_STATE = ((inputs_previous_read ^ inputs_current_read) << 4) | (inputs_current_read & 0x0F);
		core_func_send_event(ADD_REG_DIGITAL_INPUTS_STATE, true);
		
		inputs_previous_read = inputs_current_read;
	}
	
	/* MotorStopped is sent by the 1 ms callback once the motor has stopped */
	
	reti();
}

/************************************************************************/ 
/* ERROR_M0                                                             */
/************************************************************************/
ISR(PORTC_INT1_vect, ISR_NAKED)
{
	io_pin2in(&PORTC, 2, PULL_IO_UP, SENSE_IO_NO_INT_USED);
	io_set_int(&PORTC, INT_LEVEL_OFF, 1, (1<<2), false);
	
	uint8_t prev_reg = app_regs.REG_MOTORS_ERROR_DETECTTION;	
	
	clr_DRIVE_ENABLE_M0;
	if_moving_stop_rotation(0);   /* reported by the 1 ms callback */
	homing_lost(B_MOTOR0);
	clr_LED_M0;
		
	app_regs.REG_MOTORS_ERROR_DETECTTION |= B_MOTOR0;
		
	if (prev_reg != app_regs.REG_MOTORS_ERROR_DETECTTION)
	{
		core_func_send_event(ADD_REG_MOTORS_ERROR_DETECTTION, true);
	}
	
	reti();
}

/************************************************************************/ 
/* ERROR_M1                                                             */
/************************************************************************/
ISR(PORTD_INT1_vect, ISR_NAKED)
{
	io_pin2in(&PORTD, 2, PULL_IO_UP, SENSE_IO_NO_INT_USED);
	io_set_int(&PORTD, INT_LEVEL_OFF, 1, (1<<2), false);
	
	uint8_t prev_reg = app_regs.REG_MOTORS_ERROR_DETECTTION;
	
	clr_DRIVE_ENABLE_M1;
	if_moving_stop_rotation(1);   /* reported by the 1 ms callback */
	homing_lost(B_MOTOR1);
	clr_LED_M1;
		
	app_regs.REG_MOTORS_ERROR_DETECTTION |= B_MOTOR1;
		
	if (prev_reg != app_regs.REG_MOTORS_ERROR_DETECTTION)
	{
		core_func_send_event(ADD_REG_MOTORS_ERROR_DETECTTION, true);
	}
	
	reti();
}

/************************************************************************/ 
/* ERROR_M2                                                             */
/************************************************************************/
ISR(PORTE_INT1_vect, ISR_NAKED)
{
	io_pin2in(&PORTE, 2, PULL_IO_UP, SENSE_IO_NO_INT_USED);
	io_set_int(&PORTE, INT_LEVEL_OFF, 1, (1<<2), false);
	
	uint8_t prev_reg = app_regs.REG_MOTORS_ERROR_DETECTTION;
	
	clr_DRIVE_ENABLE_M2;
	if_moving_stop_rotation(2);   /* reported by the 1 ms callback */
	homing_lost(B_MOTOR2);
	clr_LED_M2;
	
	app_regs.REG_MOTORS_ERROR_DETECTTION |= B_MOTOR2;
	
	if (prev_reg != app_regs.REG_MOTORS_ERROR_DETECTTION)
	{
		core_func_send_event(ADD_REG_MOTORS_ERROR_DETECTTION, true);
	}
	
	reti();
}

/************************************************************************/ 
/* ERROR_M3                                                             */
/************************************************************************/
ISR(PORTJ_INT1_vect, ISR_NAKED)
{
	io_pin2in(&PORTJ, 5, PULL_IO_UP, SENSE_IO_NO_INT_USED);
	io_set_int(&PORTJ, INT_LEVEL_OFF, 1, (1<<5), false);
	
	uint8_t prev_reg = app_regs.REG_MOTORS_ERROR_DETECTTION;
		
	clr_DRIVE_ENABLE_M3;
	if_moving_stop_rotation(3);   /* reported by the 1 ms callback */
	homing_lost(B_MOTOR3);
	clr_LED_M3;
		
	app_regs.REG_MOTORS_ERROR_DETECTTION |= B_MOTOR3;		
		
	if (prev_reg != app_regs.REG_MOTORS_ERROR_DETECTTION)
	{
		core_func_send_event(ADD_REG_MOTORS_ERROR_DETECTTION, true);
	}
	
	reti();
}

/************************************************************************/ 
/* EMERGENCY                                                            */
/************************************************************************/
extern uint8_t motors_enabled_mask;

uint8_t enable_counter = 1;

void enable_motors (void)
{
	if (motors_enabled_mask & B_MOTOR0) set_DRIVE_ENABLE_M0;
	if (motors_enabled_mask & B_MOTOR1) set_DRIVE_ENABLE_M1;
	if (motors_enabled_mask & B_MOTOR2) set_DRIVE_ENABLE_M2;
	if (motors_enabled_mask & B_MOTOR3) set_DRIVE_ENABLE_M3;
	
	app_regs.REG_EMERGENCY_DETECTION = GM_ENABLED;
	core_func_send_event(ADD_REG_EMERGENCY_DETECTION, true);
}

ISR(PORTQ_INT1_vect, ISR_NAKED)
{
	if (read_EMERGENCY)	
	{
		/* Connector is open */
		
		if (app_regs.REG_EMERGENCY_DETECTION_MODE == GM_OPEN)
		{
			/* Enable the previous enabled motors */
		}
		else
		{
			if (enable_counter == 0)
			{
				enable_counter = 1;
				
				/* Stops are reported by the 1 ms callback */
				if_moving_stop_rotation(0);
				if_moving_stop_rotation(1);
				if_moving_stop_rotation(2);
				if_moving_stop_rotation(3);
				homing_lost(B_MOTOR0 | B_MOTOR1 | B_MOTOR2 | B_MOTOR3);
				
				clr_DRIVE_ENABLE_M0; clr_LED_M0;
				clr_DRIVE_ENABLE_M1; clr_LED_M1;
				clr_DRIVE_ENABLE_M2; clr_LED_M2;
				clr_DRIVE_ENABLE_M3; clr_LED_M3;
				
				app_regs.REG_EMERGENCY_DETECTION = GM_DISABLED;
				core_func_send_event(ADD_REG_EMERGENCY_DETECTION, true);
			}
		}
	}
	else	
	{		
		/* Connector is close */
		
		if (app_regs.REG_EMERGENCY_DETECTION_MODE == GM_CLOSED)
		{
			/* Enable the previous enabled motors */
		}
		else
		{
			if (enable_counter == 0)
			{
				enable_counter = 1;
				
				/* Stops are reported by the 1 ms callback */
				if_moving_stop_rotation(0);
				if_moving_stop_rotation(1);
				if_moving_stop_rotation(2);
				if_moving_stop_rotation(3);
				homing_lost(B_MOTOR0 | B_MOTOR1 | B_MOTOR2 | B_MOTOR3);
				
				clr_DRIVE_ENABLE_M0; clr_LED_M0;
				clr_DRIVE_ENABLE_M1; clr_LED_M1;
				clr_DRIVE_ENABLE_M2; clr_LED_M2;
				clr_DRIVE_ENABLE_M3; clr_LED_M3;
				
				app_regs.REG_EMERGENCY_DETECTION = GM_DISABLED;
				core_func_send_event(ADD_REG_EMERGENCY_DETECTION, true);
			}
		}
		
	}
	reti();
}

