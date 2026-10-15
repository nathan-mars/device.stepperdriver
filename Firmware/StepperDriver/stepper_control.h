/*
 * stepper_control.h - AVR glue between the Harp application and the motion
 * engine (motion.h). Keeps the function names the rest of the application
 * already uses.
 */
#ifndef _STEPPER_CONTROL_H_
#define _STEPPER_CONTROL_H_
#include <avr/io.h>
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
#include "motion.h"

#define MOTORS_QUANTITY MOTION_MOTORS

/* Initialise the engine; call once from core_callback_initialize_hardware(). */
bool initialize_motors(void);

/* Re-read a motor's speed registers into the engine (after a register write). */
void refresh_motor_profile(uint8_t motor_index);
/* Re-read a motor's travel-limit registers into the engine. */
void refresh_motor_limits(uint8_t motor_index);

/* Stop at the next pulse (position stays exact). Safe from any interrupt. */
void stop_rotation(uint8_t motor_index);
/* Stop if moving; returns true if it was moving. */
bool if_moving_stop_rotation(uint8_t motor_index);
/* True while a move is in progress. */
bool motor_is_moving(uint8_t motor_index);

/* Emit the MotorStopped event for the motors in the mask. */
void send_motors_stopped_event(uint8_t motor_stop_bit_mask);

/* Apply a relative request; returns the part to re-issue later. 1 ms context. */
int32_t user_sent_request(int32_t requested_steps, uint8_t motor_index);

#endif /* _STEPPER_CONTROL_H_ */
