#include "quick_movement.h"
#include "stepper_control.h"
#include "app_ios_and_regs.h"

extern AppRegs app_regs;

bool quick_movement_prepare(uint8_t motor_index, int32_t *pulses)
{
	float pulse_um, speed_mm_s, start_mm_s, accel_m_s2, distance_mm;

	if (motor_index == 1) {
		pulse_um = app_regs.REG_MOTOR1_QUICK_PULSE_DISTANCE;
		speed_mm_s = app_regs.REG_MOTOR1_QUICK_NOMINAL_SPEED;
		start_mm_s = app_regs.REG_MOTOR1_QUICK_START_SPEED;
		accel_m_s2 = app_regs.REG_MOTOR1_QUICK_ACCELERATION;
		distance_mm = app_regs.REG_MOTOR1_QUICK_DISTANCE;
	} else if (motor_index == 2) {
		pulse_um = app_regs.REG_MOTOR2_QUICK_PULSE_DISTANCE;
		speed_mm_s = app_regs.REG_MOTOR2_QUICK_NOMINAL_SPEED;
		start_mm_s = app_regs.REG_MOTOR2_QUICK_START_SPEED;
		accel_m_s2 = app_regs.REG_MOTOR2_QUICK_ACCELERATION;
		distance_mm = app_regs.REG_MOTOR2_QUICK_DISTANCE;
	} else {
		return false;
	}

	if (!(pulse_um > 0.0f) || !(speed_mm_s > 0.0f) || !(start_mm_s > 0.0f) || !(accel_m_s2 > 0.0f))
		return false;

	/* mm/s -> pulses/s: (mm/s * 1000 um/mm) / (um/pulse). m/s^2 -> pulses/s^2: * 1e6 / (um/pulse). */
	float v0   = start_mm_s * 1000.0f / pulse_um;
	float vmax = speed_mm_s * 1000.0f / pulse_um;
	float a    = accel_m_s2 * 1.0e6f / pulse_um;
	float n    = distance_mm * 1000.0f / pulse_um;

	if (n > 2.0e9f || n < -2.0e9f) return false;
	*pulses = (int32_t)(n < 0 ? n - 0.5f : n + 0.5f);

	motion_set_profile(motor_index, v0, vmax, a);
	return true;
}
