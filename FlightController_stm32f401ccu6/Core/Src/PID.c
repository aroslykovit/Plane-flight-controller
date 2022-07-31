/*
 * PID.c
 *
 *  Created on: 8 июн. 2022 г.
 *      Author: arosl
 */

#include "PID.h"

void CalculatePitchPid(float pitch, float targetPitch) {

	PitchPID.errorCurrent = targetPitch - pitch;

	if ((((PitchPID.Ki * PitchPID.errorIntegral) <= PID_MAX_VALUE)
			&& (PitchPID.errorCurrent >= 0))
			|| (((PitchPID.Ki * PitchPID.errorIntegral) >= PID_MIN_VALUE)
					&& (PitchPID.errorCurrent < 0))) {
		PitchPID.errorIntegral += PitchPID.errorCurrent * PID_dt;
	}
	PitchPID.errorDifferential =
			biquadFilterApplyDF1(&PitchPID.DTermLowpass, (PitchPID.errorCurrent - PitchPID.errorPrevious) / PID_dt);
	biquadFilterUpdateLPF(&PitchPID.DTermLowpass, 20, 1000);

	float value = PitchPID.Kp * PitchPID.errorCurrent
			+ PitchPID.Ki * PitchPID.errorIntegral
			+ PitchPID.Kd * PitchPID.errorDifferential;

	PitchPID.PWM_Duty = 1500 + value;
	if (PitchPID.PWM_Duty < PID_DUTY_CYCLE_MIN) {
		PitchPID.PWM_Duty = PID_DUTY_CYCLE_MIN;
	}
	if (PitchPID.PWM_Duty > PID_DUTY_CYCLE_MAX) {
		PitchPID.PWM_Duty = PID_DUTY_CYCLE_MAX;
	}

	PitchPID.errorPrevious = PitchPID.errorCurrent;

}

void CalculateRollPid(float roll, float targetRoll) {
	RollPID.errorCurrent = targetRoll - roll;

	if ((((RollPID.Ki * RollPID.errorIntegral) <= PID_MAX_VALUE)
			&& (RollPID.errorCurrent >= 0))
			|| (((RollPID.Ki * RollPID.errorIntegral) >= PID_MIN_VALUE)
					&& (RollPID.errorCurrent < 0))) {
		RollPID.errorIntegral += RollPID.errorCurrent * PID_dt;
	}
	RollPID.errorDifferential =
			biquadFilterApplyDF1(&RollPID.DTermLowpass, (RollPID.errorCurrent - RollPID.errorPrevious) / PID_dt);
	biquadFilterUpdateLPF(&RollPID.DTermLowpass, 20, 1000);

	float value = RollPID.Kp * RollPID.errorCurrent
			+ RollPID.Ki * RollPID.errorIntegral
			+ RollPID.Kd * RollPID.errorDifferential;

	RollPID.PWM_Duty = 1500 + value;
	if (RollPID.PWM_Duty < PID_DUTY_CYCLE_MIN) {
		RollPID.PWM_Duty = PID_DUTY_CYCLE_MIN;
	}
	if (RollPID.PWM_Duty > PID_DUTY_CYCLE_MAX) {
		RollPID.PWM_Duty = PID_DUTY_CYCLE_MAX;
	}

	RollPID.errorPrevious = RollPID.errorCurrent;
}

void CalculatePID(float pitch, float roll, float targetPitch, float targetRoll) {
	CalculatePitchPid(pitch, targetPitch);
	CalculateRollPid(roll, targetRoll);
}

void DTermFilterInit(){
	biquadFilterInitLPF(&PitchPID.DTermLowpass, 20, 1000);
	biquadFilterInitLPF(&RollPID.DTermLowpass, 20, 1000);
}

void CalculateAltPID(float alt, float targetAlt, float dt){
	AltPID.errorCurrent = targetAlt - alt;

		if ((((AltPID.Ki * AltPID.errorIntegral) <= PID_MAX_ANGLE)
				&& (AltPID.errorCurrent >= 0))
				|| (((AltPID.Ki * AltPID.errorIntegral) >= PID_MIN_ANGLE)
						&& (AltPID.errorCurrent < 0))) {
			AltPID.errorIntegral += AltPID.errorCurrent * dt;
		}
		AltPID.errorDifferential =
				(AltPID.errorCurrent - AltPID.errorPrevious) / dt;

		float value = AltPID.Kp * AltPID.errorCurrent
				+ AltPID.Ki * AltPID.errorIntegral
				+ AltPID.Kd * AltPID.errorDifferential;

		AltPID.targetAngle = value;
		if (AltPID.targetAngle < PID_MIN_ANGLE) {
			AltPID.targetAngle = PID_MIN_ANGLE;
		}
		if (AltPID.targetAngle > PID_MAX_ANGLE) {
			AltPID.targetAngle = PID_MAX_ANGLE;
		}

		AltPID.errorPrevious = AltPID.errorCurrent;
}



