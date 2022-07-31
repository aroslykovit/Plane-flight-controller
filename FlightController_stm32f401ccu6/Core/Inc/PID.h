

#include <stdint.h>
#include "main.h"
#include "filter.h"

#define PID_DUTY_CYCLE_MAX 2000
#define PID_DUTY_CYCLE_MIN 1000
#define PID_MAX_VALUE 400
#define PID_MIN_VALUE -400
#define PID_dt (float)0.001

#define PID_MAX_ANGLE 15
#define PID_MIN_ANGLE -10

struct {
	float Kp;
	float Ki;
	float Kd;

	float errorPrevious;
	float errorCurrent;
	float errorIntegral;
	float errorDifferential;

	biquadFilter_t DTermLowpass;

	uint16_t PWM_Duty;
} PitchPID, RollPID;

struct {
	float Kp;
	float Ki;
	float Kd;

	float errorPrevious;
	float errorCurrent;
	float errorIntegral;
	float errorDifferential;

	float targetAngle;
} AltPID;

void CalculatePitchPid(float pitch, float targetPitch);
void CalculateRollPid(float roll, float targetRoll);
void CalculatePID(float pitch, float roll, float targetPitch, float targetRoll);
void DTermFilterInit();
void CalculateAltPID(float alt, float targetAlt, float dt);
