#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>
#include "pid.h"
#include "tim.h"
#include "serial.h"

#define PPR 6600;
#define RTD 360 / 6600;
#define SAMPLING_TIME 0.01
#define MINUTE 60
#define DEGREE 360

typedef struct
{
	uint16_t counter;
	float rounds;
	float vel;
	float o_vel;
	float o_vel2;
	float velocity;
	float position;
    float setPoint;
    float p_ref;
    float v_ref;
    float p_ref0;
    float v_ref0;
} Motor_t;

typedef struct
{
    float dAccelMax;
    float dVelMax;
    float dPosMax;
    float dA1;
    float dA2, dB2;
    float dA3, dB3, dC3;
    float dMidStep1;
    float dMidStep2;
    float dMidStep3;
    float nTime;
//    uint8_t Direct;
} PROFILE_t;

extern void MotorReset(Motor_t *tmotor);
extern void MotorSetDuty(int nDuty, TIM_HandleTypeDef *htim);
extern void ReadEncoder(Motor_t *tmotor, TIM_HandleTypeDef *htim);
extern float MotorTuningVelocity(PID_CONTROL_t *PIDControl,Motor_t * tmotor,float vel);
extern float MotorTuningPosition(PID_CONTROL_t * PIDControl,Motor_t * tmotor, float pos);

extern void MotorTrapzoidalInit(PROFILE_t *tProfile);
extern void MotorMovePosP(PROFILE_t *tProfile, PID_CONTROL_t *tPIDControl, Motor_t * tmotor, TIM_HandleTypeDef *htim);
extern void MotorMovePosV(PROFILE_t *tProfile, PID_CONTROL_t *tPIDControl, Motor_t * tmotor, TIM_HandleTypeDef *htim);

#endif /* INC_MOTOR_H_ */
