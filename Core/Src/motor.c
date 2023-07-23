#include "motor.h"
#include "pid.h"
#include "serial.h"
#include "tim.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>



void MotorReset(Motor_t *tmotor) {
  tmotor->counter = 0;
  tmotor->o_vel = 0;
  tmotor->o_vel2 = 0;
  tmotor->vel = 0;
  tmotor->velocity = 0;
  tmotor->position = 0;
  tmotor->v_ref = 0;
  tmotor->v_ref0 = 0;
  tmotor->p_ref = 0;
  tmotor->p_ref0 = 0;
  tmotor->rounds = 0;

}

// duty cycle of motor
void MotorSetDuty(int nDuty, TIM_HandleTypeDef *htim) {
  if (nDuty == 0) {
    htim->Instance->CCR1 = 0;
    htim->Instance->CCR2 = 0;
  } else if (nDuty > 0) {
    htim->Instance->CCR1 = nDuty;
    htim->Instance->CCR2 = 0;
  } else if (nDuty < 0) {
    htim->Instance->CCR1 = 0;
    htim->Instance->CCR2 = abs(nDuty);
  }
}

void ReadEncoder(Motor_t *tmotor, TIM_HandleTypeDef *htim) {
  tmotor->counter = htim->Instance->CNT;
  int16_t temp_data = tmotor->counter;
  tmotor->rounds += temp_data / PPR;
  tmotor->vel = (float)temp_data * (MINUTE / SAMPLING_TIME) / PPR; // RPM
  tmotor->position += (float)temp_data * RTD; // degree

  // LPF
  tmotor->velocity =
      0.7 * tmotor->vel + 0.2 * tmotor->o_vel + 0.1 * tmotor->o_vel2;
  tmotor->o_vel2 = tmotor->o_vel;
  tmotor->o_vel = tmotor->vel;

  htim->Instance->CNT = 0;
}

// Turning
float MotorTuningVelocity(PID_CONTROL_t *PIDControl, Motor_t *tmotor,
                          float velocity) {
  float SetPoint = velocity;
  float Input = tmotor->velocity;
  float g_nDutyCycle = PIDCompute(PIDControl, SetPoint, Input, SAMPLING_TIME);
  return g_nDutyCycle;
}
float MotorTuningPosition(PID_CONTROL_t *PIDControl, Motor_t *tmotor,
                          float position) {
  float SetPoint = position;
  float Input = tmotor->position;
  float g_nDutyCycle = PIDCompute(PIDControl, SetPoint, Input, SAMPLING_TIME);
  return g_nDutyCycle;
}

void MotorTrapzoidalInit(PROFILE_t *tProfile) {
  //	tProfile->dAccelMax = maxAcc;
  //	tProfile->dVelMax = maxVel;
  //	tProfile->dPosMax = maxPos;

  tProfile->dA1 = 0.5f * tProfile->dAccelMax;
  tProfile->dA2 = tProfile->dVelMax;
  tProfile->dB2 =
      -0.5f * tProfile->dVelMax * tProfile->dVelMax / tProfile->dAccelMax;
  tProfile->dA3 = -0.5f * tProfile->dAccelMax;
  tProfile->dB3 = tProfile->dPosMax * tProfile->dAccelMax / tProfile->dVelMax +
                  tProfile->dVelMax;
  tProfile->dC3 =
      -0.5f * tProfile->dPosMax * tProfile->dPosMax * tProfile->dAccelMax /
          (tProfile->dVelMax * tProfile->dVelMax) -
      0.5f * tProfile->dVelMax * tProfile->dVelMax / tProfile->dAccelMax;

  tProfile->dMidStep1 = tProfile->dVelMax / tProfile->dAccelMax;
  tProfile->dMidStep2 = tProfile->dPosMax / tProfile->dVelMax;
  tProfile->dMidStep3 = tProfile->dMidStep1 + tProfile->dMidStep2;

  tProfile->nTime = 0;
}

void MotorMovePosV(PROFILE_t *tProfile, PID_CONTROL_t *tPIDControl,
                  Motor_t *tmotor, TIM_HandleTypeDef *htim) {
  int32_t g_nDutyCycle;

  float dPosTemp = 0;
  float g_dCmdVel = 0;

  // Profile trapezoidal Speed
  if (tProfile->nTime <= tProfile->dMidStep1) {
    dPosTemp = (float)(tProfile->dA1 * tProfile->nTime * tProfile->nTime);
    g_dCmdVel = 2 * tProfile->dA1 * tProfile->nTime;
  } else if (tProfile->nTime <= tProfile->dMidStep2) {
    dPosTemp = (float)(tProfile->dA2 * tProfile->nTime + tProfile->dB2);
    g_dCmdVel = tProfile->dA2;
  } else if (tProfile->nTime <= tProfile->dMidStep3) {
    dPosTemp = (float)(tProfile->dA3 * tProfile->nTime * tProfile->nTime +
                       tProfile->dB3 * tProfile->nTime + tProfile->dC3);
    g_dCmdVel = 2 * tProfile->dA3 * tProfile->nTime + tProfile->dB3;
  } else {
    dPosTemp = tProfile->dPosMax;
  }

  // Control PID
  g_nDutyCycle = (int16_t)PIDCompute(tPIDControl, g_dCmdVel, tmotor->velocity,
		  SAMPLING_TIME);
  MotorSetDuty(g_nDutyCycle, htim);

  if (tProfile->nTime > tProfile->dMidStep3) {

    dPosTemp = 0;
    g_nDutyCycle = 0;
    MotorReset(tmotor);
    PIDReset(tPIDControl);
    g_dCmdVel = 0;
    MotorSetDuty(g_nDutyCycle, htim);
    tProfile->nTime = 0;
    htim->Instance->CNT = 0;
  }

  tmotor->p_ref0 = tmotor->p_ref;
  tmotor->v_ref0 = tmotor->v_ref;

  tmotor->p_ref = dPosTemp * DEGREE;
  tmotor->v_ref = g_dCmdVel;

  tProfile->nTime += SAMPLING_TIME / MINUTE;
}

void MotorMovePosP(PROFILE_t *tProfile, PID_CONTROL_t *tPIDControl,
                  Motor_t *tmotor, TIM_HandleTypeDef *htim) {
  int32_t g_nDutyCycle;

  float dPosTemp = 0;
  float g_dCmdVel = 0;

  // Profile trapezoidal Speed
  if (tProfile->nTime <= tProfile->dMidStep1) {
    dPosTemp = (float)(tProfile->dA1 * tProfile->nTime * tProfile->nTime);
    g_dCmdVel = 2 * tProfile->dA1 * tProfile->nTime;
  } else if (tProfile->nTime <= tProfile->dMidStep2) {
    dPosTemp = (float)(tProfile->dA2 * tProfile->nTime + tProfile->dB2);
    g_dCmdVel = tProfile->dA2;
  } else if (tProfile->nTime <= tProfile->dMidStep3) {
    dPosTemp = (float)(tProfile->dA3 * tProfile->nTime * tProfile->nTime +
                       tProfile->dB3 * tProfile->nTime + tProfile->dC3);
    g_dCmdVel = 2 * tProfile->dA3 * tProfile->nTime + tProfile->dB3;
  } else {
    dPosTemp = tProfile->dPosMax;
  }

  // Control PID
  g_nDutyCycle = (int16_t)PIDCompute(tPIDControl, dPosTemp * DEGREE, tmotor->position,
		  SAMPLING_TIME);
  MotorSetDuty(g_nDutyCycle, htim);

  if (tProfile->nTime > tProfile->dMidStep3) {

    dPosTemp = 0;
    g_nDutyCycle = 0;
    MotorReset(tmotor);
    PIDReset(tPIDControl);
    g_dCmdVel = 0;
    MotorSetDuty(g_nDutyCycle, htim);
    tProfile->nTime = 0;
    htim->Instance->CNT = 0;
  }

  tmotor->p_ref0 = tmotor->p_ref;
  tmotor->v_ref0 = tmotor->v_ref;

  tmotor->p_ref = dPosTemp * DEGREE;
  tmotor->v_ref = g_dCmdVel;

  tProfile->nTime += SAMPLING_TIME / MINUTE;
}
