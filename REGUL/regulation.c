/*
 * regulation.c
 *
 * Created: 25/02/2013 17:40:29
 *  Author: Alliaume Rico
 */ 

#include "regulation.h"

int16_t PID_e_k_1[3] = {0 , 0 , 0};  // error of last sample
#ifdef PID_TYPE_A
  int16_t PID_e_k_2[3] = {0 , 0 , 0};  // error of last sample
#endif
int16_t PID_U_k_1[3] = {0 , 0 , 0};  // last command
int16_t PID_S_k_1[3] = {0 , 0 , 0};  // last gyro value
int16_t PID_S_k_2[3] = {0 , 0 , 0};  // last last gyro value

/*
* \fn void PID_init(void)
* \brief This function init pid
*/
void PID_init(void)
{
  // Init Kp Ki Kd
  PID_Kp[YAW]    = GainInADC[YAW] >> 2;
  PID_Kp[PITCH] = GainInADC[YAW] >> 2;
  PID_Kp[ROLL]  = GainInADC[ROLL] >> 2;

  PID_Ki[YAW]    = GainInADC[PITCH] >> 2;
  PID_Ki[PITCH] =  GainInADC[PITCH] >> 2;
  PID_Ki[ROLL]  =  GainInADC[PITCH] >> 2;

  PID_Kd[YAW]    = GainInADC[ROLL] >> 8;
  PID_Kd[PITCH] = GainInADC[ROLL] >> 8;
  PID_Kd[ROLL]  = GainInADC[ROLL] >> 8;

  // In function of flying mode selected by user change constant
  PID_inputMultiplication = 1;
  switch (userFlyingMode)
  {
    case NORMAL:
       PID_inputDivision = 2;
    break;
    case ACCRO:
       PID_inputDivision = 1;
    break;
    case ULTIMATE:
       PID_inputDivision = 1;
	   PID_inputMultiplication = 2;
    break;
    case NEWBIE:
      PID_inputDivision = 4;
    break;
    case DRONE:
      PID_inputDivision = 2;
    break;
  }

}

/*
* \fn int16_t PID_compute(uint8_t axis, uint8_t gyroDirection)
* \brief This function compute PID
* \param axis Axis to compute
* \param gyroDirection Gyro is reversed?
*/
int16_t PID_compute (uint8_t axis, uint8_t gyroDirection)
{
  int16_t error, p_term, i_term, d_term, S, U;

  // TODO : Add filtering of gyro values

  if (gyroDirection == GYRO_NORMAL)
  {
    S = - gyroADC[axis];
  } else {
    S = gyroADC[axis];
  }

  // -512 < S < 512
  // 0 <= GainIn <= 102
  // -100 < RxInput < 100

  // compute Error
  error = PID_inputMultiplication * (RxInput[axis] / PID_inputDivision) - S ;

  // Compute I term
  i_term = (PID_Ki[axis] * error) / 256;

  // Anti wind up
  if (i_term > 50)
    i_term = 50;

  if (i_term < -50)
    i_term = -50;

  #ifdef PID_TYPE_OLDP
  // compute P term
  p_term = (PID_Kp[axis] * error) / 256;

  // Compute output
  U = p_term;
  #endif

  #ifdef PID_TYPE_P
  // compute P term
  p_term = (PID_Kp[axis] * (error - PID_e_k_1[axis])) / 256;

  // Compute output
  U =  PID_U_k_1[axis] + p_term;
  #endif

  #ifdef PID_TYPE_PI
  // compute P term
  p_term = (PID_Kp[axis] * (error - PID_e_k_1[axis])) / 256;

  // Compute output
  U =  PID_U_k_1[axis] + p_term + i_term;
  #endif

  #ifdef PID_TYPE_PI2
  // compute P term
  p_term = (PID_Kp[axis] * error) / 128;

  i_term = (PID_Ki[axis] * (error + PID_e_k_1[axis])) / 128;

  // Compute output
  U = p_term + i_term;
  #endif

  #ifdef PID_TYPE_PID2
  // compute P term
  p_term = (PID_Kp[axis] * error) / 256;

  i_term = (PID_Ki[axis] * (error + PID_e_k_1[axis])) / 256;

  d_term = (PID_Kd[axis] * (error - PID_e_k_1[axis])) / 256;

  // Compute output
  U = p_term + i_term + d_term;
  #endif

  #ifdef PID_TYPE_A
    // compute P term
    p_term = (PID_Kp[axis] * (error - PID_e_k_1[axis])) / 256;

    // Compute D term
    d_term = ((PID_Kd[axis] * PID_SAMPLE_FREQ) * (S - 2 * PID_e_k_1[axis] + PID_e_k_2[axis]) ) / 256;

    // Compute output
    U = PID_U_k_1[axis] + p_term + i_term + d_term;
  #endif

  #ifdef PID_TYPE_B
    // compute P term
    p_term = (PID_Kp[axis] * (error - PID_e_k_1[axis])) / 256;

    // Compute D term
    d_term = ((PID_Kd[axis] * PID_SAMPLE_FREQ) * (S - 2 * PID_S_k_1[axis] + PID_S_k_2[axis])) / 256;

    // Compute output
    U = PID_U_k_1[axis] + p_term + i_term - d_term;
  #endif

  #ifdef PID_TYPE_C
    // compute P term
    p_term = (PID_Kp[axis] * (S - PID_S_k_1[axis])) / 256;

    // Compute D term
    d_term = ((PID_Kd[axis] * PID_SAMPLE_FREQ) * (S - 2 * PID_S_k_1[axis] + PID_S_k_2[axis])) / 256;

    // Compute output
    U = PID_U_k_1[axis] - p_term + i_term - d_term;
  #endif

  // Save Values
  PID_e_k_1[axis] = error;  // error of last sample
  PID_U_k_1[axis] = U;  // last command
  PID_S_k_2[axis] = PID_S_k_1[axis];  // last last gyro value
  PID_S_k_1[axis] = S;  // last gyro value

  return U;
}
