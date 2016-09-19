/*
 * regulation.h
 *
 * Created: 25/02/2013 17:40:29
 *  Author: Alliaume Rico
 */

#ifndef REGULATION_H_
#define REGULATION_H_

// Include
#include "../kkdrone.h"

int16_t PID_e_k_1[3];  // error of last sample
int16_t PID_U_k_1[3];  // last command
int16_t PID_S_k_1[3];  // last gyro value
int16_t PID_S_k_2[3];  // last last gyro value

int16_t PID_Kp[3]; // Constant coefficient
int16_t PID_Ki[3]; // Integrator coefficient
int16_t PID_Kd[3]; // Derivate coefficient

uint8_t PID_inputDivision;
uint8_t PID_inputMultiplication;

#define PID_SAMPLE_PERIOD (1 / ESC_RATE)
#define PID_SAMPLE_FREQ ESC_RATE

// Select PID Type. Choose between A B or C
//#define PID_TYPE_A
//#define PID_TYPE_B
#define PID_TYPE_PI2

// Prototypes
int16_t PID_compute (uint8_t axis, uint8_t gyroDirection);
void PID_init(void);

#endif /* REGULATION_H_ */