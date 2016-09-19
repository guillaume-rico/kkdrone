/*
 * ADC.h
 *
 * Created: 02/02/2013 19:51:30
 *  Author: Alliaume Rico
 */ 


#ifndef ADC_H_
#define ADC_H_

// Include
#include "../kkdrone.h"

// Prototype
void Init_ADC(void);
void ReadGyros(bool calibrate);
void CalibrateGyros(void);
void ReadGainPots(void);

#endif /* ADC_H_ */