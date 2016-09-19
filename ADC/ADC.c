/*
 * ADC.c
 *
 * Created: 02/02/2013 19:51:20
 *  Author: worker
 */ 

#include "ADC.h"

/*
 * \fn void Init_ADC(void)
 * \brief Init ADC
 */
void Init_ADC(void)
{
  DIDR0   = 0b00111111;    // Digital Input Disable Register - ADC5..0 Digital Input Disable
  ADCSRB   = 0b00000000;   // ADC Control and Status Register B - ADTS2:0
}

/*
 * \fn void read_adc(uint8_t channel)
 * \brief This function read ADC channel
 * \param[in] channel Channel to read
 * 
 */
void read_adc(uint8_t channel)
{
  ADMUX   = channel;            // set channel
  ADCSRA   = 0b11000110;          // ADEN, ADSC, ADPS1,2

  while (ADCSRA & (1 << ADSC));  // wait to complete

}

/*
 * \fn void ReadGainPots(void)
 * \brief This function populates following vars:
 * GainInADC[pot-name] holds the raw ADC values
 * Saved value is between 0 and 1024
 */
void ReadGainPots(void)
{
  read_adc( 3 );      // read roll gain ADC3
  GainInADC[ROLL] = ADCL;
  GainInADC[ROLL] += ((uint16_t) ADCH <<8);
  GainInADC[ROLL] = 1024 - GainInADC[ROLL]; 

  read_adc( 4 );      // read pitch gain ADC4
  GainInADC[PITCH] = ADCL;
  GainInADC[PITCH] += ((uint16_t) ADCH <<8);
  GainInADC[PITCH] = 1024 - GainInADC[PITCH]; 

  read_adc( 5 );      // read yaw gain ADC5
  GainInADC[YAW] = ADCL;
  GainInADC[YAW] += ((uint16_t) ADCH <<8);
  GainInADC[YAW] = 1024 - GainInADC[YAW]; 
}

/*
 * \fn void ReadGyros(bool calibrate)
 * \brief This function read gyros
 * \param[in] calibrate Define if calibration has been already made. If this is the case, remove gyro offset.
 * 
 */
void ReadGyros(bool calibrate)
{
  read_adc( 2 );      // read roll gyro ADC2
  gyroADC[ROLL] = ADCL;
  gyroADC[ROLL] += ((uint16_t) ADCH <<8);
  if (!calibrate)  gyroADC[ROLL]   -= gyroZero[ROLL];      //remove offset from gyro output

  read_adc( 1 );      // read pitch gyro ADC1
  gyroADC[PITCH] = ADCL;
  gyroADC[PITCH] += ((uint16_t) ADCH <<8);
  if (!calibrate)  gyroADC[PITCH] -= gyroZero[PITCH];

  read_adc( 0 );      // read yaw gyro ADC0
  gyroADC[YAW] = ADCL;
  gyroADC[YAW] += ((uint16_t) ADCH <<8);
  if (!calibrate)  gyroADC[YAW]   -= gyroZero[YAW];

}
