/*
 * config.c
 *
 * Created: 02/03/2013 13:04:01
 * Author: Alliaume RICO
 */ 

#include "config.h"

/*
* \fn void configuration_throttle (void) 
* \brief This function check if user want to do throttle configuration and make it.
*/
void configuration_throttle (void) 
{
 uint8_t i;

  // flash LED 3 times
  for (i=0;i<3;i++)
  {
    LED = 1;
    _delay_ms(1000);
    LED = 0;
    _delay_ms(1000);
  }

  Armed = true;  // override so that output_motor_pwm() won't quit early

  PWM_Low_Pulse_Interval = ((1000000UL / 50) - 2000)/10;  // set to 50Hz

  #ifdef TRI_COPTER
  ServoPPMRateDivider = 1;  // since we have already set to 50Hz
  #endif

  while (1)  // loop forever
  {
     
    RxGetChannels();

    MotorOut1 = RxInput[COLLECTIVE];
    MotorOut2 = RxInput[COLLECTIVE];
    MotorOut3 = RxInput[COLLECTIVE];
    #ifdef TRI_COPTER
    ServoPPMRateDivider = 1;  // This section already outputs @ 50Hz
    MotorOut4 = RxInput[YAW];
    #else
    MotorOut4 = RxInput[COLLECTIVE];
    #endif
    #if defined(HEX_COPTER) || defined(Y6_COPTER)
    MotorOut5 = RxInput[COLLECTIVE];
    MotorOut6 = RxInput[COLLECTIVE];
    #endif
    output_motor_ppm();  // this regulates rate at which we output signals
  }

};

/*
* \fn void configuration_clear (void)
* \brief This function clear configuration
*/
void configuration_clear (void)
{
    Set_EEPROM_Default_Config();
    while ( 1 );
}

/*
* \fn void configuration_stickCentering (void)
* \brief This function update zero of stick
*/
void configuration_stickCentering (void)
{
  uint16_t RxChannel1ZeroOffset, RxChannel2ZeroOffset, RxChannel4ZeroOffset;
  uint8_t i;

  // set offsets to zero (otherwise we affect what we want to calibrate !!)
  Config.RxChannel1ZeroOffset  = 0;
  Config.RxChannel2ZeroOffset  = 0;
  Config.RxChannel4ZeroOffset  = 0;

  // flash LED 4 times
  for (i=0;i<4;i++)
  {
    LED = 1;
    _delay_ms(1000);
    LED = 0;
    _delay_ms(1000);
  }
  RxChannel1ZeroOffset = RxChannel2ZeroOffset = RxChannel4ZeroOffset = 0;
    
  for (i=0;i<4;i++)
  {
    RxGetChannels();

    RxChannel1ZeroOffset += RxInput[ROLL];
    RxChannel2ZeroOffset += RxInput[PITCH];
    RxChannel4ZeroOffset += RxInput[YAW];

    _delay_ms(100);
  }
  // nb RxGetChannels() divides RxInXXX by 4 so we won't here
  Config.RxChannel1ZeroOffset  = RxChannel1ZeroOffset;
  Config.RxChannel2ZeroOffset  = RxChannel2ZeroOffset;
  Config.RxChannel3ZeroOffset  = 1120;
  Config.RxChannel4ZeroOffset  = RxChannel4ZeroOffset;

  // Store gyro direction to EEPROM
  Save_Config_to_EEPROM();

  // flash LED 1 time
  LED = 1;
  _delay_ms(2000);
  LED = 0;
}

/*
* \fn void configuration_gyroReverse (void)
* \brief This function can reverse a gyro
*/
void configuration_gyroReverse (void)
{
  uint8_t i;
  // flash LED 5 times
  for (i=0;i<5;i++)
  {
    LED = 1;
    _delay_ms(1000);
    LED = 0;
    _delay_ms(1000);
  }

  while(1)
  {
    RxGetChannels();

    if (RxInput[ROLL] < -30 || RxInput[ROLL] > 30)
    {  // Left / Right
      if (Config.RollGyroDirection == GYRO_NORMAL)
      {
        Config.RollGyroDirection = GYRO_REVERSED;
      }
      else
      {
        Config.RollGyroDirection = GYRO_NORMAL;
      }
      // Save the new config in EEPROM
      Save_Config_to_EEPROM();
      // Blink LED forever
      while (1)
      {
        LED = 1;
        _delay_ms(1000);
        LED = 0;
        _delay_ms(1000);
      }
    }
    else if (RxInput[PITCH] < -30 || RxInput[PITCH] > 30)
    { // normal(up)
      if (Config.PitchGyroDirection == GYRO_NORMAL)
      {
        Config.PitchGyroDirection = GYRO_REVERSED;
      }
      else
      {
        Config.PitchGyroDirection = GYRO_NORMAL;
      }
      // Save the new config in EEPROM
      Save_Config_to_EEPROM();
      // Blink LED forever
      while (1)
      {
        LED = 1;
        _delay_ms(1000);
        LED = 0;
        _delay_ms(1000);
      }
    }
    else if (RxInput[YAW] < -30|| RxInput[YAW] > 30)
    { // normal(left)
      if (Config.YawGyroDirection == GYRO_NORMAL)
      {
        Config.YawGyroDirection = GYRO_REVERSED;
      }
      else
      {
        Config.YawGyroDirection = GYRO_NORMAL;
      }
      // Save the new config in EEPROM
      Save_Config_to_EEPROM();
      // Blink LED forever
      while (1)
      {
        LED = 1;
        _delay_ms(1000);
        LED = 0;
        _delay_ms(1000);
      }
    }
  }
}