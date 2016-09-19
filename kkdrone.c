/*
 * kkdrone.c
 *
 * Created: 02/02/2013 17:09:14
 *
 * Author: Alliaume RICO
 *
 * History :
 * Version 2.1 :
 * - Add fly mode (Newbie, normal, accro, ultimate)
 * Version 2.0 :
 * -Add PID as 4.7 of KuK
 * -Reverse pots
 * -Change LED timing
 * -Change default config
 * Version 1.0 :
 * - Version 1.1 of Mike Berton
 * inspired by KKmulticopter
 * based on assembly code by Rolf R Bakke
 *
 * Config Modes (at startup)
 * =========================
 *
 * Stick Centering
 * ---------------
 * set Pitch Gain pot to zero
 * set Tx trims to center
 * power on
 * led flashes 3 times
 * restore gain pot
 * use as normal
 *
 * Gyro direction reversing
 * ------------------------
 * set Roll Gain pot to zero
 * power on
 * led flashes 3 times
 * move stick Up/Left for Normal, Down/Right for reverse
 * eg: to set Roll Gyro Normal, move Tx Roll stick left, or to reverse it move stick right
 *
 * ESC throttle calibration
 * ------------------------
 * set Yaw Gain pot to zero
 * put throttle stick to full
 * power on
 * led flashes 3 times
 * wait for motor signal (double bleep)
 * throttle to zero
 * wait for motor confirm signal
 * power off
 * restore gain pot
 *
 * Clear all settings (gyro & stick centering)
 * -------------------------------------------
 * set Roll, Pitch & Yaw Gain pots to zero
 * power on, wait a few seconds, power off
 * restore gain pots
 *
 * Select mode
 * -------------------------------------------
 * When arming, the roll/pitch is used to select mode
 * Normal : center
 * Beginner : Bottom Left
 * Accro : Top left
 * Ultimate : Top right
 * Drone bottom right
 */ 


/*


Tri   
       
     M1 CW      M2 CCW
      \       / 
       \ --- /
        |   |
         --- 
          |  
          |
          M3    M4=Tail Servo  


Quad
            M1 CW
            |
            |
            |
          +---+
CCW M2----|   |----M3 CCW
          +---+
            |
            |
            |
            M4 CW

Quad-X
       
      M2 CW     M4 CCW
        \       / 
         \ --- /
          |   |
         / --- \
        /       \ 
      M1  CCW    M3 CW
 
Hex
            M1 CW
            |
     M6     |      M2 CCW
       \    |     /
        \ +---+  /
         -|   |- 
        / +---+  \
       /    |     \
     M5     |      M3 CW
            |
            M4

Y6
       
     M1/4        M2/5    M1->3 = CW
      \       /          M4->6 = CCW
       \ --- /
        |   |
         --- 
          |  
          |
          M3/6

*/

/* ----------- Main Code -----------  */

#include "kkdrone.h"

int16_t gyroZero[3] = {0,0,0};  // used for calibrating Gyros on ground

bool output_motor_high = false;
uint16_t PWM_Low_Pulse_Interval = PWM_LOW_PULSE_INTERVAL;    // non-const version of PWM_LOW_PULSE_INTERVAL

/*
* \fn int main(void)
* \brief Main function
*/
int main(void)
{

  setup();

  while (1)
  {
    loop();
  }

  return 1;
}

/*
* \fn void setup(void)
* \brief Hardware and software initialization
*/
void setup(void)
{

  MCUCR |= (1<<PUD);  // Pull-up Disable

  // Define Input And Output pins
  RX_ROLL_DIR     = INPUT;
  RX_PITCH_DIR   = INPUT;
  RX_COLL_DIR    = INPUT;
  RX_YAW_DIR  = INPUT;

  GYRO_YAW_DIR    = INPUT;
  GYRO_PITCH_DIR  = INPUT;
  GYRO_ROLL_DIR      = INPUT;
  GAIN_YAW_DIR        = INPUT;
  GAIN_PITCH_DIR    = INPUT;
  GAIN_ROLL_DIR      = INPUT;

  M1_DIR              = OUTPUT;
  M2_DIR              = OUTPUT;
  M3_DIR              = OUTPUT;
  M4_DIR              = OUTPUT;
#if defined(HEX_COPTER) || defined(Y6_COPTER)
  M5_DIR              = OUTPUT;
  M6_DIR              = OUTPUT;
#endif
  LED_DIR            = OUTPUT;

  LED       = 0;
  RX_ROLL   = 0;
  RX_PITCH   = 0;
  RX_COLL    = 0;
  RX_YAW     = 0;

  // pin change interrupt enables
  PCICR |= (1 << PCIE0);      // PCINT0..7    
  PCICR |= (1 << PCIE2);      // PCINT16..23

  // pin change masks
  PCMSK0 |= (1 << PCINT7);    // PB7
  PCMSK2 |= (1 << PCINT17);    // PD1
  // external interrupts
  EICRA  = (1 << ISC00) | (1 << ISC10);  // Any change INT0, INT1
  EIMSK  = (1 << INT0) | (1 << INT1);    // External Interrupt Mask Register
  EIFR |= (1 << INTF0) | (1 << INTF1);

  // timer0 (8bit) - run @ 8MHz
  // used to control ESC/servo pulse length
  TCCR0A = 0;            // normal operation
  TCCR0B = (1 << CS00);  // clk/0
  TIMSK0 = 0;           // no interrupts

  // timer1 (16bit) - run @ 1Mhz
  // used to measure Rx Signals & control ESC/servo output rate
  TCCR1A = 0;
  TCCR1B = (1 << CS11);

  // timer2 8bit - run @ 8MHz / 1024 = 7812.5KHz
  // and Stick-Arming
  TCCR2A = 0;  
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);  // /1024
  TIMSK2 = 0;
  TIFR2  = 0;
  TCNT2 = 0;    // reset counter

#ifdef TRI_COPTER
  // calculate Servo Rate divider
  ServoPPMRateDivider = 0;
  do {
    ServoPPMRateDivider++;
    i = ESC_RATE / ServoPPMRateDivider;
  } while (i>50);
#endif

  Initial_EEPROM_Config_Load();  // loads config at start-up 

  Init_ADC();

  GyroCalibrated = false;

  Armed = false;

  RxChannelsUpdatingFlag = 0;

  RxChannel1 = Config.RxChannel1ZeroOffset;    // prime the channels 1520;
  RxChannel2 = Config.RxChannel2ZeroOffset;    // 1520;
  RxChannel3 = Config.RxChannel3ZeroOffset;    // 1120;
  RxChannel4 = Config.RxChannel4ZeroOffset;    // 1520;

  // flash LED
  LED = 1;
  _delay_ms(150);
  LED = 0;

  sei();                // Global Interrupts 

  // 2 second delay
  _delay_ms(1500);
  
  ReadGainPots();
  ReadGainPots();

  // If user set all pots to 0 , clear config.
  if (GainInADC[PITCH] < (UC_ADC_MAX*5)/100 &&
      GainInADC[ROLL]  < (UC_ADC_MAX*5)/100 &&
      GainInADC[YAW]   < (UC_ADC_MAX*5)/100 )
  {
    configuration_clear();
  }

  // Stick Centering
  if (GainInADC[PITCH] < (UC_ADC_MAX*5)/100)    // less than 5%
  {
    configuration_stickCentering();
  }

  // Gyro direction reversing
  if (GainInADC[ROLL] < (UC_ADC_MAX*5)/100)    // less than 5% (5/100) * 1023 = 51 
  {
    configuration_gyroReverse();
  }

  // ESC throttle calibration
  if (GainInADC[YAW] < (UC_ADC_MAX*5)/100)    // less than 5%
  {
    configuration_throttle();
  }
}

/*
* \fn void loop(void)
* \brief Main loop
*/
void loop(void)
{
  static uint16_t Change_Arming=0;
  static uint8_t Arming_TCNT2=0;

  int16_t MotorOut=0;

  RxGetChannels();

  if ( RxInput[COLLECTIVE] < STICKARM_MIN_COLLECTIVE) {
    // check for stick arming (Timer2 @ 8MHz/1024 = 7812.5KHz)
    // arm: yaw right (>60), dis-arm: yaw left (<-60)
    Change_Arming += (uint8_t) (TCNT2 - Arming_TCNT2);
    Arming_TCNT2 = TCNT2;

#ifdef STICKARM_LEFT
    if (Armed) {
      if (RxInput[YAW]<STICKARM_POINT)   Change_Arming = 0;    // re-set count
    } else {
      if (RxInput[YAW]>-STICKARM_POINT)   Change_Arming = 0;    // re-set count
    }
#else
    if (!Armed) {  
      if (RxInput[YAW]<STICKARM_POINT)   Change_Arming = 0;    // re-set count
    } else {
      if (RxInput[YAW]>-STICKARM_POINT)   Change_Arming = 0;    // re-set count
    }
#endif
    // 3Sec / 0.000128 = 23437 = 0x5B8D or 
    // 2.5Sec / 0.000128 = 19531 = 0x4C4B
    // 1Sec / 0.000128 = 7812 = 0x1E84
    if (Change_Arming>0x1E84)
    {
      Armed = ! Armed;
      LED = 0;
      if (Armed) {
        output_motor_high = false;  // re-set 1st time flag
        userFlyingMode = NORMAL;
        // Check fly user fly mode
        // Ultimate mode (Top-right right hand)
        if (RxInput[ROLL] > MODE_SELECT_POINT && RxInput[PITCH]>MODE_SELECT_POINT)
        {
           userFlyingMode = ULTIMATE;
        }
        // Accro mode (Top-left right hand)
        else if (RxInput[ROLL] < -MODE_SELECT_POINT && RxInput[PITCH]>MODE_SELECT_POINT)
        {
           userFlyingMode = ACCRO;
        }
        // Newbie mode (Bottom-left right hand)
        else if (RxInput[ROLL]<-MODE_SELECT_POINT && RxInput[PITCH]<-MODE_SELECT_POINT)
        { 
           userFlyingMode = NEWBIE;
        }
        // Drone mode (Bottom-right right hand)
        else if (RxInput[ROLL]>MODE_SELECT_POINT && RxInput[PITCH]<-MODE_SELECT_POINT)
        {
           userFlyingMode = DRONE;
		}        
        
        CalibrateGyros();

        // Init PID constant
        PID_init();
        LED = 1;
      }  else if (output_motor_high) {
        output_motor_ppm();          // turn off
      }
      return;
    }
  }


  //--- Read gyros ---
  ReadGyros(false);

  //--- Start mixing by setting collective to motor input 1,2,3 and 4 ---
  if (RxInput[COLLECTIVE] > MAX_COLLECTIVE) RxInput[COLLECTIVE] = MAX_COLLECTIVE;
  MotorOut1 = RxInput[COLLECTIVE];
  MotorOut2 = RxInput[COLLECTIVE];
  MotorOut3 = RxInput[COLLECTIVE];
#ifndef TRI_COPTER
  MotorOut4 = RxInput[COLLECTIVE];
#endif
#if defined(HEX_COPTER) || defined(Y6_COPTER)
  MotorOut5 = RxInput[COLLECTIVE];
  MotorOut6 = RxInput[COLLECTIVE];
#endif


  //--- Calculate roll (Right Vs Left) gyro output ---
  MotorOut = PID_compute(ROLL,Config.RollGyroDirection);

  //--- (Add)Adjust roll gyro output to motors
#ifdef TRI_COPTER
  MotorOut  = (MotorOut * 20)/23;  //MotorOut 60= 0.866 ~ 20/23 or possibly 7/8
  MotorOut1 += MotorOut;
  MotorOut2 -= MotorOut;
#elif defined(QUAD_COPTER)
  MotorOut2 += MotorOut;
  MotorOut3 -= MotorOut;
#elif defined(QUAD_X_COPTER)
  MotorOut  = (MotorOut >> 1);  //was:MotorOut  = (MotorOut * 20)/23;
  MotorOut1 += MotorOut;
  MotorOut2 += MotorOut;
  MotorOut3 -= MotorOut;
  MotorOut4 -= MotorOut;
#elif defined(HEX_COPTER)
  // Was MotorOut  = (MotorOut * 20)/23;  //MotorOut  *= 0.866;  // Sine 60
  MotorOut  = (MotorOut >> 1);
  MotorOut2 -= MotorOut;
  MotorOut3 -= MotorOut;
  MotorOut5 += MotorOut;
  MotorOut6 += MotorOut;
#elif defined(Y6_COPTER)
  MotorOut  = (MotorOut * 20)/23;  //MotorOut  *= 0.866;  // Sine 60
  MotorOut1 += MotorOut;
  MotorOut4 += MotorOut;
  MotorOut2 -= MotorOut;
  MotorOut5 -= MotorOut;
#else
#error No Copter configuration defined !!!!
#endif

  //--- Calculate pitch (Front Vs Back) gyro output ---
  MotorOut = PID_compute(PITCH,Config.PitchGyroDirection);

  //--- (Add)Adjust pitch gyro output to motors
#ifdef TRI_COPTER
  MotorOut3 -= MotorOut;
  MotorOut = (MotorOut >> 1);  // cosine of 60
  MotorOut1 += MotorOut;
  MotorOut2 += MotorOut;
#elif defined(QUAD_COPTER)
  MotorOut1 += MotorOut;
  MotorOut4 -= MotorOut;
#elif defined(QUAD_X_COPTER)
  MotorOut  = (MotorOut >> 1);  //23;  //Sine 60 = 0.866 20/23 = 0.869
  MotorOut1 += MotorOut;
  MotorOut2 -= MotorOut;
  MotorOut3 += MotorOut;
  MotorOut4 -= MotorOut;
#elif defined(HEX_COPTER)
  MotorOut  = (MotorOut >> 1); // Was not defined
  MotorOut1 -= MotorOut;
  MotorOut4 += MotorOut;
  MotorOut = (MotorOut >> 1); // Was MotorOut = (MotorOut >> 1);
  MotorOut2 -= MotorOut;  
  MotorOut3 += MotorOut;
  MotorOut5 += MotorOut;
  MotorOut6 -= MotorOut;
#elif defined(Y6_COPTER)
  MotorOut3 -= MotorOut;
  MotorOut6 -= MotorOut;
  MotorOut = (MotorOut >> 1);  // cosine of 60
  MotorOut1 += MotorOut;
  MotorOut4 += MotorOut;
  MotorOut2 += MotorOut;
  MotorOut5 += MotorOut;
#else
#error No Copter configuration defined !!!!
#endif


  //--- Calculate yaw gyro output ---
#ifdef TRI_COPTER

  GainAttenuator = RxInput[YAW];
  if (GainAttenuator < 0) GainAttenuator = - GainAttenuator;
  GainAttenuator -= 5;
  if (GainAttenuator < 0) GainAttenuator = 0;
  GainAttenuator = (GainAttenuator >> 2);        // div by 4
  GainAttenuator = 64 - GainAttenuator;
  if (GainAttenuator < 0) GainAttenuator = 0;
  GainAttenuator = (GainAttenuator >> 6);        // div by 64

  GainAttenuator *= GainInADC[YAW] / 10;            // was 0->1023 now 0->102
  GainAttenuator *= YAW_GAIN_MULTIPLIER;    // x Gain (was 0.003) = x3/1000
  gyroADC[YAW] *= GainAttenuator;

  gyroADC[YAW] /= 100;            // so now /100 rather than /1000
  if (Config.YawGyroDirection == GYRO_NORMAL)  {  // scale gyro output
    MotorOut = RxInput[YAW] + gyroADC[YAW];
  } else {
    MotorOut = RxInput[YAW] - gyroADC[YAW];
  }
  MotorOut4 = 50 + MotorOut;
#else


  MotorOut = PID_compute(YAW,Config.YawGyroDirection);

  //--- (Add)Adjust yaw gyro (Z axis rotation) output to motors
#ifdef QUAD_COPTER
  MotorOut1 -= MotorOut;
  MotorOut2 += MotorOut;
  MotorOut3 += MotorOut;
  MotorOut4 -= MotorOut;
#elif defined(QUAD_X_COPTER)
  MotorOut1 += MotorOut;
  MotorOut2 -= MotorOut;
  MotorOut3 -= MotorOut;
  MotorOut4 += MotorOut;
#elif defined(HEX_COPTER)
  MotorOut1 -= MotorOut;
  MotorOut2 += MotorOut;
  MotorOut3 -= MotorOut;
  MotorOut4 += MotorOut;
  MotorOut5 -= MotorOut;
  MotorOut6 += MotorOut;
#elif defined(Y6_COPTER)
  MotorOut1 -= MotorOut;
  MotorOut2 -= MotorOut;
  MotorOut3 -= MotorOut;
  MotorOut4 += MotorOut;
  MotorOut5 += MotorOut;
  MotorOut6 += MotorOut;
#else
#error No Copter configuration defined !!!!
#endif

#endif

  //--- Limit the lowest value to avoid stopping of motor if motor value is under-saturated ---
  if ( MotorOut1 < 10 )  MotorOut1 = 10;    // this is the motor idle level
  if ( MotorOut2 < 10 )  MotorOut2 = 10;  
  if ( MotorOut3 < 10 )  MotorOut3 = 10;
#ifndef TRI_COPTER
  if ( MotorOut4 < 10 )  MotorOut4 = 10;  
#endif
#if defined(HEX_COPTER) || defined(Y6_COPTER)
  if ( MotorOut5 < 10 )  MotorOut5 = 10;  
  if ( MotorOut6 < 10 )  MotorOut6 = 10;  
#endif  
  //--- Output to motor ESC's ---

  if (RxInput[COLLECTIVE] < STICKARM_MIN_COLLECTIVE + 1 || !Armed || !GyroCalibrated)  // turn off motors if collective below 1% ???
  {                                                  // or  if gyros not calibrated
    MotorOut1 = 0;
    MotorOut2 = 0;
    MotorOut3 = 0;
#ifndef TRI_COPTER
    MotorOut4 = 0;
#else
    MotorOut4 = 50;
#endif
#if defined(HEX_COPTER) || defined(Y6_COPTER)
    MotorOut5 = 0;
    MotorOut6 = 0;
#endif
  }

  if (Armed) output_motor_ppm();    // output ESC signal
}

/*
* \fn void CalibrateGyros(void)
* \brief Calibrate Gyro
*/
void CalibrateGyros(void)
{
  uint8_t i;

  ReadGainPots();  // about time we did this !

  // get/set gyro zero value (average of 32 readings)
  gyroZero[ROLL]  = 0;            
  gyroZero[PITCH] = 0;  
  gyroZero[YAW]   = 0;

  for (i=0;i<32;i++)
  {
    ReadGyros(true);
  
    gyroZero[ROLL]  += gyroADC[ROLL];            
    gyroZero[PITCH] += gyroADC[PITCH];  
    gyroZero[YAW]   += gyroADC[YAW];
  }

  gyroZero[ROLL]  = (gyroZero[ROLL]  >> 5);            
  gyroZero[PITCH] = (gyroZero[PITCH] >> 5);
  gyroZero[YAW]   = (gyroZero[YAW]   >> 5);

  GyroCalibrated = true;

}

/*
* \fn void RxGetChannels(void)
* \brief Get and scale RX channel inputs
* RxChannelX is between 1000 and 2000, then remove 1520 and then divide by 4
* RxInput is between -130 and 120
* Else for RxInput[COLLECTIVE] : 1000 --> 2000 , remove 1120 , divide per 8
* RxInput[COLLECTIVE] is between -15 and 110
*/
void RxGetChannels(void)
{
  static int16_t RxChannel;

  while ( RxChannelsUpdatingFlag );

  RxChannel = RxChannel1;
  RxChannel -= Config.RxChannel1ZeroOffset;        // normalize
  RxInput[ROLL] = (RxChannel >> 2);

  while ( RxChannelsUpdatingFlag );

  RxChannel = RxChannel2;
  RxChannel -= Config.RxChannel2ZeroOffset;        // normalize
  RxInput[PITCH] = (RxChannel >> 2);

  while ( RxChannelsUpdatingFlag );

  RxChannel = RxChannel3;
  RxChannel -= Config.RxChannel3ZeroOffset;        // scale 0->100
  RxInput[COLLECTIVE] = (RxChannel >> 3);

  while ( RxChannelsUpdatingFlag );

  RxChannel = RxChannel4;
  RxChannel -= Config.RxChannel4ZeroOffset;        // normalize
  RxInput[YAW] = (RxChannel >> 2);

}

/*
* \fn void output_motor_ppm(void)
* \brief CSend command to motor
*/
void output_motor_ppm(void)
{
  static uint8_t i;
  static uint16_t MotorStartTCNT1, ElapsedTCNT1;
  static int16_t MotorAdjust;
  static uint16_t PWM_Low_Count;
  static int8_t num_of_10uS;  
#ifdef TRI_COPTER
  static uint8_t ServoPPMRateCount;
#endif


  // if ESC's are high, we need to turn them off
  if (output_motor_high)
  {

    // set motor limits (0 -> 100)
    if ( MotorOut1 < 0 ) MotorOut1 = 0;
    else if ( MotorOut1 > 100 ) MotorOut1 = 100;
    if ( MotorOut2 < 0 ) MotorOut2 = 0;
    else if ( MotorOut2 > 100 ) MotorOut2 = 100;
    if ( MotorOut3 < 0 ) MotorOut3 = 0;
    else if ( MotorOut3 > 100 ) MotorOut3 = 100;
    if ( MotorOut4 < 0 ) MotorOut4 = 0;
    else if ( MotorOut4 > 100 ) MotorOut4 = 100;
#if defined(HEX_COPTER) || defined(Y6_COPTER)
    if ( MotorOut5 < 0 ) MotorOut5 = 0;
    else if ( MotorOut5 > 100 ) MotorOut5 = 100;
    if ( MotorOut6 < 0 ) MotorOut6 = 0;
    else if ( MotorOut6 > 100 ) MotorOut6 = 100;
#endif

    // now calculate the time already passed that Motors were HIGH
    ElapsedTCNT1 = (TCNT1 - MotorStartTCNT1);

    // start output timer
    TIFR0 &= ~(1 << TOV0);  // clr overflow
    TCNT0 = 0;              // reset counter

    // convert into 10uS intervals
    num_of_10uS = (ElapsedTCNT1 / 10) + 1;
    MotorAdjust = 100 - num_of_10uS;

    // add adjustment (1mS - time already gone) to all channels
    MotorOut1 += MotorAdjust;
    MotorOut2 += MotorAdjust;
    MotorOut3 += MotorAdjust;
    MotorOut4 += MotorAdjust;
#if defined(HEX_COPTER) || defined(Y6_COPTER)
    MotorOut5 += MotorAdjust;
    MotorOut6 += MotorAdjust;
#endif

    // keep signal on for correct time
    // MotorOutX = 100 -> 200
    // Pulse len = 1   -> 2    mS

    TIFR0 &= ~(1 << TOV0);  // clr overflow
    TCNT0 = 0;              // reset counter

    for (i=num_of_10uS;i<200;i++)  
    {
      while (TCNT0 < 80);    // 10uS @ 8MHz = 80 // 10 @ 1MHz = 10uS
      TCNT0 -= 80;

      if (MotorOut1) 
      {
        MotorOut1--;
        if (MotorOut1==0) M1 = 0;
      }
      if (MotorOut2) 
      {
        MotorOut2--;
        if (MotorOut2==0) M2 = 0;
      }
      if (MotorOut3) 
      {
        MotorOut3--;
        if (MotorOut3==0) M3 = 0;
      }
      if (MotorOut4) 
      {
        MotorOut4--;
        if (MotorOut4==0) M4 = 0;
      }
  #if defined(HEX_COPTER) || defined(Y6_COPTER)
      if (MotorOut5) 
      {
        MotorOut5--;
        if (MotorOut5==0) M5 = 0;
      }
      if (MotorOut6) 
      {
        MotorOut6--;
        if (MotorOut6==0) M6 = 0;
      }
  #endif
    }

    //Now wait low signal interval
    PWM_Low_Count = PWM_Low_Pulse_Interval - 1;

    TIFR0 &= ~(1 << TOV0);  // clr overflow
    TCNT0 = 0;              // reset counter

    while (PWM_Low_Count--)
    {
      while (TCNT0 < 80);    // 20 @ 2MHz = 10uS
      TCNT0 -= 80;
    }
  }

  if (! Armed) return;

  // Log PWM signal HIGH  
  MotorStartTCNT1 = TCNT1;
  output_motor_high = true;

  // turn on pins
  M1 = 1;
  M2 = 1;
  M3 = 1;
#ifdef TRI_COPTER
  if(ServoPPMRateCount==ServoPPMRateDivider)
  {
    M4 = 1;
    ServoPPMRateCount = 1;
  } else {
    ServoPPMRateCount++;
  }
#else
  M4 = 1;
#endif
#if defined(HEX_COPTER) || defined(Y6_COPTER)
  M5 = 1;
  M6 = 1;
#endif

}

