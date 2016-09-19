/*
 * kkdrone.h
 *
 * Created: 02/02/2013 19:21:13
 * Author: Alliaume RICO
 */ 


#ifndef KKDRONE_H_
#define KKDRONE_H_

/* ----------- Configuration -----------  */
// Don't forget to set your device in Project/Configuration Options

// enable ONE of these lines
//#define TRI_COPTER
//#define QUAD_COPTER
//#define QUAD_X_COPTER
#define HEX_COPTER
//#define Y6_COPTER


// defines output rate to ESC/Servo
// either define by setting ESC_RATE (Max is approx 495Hz)

#define ESC_RATE 450  // in Hz
#define PWM_LOW_PULSE_INTERVAL  ((1000000 / ESC_RATE) - 2000)/10

//or define by setting PWM_LOW_PULSE_INTERVAL (minimum is 1)

//#define PWM_LOW_PULSE_INTERVAL 50    // this equates to 400Hz
//#define ESC_RATE = 10000000 / ( (PWM_LOW_PULSE_INTERVAL * 10) + 20000 )

//PWM_LOW_PULSE_INTERVAL is the number of 10uS that PWM signal is low


// Adjust these:
//     down if you have too much gyro assistance
//     up if you have maxxed your gyro gain
#define ROLL_GAIN_MULTIPLIER 3
#define PITCH_GAIN_MULTIPLIER 3
#define YAW_GAIN_MULTIPLIER 3

// Stick scaling
// 'normal' is 1
// to double stick effect, subtract by 1 (but do not go -ve!)
// to half, add 1
#define STICK_DIVIDER 1

// Stick Arming
// if you cannot either arm or disarm, lower this value
#define STICKARM_POINT 60  // defines how far stick must be moved
#define MODE_SELECT_POINT 60  // defines how far stick must be moved

// set one of the following lines
//#define STICKARM_LEFT
#define STICKARM_RIGHT
// Min Throttle for arming /disarming (0 --> 100)
#define STICKARM_MIN_COLLECTIVE 5


// Max Collective
// limits the maximum stick collective (range 80->100  100=Off)
// this allows gyros to stabilize better when full throttle applied
#define MAX_COLLECTIVE 95

#define F_CPU 1000000UL

//Include
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "typedefs.h"
#include "io_cfg.h"
#include "ISR/ISR.h"
#include "EEPROM/EEPROM.h"
#include "ADC/ADC.h"
#include "REGUL/regulation.h"
#include "CONFIG/config.h"

// Prototype
void setup(void);
void loop(void);

void RxGetChannels(void);
void read_adc(uint8_t channel);

void output_motor_ppm(void);

#define EEPROM_DATA_START_POS 10    // allows moving (not really necessary)

// Max ADC
#define UC_ADC_MAX 1023      // used to invert ADC reading of uC

enum GyroDirection { GYRO_NORMAL = 0, GYRO_REVERSED };

enum GyroArrayIndex { ROLL = 0, PITCH, YAW, COLLECTIVE };

// User flying mode
uint8_t userFlyingMode;
enum FlyingMode {NORMAL = 0, ACCRO, ULTIMATE, NEWBIE, DRONE};


// eeProm data structure
typedef struct Config_Struct CONFIG_STRUCT;
struct Config_Struct
{
  uint8_t  setup;          // byte to identify if already setup

  uint8_t RollGyroDirection;
  uint8_t PitchGyroDirection;
  uint8_t YawGyroDirection;

  // allows setting to zero
  uint16_t RxChannel1ZeroOffset;
  uint16_t RxChannel2ZeroOffset;
  uint16_t RxChannel3ZeroOffset;  // currently fixed
  uint16_t RxChannel4ZeroOffset;

};

CONFIG_STRUCT Config;    // Holds configuration (from eeProm)

bool GyroCalibrated;
volatile BOOL RxChannelsUpdatingFlag;

bool Armed;

uint16_t GainInADC[3];  // ADC result

#ifdef TRI_COPTER
int16_t GainAttenuator;
uint16_t ServoPPMRateDivider = 0;
int8_t RxInYawAdjust;
#endif

volatile uint16_t RxChannel1;    // ISR vars
volatile uint16_t RxChannel2;
volatile uint16_t RxChannel3;
volatile uint16_t RxChannel4;

uint16_t RxChannel1Start;        // ISR vars (could be local static)
uint16_t RxChannel2Start;
uint16_t RxChannel3Start;
uint16_t RxChannel4Start;

int16_t RxInput[4]; // Imput pulse duration

int16_t gyroADC[3];              // Holds Gyro ADC's
int16_t gyroZero[3];  // used for calibrating Gyros on ground


bool output_motor_high;
uint16_t PWM_Low_Pulse_Interval;    // non-const version of PWM_LOW_PULSE_INTERVAL

int16_t MotorOut1;
int16_t MotorOut2;
int16_t MotorOut3;
int16_t MotorOut4;
#if defined(HEX_COPTER) || defined(Y6_COPTER)
int16_t MotorOut5;
int16_t MotorOut6;
#endif


#endif /* KKDRONE_H_ */