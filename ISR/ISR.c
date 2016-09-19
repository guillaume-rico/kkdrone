/*
 * ISR.c
 *
 * Created: 02/02/2013 19:40:42
 * Author: Alliaume Rico
 * TCNT1 work at 1MHz
 * RxChannelX is between 1000 and 2000
 */ 

#include "ISR.h"
 
/*
* \fn ISR(PCINT2_vect)
* \brief Started on edge detection on RX_ROLL
*/
ISR(PCINT2_vect)
{

  if ( RX_ROLL )    // rising
  {
    RxChannel1Start = TCNT1;
  } else {        // falling
    RxChannelsUpdatingFlag = 1;
    RxChannel1 = TCNT1 - RxChannel1Start;
    RxChannelsUpdatingFlag = 0;
  }
}

/*
* \fn ISR(INT0_vect)
* \brief Started on edge detection on RX_PITCH
*/
ISR(INT0_vect)
{
  if (RX_PITCH)    
  {
    RxChannel2Start = TCNT1;

  } else {        // falling
    RxChannelsUpdatingFlag = 1;
    RxChannel2 = TCNT1 - RxChannel2Start;
    RxChannelsUpdatingFlag = 0;
  }
}

/*
* \fn ISR(INT1_vect)
* \brief Started on edge detection on RX_COLL
*/
ISR(INT1_vect)
{
  if (RX_COLL)    
  {
    RxChannel3Start = TCNT1;

  } else {        // falling
    RxChannelsUpdatingFlag = 1;
    RxChannel3 = TCNT1 - RxChannel3Start;
    RxChannelsUpdatingFlag = 0;
  }
}

/*
* \fn ISR(PCINT0_vect)
* \brief Started on edge detection on RX_YAW
*/
ISR(PCINT0_vect)
{

  if ( RX_YAW )    // rising
  {
    RxChannel4Start = TCNT1;

  } else {        // falling
    RxChannelsUpdatingFlag = 1;
    RxChannel4 = TCNT1 - RxChannel4Start;
    RxChannelsUpdatingFlag = 0;
  }
}
