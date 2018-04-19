/*!
 * \file      main.c
 *
 * \brief     LoRaMac classA device implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */

/*! \file classA/LoRaMote/main.c */

#include "utilities.h"
#include "board.h"
#include "gpio.h"
#include "gps.h"
#include "timer.h"

static bool gps_enabled = false;

/*!
 * Timer to handle gps state
 */
static TimerEvent_t GpsTimer;

/*!
 * LED GPIO pins objects
 */
extern Gpio_t Led2;


void OnGpsTimerEvent(void) {
  if (gps_enabled) {
    GpsStop();
    GpioWrite (&Led2, 1); 
    gps_enabled = false;
  } else {
    GpsStart();
    GpioWrite (&Led2, 0); 
    gps_enabled = true;
  }

  TimerReset(&GpsTimer);
}

/**
 * Main application entry point.
 */
int main( void )
{
    BoardInitMcu( );
    BoardInitPeriph( );
    
    if (gps_enabled) {
        GpsStart();
    } else {
        GpsStop();
    }

    TimerInit( &GpsTimer, OnGpsTimerEvent );
    TimerSetValue( &GpsTimer, 15000 );
    TimerStart( &GpsTimer );
       

    while( 1 ) { }
}
