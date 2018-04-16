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
#include "mpl3115.h"
#include "LoRaMac.h"

/*!
 * Timer to handle the state of LED1
 */
static TimerEvent_t Led1Timer;

/*!
 * Timer to handle the state of LED2
 */
static TimerEvent_t Led2Timer;

/*!
 * Timer to handle the state of LED3
 */
static TimerEvent_t Led3Timer;

/*!
 * Indicates if a new packet can be sent
 */
/*!
 * LED GPIO pins objects
 */
extern Gpio_t Led1;
extern Gpio_t Led2;
extern Gpio_t Led3;

/*!
 * \brief Function executed on Led 1 Timeout event
 */
static void OnLed1TimerEvent( void )
{
    //TimerStop( &Led1Timer );
    TimerReset(&Led1Timer);
    // Switch LED 1 OFF
    GpioWrite( &Led1, GpioRead(&Led1) ^ 1);
}

/*!
 * \brief Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent( void )
{
    //TimerStop( &Led2Timer );
    
    TimerReset(&Led2Timer);
    GpioWrite( &Led2, GpioRead(&Led2) ^ 1);
}

/*!
 * \brief Function executed on Led 2 Timeout event
 */
static void OnLed3TimerEvent( void )
{
    TimerReset(&Led3Timer);
    GpioWrite( &Led3, GpioRead(&Led3) ^ 1);
}

/**
 * Main application entry point.
 */
int main( void )
{
    BoardInitMcu( );
    BoardInitPeriph( );

    GpioWrite( &Led1, 0 );
    GpioWrite( &Led2, 0 );
    GpioWrite( &Led3, 0 );


    TimerInit( &Led1Timer, OnLed1TimerEvent );
    TimerSetValue( &Led1Timer, 1000 );

    TimerInit( &Led2Timer, OnLed2TimerEvent );
    TimerSetValue( &Led2Timer, 300 );
    
    TimerInit( &Led3Timer, OnLed3TimerEvent );
    TimerSetValue( &Led3Timer, 25 );

    
    TimerStart( &Led1Timer );
    TimerStart( &Led2Timer );
    TimerStart( &Led3Timer );


    while( 1 )
    {
       
    }
}
