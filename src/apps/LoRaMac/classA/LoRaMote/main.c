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
#include "delay.h"

/*!
 * Timer to handle the state of LED1
 */
static TimerEvent_t Led1Timer;

/*!
 * Timer to handle the state of LED2
 */
static TimerEvent_t Led2Timer;

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
    GpioToggle( &Led2);
}

void Empty(void)
{
   GpioWrite (&Led3, 0);   
}
    Gpio_t Gpio_t_Ext3;

/**
 * Main application entry point.
 */
int main( void )
{
    BoardInitMcu( );
    BoardInitPeriph( );

    GpioWrite( &Led1, 1 );
    GpioWrite( &Led2, 1 );
    GpioWrite( &Led3, 1 );

    TimerInit( &Led1Timer, OnLed1TimerEvent );
    TimerSetValue( &Led1Timer, 1000 );

    TimerInit( &Led2Timer, OnLed2TimerEvent );
    TimerSetValue( &Led2Timer, 300 );
    
    TimerStart( &Led1Timer );
    TimerStart( &Led2Timer );

    Gpio_t_Ext3.pin = IOE_3;
    Gpio_t_Ext3.pinIndex = 1<<3;

    GpioInit( &Gpio_t_Ext3, IOE_3, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioSetInterrupt(&Gpio_t_Ext3,  IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, Empty);
       

    while( 1 ) { 
      //GpioWrite( &Led3, GpioRead(&Gpio_t_Ext3));
    // DelayMs(100);
    }
}
