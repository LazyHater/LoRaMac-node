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
#include "Commisioning.h"


#ifndef ACTIVE_REGION

#warning "No active region defined, LORAMAC_REGION_EU868 will be used as default."

#define ACTIVE_REGION LORAMAC_REGION_EU868

#endif

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            5000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * Default datarate
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_0

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1

#if defined( REGION_EU868 )

#include "LoRaMacTest.h"

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        false

#endif

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            2

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;
static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;

/*!
 * Device address
 */
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;


/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize = 16;
static uint8_t AppDataSizeBackup = 16;
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           242

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxNextPacketTimer;

/*!
 * Specifies the state of the application LED
 */
static bool AppLedStateOn = false;

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
static bool NextTx = true;

/*!
 * Device states
 */
static enum eDeviceState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
} DeviceState;


/*!
 * LED GPIO pins objects
 */
extern Gpio_t Led1; // red
extern Gpio_t Led2; // green
extern Gpio_t Led3; // orange

void onPacketReveiveEvent (uint8_t port, uint8_t* data, uint8_t data_len);

/*!
 * \brief   Prepares the payload of the frame
 */
static void PrepareTxFrame( uint8_t port )
{
    const LoRaMacRegion_t region = ACTIVE_REGION;

    switch( port )
    {
    case 2:
        switch( region )
        {
            case LORAMAC_REGION_CN779:
            case LORAMAC_REGION_EU868:
            case LORAMAC_REGION_IN865:
            case LORAMAC_REGION_KR920:
            {
                uint16_t pressure = 0;
                int16_t altitudeBar = 0;
                int16_t temperature = 0;
                int32_t latitude, longitude = 0;
                int16_t altitudeGps = 0xFFFF;
                uint8_t batteryLevel = 0;

                pressure = ( uint16_t )( MPL3115ReadPressure( ) / 10 );             // in hPa / 10
                temperature = ( int16_t )( MPL3115ReadTemperature( ) * 100 );       // in °C * 100
                altitudeBar = ( int16_t )( MPL3115ReadAltitude( ) * 10 );           // in m * 10
                batteryLevel = BoardGetBatteryLevel( );                             // 1 (very low) to 254 (fully charged)
                GpsGetLatestGpsPositionBinary( &latitude, &longitude );
                altitudeGps = GpsGetLatestGpsAltitude( );                           // in m

                AppDataSizeBackup = AppDataSize = 16;
                AppData[0] = AppLedStateOn;
                AppData[1] = ( pressure >> 8 ) & 0xFF;
                AppData[2] = pressure & 0xFF;
                AppData[3] = ( temperature >> 8 ) & 0xFF;
                AppData[4] = temperature & 0xFF;
                AppData[5] = ( altitudeBar >> 8 ) & 0xFF;
                AppData[6] = altitudeBar & 0xFF;
                AppData[7] = batteryLevel;
                AppData[8] = ( latitude >> 16 ) & 0xFF;
                AppData[9] = ( latitude >> 8 ) & 0xFF;
                AppData[10] = latitude & 0xFF;
                AppData[11] = ( longitude >> 16 ) & 0xFF;
                AppData[12] = ( longitude >> 8 ) & 0xFF;
                AppData[13] = longitude & 0xFF;
                AppData[14] = ( altitudeGps >> 8 ) & 0xFF;
                AppData[15] = altitudeGps & 0xFF;
                break;
            }
            case LORAMAC_REGION_AS923:
            case LORAMAC_REGION_AU915:
            case LORAMAC_REGION_US915:
            case LORAMAC_REGION_US915_HYBRID:
            
            default:
                // Unsupported region.
                break;
        }
        break;
    default:
        break;
    }
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool SendFrame( void )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
    else
    {
        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    }

    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return false;
    }
    return true;
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    TimerStop( &TxNextPacketTimer );

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else
        {
            // Network not joined yet. Try to join again
            MlmeReq_t mlmeReq;
            mlmeReq.Type = MLME_JOIN;
            mlmeReq.Req.Join.DevEui = DevEui;
            mlmeReq.Req.Join.AppEui = AppEui;
            mlmeReq.Req.Join.AppKey = AppKey;
            mlmeReq.Req.Join.Datarate = LORAWAN_DEFAULT_DATARATE;

            if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
            {
                DeviceState = DEVICE_STATE_SLEEP;
            }
            else
            {
                DeviceState = DEVICE_STATE_CYCLE;
            }
        }
    }
}

/*!
 * \brief Function executed on Led 1 Timeout event
 */
static void OnLed1TimerEvent( void )
{
    TimerStop( &Led1Timer );
    // Switch LED 1 OFF
    GpioWrite( &Led1, 1 );
}

/*!
 * \brief Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent( void )
{
    TimerStop( &Led2Timer );
    // Switch LED 2 OFF
    GpioWrite( &Led2, 1 );
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }

        // Switch LED 1 ON
        GpioWrite( &Led1, 0 );
        TimerStart( &Led1Timer );
    }
    NextTx = true;
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            break;
        }
        case MCPS_CONFIRMED:
        {
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    if( mcpsIndication->FramePending == true )
    {
        // The server signals that it has pending data to be sent.
        // We schedule an uplink as soon as possible to flush the server.
        OnTxNextPacketTimerEvent( );
    }
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot

    if( mcpsIndication->RxData == true )
    {
        onPacketReveiveEvent(mcpsIndication->Port, mcpsIndication->Buffer, mcpsIndication->BufferSize );
    }

    // Switch LED 2 ON for each received downlink
    GpioWrite( &Led2, 0 );
    TimerStart( &Led2Timer );
}



/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    switch( mlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Status is OK, node has joined the network
                DeviceState = DEVICE_STATE_SEND;
            }
            else
            {
                // Join was not successful. Try to join again
                MlmeReq_t mlmeReq;
                mlmeReq.Type = MLME_JOIN;
                mlmeReq.Req.Join.DevEui = DevEui;
                mlmeReq.Req.Join.AppEui = AppEui;
                mlmeReq.Req.Join.AppKey = AppKey;
                mlmeReq.Req.Join.Datarate = LORAWAN_DEFAULT_DATARATE;

                if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
                {
                    DeviceState = DEVICE_STATE_SLEEP;
                }
                else
                {
                    DeviceState = DEVICE_STATE_CYCLE;
                }
            }
            break;
        }
        case MLME_LINK_CHECK:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
 
            }
            break;
        }
        default:
            break;
    }
    NextTx = true;
}

/*!
 * \brief   MLME-Indication event function
 *
 * \param   [IN] mlmeIndication - Pointer to the indication structure.
 */
static void MlmeIndication( MlmeIndication_t *mlmeIndication )
{
    switch( mlmeIndication->MlmeIndication )
    {
        case MLME_SCHEDULE_UPLINK:
        {// The MAC signals that we shall provide an uplink as soon as possible
            OnTxNextPacketTimerEvent( );
            break;
        }
        default:
            break;
    }
}

void onPacketReveiveEvent (uint8_t port, uint8_t* data, uint8_t data_len) {
 if( data_len == 1 )
    {
        AppLedStateOn = data[0] & 0x01;
        if ( AppLedStateOn & 0x01 ) {
            GpioWrite( &Led3,  0 );
        } else {
            GpioWrite( &Led3,  1 );
        }
    }
}

/**
 * Main application entry point.
 */
int main( void )
{
    LoRaMacPrimitives_t LoRaMacPrimitives;
    LoRaMacCallback_t LoRaMacCallbacks;
    MibRequestConfirm_t mibReq;

    BoardInitMcu( );
    BoardInitPeriph( );

    DeviceState = DEVICE_STATE_INIT;
     
    while( 1 )
    {
        switch( DeviceState )
        {
            case DEVICE_STATE_INIT:
            {
                LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
                LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
                LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
                LoRaMacPrimitives.MacMlmeIndication = MlmeIndication;
                LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, ACTIVE_REGION );

                TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );

                TimerInit( &Led1Timer, OnLed1TimerEvent );
                TimerSetValue( &Led1Timer, 1000 );

                TimerInit( &Led2Timer, OnLed2TimerEvent );
                TimerSetValue( &Led2Timer, 1000 );

                mibReq.Type = MIB_ADR;
                mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_PUBLIC_NETWORK;
                mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
                LoRaMacMibSetRequestConfirm( &mibReq );

                LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON ); // important line to get 5sec interval

                DeviceState = DEVICE_STATE_JOIN;
                break;
            }
            case DEVICE_STATE_JOIN:
            {
                mibReq.Type = MIB_NET_ID;
                mibReq.Param.NetID = LORAWAN_NETWORK_ID;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_DEV_ADDR;
                mibReq.Param.DevAddr = DevAddr;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NWK_SKEY;
                mibReq.Param.NwkSKey = NwkSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_APP_SKEY;
                mibReq.Param.AppSKey = AppSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NETWORK_JOINED;
                mibReq.Param.IsNetworkJoined = true;
                LoRaMacMibSetRequestConfirm( &mibReq );

                DeviceState = DEVICE_STATE_SEND;
                break;
            }
            case DEVICE_STATE_SEND:
            {
                if( NextTx == true )
                {
                    PrepareTxFrame( AppPort );

                    NextTx = SendFrame( );
                }

                // Schedule next packet transmission
                TxDutyCycleTime = 5000 + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
                
                DeviceState = DEVICE_STATE_CYCLE;
                break;
            }
            case DEVICE_STATE_CYCLE:
            {
                DeviceState = DEVICE_STATE_SLEEP;

                // Schedule next packet transmission
                TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime );
                TimerStart( &TxNextPacketTimer );
                break;
            }
            case DEVICE_STATE_SLEEP:
            {
                // Wake up through events
                TimerLowPowerHandler( );
                break;
            }
            default:
            {
                DeviceState = DEVICE_STATE_INIT;
                break;
            }
        }
    }
}
