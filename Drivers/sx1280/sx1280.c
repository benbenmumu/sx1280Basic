#include "sx1280.h"
#include "sx1280-hal.h"
#include "rangingcorrection.h"

/*!
 * \brief Radio registers definition
 *
 */
typedef struct
{
    uint16_t      Addr;                             //!< The address of the register
    uint8_t       Value;                            //!< The value of the register
}RadioRegisters_t;

/*!
 * \brief Radio hardware registers initialization definition
 */
// { Address, RegValue }
#define RADIO_INIT_REGISTERS_VALUE  { NULL }

/*!
 * \brief Radio hardware registers initialization
 */
const RadioRegisters_t RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;

/*!
 * \brief Holds the internal operating mode of the radio
 */
static SX1280_RadioOperatingModes_t OperatingMode;

/*!
 * \brief Stores the current packet type set in the radio
 */
static SX1280_RadioPacketTypes_t PacketType;

/*!
 * \brief Stores the current LoRa bandwidth set in the radio
 */
static SX1280_RadioLoRaBandwidths_t LoRaBandwidth;

/*!
 * \brief Holds the polling state of the driver
 */
static bool PollingMode;

/*!
 * Hardware DIO IRQ callback initialization
 */
SX1280DioIrqHandler *DioIrq[] = { SX1280OnDioIrq };

void SX1280OnDioIrq( void );

/*!
 * \brief Holds a flag raised on radio interrupt
 */
static bool IrqState;

static SX1280_RadioCallbacks_t* RadioCallbacks;

int32_t SX1280complement2( const uint32_t num, const uint8_t bitCnt )
{
    int32_t retVal = ( int32_t )num;
    if( num >= 2<<( bitCnt - 2 ) )
    {
        retVal -= 2<<( bitCnt - 1 );
    }
    return retVal;
}

void SX1280Init( SX1280_RadioCallbacks_t *callbacks )
{
    RadioCallbacks = callbacks;

    SX1280HalInit( DioIrq );
}

void SX1280SetRegistersDefault( void )
{
    for( int16_t i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
    {
        SX1280HalWriteRegister( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }
}

uint16_t SX1280GetFirmwareVersion( void )
{
    return( ( ( SX1280HalReadRegister( SX1280_REG_LR_FIRMWARE_VERSION_MSB ) ) << 8 ) | ( SX1280HalReadRegister( SX1280_REG_LR_FIRMWARE_VERSION_MSB + 1 ) ) );
}

SX1280_RadioStatus_t SX1280GetStatus( void )
{
    uint8_t stat = 0;
    SX1280_RadioStatus_t status;

    SX1280HalReadCommand( SX1280_RADIO_GET_STATUS, ( uint8_t * )&stat, 1 );
    status.Value = stat;
    return status;
}

SX1280_RadioOperatingModes_t SX1280GetOpMode( void )
{
    return OperatingMode;
}

void SX1280SetSleep( SX1280_SleepParams_t sleepConfig )
{
    uint8_t sleep = ( sleepConfig.WakeUpRTC << 3 ) |
                    ( sleepConfig.InstructionRamRetention << 2 ) |
                    ( sleepConfig.DataBufferRetention << 1 ) |
                    ( sleepConfig.DataRamRetention );

    OperatingMode = SX1280_MODE_SLEEP;
    SX1280HalWriteCommand( SX1280_RADIO_SET_SLEEP, &sleep, 1 );
}

void SX1280SetStandby( SX1280_RadioStandbyModes_t standbyConfig )
{
    SX1280HalWriteCommand( SX1280_RADIO_SET_STANDBY, ( uint8_t* )&standbyConfig, 1 );
    if( standbyConfig == SX1280_STDBY_RC )
    {
        OperatingMode = SX1280_MODE_STDBY_RC;
    }
    else
    {
        OperatingMode = SX1280_MODE_STDBY_XOSC;
    }
}

void SX1280SetFs( void )
{
    SX1280HalWriteCommand( SX1280_RADIO_SET_FS, 0, 0 );
    OperatingMode = SX1280_MODE_FS;
}

void SX1280SetTx( SX1280_TickTime_t timeout )
{
    uint8_t buf[3];
    buf[0] = timeout.Step;
    buf[1] = ( uint8_t )( ( timeout.NbSteps >> 8 ) & 0x00FF );
    buf[2] = ( uint8_t )( timeout.NbSteps & 0x00FF );

    SX1280ClearIrqStatus( SX1280_IRQ_RADIO_ALL );

    // If the radio is doing ranging operations, then apply the specific calls
    // prior to SetTx
    if( SX1280GetPacketType( ) == SX1280_PACKET_TYPE_RANGING )
    {
        SX1280SetRangingRole( SX1280_RADIO_RANGING_ROLE_MASTER );
    }
    SX1280HalWriteCommand( SX1280_RADIO_SET_TX, buf, 3 );
    OperatingMode = SX1280_MODE_TX;
}

void SX1280SetRx( SX1280_TickTime_t timeout )
{
    uint8_t buf[3];
    buf[0] = timeout.Step;
    buf[1] = ( uint8_t )( ( timeout.NbSteps >> 8 ) & 0x00FF );
    buf[2] = ( uint8_t )( timeout.NbSteps & 0x00FF );

    SX1280ClearIrqStatus( SX1280_IRQ_RADIO_ALL );

    // If the radio is doing ranging operations, then apply the specific calls
    // prior to SetRx
    if( SX1280GetPacketType( ) == SX1280_PACKET_TYPE_RANGING )
    {
        SX1280SetRangingRole( SX1280_RADIO_RANGING_ROLE_SLAVE );
    }
    SX1280HalWriteCommand( SX1280_RADIO_SET_RX, buf, 3 );
    OperatingMode = SX1280_MODE_RX;
}

void SX1280SetRxDutyCycle( SX1280_RadioTickSizes_t Step, uint16_t NbStepRx, uint16_t RxNbStepSleep )
{
    uint8_t buf[5];

    buf[0] = Step;
    buf[1] = ( uint8_t )( ( NbStepRx >> 8 ) & 0x00FF );
    buf[2] = ( uint8_t )( NbStepRx & 0x00FF );
    buf[3] = ( uint8_t )( ( RxNbStepSleep >> 8 ) & 0x00FF );
    buf[4] = ( uint8_t )( RxNbStepSleep & 0x00FF );
    SX1280HalWriteCommand( SX1280_RADIO_SET_RXDUTYCYCLE, buf, 5 );
    OperatingMode = SX1280_MODE_RX;
}

void SX1280SetCad( void )
{
    SX1280HalWriteCommand( SX1280_RADIO_SET_CAD, 0, 0 );
    OperatingMode = SX1280_MODE_CAD;
}

void SX1280SetTxContinuousWave( void )
{
    SX1280HalWriteCommand( SX1280_RADIO_SET_TXCONTINUOUSWAVE, 0, 0 );
}

void SX1280SetTxContinuousPreamble( void )
{
    SX1280HalWriteCommand( SX1280_RADIO_SET_TXCONTINUOUSPREAMBLE, 0, 0 );
}

void SX1280SetPacketType( SX1280_RadioPacketTypes_t packetType )
{
    // Save packet type internally to avoid questioning the radio
    PacketType = packetType;

    SX1280HalWriteCommand( SX1280_RADIO_SET_PACKETTYPE, ( uint8_t* )&packetType, 1 );
}

SX1280_RadioPacketTypes_t SX1280GetPacketType( void )
{
    return PacketType;
}

void SX1280SetRfFrequency( uint32_t frequency )
{
    uint8_t buf[3];
    uint32_t freq = 0;

    freq = ( uint32_t )( ( double )frequency / ( double )SX1280_FREQ_STEP );
    buf[0] = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    buf[1] = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    buf[2] = ( uint8_t )( freq & 0xFF );
    SX1280HalWriteCommand( SX1280_RADIO_SET_RFFREQUENCY, buf, 3 );
}

void SX1280SetTxParams( int8_t power, SX1280_RadioRampTimes_t rampTime )
{
    uint8_t buf[2];

    // The power value to send on SPI/UART is in the range [0..31] and the
    // physical output power is in the range [-18..13]dBm
    buf[0] = power + 18;
    buf[1] = ( uint8_t )rampTime;
    SX1280HalWriteCommand( SX1280_RADIO_SET_TXPARAMS, buf, 2 );
}

void SX1280SetCadParams( SX1280_RadioLoRaCadSymbols_t cadSymbolNum )
{
    SX1280HalWriteCommand( SX1280_RADIO_SET_CADPARAMS, ( uint8_t* )&cadSymbolNum, 1 );
    OperatingMode = SX1280_MODE_CAD;
}

void SX1280SetBufferBaseAddresses( uint8_t txBaseAddress, uint8_t rxBaseAddress )
{
    uint8_t buf[2];

    buf[0] = txBaseAddress;
    buf[1] = rxBaseAddress;
    SX1280HalWriteCommand( SX1280_RADIO_SET_BUFFERBASEADDRESS, buf, 2 );
}

void SX1280SetModulationParams( SX1280_ModulationParams_t *modulationParams )
{
    uint8_t buf[3];

    // Check if required configuration corresponds to the stored packet type
    // If not, silently update radio packet type
    if( PacketType != modulationParams->PacketType )
    {
        SX1280SetPacketType( modulationParams->PacketType );
    }

    switch( modulationParams->PacketType )
    {
        case SX1280_PACKET_TYPE_GFSK:
            buf[0] = modulationParams->Params.Gfsk.BitrateBandwidth;
            buf[1] = modulationParams->Params.Gfsk.ModulationIndex;
            buf[2] = modulationParams->Params.Gfsk.ModulationShaping;
            break;

        case SX1280_PACKET_TYPE_LORA:
        case SX1280_PACKET_TYPE_RANGING:
            buf[0] = modulationParams->Params.LoRa.SpreadingFactor;
            buf[1] = modulationParams->Params.LoRa.Bandwidth;
            buf[2] = modulationParams->Params.LoRa.CodingRate;
            LoRaBandwidth = modulationParams->Params.LoRa.Bandwidth;
            break;

        case SX1280_PACKET_TYPE_FLRC:
            buf[0] = modulationParams->Params.Flrc.BitrateBandwidth;
            buf[1] = modulationParams->Params.Flrc.CodingRate;
            buf[2] = modulationParams->Params.Flrc.ModulationShaping;
            break;

        case SX1280_PACKET_TYPE_BLE:
            buf[0] = modulationParams->Params.Ble.BitrateBandwidth;
            buf[1] = modulationParams->Params.Ble.ModulationIndex;
            buf[2] = modulationParams->Params.Ble.ModulationShaping;
            break;

        case SX1280_PACKET_TYPE_NONE:
            buf[0] = NULL;
            buf[1] = NULL;
            buf[2] = NULL;
            break;
    }
    SX1280HalWriteCommand( SX1280_RADIO_SET_MODULATIONPARAMS, buf, 3 );
}

void SX1280SetPacketParams( SX1280_PacketParams_t *packetParams )
{
    uint8_t buf[7];

    // Check if required configuration corresponds to the stored packet type
    // If not, silently update radio packet type
    if( PacketType != packetParams->PacketType )
    {
        SX1280SetPacketType( packetParams->PacketType );
    }

    switch( packetParams->PacketType )
    {
        case SX1280_PACKET_TYPE_GFSK:
            buf[0] = packetParams->Params.Gfsk.PreambleLength;
            buf[1] = packetParams->Params.Gfsk.SyncWordLength;
            buf[2] = packetParams->Params.Gfsk.SyncWordMatch;
            buf[3] = packetParams->Params.Gfsk.HeaderType;
            buf[4] = packetParams->Params.Gfsk.PayloadLength;
            buf[5] = packetParams->Params.Gfsk.CrcLength;
            buf[6] = packetParams->Params.Gfsk.Whitening;
            break;

        case SX1280_PACKET_TYPE_LORA:
        case SX1280_PACKET_TYPE_RANGING:
            buf[0] = packetParams->Params.LoRa.PreambleLength;
            buf[1] = packetParams->Params.LoRa.HeaderType;
            buf[2] = packetParams->Params.LoRa.PayloadLength;
            buf[3] = packetParams->Params.LoRa.CrcMode;
            buf[4] = packetParams->Params.LoRa.InvertIQ;
            buf[5] = NULL;
            buf[6] = NULL;
            break;

        case SX1280_PACKET_TYPE_FLRC:
            buf[0] = packetParams->Params.Flrc.PreambleLength;
            buf[1] = packetParams->Params.Flrc.SyncWordLength;
            buf[2] = packetParams->Params.Flrc.SyncWordMatch;
            buf[3] = packetParams->Params.Flrc.HeaderType;
            buf[4] = packetParams->Params.Flrc.PayloadLength;
            buf[5] = packetParams->Params.Flrc.CrcLength;
            buf[6] = packetParams->Params.Flrc.Whitening;
            break;

        case SX1280_PACKET_TYPE_BLE:
            buf[0] = packetParams->Params.Ble.ConnectionState;
            buf[1] = packetParams->Params.Ble.CrcField;
            buf[2] = packetParams->Params.Ble.BlePacketType;
            buf[3] = packetParams->Params.Ble.Whitening;
            buf[4] = NULL;
            buf[5] = NULL;
            buf[6] = NULL;
            break;

        case SX1280_PACKET_TYPE_NONE:
            buf[0] = NULL;
            buf[1] = NULL;
            buf[2] = NULL;
            buf[3] = NULL;
            buf[4] = NULL;
            buf[5] = NULL;
            buf[6] = NULL;
            break;
    }
    SX1280HalWriteCommand( SX1280_RADIO_SET_PACKETPARAMS, buf, 7 );
}

void SX1280GetRxBufferStatus( uint8_t *payloadLength, uint8_t *rxStartBufferPointer )
{
    uint8_t status[2];

    SX1280HalReadCommand( SX1280_RADIO_GET_RXBUFFERSTATUS, status, 2 );

    // In case of LORA fixed header, the payloadLength is obtained by reading
    // the register REG_LR_PAYLOADLENGTH
    if( ( SX1280GetPacketType( ) == SX1280_PACKET_TYPE_LORA ) && ( SX1280HalReadRegister( SX1280_REG_LR_PACKETPARAMS ) >> 7 == 1 ) )
    {
        *payloadLength = SX1280HalReadRegister( SX1280_REG_LR_PAYLOADLENGTH );
    }
    else if( SX1280GetPacketType( ) == SX1280_PACKET_TYPE_BLE )
    {
        // In the case of BLE, the size returned in status[0] do not include the 2-byte length PDU header
        // so it is added there
        *payloadLength = status[0] + 2;
    }
    else
    {
        *payloadLength = status[0];
    }

    *rxStartBufferPointer = status[1];
}

void SX1280GetPacketStatus( SX1280_PacketStatus_t *pktStatus )
{
    uint8_t status[5];

    SX1280HalReadCommand( SX1280_RADIO_GET_PACKETSTATUS, status, 5 );

    pktStatus->packetType = SX1280GetPacketType( );
    switch( pktStatus->packetType )
    {
        case SX1280_PACKET_TYPE_GFSK:
            pktStatus->Params.Gfsk.RssiAvg = -status[0] / 2;
            pktStatus->Params.Gfsk.RssiSync = -status[1] / 2;

            pktStatus->Params.Gfsk.ErrorStatus.SyncError = ( status[2] >> 6 ) & 0x01;
            pktStatus->Params.Gfsk.ErrorStatus.LengthError = ( status[2] >> 5 ) & 0x01;
            pktStatus->Params.Gfsk.ErrorStatus.CrcError = ( status[2] >> 4 ) & 0x01;
            pktStatus->Params.Gfsk.ErrorStatus.AbortError = ( status[2] >> 3 ) & 0x01;
            pktStatus->Params.Gfsk.ErrorStatus.HeaderReceived = ( status[2] >> 2 ) & 0x01;
            pktStatus->Params.Gfsk.ErrorStatus.PacketReceived = ( status[2] >> 1 ) & 0x01;
            pktStatus->Params.Gfsk.ErrorStatus.PacketControlerBusy = status[2] & 0x01;

            pktStatus->Params.Gfsk.TxRxStatus.RxNoAck = ( status[3] >> 5 ) & 0x01;
            pktStatus->Params.Gfsk.TxRxStatus.PacketSent = status[3] & 0x01;

            pktStatus->Params.Gfsk.SyncAddrStatus = status[4] & 0x07;
            break;

        case SX1280_PACKET_TYPE_LORA:
        case SX1280_PACKET_TYPE_RANGING:
            pktStatus->Params.LoRa.RssiPkt = -status[0] / 2;
            ( status[1] < 128 ) ? ( pktStatus->Params.LoRa.SnrPkt = status[1] / 4 ) : ( pktStatus->Params.LoRa.SnrPkt = ( ( status[1] - 256 ) /4 ) );

            pktStatus->Params.LoRa.ErrorStatus.SyncError = ( status[2] >> 6 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.LengthError = ( status[2] >> 5 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.CrcError = ( status[2] >> 4 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.AbortError = ( status[2] >> 3 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.HeaderReceived = ( status[2] >> 2 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.PacketReceived = ( status[2] >> 1 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.PacketControlerBusy = status[2] & 0x01;

            pktStatus->Params.LoRa.TxRxStatus.RxNoAck = ( status[3] >> 5 ) & 0x01;
            pktStatus->Params.LoRa.TxRxStatus.PacketSent = status[3] & 0x01;

            pktStatus->Params.LoRa.SyncAddrStatus = status[4] & 0x07;
            break;

        case SX1280_PACKET_TYPE_FLRC:
            pktStatus->Params.Flrc.RssiAvg = -status[0] / 2;
            pktStatus->Params.Flrc.RssiSync = -status[1] / 2;

            pktStatus->Params.Flrc.ErrorStatus.SyncError = ( status[2] >> 6 ) & 0x01;
            pktStatus->Params.Flrc.ErrorStatus.LengthError = ( status[2] >> 5 ) & 0x01;
            pktStatus->Params.Flrc.ErrorStatus.CrcError = ( status[2] >> 4 ) & 0x01;
            pktStatus->Params.Flrc.ErrorStatus.AbortError = ( status[2] >> 3 ) & 0x01;
            pktStatus->Params.Flrc.ErrorStatus.HeaderReceived = ( status[2] >> 2 ) & 0x01;
            pktStatus->Params.Flrc.ErrorStatus.PacketReceived = ( status[2] >> 1 ) & 0x01;
            pktStatus->Params.Flrc.ErrorStatus.PacketControlerBusy = status[2] & 0x01;

            pktStatus->Params.Flrc.TxRxStatus.RxPid = ( status[3] >> 6 ) & 0x03;
            pktStatus->Params.Flrc.TxRxStatus.RxNoAck = ( status[3] >> 5 ) & 0x01;
            pktStatus->Params.Flrc.TxRxStatus.RxPidErr = ( status[3] >> 4 ) & 0x01;
            pktStatus->Params.Flrc.TxRxStatus.PacketSent = status[3] & 0x01;

            pktStatus->Params.Flrc.SyncAddrStatus = status[4] & 0x07;
            break;

        case SX1280_PACKET_TYPE_BLE:
            pktStatus->Params.Ble.RssiAvg = -status[0] / 2;
            pktStatus->Params.Ble.RssiSync = -status[1] / 2;

            pktStatus->Params.Ble.ErrorStatus.SyncError = ( status[2] >> 6 ) & 0x01;
            pktStatus->Params.Ble.ErrorStatus.LengthError = ( status[2] >> 5 ) & 0x01;
            pktStatus->Params.Ble.ErrorStatus.CrcError = ( status[2] >> 4 ) & 0x01;
            pktStatus->Params.Ble.ErrorStatus.AbortError = ( status[2] >> 3 ) & 0x01;
            pktStatus->Params.Ble.ErrorStatus.HeaderReceived = ( status[2] >> 2 ) & 0x01;
            pktStatus->Params.Ble.ErrorStatus.PacketReceived = ( status[2] >> 1 ) & 0x01;
            pktStatus->Params.Ble.ErrorStatus.PacketControlerBusy = status[2] & 0x01;

            pktStatus->Params.Ble.TxRxStatus.PacketSent = status[3] & 0x01;

            pktStatus->Params.Ble.SyncAddrStatus = status[4] & 0x07;
            break;

        case SX1280_PACKET_TYPE_NONE:
            // In that specific case, we set everything in the pktStatus to zeros
            // and reset the packet type accordingly
            memset( pktStatus, 0, sizeof( SX1280_PacketStatus_t ) );
            pktStatus->packetType = SX1280_PACKET_TYPE_NONE;
            break;
    }
}

uint8_t SX1280GetRssiInst( void )
{
    uint8_t raw = 0;

    SX1280HalReadCommand( SX1280_RADIO_GET_RSSIINST, &raw, 1 );

    // return ( int8_t )( -raw / 2 );
    return raw;
}

void SX1280SetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask )
{
    uint8_t buf[8];

    buf[0] = ( uint8_t )( ( irqMask >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( irqMask & 0x00FF );
    buf[2] = ( uint8_t )( ( dio1Mask >> 8 ) & 0x00FF );
    buf[3] = ( uint8_t )( dio1Mask & 0x00FF );
    buf[4] = ( uint8_t )( ( dio2Mask >> 8 ) & 0x00FF );
    buf[5] = ( uint8_t )( dio2Mask & 0x00FF );
    buf[6] = ( uint8_t )( ( dio3Mask >> 8 ) & 0x00FF );
    buf[7] = ( uint8_t )( dio3Mask & 0x00FF );
    SX1280HalWriteCommand( SX1280_RADIO_SET_DIOIRQPARAMS, buf, 8 );
}

uint16_t SX1280GetIrqStatus( void )
{
    uint8_t irqStatus[2];

    SX1280HalReadCommand( SX1280_RADIO_GET_IRQSTATUS, irqStatus, 2 );

    return ( irqStatus[0] << 8 ) | irqStatus[1];
}

void SX1280ClearIrqStatus( uint16_t irq )
{
    uint8_t buf[2];

    buf[0] = ( uint8_t )( ( ( uint16_t )irq >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( ( uint16_t )irq & 0x00FF );
    SX1280HalWriteCommand( SX1280_RADIO_CLR_IRQSTATUS, buf, 2 );
}

void SX1280Calibrate( SX1280_CalibrationParams_t calibParam )
{
    uint8_t cal = ( calibParam.ADCBulkPEnable << 5 ) |
                  ( calibParam.ADCBulkNEnable << 4 ) |
                  ( calibParam.ADCPulseEnable << 3 ) |
                  ( calibParam.PLLEnable << 2 ) |
                  ( calibParam.RC13MEnable << 1 ) |
                  ( calibParam.RC64KEnable );

    SX1280HalWriteCommand( SX1280_RADIO_CALIBRATE, &cal, 1 );
}

void SX1280SetRegulatorMode( SX1280_RadioRegulatorModes_t mode )
{
    SX1280HalWriteCommand( SX1280_RADIO_SET_REGULATORMODE, ( uint8_t* )&mode, 1 );
}

void SX1280SetSaveContext( void )
{
    SX1280HalWriteCommand( SX1280_RADIO_SET_SAVECONTEXT, 0, 0 );
}

void SX1280SetAutoTx( uint16_t time )
{
    uint16_t compensatedTime = time - ( uint16_t )SX1280_AUTO_RX_TX_OFFSET;
    uint8_t buf[2];

    buf[0] = ( uint8_t )( ( compensatedTime >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( compensatedTime & 0x00FF );
    SX1280HalWriteCommand( SX1280_RADIO_SET_AUTOTX, buf, 2 );
}

void SX1280StopAutoTx( void )
{
    uint8_t buf[2] = {0x00, 0x00};
    SX1280HalWriteCommand( SX1280_RADIO_SET_AUTOTX, buf, 2 );
}

void SX1280SetAutoFS( uint8_t enable )
{
    SX1280HalWriteCommand( SX1280_RADIO_SET_AUTOFS, &enable, 1 );
}

void SX1280SetLongPreamble( uint8_t enable )
{
    SX1280HalWriteCommand( SX1280_RADIO_SET_LONGPREAMBLE, &enable, 1 );
}

void SX1280SetPayload( uint8_t *buffer, uint8_t size )
{
    SX1280HalWriteBuffer( 0x00, buffer, size );
}

uint8_t SX1280GetPayload( uint8_t *buffer, uint8_t *size , uint8_t maxSize )
{
    uint8_t offset;

    SX1280GetRxBufferStatus( size, &offset );
    if( *size > maxSize )
    {
        return 1;
    }
    SX1280HalReadBuffer( offset, buffer, *size );
    return 0;
}

void SX1280SendPayload( uint8_t *payload, uint8_t size, SX1280_TickTime_t timeout )
{
    SX1280SetPayload( payload, size );
    SX1280SetTx( timeout );
}

uint8_t SX1280SetSyncWord( uint8_t syncWordIdx, uint8_t *syncWord )
{
    uint16_t addr;
    uint8_t syncwordSize = 0;

    switch( SX1280GetPacketType( ) )
    {
        case SX1280_PACKET_TYPE_GFSK:
            syncwordSize = 5;
            switch( syncWordIdx )
            {
                case 1:
                    addr = SX1280_REG_LR_SYNCWORDBASEADDRESS1;
                    break;

                case 2:
                    addr = SX1280_REG_LR_SYNCWORDBASEADDRESS2;
                    break;

                case 3:
                    addr = SX1280_REG_LR_SYNCWORDBASEADDRESS3;
                    break;

                default:
                    return 1;
            }
            break;

        case SX1280_PACKET_TYPE_FLRC:
            // For FLRC packet type, the SyncWord is one byte shorter and
            // the base address is shifted by one byte
            syncwordSize = 4;
            switch( syncWordIdx )
            {
                case 1:
                    addr = SX1280_REG_LR_SYNCWORDBASEADDRESS1 + 1;
                    break;

                case 2:
                    addr = SX1280_REG_LR_SYNCWORDBASEADDRESS2 + 1;
                    break;

                case 3:
                    addr = SX1280_REG_LR_SYNCWORDBASEADDRESS3 + 1;
                    break;

                default:
                    return 1;
            }
            break;

        case SX1280_PACKET_TYPE_BLE:
            // For Ble packet type, only the first SyncWord is used and its
            // address is shifted by one byte
            syncwordSize = 4;
            switch( syncWordIdx )
            {
                case 1:
                    addr = SX1280_REG_LR_SYNCWORDBASEADDRESS1 + 1;
                    break;

                default:
                    return 1;
            }
            break;

        default:
            return 1;
    }
    SX1280HalWriteRegisters( addr, syncWord, syncwordSize );
    return 0;
}

void SX1280SetSyncWordErrorTolerance( uint8_t ErrorBits )
{
    ErrorBits = ( SX1280HalReadRegister( SX1280_REG_LR_SYNCWORDTOLERANCE ) & 0xF0 ) | ( ErrorBits & 0x0F );
    SX1280HalWriteRegister( SX1280_REG_LR_SYNCWORDTOLERANCE, ErrorBits );
}

void SX1280SetCrcSeed( uint16_t seed )
{
    uint8_t val[2];

    val[0] = ( uint8_t )( seed >> 8 ) & 0xFF;
    val[1] = ( uint8_t )( seed  & 0xFF );

    switch( SX1280GetPacketType( ) )
    {
        case SX1280_PACKET_TYPE_GFSK:
        case SX1280_PACKET_TYPE_FLRC:
            SX1280HalWriteRegisters( SX1280_REG_LR_CRCSEEDBASEADDR, val, 2 );
            break;

        default:
            break;
    }
}

void SX1280SetBleAccessAddress( uint32_t accessAddress )
{
    SX1280HalWriteRegister( SX1280_REG_LR_BLE_ACCESS_ADDRESS, ( accessAddress >> 24 ) & 0x000000FF );
    SX1280HalWriteRegister( SX1280_REG_LR_BLE_ACCESS_ADDRESS + 1, ( accessAddress >> 16 ) & 0x000000FF );
    SX1280HalWriteRegister( SX1280_REG_LR_BLE_ACCESS_ADDRESS + 2, ( accessAddress >> 8 ) & 0x000000FF );
    SX1280HalWriteRegister( SX1280_REG_LR_BLE_ACCESS_ADDRESS + 3, accessAddress & 0x000000FF );
}

void SX1280SetBleAdvertizerAccessAddress( void )
{
    SX1280SetBleAccessAddress( SX1280_BLE_ADVERTIZER_ACCESS_ADDRESS );
}

void SX1280SetCrcPolynomial( uint16_t polynomial )
{
    uint8_t val[2];

    val[0] = ( uint8_t )( polynomial >> 8 ) & 0xFF;
    val[1] = ( uint8_t )( polynomial  & 0xFF );

    switch( SX1280GetPacketType( ) )
    {
        case SX1280_PACKET_TYPE_GFSK:
        case SX1280_PACKET_TYPE_FLRC:
            SX1280HalWriteRegisters( SX1280_REG_LR_CRCPOLYBASEADDR, val, 2 );
            break;

        default:
            break;
    }
}

void SX1280SetWhiteningSeed( uint8_t seed )
{
    switch( SX1280GetPacketType( ) )
    {
        case SX1280_PACKET_TYPE_GFSK:
        case SX1280_PACKET_TYPE_FLRC:
        case SX1280_PACKET_TYPE_BLE:
            SX1280HalWriteRegister( SX1280_REG_LR_WHITSEEDBASEADDR, seed );
            break;

        default:
            break;
    }
}

void SX1280EnableManualGain( void )
{
    SX1280HalWriteRegister( SX1280_REG_ENABLE_MANUAL_GAIN_CONTROL, SX1280HalReadRegister( SX1280_REG_ENABLE_MANUAL_GAIN_CONTROL ) | SX1280_MASK_MANUAL_GAIN_CONTROL );
    SX1280HalWriteRegister( SX1280_REG_DEMOD_DETECTION, SX1280HalReadRegister( SX1280_REG_DEMOD_DETECTION ) & SX1280_MASK_DEMOD_DETECTION );
}

void SX1280DisableManualGain( void )
{
    SX1280HalWriteRegister( SX1280_REG_ENABLE_MANUAL_GAIN_CONTROL, SX1280HalReadRegister( SX1280_REG_ENABLE_MANUAL_GAIN_CONTROL ) & ~SX1280_MASK_MANUAL_GAIN_CONTROL );
    SX1280HalWriteRegister( SX1280_REG_DEMOD_DETECTION, SX1280HalReadRegister( SX1280_REG_DEMOD_DETECTION ) | ~SX1280_MASK_DEMOD_DETECTION );
}

void SX1280SetManualGainValue( uint8_t gain )
{
    SX1280HalWriteRegister( SX1280_REG_MANUAL_GAIN_VALUE, ( SX1280HalReadRegister( SX1280_REG_MANUAL_GAIN_VALUE ) & SX1280_MASK_MANUAL_GAIN_VALUE ) | gain );
}

void SX1280SetLNAGainSetting( const SX1280_RadioLnaSettings_t lnaSetting )
{
    switch( lnaSetting )
    {
        case SX1280_LNA_HIGH_SENSITIVITY_MODE:
        {
            SX1280HalWriteRegister( SX1280_REG_LNA_REGIME, SX1280HalReadRegister( SX1280_REG_LNA_REGIME ) | SX1280_MASK_LNA_REGIME );
            break;
        }
        case SX1280_LNA_LOW_POWER_MODE:
        {
            SX1280HalWriteRegister( SX1280_REG_LNA_REGIME, SX1280HalReadRegister( SX1280_REG_LNA_REGIME ) & ~SX1280_MASK_LNA_REGIME );
            break;
        }
    }
}

void SX1280SetRangingIdLength( SX1280_RadioRangingIdCheckLengths_t length )
{
    switch( SX1280GetPacketType( ) )
    {
        case SX1280_PACKET_TYPE_RANGING:
            SX1280HalWriteRegister( SX1280_REG_LR_RANGINGIDCHECKLENGTH, ( ( ( ( uint8_t )length ) & 0x03 ) << 6 ) | ( SX1280HalReadRegister( SX1280_REG_LR_RANGINGIDCHECKLENGTH ) & 0x3F ) );
            break;

        default:
            break;
    }
}

void SX1280SetDeviceRangingAddress( uint32_t address )
{
    uint8_t addrArray[] = { address >> 24, address >> 16, address >> 8, address };

    switch( SX1280GetPacketType( ) )
    {
        case SX1280_PACKET_TYPE_RANGING:
            SX1280HalWriteRegisters( SX1280_REG_LR_DEVICERANGINGADDR, addrArray, 4 );
            break;

        default:
            break;
    }
}

void SX1280SetRangingRequestAddress( uint32_t address )
{
    uint8_t addrArray[] = { address >> 24, address >> 16, address >> 8, address };

    switch( SX1280GetPacketType( ) )
    {
        case SX1280_PACKET_TYPE_RANGING:
            SX1280HalWriteRegisters( SX1280_REG_LR_REQUESTRANGINGADDR, addrArray, 4 );
            break;

        default:
            break;
    }
}

double SX1280GetRangingResult( SX1280_RadioRangingResultTypes_t resultType )
{
    uint32_t valLsb = 0;
    double val = 0.0;

    switch( SX1280GetPacketType( ) )
    {
        case SX1280_PACKET_TYPE_RANGING:
            SX1280SetStandby( SX1280_STDBY_XOSC );
            SX1280HalWriteRegister( 0x97F, SX1280HalReadRegister( 0x97F ) | ( 1 << 1 ) ); // enable LORA modem clock
            SX1280HalWriteRegister( SX1280_REG_LR_RANGINGRESULTCONFIG, ( SX1280HalReadRegister( SX1280_REG_LR_RANGINGRESULTCONFIG ) & SX1280_MASK_RANGINGMUXSEL ) | ( ( ( ( uint8_t )resultType ) & 0x03 ) << 4 ) );
            valLsb = ( ( SX1280HalReadRegister( SX1280_REG_LR_RANGINGRESULTBASEADDR ) << 16 ) | ( SX1280HalReadRegister( SX1280_REG_LR_RANGINGRESULTBASEADDR + 1 ) << 8 ) | ( SX1280HalReadRegister( SX1280_REG_LR_RANGINGRESULTBASEADDR + 2 ) ) );
            SX1280SetStandby( SX1280_STDBY_RC );

            // Convertion from LSB to distance. For explanation on the formula, refer to Datasheet of SX1280
            switch( resultType )
            {
                case SX1280_RANGING_RESULT_RAW:
                    // Convert the ranging LSB to distance in meter
                    val = ( double )SX1280complement2( valLsb, 24 ) / ( double )SX1280GetLoRaBandwidth( ) * 36621.09375;
                    break;

                case SX1280_RANGING_RESULT_AVERAGED:
                case SX1280_RANGING_RESULT_DEBIASED:
                case SX1280_RANGING_RESULT_FILTERED:
                    val = ( double )valLsb * 20.0 / 100.0;
                    break;

                default:
                    val = 0.0;
            }
            break;

        default:
            break;
    }
    return val;
}

uint8_t SX1280GetRangingPowerDeltaThresholdIndicator( void )
{
    SX1280SetStandby( SX1280_STDBY_XOSC );
    SX1280HalWriteRegister( 0x97F, SX1280HalReadRegister( 0x97F ) | ( 1 << 1 ) ); // enable LoRa modem clock
    SX1280HalWriteRegister( SX1280_REG_LR_RANGINGRESULTCONFIG, ( SX1280HalReadRegister( SX1280_REG_LR_RANGINGRESULTCONFIG ) & SX1280_MASK_RANGINGMUXSEL ) | ( ( ( ( uint8_t )SX1280_RANGING_RESULT_RAW ) & 0x03 ) << 4 ) ); // Select raw results
    return SX1280HalReadRegister( SX1280_REG_RANGING_RSSI );
}

void SX1280SetRangingCalibration( uint16_t cal )
{
    switch( SX1280GetPacketType( ) )
    {
        case SX1280_PACKET_TYPE_RANGING:
            SX1280HalWriteRegister( SX1280_REG_LR_RANGINGRERXTXDELAYCAL, ( uint8_t )( ( cal >> 8 ) & 0xFF ) );
            SX1280HalWriteRegister( SX1280_REG_LR_RANGINGRERXTXDELAYCAL + 1, ( uint8_t )( ( cal ) & 0xFF ) );
            break;

        default:
            break;
    }
}

void SX1280RangingClearFilterResult( void )
{
    uint8_t regVal = SX1280HalReadRegister( SX1280_REG_LR_RANGINGRESULTCLEARREG );

    // To clear result, set bit 5 to 1 then to 0
    SX1280HalWriteRegister( SX1280_REG_LR_RANGINGRESULTCLEARREG, regVal | ( 1 << 5 ) );
    SX1280HalWriteRegister( SX1280_REG_LR_RANGINGRESULTCLEARREG, regVal & ( ~( 1 << 5 ) ) );
}

void SX1280RangingSetFilterNumSamples( uint8_t num )
{
    // Silently set 8 as minimum value
    SX1280HalWriteRegister( SX1280_REG_LR_RANGINGFILTERWINDOWSIZE, ( num < SX1280_DEFAULT_RANGING_FILTER_SIZE ) ? SX1280_DEFAULT_RANGING_FILTER_SIZE : num );
}

int8_t SX1280ParseHexFileLine( char* line )
{
    uint16_t addr;
    uint16_t n;
    uint8_t code;
    uint8_t bytes[256];

    if( SX1280GetHexFileLineFields( line, bytes, &addr, &n, &code ) != 0 )
    {
        if( code == 0 )
        {
            SX1280HalWriteRegisters( addr, bytes, n );
        }
        if( code == 1 )
        { // end of file
            //return 2;
        }
        if( code == 2 )
        { // begin of file
            //return 3;
        }
    }
    else
    {
        return 0;
    }
    return 1;
}

void SX1280SetRangingRole( SX1280_RadioRangingRoles_t role )
{
    uint8_t buf[1];

    buf[0] = role;
    SX1280HalWriteCommand( SX1280_RADIO_SET_RANGING_ROLE, &buf[0], 1 );
}

int8_t SX1280GetHexFileLineFields( char* line, uint8_t *bytes, uint16_t *addr, uint16_t *num, uint8_t *code )
{
    uint16_t sum, len, cksum;
    char *ptr;

    *num = 0;
    if( line[0] != ':' )
    {
        return 0;
    }
    if( strlen( line ) < 11 )
    {
        return 0;
    }
    ptr = line + 1;
    if( !sscanf( ptr, "%02hx", &len ) )
    {
        return 0;
    }
    ptr += 2;
    if( strlen( line ) < ( 11 + ( len * 2 ) ) )
    {
        return 0;
    }
    if( !sscanf( ptr, "%04hx", addr ) )
    {
        return 0;
    }
    ptr += 4;
    if( !sscanf( ptr, "%02hhx", code ) )
    {
        return 0;
    }
    ptr += 2;
    sum = ( len & 255 ) + ( ( *addr >> 8 ) & 255 ) + ( *addr & 255 ) + ( ( *code >> 8 ) & 255 ) + ( *code & 255 );
    while( *num != len )
    {
        if( !sscanf( ptr, "%02hhx", &bytes[*num] ) )
        {
            return 0;
        }
        ptr += 2;
        sum += bytes[*num] & 255;
        ( *num )++;
        if( *num >= 256 )
        {
            return 0;
        }
    }
    if( !sscanf( ptr, "%02hx", &cksum ) )
    {
        return 0;
    }
    if( ( ( sum & 255 ) + ( cksum & 255 ) ) & 255 )
    {
        return 0; // checksum error
    }

    return 1;
}

double SX1280GetFrequencyError( )
{
    uint8_t efeRaw[3] = {0};
    uint32_t efe = 0;
    double efeHz = 0.0;

    switch( SX1280GetPacketType( ) )
    {
        case SX1280_PACKET_TYPE_LORA:
        case SX1280_PACKET_TYPE_RANGING:
            efeRaw[0] = SX1280HalReadRegister( SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB );
            efeRaw[1] = SX1280HalReadRegister( SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 1 );
            efeRaw[2] = SX1280HalReadRegister( SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 2 );
            efe = ( efeRaw[0]<<16 ) | ( efeRaw[1]<<8 ) | efeRaw[2];
            efe &= SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK;

            efeHz = 1.55 * ( double )SX1280complement2( efe, 20 ) / ( 1600.0 / ( double )SX1280GetLoRaBandwidth( ) * 1000.0 );
            break;

        case SX1280_PACKET_TYPE_NONE:
        case SX1280_PACKET_TYPE_BLE:
        case SX1280_PACKET_TYPE_FLRC:
        case SX1280_PACKET_TYPE_GFSK:
            break;
    }

    return efeHz;
}

void SX1280SetPollingMode( void )
{
    PollingMode = true;
}

int32_t SX1280GetLoRaBandwidth( )
{
    int32_t bwValue = 0;

    switch( LoRaBandwidth )
    {
        case SX1280_LORA_BW_0200:
            bwValue = 203125;
            break;

        case SX1280_LORA_BW_0400:
            bwValue = 406250;
            break;

        case SX1280_LORA_BW_0800:
            bwValue = 812500;
            break;

        case SX1280_LORA_BW_1600:
            bwValue = 1625000;
            break;

        default:
            bwValue = 0;
    }
    return bwValue;
}

double SX1280GetRangingCorrectionPerSfBwGain( const SX1280_RadioLoRaSpreadingFactors_t sf, const SX1280_RadioLoRaBandwidths_t bw, const int8_t gain){
    uint8_t sf_index, bw_index;
    
    switch(sf){
        case SX1280_LORA_SF5:
            sf_index = 0;
            break;
        case SX1280_LORA_SF6:
            sf_index = 1;
            break;
        case SX1280_LORA_SF7:
            sf_index = 2;
            break;
        case SX1280_LORA_SF8:
            sf_index = 3;
            break;
        case SX1280_LORA_SF9:
            sf_index = 4;
            break;
        case SX1280_LORA_SF10:
            sf_index = 5;
            break;
		case SX1280_LORA_SF11:
			sf_index = 6;
			break;
		case SX1280_LORA_SF12:
            sf_index = 7;
            break;
    }
    switch(bw){
        case SX1280_LORA_BW_0400:
            bw_index = 0;
            break;
        case SX1280_LORA_BW_0800:
            bw_index = 1;
            break;
        case SX1280_LORA_BW_1600:
            bw_index = 2;
            break;
    }
    
    double correction = RangingCorrectionPerSfBwGain[sf_index][bw_index][gain];
    return correction;
}

double SX1280ComputeRangingCorrectionPolynome(const SX1280_RadioLoRaSpreadingFactors_t sf, const SX1280_RadioLoRaBandwidths_t bw, const double median){
    uint8_t sf_index, bw_index;

    switch(sf){
        case SX1280_LORA_SF5:
            sf_index = 0;
            break;
        case SX1280_LORA_SF6:
            sf_index = 1;
            break;
        case SX1280_LORA_SF7:
            sf_index = 2;
            break;
        case SX1280_LORA_SF8:
            sf_index = 3;
            break;
        case SX1280_LORA_SF9:
            sf_index = 4;
            break;
        case SX1280_LORA_SF10:
            sf_index = 5;
            break;
		case SX1280_LORA_SF11:
			sf_index = 6;
			break;
		case SX1280_LORA_SF12:
            sf_index = 7;
            break;
    }
    switch(bw){
        case SX1280_LORA_BW_0400:
            bw_index = 0;
            break;
        case SX1280_LORA_BW_0800:
            bw_index = 1;
            break;
        case SX1280_LORA_BW_1600:
            bw_index = 2;
            break;
    }
    const RangingCorrectionPolynomes_t *polynome = RangingCorrectionPolynomesPerSfBw[sf_index][bw_index];
    double correctedValue = 0.0;
    double correctionCoeff = 0;
    for(uint8_t order = 0; order < polynome->order; order++){
        correctionCoeff = polynome->coefficients[order] * pow(median, polynome->order - order - 1);
        correctedValue += correctionCoeff;
    }
    return correctedValue;
}

void SX1280SetInterruptMode( void )
{
    PollingMode = false;
}

void SX1280OnDioIrq( void )
{
    /*
     * When polling mode is activated, it is up to the application to call
     * ProcessIrqs( ). Otherwise, the driver automatically calls ProcessIrqs( )
     * on radio interrupt.
     */
    if( PollingMode == true )
    {
        IrqState = true;
    }
    else
    {
//        SX1280ProcessIrqs( );
    }
}






