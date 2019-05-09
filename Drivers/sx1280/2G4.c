#include "2G4.h"

/*!
 * \brief The size of the buffer
 */
uint8_t LORA2G4_BufferSize = LORA2G4_BUFFER_SIZE;

/*!
 * \brief The buffer
 */
uint8_t LORA2G4_Buffer[LORA2G4_BUFFER_SIZE];

/*!
 * \brief Mask of IRQs to listen to in rx mode
 */
uint16_t LORA2G4_RxIrqMask = SX1280_IRQ_RX_DONE | SX1280_IRQ_RX_TX_TIMEOUT;

/*!
 * \brief Mask of IRQs to listen to in tx mode
 */
uint16_t LORA2G4_TxIrqMask = SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_TX_TIMEOUT;

SX1280_PacketParams_t LORA2G4_packetParams;

SX1280_PacketStatus_t LORA2G4_packetStatus;

SX1280_ModulationParams_t LORA2G4_modulationParams;

void LORA2G4Init( void )
{
    SX1280HalReset();

    SX1280SetRegulatorMode(SX1280_USE_DCDC);
    HAL_Delay(500);

    LORA2G4_modulationParams.PacketType = SX1280_PACKET_TYPE_LORA;
    LORA2G4_modulationParams.PacketType = SX1280_PACKET_TYPE_LORA;
    LORA2G4_modulationParams.Params.LoRa.SpreadingFactor = SX1280_LORA_SF10;
    LORA2G4_modulationParams.Params.LoRa.Bandwidth = SX1280_LORA_BW_1600;
    LORA2G4_modulationParams.Params.LoRa.CodingRate = SX1280_LORA_CR_LI_4_7;

    LORA2G4_packetParams.PacketType = SX1280_PACKET_TYPE_LORA;
    LORA2G4_packetParams.Params.LoRa.PreambleLength = 0x12;
    LORA2G4_packetParams.Params.LoRa.HeaderType = SX1280_LORA_PACKET_VARIABLE_LENGTH;
    LORA2G4_packetParams.Params.LoRa.PayloadLength = LORA2G4_BUFFER_SIZE;
    LORA2G4_packetParams.Params.LoRa.CrcMode = SX1280_LORA_CRC_ON;
    LORA2G4_packetParams.Params.LoRa.InvertIQ = SX1280_LORA_IQ_NORMAL;

    SX1280SetStandby(SX1280_STDBY_RC);
    SX1280SetPacketType(LORA2G4_modulationParams.PacketType);
    SX1280SetModulationParams(&LORA2G4_modulationParams);
    SX1280SetPacketParams(&LORA2G4_packetParams);
    SX1280SetRfFrequency(LORA2G4_RF_FREQUENCY);
    SX1280SetBufferBaseAddresses(0x00, 0x00);
}

void LORA2G4SetPayloadLength(uint8_t length)
{
    LORA2G4_packetParams.Params.LoRa.PayloadLength = length;

    SX1280SetPacketParams(&LORA2G4_packetParams);
}

void LORA2G4SendData(uint8_t* txBuffer, uint8_t size)
{
    LORA2G4SetPayloadLength(size);
    SX1280SetTxParams(LORA2G4_TX_OUTPUT_POWER, SX1280_RADIO_RAMP_02_US);
    SX1280SetDioIrqParams(LORA2G4_TxIrqMask, LORA2G4_TxIrqMask, SX1280_IRQ_RADIO_NONE, SX1280_IRQ_RADIO_NONE);
    SX1280SendPayload(txBuffer, size, (SX1280_TickTime_t){LORA2G4_RX_TIMEOUT_TICK_SIZE, LORA2G4_TX_TIMEOUT_VALUE});
}

void LORA2G4SetRx( void )
{
    SX1280SetDioIrqParams(LORA2G4_RxIrqMask, LORA2G4_RxIrqMask, SX1280_IRQ_RADIO_NONE, SX1280_IRQ_RADIO_NONE);
    SX1280SetRx((SX1280_TickTime_t) {LORA2G4_RX_TIMEOUT_TICK_SIZE, LORA2G4_RX_TIMEOUT_VALUE});
}
