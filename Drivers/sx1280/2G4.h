#ifndef __2G4_H__
#define __2G4_H__

#include "main.h"
#include "stm32l4xx_hal.h"
#include "sx1280.h"
#include "sx1280-hal.h"

/*!
 * \brief Defines the nominal frequency
 */
#define LORA2G4_RF_FREQUENCY                                2426000000 // Hz

/*!
 * \brief Defines the output power in dBm
 *
 * \remark The range of the output power is [-18..+13] dBm
 */
#define LORA2G4_TX_OUTPUT_POWER                             13

/*!
 * \brief Defines the buffer size, i.e. the payload size
 */
#define LORA2G4_BUFFER_SIZE                                 20

/*!
 * \brief Number of tick size steps for tx timeout
 */
#define LORA2G4_TX_TIMEOUT_VALUE                            10000 // ms

/*!
 * \brief Number of tick size steps for rx timeout
 */
#define LORA2G4_RX_TIMEOUT_VALUE                            0xffff // ms

/*!
 * \brief Size of ticks (used for Tx and Rx timeout)
 */
#define LORA2G4_RX_TIMEOUT_TICK_SIZE                        SX1280_RADIO_TICK_SIZE_1000_US

/*!
 * \brief The device address
 */
#define LORA2G4_RANGING_DIVICE                                      0x00010001

void LORA2G4SetParams( void );
void LORA2G4Init( void );
void LORA2G4InitRanging(SX1280_RadioRangingRoles_t role);
void LORA2G4SetPayloadLength(uint8_t length);
void LORA2G4SendData(uint8_t* txBuffer, uint8_t size);
void LORA2G4SetRx( void );


#endif /*__2G4_H__*/
