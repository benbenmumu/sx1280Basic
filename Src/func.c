#include "func.h"
#include "usart.h"

uint8_t Hex2Str[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

void SendHexToStr(uint8_t HexData)
{
    uint8_t HexStr[2] = {'0', '0'};
    
    HexStr[0] = Hex2Str[(HexData & 0xf0) >> 4];
    HexStr[1] = Hex2Str[HexData & 0x0f];
    
    while(HAL_UART_Transmit_IT(&huart1, HexStr, 2) == HAL_BUSY);
}


















