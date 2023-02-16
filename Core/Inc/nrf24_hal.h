//
// Created by Decke on 11/02/2023.
//

#ifndef NRF24_H
#define NRF24_H

#include "stm32f1xx_hal.h"


#define NRF24_MAX_TIMEOUT 1000


#define NRF24_CMD_W_TX_PAYLOAD_NOACK  0b10110000 //Buffer for sending packet but not expecting ACK
#define NRF24_CMD_FLUSH_TX 0b11100001
#define NRF24_CMD_FLUSH_RX 0b11100010
#define NRF24_CMD_R_RX_PAYLOAD 0b01100001

#define NRF24_REG_CONFIG 0X00
#define NRF24_REG_ENAA 0X01
#define NRF24_REG_RXADDR 0X02
#define NRF24_REG_SETUP_AW 0X03
#define NRF24_REG_SETUP_RETR 0X04 //re-transmission
#define NRF24_REG_RF_CH 0X05 //Radio frequency channel
#define NRF24_REG_RF_SETUP 0X06
#define NRF24_REG_STATUS 0X07
#define NRF24_REG_TX_ADDR 0X10 //Transmisit address 8bytes

#define NRF24_REG_RX_ADDR_P0 0x0A
#define NRF24_REG_RX_ADDR_P1 0x0B
#define NRF24_REG_RX_ADDR_P2 0x0C
#define NRF24_REG_RX_ADDR_P3 0x0D
#define NRF24_REG_RX_ADDR_P4 0x0E
#define NRF24_REG_RX_ADDR_P5 0x0F


#define NRF24_REG_RX_PW_P0 0X11
#define NRF24_REG_RX_PW_P1 0X12
#define NRF24_REG_RX_PW_P2 0X13
#define NRF24_REG_RX_PW_P3 0X14
#define NRF24_REG_RX_PW_P4 0X15


#define NRF24_REG_FIFO_STATUS 0X17

#define NRF24_REG_DYNPD 0X1C
#define NRF24_REG_FEATURE 0X1D


typedef enum {
    NRF24_OK = 0x00,
    NRF24_ERROR = 0x01
} NRF24_ResultTypeDef;

typedef struct {
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *CE_PinPort;
    uint16_t CE_Pin;
    GPIO_TypeDef *CS_PinPort;
    uint16_t CS_Pin;

} NRF24_TypeDef;


uint8_t NRF24_ReadSingleByte(NRF24_TypeDef *hnrf, uint8_t command);

void NRF24_WriteSingleByte(NRF24_TypeDef *hnrf, uint8_t command, uint8_t data);

void NRF24_ReadMultiByte(NRF24_TypeDef *hnrf, uint8_t commandAddress, uint8_t size, uint8_t *dataOut);

void NRF24_WriteMultiByte(NRF24_TypeDef *hnrf, uint8_t commandAddress, uint8_t size, uint8_t *dataIn);

uint8_t NRF24_ReadSingleByteReg(NRF24_TypeDef *hnrf, uint8_t regAddress);

void NRF24_ReadMultiByteReg(NRF24_TypeDef *hnrf, uint8_t address, uint8_t size, uint8_t *dataOut);

void NRF24_WriteMultiByteReg(NRF24_TypeDef *hnrf, uint8_t address, uint8_t size, uint8_t *dataIn);

NRF24_ResultTypeDef NRF24_CheckAvailable(NRF24_TypeDef *hnrf);

void NRF24_WriteSingleByteReg(NRF24_TypeDef *hnrf, uint8_t regAddress, uint8_t data);

NRF24_ResultTypeDef
NRF24_BroadcastBytesToAddress(NRF24_TypeDef *hnrf, uint8_t *receiverAddress, uint8_t *data, uint8_t size);

#endif //NRF24_H
