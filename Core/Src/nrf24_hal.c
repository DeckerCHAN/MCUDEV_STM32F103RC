//
// Created by Decke on 11/02/2023.
//

#include "nrf24_hal.h"
#include "string.h"



void NRF24_Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}


NRF24_ResultTypeDef NRF24_CheckAvailable(NRF24_TypeDef *hnrf){
    NRF24_WriteMultiByteReg(hnrf, NRF24_REG_TX_ADDR, 5 , (uint8_t [] ){ 0x01,0x02,0x03,0x04,0x05,});
    uint8_t dataOut [5] = {0};
    NRF24_ReadMultiByteReg(hnrf, NRF24_REG_TX_ADDR, 5 , dataOut);

    for(uint8_t i; i < 5; i++){
        if (dataOut[i] != i+1){
            return NRF24_ERROR;
        }
    }

    return NRF24_OK;
}

void NRF24_WriteSingleByteReg(NRF24_TypeDef *hnrf, uint8_t regAddress, uint8_t data) {
    uint8_t commandAddress = 0b00100000 | regAddress;
    NRF24_WriteSingleByte(hnrf, commandAddress, data);
}

uint8_t NRF24_ReadSingleByteReg(NRF24_TypeDef *hnrf, uint8_t regAddress) {
    uint8_t commandAddress = 0b00000000 | regAddress;
    return NRF24_ReadSingleByte(hnrf, commandAddress);
}

uint8_t NRF24_ReadSingleByte(NRF24_TypeDef *hnrf, uint8_t command) {
    uint8_t txData[2] = {command, 0x00};
    uint8_t rxData[2];
    HAL_GPIO_WritePin(hnrf->CS_PinPort, hnrf->CS_Pin, GPIO_PIN_RESET);

    if (HAL_SPI_TransmitReceive(hnrf->spi, txData, rxData, 2, NRF24_MAX_TIMEOUT) != HAL_OK){
        NRF24_Error_Handler();
    }
    while (HAL_SPI_GetState(hnrf->spi) != HAL_SPI_STATE_READY);
    HAL_GPIO_WritePin(hnrf->CS_PinPort, hnrf->CS_Pin, GPIO_PIN_SET);

    return rxData[1];
}

void NRF24_WriteSingleByte(NRF24_TypeDef *hnrf, uint8_t command, uint8_t data) {
    uint8_t txData[2] = {command, data};
    uint8_t rxData[2];

    HAL_GPIO_WritePin(hnrf->CS_PinPort, hnrf->CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hnrf->spi, txData, rxData, 2, NRF24_MAX_TIMEOUT);

    while (HAL_SPI_GetState(hnrf->spi) != HAL_SPI_STATE_READY);
    HAL_GPIO_WritePin(hnrf->CS_PinPort, hnrf->CS_Pin, GPIO_PIN_SET);
}


void NRF24_ReadMultiByte(NRF24_TypeDef *hnrf, uint8_t commandAddress, uint8_t size, uint8_t *dataOut) {
    uint8_t txData[32 + 1] = {0};
    txData[0] = commandAddress;
    uint8_t rxData[32 + 1] = {0};
    HAL_GPIO_WritePin(hnrf->CS_PinPort, hnrf->CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hnrf->spi, txData, rxData, size + 1, NRF24_MAX_TIMEOUT);
    while (HAL_SPI_GetState(hnrf->spi) != HAL_SPI_STATE_READY);
    HAL_GPIO_WritePin(hnrf->CS_PinPort, hnrf->CS_Pin, GPIO_PIN_SET);
    memcpy(dataOut, rxData + 1, size);
}

void NRF24_ReadMultiByteReg(NRF24_TypeDef *hnrf, uint8_t address, uint8_t size, uint8_t *dataOut) {
    uint8_t commandAddress = 0b00000000 | address;
    NRF24_ReadMultiByte(hnrf, commandAddress, size, dataOut);
}

void NRF24_WriteMultiByteReg(NRF24_TypeDef *hnrf, uint8_t address, uint8_t size, uint8_t *dataIn) {
    uint8_t commandAddress = 0b00100000 | address;
    NRF24_WriteMultiByte(hnrf, commandAddress, size, dataIn);
}

void NRF24_WriteMultiByte(NRF24_TypeDef *hnrf, uint8_t commandAddress, uint8_t size, uint8_t *dataIn) {
    uint8_t txData[32 + 1] = {0};
    uint8_t rxData[32 + 1] = {0};
    txData[0] = commandAddress;
    memcpy(txData + 1, dataIn, size);

    HAL_GPIO_WritePin(hnrf->CS_PinPort, hnrf->CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hnrf->spi, txData, rxData, size + 1, NRF24_MAX_TIMEOUT);

    while (HAL_SPI_GetState(hnrf->spi) != HAL_SPI_STATE_READY);
    HAL_GPIO_WritePin(hnrf->CS_PinPort, hnrf->CS_Pin, GPIO_PIN_SET);

}

void NRF24_SwitchToRx(NRF24_TypeDef *hnrf) {
    NRF24_WriteSingleByteReg(hnrf, NRF24_REG_CONFIG,   NRF24_ReadSingleByteReg(hnrf, NRF24_REG_CONFIG) | (0b00000001 << 0) );
}


void NRF24_SwitchToTx(NRF24_TypeDef *hnrf) {
    NRF24_WriteSingleByteReg(hnrf, NRF24_REG_CONFIG,   NRF24_ReadSingleByteReg(hnrf, NRF24_REG_CONFIG) & ~(0b00000000 << 0) );
}


NRF24_ResultTypeDef
NRF24_ReceivePacketBlocking(NRF24_TypeDef *hnrf, uint8_t *dataOut, uint8_t size) {
    HAL_GPIO_WritePin(hnrf->CE_PinPort, hnrf->CE_Pin, GPIO_PIN_RESET);


    //Flush RX
    NRF24_WriteSingleByte(hnrf, NRF24_CMD_FLUSH_RX, 0xFF);

    //Clear curent IRQ
    NRF24_WriteSingleByteReg(hnrf, NRF24_REG_STATUS,0b01111110);

    NRF24_SwitchToRx(hnrf);
    HAL_GPIO_WritePin(hnrf->CE_PinPort, hnrf->CE_Pin, GPIO_PIN_SET);

    //Wait until TX_DS is high
    while ((NRF24_ReadSingleByteReg(hnrf, NRF24_REG_STATUS) & 0b01000000) != 0b01000000){
        NRF24_ReadSingleByteReg(hnrf, NRF24_REG_FIFO_STATUS);
    }

    HAL_GPIO_WritePin(hnrf->CE_PinPort, hnrf->CE_Pin, GPIO_PIN_RESET);

    NRF24_ReadMultiByte(hnrf, NRF24_CMD_R_RX_PAYLOAD, size, dataOut);

    //Flush RX
    NRF24_WriteSingleByte(hnrf, NRF24_CMD_FLUSH_RX, 0xFF);

}

NRF24_ResultTypeDef
NRF24_BroadcastBytesToAddress(NRF24_TypeDef *hnrf, uint8_t *receiverAddress, uint8_t *data, uint8_t size) {

    HAL_GPIO_WritePin(hnrf->CE_PinPort, hnrf->CE_Pin , GPIO_PIN_RESET);

    uint8_t fifoStatus = NRF24_ReadSingleByteReg(hnrf, NRF24_REG_FIFO_STATUS);
    uint8_t isTxEmpty = fifoStatus & 0b00010000;
    if (!isTxEmpty) {
        return NRF24_ERROR;
    }
    NRF24_SwitchToTx(hnrf);
    //Set TX Address
    //Broadcasting no need to set P0 address
    NRF24_WriteMultiByteReg(hnrf, NRF24_REG_TX_ADDR,5 , receiverAddress);


    //Clear out TX_DS amd MAX_RT
    NRF24_WriteSingleByteReg(hnrf , NRF24_REG_STATUS,NRF24_ReadSingleByteReg(hnrf, NRF24_REG_STATUS) | 0b00110000 );

    //Write Data to No_ACK
    NRF24_WriteMultiByte(hnrf, NRF24_CMD_W_TX_PAYLOAD_NOACK, size , data);
    HAL_GPIO_WritePin(hnrf->CE_PinPort, hnrf->CE_Pin , GPIO_PIN_SET);


    //Wait until TX_DS is high
    while ((NRF24_ReadSingleByteReg(hnrf, NRF24_REG_STATUS) & 0b00100000 ) != 0b00100000 );

    //Turn off CE
    HAL_GPIO_WritePin(hnrf->CE_PinPort, hnrf->CE_Pin , GPIO_PIN_RESET);


    //Clear out TX_DS amd MAX_RT
    NRF24_WriteSingleByteReg(hnrf , NRF24_REG_STATUS,NRF24_ReadSingleByteReg(hnrf, NRF24_REG_STATUS) | 0b00110000 );

    return NRF24_OK;
}