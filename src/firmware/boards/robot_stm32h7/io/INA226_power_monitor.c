/**
  ********************************************************************************
  * @brief   STM32 HAL Library for INA226 Current/Power Monitor
  * @date    Feb 2016
  * @version 1.0
  * @author  George Christidis
  ********************************************************************************
  * @details
            This library contains the necessary functions to initialize, read and
            write data to the TI INA226 Current/Power Monitor using the I2C
            protocol.
    ******************************************************************************
    */

#include "INA226_power_monitor.h"

float32_t INA226_getBusV(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress)
{
    return (INA226_getBusVReg(I2CHandler, DevAddress));
}

float32_t INA226_getCurrent(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress)
{
    return (INA226_getCurrentReg(I2CHandler, DevAddress) * INA226_CURRENTLSB_INV);
}

float32_t INA226_getPower(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress)
{
    return (INA226_getPowerReg(I2CHandler, DevAddress) * INA226_POWERLSB_INV);
}

uint8_t INA226_setConfig(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress,
                         uint16_t ConfigWord)
{
    uint8_t SentTable[3];
    SentTable[0] = INA226_CONFIG;
    SentTable[1] = (ConfigWord & 0xFF00) >> 8;
    SentTable[2] = (ConfigWord & 0x00FF);
    return HAL_I2C_Master_Transmit(I2CHandler, DevAddress, SentTable, 3,
                                   INA226_I2CTIMEOUT);
}

uint16_t INA226_getConfig(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress)
{
    uint8_t SentTable[1] = {INA226_CONFIG};
    uint8_t ReceivedTable[2];
    HAL_I2C_Master_Transmit(I2CHandler, DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
    if (HAL_I2C_Master_Receive(I2CHandler, DevAddress, ReceivedTable, 2,
                               INA226_I2CTIMEOUT) != HAL_OK)
        return 0xFF;
    else
        return ((uint16_t)ReceivedTable[0] << 8 | ReceivedTable[1]);
}

uint16_t INA226_getShuntV(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress)
{
    uint8_t SentTable[1] = {INA226_SHUNTV};
    uint8_t ReceivedTable[2];
    HAL_I2C_Master_Transmit(I2CHandler, DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
    if (HAL_I2C_Master_Receive(I2CHandler, DevAddress, ReceivedTable, 2,
                               INA226_I2CTIMEOUT) != HAL_OK)
        return 0xFF;
    else
        return ((uint16_t)ReceivedTable[0] << 8 | ReceivedTable[1]);
}

uint16_t INA226_getBusVReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress)
{
    uint8_t SentTable[1] = {INA226_BUSV};
    uint8_t ReceivedTable[2];
    HAL_I2C_Master_Transmit(I2CHandler, DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
    if (HAL_I2C_Master_Receive(I2CHandler, DevAddress, ReceivedTable, 2,
                               INA226_I2CTIMEOUT) != HAL_OK)
        return 0xFF;
    else
        return ((uint16_t)ReceivedTable[0] << 8 | ReceivedTable[1]);
}

uint8_t INA226_setCalibrationReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress,
                                 uint16_t ConfigWord)
{
    uint8_t SentTable[3];
    SentTable[0] = INA226_CALIB;
    SentTable[1] = (ConfigWord & 0xFF00) >> 8;
    SentTable[2] = (ConfigWord & 0x00FF);
    return HAL_I2C_Master_Transmit(I2CHandler, DevAddress, SentTable, 3,
                                   INA226_I2CTIMEOUT);
}

uint16_t INA226_getCalibrationReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress)
{
    uint8_t SentTable[1] = {INA226_CALIB};
    uint8_t ReceivedTable[2];
    HAL_I2C_Master_Transmit(I2CHandler, DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
    if (HAL_I2C_Master_Receive(I2CHandler, DevAddress, ReceivedTable, 2,
                               INA226_I2CTIMEOUT) != HAL_OK)
        return 0xFF;
    else
        return ((uint16_t)ReceivedTable[0] << 8 | ReceivedTable[1]);
}

uint16_t INA226_getPowerReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress)
{
    uint8_t SentTable[1] = {INA226_POWER};
    uint8_t ReceivedTable[2];
    HAL_I2C_Master_Transmit(I2CHandler, DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
    if (HAL_I2C_Master_Receive(I2CHandler, DevAddress, ReceivedTable, 2,
                               INA226_I2CTIMEOUT) != HAL_OK)
        return 0xFF;
    else
        return ((uint16_t)ReceivedTable[0] << 8 | ReceivedTable[1]);
}

uint16_t INA226_getCurrentReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress)
{
    uint8_t SentTable[1] = {INA226_CURRENT};
    uint8_t ReceivedTable[2];
    HAL_I2C_Master_Transmit(I2CHandler, DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
    if (HAL_I2C_Master_Receive(I2CHandler, DevAddress, ReceivedTable, 2,
                               INA226_I2CTIMEOUT) != HAL_OK)
        return 0xFF;
    else
        return ((uint16_t)ReceivedTable[0] << 8 | ReceivedTable[1]);
}

uint16_t INA226_getManufID(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress)
{
    uint8_t SentTable[1] = {INA226_MANUF_ID};
    uint8_t ReceivedTable[2];
    HAL_I2C_Master_Transmit(I2CHandler, DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
    if (HAL_I2C_Master_Receive(I2CHandler, DevAddress, ReceivedTable, 2,
                               INA226_I2CTIMEOUT) != HAL_OK)
        return 0xFF;
    else
        return ((uint16_t)ReceivedTable[0] << 8 | ReceivedTable[1]);
}

uint16_t INA226_getDieID(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress)
{
    uint8_t SentTable[1] = {INA226_DIE_ID};
    uint8_t ReceivedTable[2];
    HAL_I2C_Master_Transmit(I2CHandler, DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
    if (HAL_I2C_Master_Receive(I2CHandler, DevAddress, ReceivedTable, 2,
                               INA226_I2CTIMEOUT) != HAL_OK)
        return 0xFF;
    else
        return ((uint16_t)ReceivedTable[0] << 8 | ReceivedTable[1]);
}

uint8_t INA226_setMaskEnable(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress,
                             uint16_t ConfigWord)
{
    uint8_t SentTable[3];
    SentTable[0] = INA226_MASK;
    SentTable[1] = (ConfigWord & 0xFF00) >> 8;
    SentTable[2] = (ConfigWord & 0x00FF);
    return HAL_I2C_Master_Transmit(I2CHandler, DevAddress, SentTable, 3,
                                   INA226_I2CTIMEOUT);
}

uint16_t INA226_getMaskEnable(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress)
{
    uint8_t SentTable[1] = {INA226_MASK};
    uint8_t ReceivedTable[2];
    HAL_I2C_Master_Transmit(I2CHandler, DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
    if (HAL_I2C_Master_Receive(I2CHandler, DevAddress, ReceivedTable, 2,
                               INA226_I2CTIMEOUT) != HAL_OK)
        return 0xFF;
    else
        return ((uint16_t)ReceivedTable[0] << 8 | ReceivedTable[1]);
}

uint8_t INA226_setAlertLimit(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress,
                             uint16_t ConfigWord)
{
    uint8_t SentTable[3];
    SentTable[0] = INA226_ALERTL;
    SentTable[1] = (ConfigWord & 0xFF00) >> 8;
    SentTable[2] = (ConfigWord & 0x00FF);
    return HAL_I2C_Master_Transmit(I2CHandler, DevAddress, SentTable, 3,
                                   INA226_I2CTIMEOUT);
}

uint16_t INA226_getAlertLimit(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress)
{
    uint8_t SentTable[1] = {INA226_ALERTL};
    uint8_t ReceivedTable[2];
    HAL_I2C_Master_Transmit(I2CHandler, DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
    if (HAL_I2C_Master_Receive(I2CHandler, DevAddress, ReceivedTable, 2,
                               INA226_I2CTIMEOUT) != HAL_OK)
        return 0xFF;
    else
        return ((uint16_t)ReceivedTable[0] << 8 | ReceivedTable[1]);
}
