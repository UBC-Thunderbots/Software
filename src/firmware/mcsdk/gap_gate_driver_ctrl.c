/**
  ******************************************************************************
  * @file    gap_gate_driver_ctrl.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the GAP component of the Motor Control SDK that provides support 
  *          the STGAPxx galvanically isolated gate drivers family.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup GAP_GATE_DRIVER_CTRL
  */

/* Includes ------------------------------------------------------------------*/
#include "gap_gate_driver_ctrl.h"

#include "mc_type.h"

/**
 *  @addtogroup MCSDK
 * @{
 */

/**
  * @defgroup GAP_GATE_DRIVER_CTRL STGAP1x controller
  * @brief A component to interface with the configuration and diagnostic 
  *        features of STGAP1x gate drivers through SPI
  * 
  * The STGAP1x gate drivers family offers galvanically isolated gate drivers
  * for high-power MOSFET and IGBT applications. These devices provide advanced
  * configuration and diagnostic features that are accessible through their SPI
  * interface. Several STGAP1x devices can be daisy-chained on an SPI bus.
  * 
  * The STGAP1x controller component allows for configuring any number of STGAP1x 
  * devices daisy-chained on an SPI bus and for accessing their diagnostic 
  * registers.
  *
  * The GAP driver configuration is performed at the first steps of the MCboot().
  * In a MCSDK project up to seven gate drivers might be configured:
  *
  *  1 brake STGAP1AS_BRAKE
  *
  *  3 high and low side PWM signals
                    STGAP1AS_UH,
                    STGAP1AS_UL,
                    STGAP1AS_VH,
                    STGAP1AS_VL,
                    STGAP1AS_WH,
                    STGAP1AS_WL.
  *
  * More information on STGAP1x can be find on st.com: [Isolated Gate Drivers](https://www.st.com/en/motor-drivers/isolated-gate-drivers.html)
  *
  * @{
  */

/* Private defines -----------------------------------------------------------*/
#define GAP_STARTCONFIG                 0x2A /**< @brief CRC data computation value to start GAP driver configuration. */
#define GAP_STOPCONFIG                  0x3A /**< @brief CRC data computation value to stop GAP driver configuration. */
#define GAP_WRITEREG                    0x80 /**< @brief CRC data computation value for writing register. */
#define GAP_READREG                     0xA0 /**< @brief CRC data computation value for reading register. */
#define GAP_RESETREG                    0xC0 /**< @brief CRC data computation value to reset a register. */
#define GAP_RESETSTATUS                 0xD0 /**< @brief CRC data computation value to reset a status register. */
#define GAP_GLOBALRESET                 0xEA /**< @brief CRC data computation value to reset GAP driver. */
#define GAP_SLPEEP                      0xF5 /**< @brief CRC data computation value to set GAP driver in sleep mode. */
#define GAP_NOP                         0x00 /**< @brief CRC data computation value for no operation. */

#define GAP_ERROR_CODE_FROM_DEVICE_MASK (uint32_t)(0xFF000000) /**< @brief ERROR code mask */

#define WAITTIME                        5000 /**< @brief 860u wait time duration. */
#define WAITTRCHK                       50   /**< @brief 8.6u wait time duration. */
#define WAITTSENSECHK                   50   /**< @brief 8.6u wait time duration. */
#define WAITTGCHK                       50   /**< @brief 8.6u wait time duration. */
#define WAITTDESATCHK                   50   /**< @brief 8.6u wait time duration. */

/* Global variables ----------------------------------------------------------*/
static void GAP_SD_Deactivate(GAP_Handle_t *pHandle_t);
static void GAP_SD_Activate(GAP_Handle_t *pHandle_t);
static void GAP_CS_Deactivate(GAP_Handle_t *pHandle_t);
static void GAP_CS_Activate(GAP_Handle_t *pHandle_t);
static uint16_t GAP_SPI_Send(GAP_Handle_t *pHandle_t, uint16_t value);

/**
  * @brief  Checks errors of GAP devices
  * @param  pHandle Handle of the GAP component GAP_Handle_t.
  * @param  errorNow Buffer of returned bitfields containing error flags
  *         coming from GAP device that are currently active.\n
  *         The buffer have to be provided from the caller.
  * @param  errorOccurred Buffer of returned bitfields containing error flags
  *         coming from GAP device that are over.\n
  *         The buffer have to be provided from the caller.
  * @retval bool It returns false if an error occurs, otherwise return true.
  */
__weak bool GAP_CheckErrors(GAP_Handle_t *pHandle, uint32_t *error_now, uint32_t *error_occurred)
{
  bool ret_val = false;
  uint32_t errorFromDevices[MAX_DEVICES_NUMBER];

  if ((error_now) && (error_occurred))
  {
    uint8_t index1, index2;
    uint8_t ret_read[MAX_DEVICES_NUMBER];

//    /* If current error is device not programmable try to re-configure before
//       read the status registers */
//    if ((pDVars->wGAP_ErrorsNow[0] & GAP_ERROR_CODE_DEVICES_NOT_PROGRAMMABLE) ||
//        (pDVars->wGAP_ErrorsNow[0] & GAP_ERROR_CODE_SPI_CRC))
//    {
//      if (GAP_Configuration(pHandle))
//      {
//        pDVars->wGAP_ErrorsNow[0] &= ~GAP_ERROR_CODE_DEVICES_NOT_PROGRAMMABLE;
//        pDVars->wGAP_ErrorsNow[0] &= ~GAP_ERROR_CODE_SPI_CRC;
//      }
//    }


    index2 = pHandle->DeviceNum;
    ret_val = GAP_ReadRegs(pHandle, ret_read, STATUS1);
    for (index1 = 0; index1 < index2; index1 ++)
    {
      errorFromDevices[index1] = (ret_read[index1] << 16);
    }
    ret_val = GAP_ReadRegs(pHandle, ret_read, STATUS2);
    for (index1 = 0; index1 < index2; index1 ++)
    {
      /* Clear GATE bit from STATUS2 - no error if 1 */
      ret_read[index1] &= 0xFE;
      errorFromDevices[index1] |= (ret_read[index1] << 8);
    }
    ret_val = GAP_ReadRegs(pHandle, ret_read, STATUS3);
    for (index1 = 0; index1 < index2; index1 ++)
    {
      errorFromDevices[index1] |= ret_read[index1];
    }

    for (index1 = 0; index1 < index2; index1 ++)
    {
      pHandle->GAP_ErrorsNow[index1] &= GAP_ERROR_CODE_FROM_DEVICE_MASK;
      pHandle->GAP_ErrorsNow[index1] |= errorFromDevices[index1];
      pHandle->GAP_ErrorsOccurred[index1] |= pHandle->GAP_ErrorsNow[index1];
      error_now[index1] = pHandle->GAP_ErrorsNow[index1];
      error_occurred[index1] = pHandle->GAP_ErrorsOccurred[index1];
    }
  }
  return ret_val;
}

/**
  * @brief  Clears the fault state of GAP devices.
  * @param  pHandle Handle of the GAP component GAP_Handle_t.
  * @retval none.
  */
__weak void GAP_FaultAck(GAP_Handle_t *pHandle)
{
  uint8_t index1, index2;
  uint16_t value;

  GAP_SD_Activate(pHandle);
  value = GAP_CRCCalculate(GAP_RESETSTATUS, 0xFF);
  GAP_CS_Activate(pHandle);
  index2 = pHandle->DeviceNum;

  for (index1 = 0; index1 < index2; index1 ++)
  {
    GAP_SPI_Send(pHandle, value);
  }
  GAP_CS_Deactivate(pHandle);
  GAP_SD_Deactivate(pHandle);
  wait(WAITTIME);

  for (index1 = 0; index1 < index2; index1 ++)
  {
    pHandle->GAP_ErrorsOccurred[index1] = GAP_ERROR_CLEAR;
  }
}

/**
  * @brief  Programs the GAP devices with the settled parameters.
  * @param  pHandle Handle of the GAP component GAP_Handle_t.
  * @retval bool It returns false if at least one device is not programmable,
  *         otherwise return true.
  */
__weak bool GAP_Configuration(GAP_Handle_t *pHandle)
{
  bool ret_val;

  LL_SPI_Enable(pHandle->SPIx);

  /* Configure devices with settled parameters */
  GAP_DevicesConfiguration(pHandle);

  /* Verify if device has been programmed */
  ret_val = GAP_IsDevicesProgrammed(pHandle);

  if (!ret_val)
  {
    /* At least one device is not programmable */
    pHandle->GAP_ErrorsNow[0] |= GAP_ERROR_CODE_DEVICES_NOT_PROGRAMMABLE;
    pHandle->GAP_ErrorsOccurred[0] |= GAP_ERROR_CODE_DEVICES_NOT_PROGRAMMABLE;
  }
  return ret_val;
}

/**
  * @brief  Checks if the GAP devices are programmed with the settled parameters.
  * @param  pHandle Handle of the GAP component GAP_Handle_t.
  * @retval True if the GAP devices are already programmed with the settled
  *         parameters.
  */
__weak bool GAP_IsDevicesProgrammed(GAP_Handle_t *pHandle)
{
  bool ret_val = true;
  uint8_t index1, index2 = pHandle->DeviceNum;
  uint8_t read_reg_values[MAX_DEVICES_NUMBER];

  GAP_ReadRegs(pHandle, read_reg_values, CFG1);

  for (index1 = 0; index1 < index2; index1 ++)
  {
    if ((pHandle->DeviceParams[index1].CFG1 & CFG1_REG_MASK) != read_reg_values[index1])
    {
      ret_val = false;
      break;
    }
  }

  if (ret_val)
  {
    GAP_ReadRegs(pHandle, read_reg_values, CFG2);
    for (index1 = 0; index1 < index2; index1 ++)
    {
      if ((pHandle->DeviceParams[index1].CFG2 & CFG2_REG_MASK) != read_reg_values[index1])
      {
        ret_val = false;
        break;
      }
    }
  }
  if (ret_val)
  {
    GAP_ReadRegs(pHandle, read_reg_values, CFG3);
    for (index1 = 0; index1 < index2; index1 ++)
    {
      if ((pHandle->DeviceParams[index1].CFG3 & CFG3_REG_MASK) != read_reg_values[index1])
      {
        ret_val = false;
        break;
      }
    }
  }
  if (ret_val)
  {
    GAP_ReadRegs(pHandle, read_reg_values, CFG4);
    for (index1 = 0; index1 < index2; index1 ++)
    {
      if ((pHandle->DeviceParams[index1].CFG4 & CFG4_REG_MASK) != read_reg_values[index1])
      {
        ret_val = false;
        break;
      }
    }
  }
  if (ret_val)
  {
    GAP_ReadRegs(pHandle, read_reg_values, CFG5);
    for (index1 = 0; index1 < index2; index1 ++)
    {
      if ((pHandle->DeviceParams[index1].CFG5 & CFG5_REG_MASK) != read_reg_values[index1])
      {
        ret_val = false;
        break;
      }
    }
  }
  if (ret_val)
  {
    GAP_ReadRegs(pHandle, read_reg_values, DIAG1);
    for (index1 = 0; index1 < index2; index1 ++)
    {
      if ((pHandle->DeviceParams[index1].DIAG1 & DIAG1_REG_MASK)  != read_reg_values[index1])
      {
        ret_val = false;
        break;
      }
    }
  }
  if (ret_val)
  {
    GAP_ReadRegs(pHandle, read_reg_values, DIAG2);
    for (index1 = 0; index1 < index2; index1 ++)
    {
      if ((pHandle->DeviceParams[index1].DIAG2 & DIAG2_REG_MASK) != read_reg_values[index1])
      {
        ret_val = false;
        break;
      }
    }
  }
  return ret_val;
}

/**
  * @brief  Programs the GAP devices with the settled parameters.
  * @param  pHandle Handle of the GAP component GAP_Handle_t.
  * @retval bool It returns false if an error occurs, otherwise return true.
  */
__weak bool GAP_DevicesConfiguration(GAP_Handle_t *pHandle)
{
  bool ret_val = true;
  uint8_t index1, DeviceNum = pHandle->DeviceNum;
  uint8_t write_reg_values[MAX_DEVICES_NUMBER];

  GAP_StartConfig(pHandle);

  /* Global Reset before programming */
  GAP_GlobalReset(pHandle);

  for (index1 = 0; index1 < DeviceNum; index1 ++)
  {
    write_reg_values[index1] = pHandle->DeviceParams[index1].CFG1;
  }
  ret_val = GAP_WriteRegs(pHandle, write_reg_values, CFG1);

  if (ret_val)
  {
    for (index1 = 0; index1 < DeviceNum; index1 ++)
    {
      write_reg_values[index1] = pHandle->DeviceParams[index1].CFG2;
    }
    ret_val = GAP_WriteRegs(pHandle, write_reg_values, CFG2);
  }

  if (ret_val)
  {
    for (index1 = 0; index1 < DeviceNum; index1 ++)
    {
      write_reg_values[index1] = pHandle->DeviceParams[index1].CFG3;
    }
    ret_val = GAP_WriteRegs(pHandle, write_reg_values, CFG3);
  }
  if (ret_val)
  {
    for (index1 = 0; index1 < DeviceNum; index1 ++)
    {
      write_reg_values[index1] = pHandle->DeviceParams[index1].CFG4;
    }
    ret_val = GAP_WriteRegs(pHandle, write_reg_values, CFG4);
  }
  if (ret_val)
  {
    for (index1 = 0; index1 < DeviceNum; index1 ++)
    {
      write_reg_values[index1] = pHandle->DeviceParams[index1].CFG5;
    }
    ret_val = GAP_WriteRegs(pHandle, write_reg_values, CFG5);
  }
  if (ret_val)
  {
    for (index1 = 0; index1 < DeviceNum; index1 ++)
    {
      write_reg_values[index1] = pHandle->DeviceParams[index1].DIAG1;
    }
    ret_val = GAP_WriteRegs(pHandle, write_reg_values, DIAG1);
  }
  if (ret_val)
  {
    for (index1 = 0; index1 < DeviceNum; index1 ++)
    {
      write_reg_values[index1] = pHandle->DeviceParams[index1].DIAG2;
    }
    ret_val = GAP_WriteRegs(pHandle, write_reg_values, DIAG2);
  }

  GAP_StopConfig(pHandle);

  /* Fault reset */
  GAP_FaultAck(pHandle);

  return ret_val;
}

/**
  * @brief  Calculates CRC from data and creates 16bit value with data as MSB and
  *         CRC as LSB.
  * @param  data 8bit value used to calculate CRC.
  * @retval uint16_t It returns the 16bit value with data as MSB and
  *         CRC as LSB.
  */
__weak uint16_t GAP_CRCCalculate(uint8_t data, uint8_t crc_initial_value)
{
  uint8_t crc = crc_initial_value;
  uint8_t poly = 0x07;
  uint8_t crc_temp;
  uint8_t crctab[8];
  uint8_t index1, index2;

  uint16_t value;
  value = data;
  value <<= 8;

  for (index2 = 0; index2 < 8; index2 ++)
  {
    crctab[index2] = (crc >> index2) & 0x1;
  }

  for (index2 = 0; index2 < 8; index2 ++)
  {
    crc_temp = (crctab[7] << 7) + (crctab[6] << 6) + (crctab[5] << 5) + (crctab[4] << 4) + (crctab[3] << 3) +
               (crctab[2] << 2) + (crctab[1] << 1) + (crctab[0]);
    crctab[0] = ((data >> (7 - index2)) & 0x1) ^ crctab[7];

    for (index1 = 1; index1 < 8; index1 ++)
    {
      crctab[index1] = (crctab[0] & ((poly >> index1) & 0x1)) ^ ((crc_temp >> (index1 - 1)) & 0x1);
    }
  }

  crc = (crctab[7] << 7) + (crctab[6] << 6) + (crctab[5] << 5) + (crctab[4] << 4) + (crctab[3] << 3) +
        (crctab[2] << 2) + (crctab[1] << 1) + (crctab[0] << 0);
  crc ^= 0xFF;

  value |= crc;
  return value;
}

/**
  * @brief  Verifies the CRC from dataIn and extracts the 8bit data value (out).
  * @param  out Reference for the extracted 8bit data value.
  * @param  dataIn 16bit value with data as MSB and CRC as LSB.
  * @retval bool It returns true if CRC is correct, false otherwise.
  */
__weak bool GAP_CRCCheck(uint8_t *out, uint16_t data_in)
{
  bool ret_val = false;
  uint8_t data = (uint8_t)(data_in >> 8);
  uint8_t received_crc = (uint8_t)(data_in);
  uint8_t crc = (uint8_t)(GAP_CRCCalculate(data, 0xFF)) ^ 0xFF;

  if (crc == received_crc)
  {
    *out = data;
    ret_val = true;
  }
  return ret_val;
}

/**
  * @brief  Waits a time interval proportional to count.
  * @param  count Number of count to be waited.
  * @retval none
  */
__weak void wait(uint16_t count)
{
  volatile uint16_t wait_cnt;

  for (wait_cnt = 0; wait_cnt < count; wait_cnt ++)
  {
    /* Nothing to do */
  }
}

/**
  * @brief  Returns the register mask starting from it address.
  * @param  reg Register address GAP_Registers_Handle_t.
  * @retval uint8_t Mask to be and-ed bit wise to data to filter it.
  */
__weak uint8_t GAP_RegMask(GAP_Registers_Handle_t reg)
{
  uint8_t ret_val;
  switch (reg)
  {
    case CFG1:
    {
      ret_val = CFG1_REG_MASK;
    }
    break;
    case CFG2:
    {
      ret_val = CFG2_REG_MASK;
    }
    break;
    case CFG3:
    {
      ret_val = CFG3_REG_MASK;
    }
    break;
    case CFG4:
    {
      ret_val = CFG4_REG_MASK;
    }
    break;
    case CFG5:
    {
      ret_val = CFG5_REG_MASK;
    }
    break;
    case STATUS1:
    {
      ret_val = STATUS1_REG_MASK;
    }
    break;
    case STATUS2:
    {
      ret_val = STATUS2_REG_MASK;
    }
    break;
    case STATUS3:
    {
      ret_val = STATUS3_REG_MASK;
    }
    break;
    case TEST1:
    {
      ret_val = TEST1_REG_MASK;
    }
    break;
    case DIAG1:
    {
      ret_val = DIAG1_REG_MASK;
    }
    break;
    case DIAG2:
    {
      ret_val = DIAG2_REG_MASK;
    }
    break;
    default:
    {
      ret_val = 0x00;
    }
    break;
  }
  return ret_val;
}

/**
  * @brief  Reads all data in the daisy chain related to register reg.
  * @param  pHandle Handle of the GAP component GAP_Handle_t.
  * @param  pDataRead Pointer to the buffer in which will be stored the readed
  *         data. The buffer have to be provided from the caller.
  * @param  reg Register to be read. It must be one of the register defined in
  *         GAP_Registers_Handle_t.
  * @retval bool It returns false if an error occurs, otherwise return true.
  */
__weak bool GAP_ReadRegs(GAP_Handle_t *pHandle, uint8_t *pDataRead, GAP_Registers_Handle_t reg)
{
  bool ret_val = false;
  uint8_t index1;
  uint8_t device;
  uint8_t data;
  uint16_t value;

  if (pDataRead)
  {
    value = GAP_CRCCalculate(GAP_READREG | reg, 0xFF);
    GAP_CS_Activate(pHandle);

    for (index1 = 0; index1 < pHandle->DeviceNum; index1 ++)
    {
      GAP_SPI_Send(pHandle, value);
    }

    GAP_CS_Deactivate(pHandle);
    wait(WAITTIME);
    value = GAP_CRCCalculate(GAP_NOP, 0xFF);
    GAP_CS_Activate(pHandle);

    ret_val = true;

    for (index1 = 0; index1 < pHandle->DeviceNum; index1 ++)
    {
      device = pHandle->DeviceNum - index1 - 1;

      if (pHandle->DeviceParams[device].CFG1 & GAP_CFG1_CRC_SPI)
      {
        if (GAP_CRCCheck(&data, GAP_SPI_Send(pHandle, value)))
        {
          pDataRead[device] = data & GAP_RegMask(reg);
        }
        else
        {
          pDataRead[device] = 0x00;
          pHandle->GAP_ErrorsNow[0] |= GAP_ERROR_CODE_SPI_CRC;
          pHandle->GAP_ErrorsOccurred[0] |= GAP_ERROR_CODE_SPI_CRC;
          ret_val = false;
        }
      }
      else
      {
        pDataRead[device] = (uint8_t)(GAP_SPI_Send(pHandle, value) >> 8) & GAP_RegMask(reg);
      }
    }
    GAP_CS_Deactivate(pHandle);
  }
  return ret_val;
}

/**
  * @brief  Switches the device to the configuration mode allowing writing configuration values in configuration registers.
  * @param  pHandle Handle of the GAP component GAP_Handle_t.
  * @retval none.
  */
__weak void GAP_StartConfig(GAP_Handle_t *pHandle)
{
  uint8_t index;
  uint16_t value;

  GAP_SD_Activate(pHandle);

  value = GAP_CRCCalculate(GAP_STARTCONFIG, 0xFF);
  GAP_CS_Activate(pHandle);

  for (index = 0; index < pHandle->DeviceNum; index ++)
  {
    GAP_SPI_Send(pHandle, value);
  }
  GAP_CS_Deactivate(pHandle);
  wait(WAITTIME);
}

/**
  * @brief  Quits the configuration mode and make all changes effective.
  * @param  pHandle Handle of the GAP component GAP_Handle_t.
  * @retval none.
  */
__weak void GAP_StopConfig(GAP_Handle_t *pHandle)
{
  uint8_t index;
  uint16_t value;

  value = GAP_CRCCalculate(GAP_STOPCONFIG, 0xFF);
  GAP_CS_Activate(pHandle);

  for (index = 0; index < pHandle->DeviceNum; index ++)
  {
    GAP_SPI_Send(pHandle, value);
  }
  GAP_CS_Deactivate(pHandle);
  wait(WAITTIME);
  GAP_SD_Deactivate(pHandle);
}

/**
  * @brief  Writes data in the daisy chain into the register reg.
  * @param  pHandle Handle of the GAP component GAP_Handle_t.
  * @param  pDataWrite Pointer to the buffer in which are stored the data
  *         to be written in the register reg. The buffer have to be provided
  *         from the caller.
  * @param  reg Register to be write. It must be one of the register defined in
  *         GAP_Registers_Handle_t.
  * @retval bool It returns false if an error occurs, otherwise return true.
  */
__weak bool GAP_WriteRegs(GAP_Handle_t *pHandle, uint8_t *pDataWrite, GAP_Registers_Handle_t reg)
{
  bool ret_val = false;

  if (pDataWrite)
  {
    uint8_t crc;
    uint8_t index;
    uint16_t value;

    value = GAP_CRCCalculate(GAP_WRITEREG | reg, 0xFF);
    crc = (uint8_t)(value);
    GAP_CS_Activate(pHandle);

    for (index = 0; index < pHandle->DeviceNum; index ++)
    {
      GAP_SPI_Send(pHandle, value);
    }
    GAP_CS_Deactivate(pHandle);
    wait(WAITTIME);

    GAP_CS_Activate(pHandle);
    ret_val = true;

    for (index = 0; index < pHandle->DeviceNum; index ++)
    {
      value = GAP_CRCCalculate(pDataWrite[index], crc ^ 0xFF);
      GAP_SPI_Send(pHandle, value);
    }

    GAP_CS_Deactivate(pHandle);
    wait(WAITTIME);
  }
  return ret_val;
}

/**
  * @brief  Resets all the registers to the default and releases all the failure flag.
  * @param  pHandle Handle of the GAP component GAP_Handle_t.
  * @retval none.
  */
__weak void GAP_GlobalReset(GAP_Handle_t *pHandle)
{
  uint8_t index;
  uint16_t value;

  value = GAP_CRCCalculate(GAP_GLOBALRESET, 0xFF);
  GAP_CS_Activate(pHandle);

  for (index = 0; index < pHandle->DeviceNum; index ++)
  {
    GAP_SPI_Send(pHandle, value);
  }
  GAP_CS_Deactivate(pHandle);
  wait(WAITTIME);
}

/**
  * @brief  Resets selected status register.
  * @param  pHandle Handle of the GAP component GAP_Handle_t.
  * @param  reg Register to be reset. It must be one of the STATUS register
   *        defined in GAP_Registers_Handle_t.
  * @retval bool It returns false if an error occurs, otherwise return true.
  */
__weak bool GAP_ResetStatus(GAP_Handle_t *pHandle, GAP_Registers_Handle_t reg)
{
  bool ret_val = false;
  uint8_t index;
  uint16_t value;
  GAP_SD_Activate(pHandle);
  value = GAP_CRCCalculate(GAP_RESETREG | reg, 0xFF);
  GAP_CS_Activate(pHandle);
  for (index = 0; index < pHandle->DeviceNum; index ++)
  {
    GAP_SPI_Send(pHandle, value);
  }
  GAP_CS_Deactivate(pHandle);
  GAP_SD_Deactivate(pHandle);
  return ret_val;
}

/**
  * @brief  It performs the selected test on each GAP devices.
  * @attention pHandle function should be called just in IDLE state.
  * @param  pHandle Handle of the GAP component GAP_Handle_t.
  * @param  testMode Test mode to be executed. It shall be one of the
  *         test modes present in the GAP_TestMode_t.
  * @retval bool It returns true if an error occurs, otherwise return false.
  */
__weak bool GAP_Test(GAP_Handle_t *pHandle, GAP_TestMode_t testMode)
{
  bool ret_val = false;
  bool invertResult = false;

  uint8_t testModeData;
  uint8_t clr[MAX_DEVICES_NUMBER];
  uint8_t data[MAX_DEVICES_NUMBER];
  uint8_t index1, index2 = pHandle->DeviceNum;
  uint8_t statusMask;
  uint16_t wait_cnt;

  switch (testMode)
  {
    case GOFF_CHK:
    {
      testModeData = GAP_TEST1_GOFFCHK;
      wait_cnt = WAITTGCHK;
      statusMask = GAP_STATUS1_DESAT;
    }
    break;
    case GON_CHK:
    {
      testModeData = GAP_TEST1_GONCHK;
      wait_cnt = WAITTGCHK;
      statusMask = GAP_STATUS1_TSD;
    }
    break;
    case GAP_TEST1_DESCHK:
    {
      testModeData = DESAT_CHK;
      wait_cnt = WAITTDESATCHK;
      statusMask = GAP_STATUS1_DESAT;
      invertResult = true;
    }
    break;
    case SENSE_RESISTOR_CHK:
    {
      testModeData = GAP_TEST1_RCHK;
      wait_cnt = WAITTRCHK;
      statusMask = GAP_STATUS1_SENSE;
    }
    break;
    case SENSE_COMPARATOR_CHK:
    {
      testModeData = GAP_TEST1_SNSCHK;
      wait_cnt = WAITTSENSECHK;
      statusMask = GAP_STATUS1_SENSE;
      invertResult = true;
    }
    break;
    default:
    {
      ret_val = true;
    }
    break;
  }

  for (index1 = 0; index1 < index2; index1 ++)
  {
    data[index1] = testModeData;
    clr[index1] = 0x00;
  }
  GAP_WriteRegs(pHandle, data, TEST1);
  wait(wait_cnt);
  GAP_ReadRegs(pHandle, data, STATUS1);
  /* Clear TEST1 regs */
  GAP_WriteRegs(pHandle, clr, TEST1);
  /* Clear STATUS1 regs */
  GAP_ResetStatus(pHandle, STATUS1);

  for (index1 = 0; index1 < index2; index1 ++)
  {
    if (invertResult)
    {
      if ((data[index1] & statusMask) == 0)
      {
        ret_val = true;
      }
    }
    else
    {
      if ((data[index1] & statusMask) != 0)
      {
        ret_val = true;
      }
    }
  }

  return ret_val;
}

/**
  * @brief  This function sends a 16bit value through the configured SPI and
  *         returns the 16-bit value received during communication.
  * @param  pHandle_t Handle of the GAP component GAP_Handle_t.
  * @param  value Value to be sent through SPI.
  * @retval uint16_t Received 16bit value.
  */
uint16_t GAP_SPI_Send(GAP_Handle_t *pHandle_t, uint16_t value)
{
  SPI_TypeDef *SPIx = pHandle_t->SPIx;
  /* Wait for SPI Tx buffer empty */
  while (LL_SPI_IsActiveFlag_TXE(SPIx) == 0);
  /* Send SPI data */
  LL_SPI_TransmitData16(SPIx, value);
  /* Wait for SPIz data reception */
  while (LL_SPI_IsActiveFlag_RXNE(SPIx) == 0);
  /* Read SPIz received data */
  return  LL_SPI_ReceiveData16(SPIx);
}

/**
  * @brief  Deactivates SD pin.
  * @param  pHandle_t Handle of the GAP component GAP_Handle_t.
  * @retval none.
  */
void GAP_SD_Deactivate(GAP_Handle_t *pHandle_t)
{
  LL_GPIO_SetOutputPin(pHandle_t->NSDPort, pHandle_t->NSDPin);
}

/**
  * @brief  Activates SD pin.
  * @param  pHandle_t Handle of the GAP component GAP_Handle_t.
  * @retval none.
  */
void GAP_SD_Activate(GAP_Handle_t *pHandle_t)
{
  LL_GPIO_ResetOutputPin(pHandle_t->NSDPort, pHandle_t->NSDPin);
}

/**
  * @brief  Deactivates CS pin.
  * @param  pHandle_t Handle of the GAP component GAP_Handle_t.
  * @retval none.
  */
void GAP_CS_Deactivate(GAP_Handle_t *pHandle_t)
{
  LL_GPIO_SetOutputPin(pHandle_t->NCSPort, pHandle_t->NCSPin);
}

/**
  * @brief  Activates CS pin.
  * @param  pHandle_t Handle of the GAP component GAP_Handle_t.
  * @retval none.
  */
void GAP_CS_Activate(GAP_Handle_t *pHandle_t)
{
  LL_GPIO_ResetOutputPin(pHandle_t->NCSPort, pHandle_t->NCSPin);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/

