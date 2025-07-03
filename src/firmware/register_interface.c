/**
  ******************************************************************************
  * @file    register_interface.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the register access for the MCP protocol
  *
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
  */

#include "firmware/mcsdk/mc_type.h"
#include "string.h"
#include "register_interface.h"
#include "mc_config.h"
#include "mc_configuration_registers.h"

static PID_Handle_t *pPIDSpeed[NBR_OF_MOTORS] = { &PIDSpeedHandle_M1 };
static ENCODER_Handle_t *pEncoder[NBR_OF_MOTORS] = {&ENCODER_M1};

static uint8_t RI_SetReg (uint16_t dataID, uint8_t * data, uint16_t *size, int16_t dataAvailable);
static uint8_t RI_GetReg (uint16_t dataID, uint8_t * data, uint16_t *size, int16_t maxSize);
static uint8_t RI_MovString(const char_t * srcString, char_t * destString, uint16_t *size, int16_t maxSize);
__weak uint8_t RI_SetRegCommandParser (MCP_Handle_t * pHandle, uint16_t txSyncFreeSpace)
{
  uint8_t retVal = MCP_CMD_OK;
#ifdef NULL_PTR_REG_INT
  if (MC_NULL == pHandle)
  {
    retVal = MCP_CMD_NOK;
  }
  else
  {
#endif
    uint16_t * dataElementID;
    uint8_t * rxData = pHandle->rxBuffer;
    uint8_t * txData = pHandle->txBuffer;
    int16_t rxLength = pHandle->rxLength;
    uint16_t size = 0U;
    uint8_t number_of_item =0;
    pHandle->txLength = 0;
    uint8_t accessResult;
    while (rxLength > 0)
    {
       number_of_item++;
      dataElementID = (uint16_t *) rxData;
      rxLength = rxLength-MCP_ID_SIZE; // We consume 2 byte in the DataID
      rxData = rxData+MCP_ID_SIZE; // Shift buffer to the next data
      accessResult = RI_SetReg (*dataElementID,rxData,&size,rxLength);

      /* Prepare next data*/
      rxLength = (int16_t) (rxLength - size);
      rxData = rxData+size;
      /* If there is only one CMD in the buffer, we do not store the result */
        if ((1U == number_of_item) && (0 == rxLength))
      {
        retVal = accessResult;
      }
      else
      {/* Store the result for each access to be able to report failling access */
        if (txSyncFreeSpace !=0 )
        {
          *txData = accessResult;
          txData = txData+1;
          pHandle->txLength++;
          txSyncFreeSpace--; /* decrement one by one no wraparound possible */
          retVal = (accessResult != MCP_CMD_OK) ? MCP_CMD_NOK : retVal;
          if ((accessResult == MCP_ERROR_BAD_DATA_TYPE) || (accessResult == MCP_ERROR_BAD_RAW_FORMAT))
          { /* From this point we are not able to continue to decode CMD buffer*/
            /* We stop the parsing */
            rxLength = 0;
          }
        }
        else
        {
          /* Stop parsing the cmd buffer as no space to answer */
          /* If we reach this state, chances are high the command was badly formated or received */
          rxLength = 0;
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
      }
    }
    /* If all accesses are fine, just one global MCP_CMD_OK is required*/
      if (MCP_CMD_OK == retVal)
    {
      pHandle->txLength = 0;
    }
    else
    {
      /* Nothing to do */
    }
#ifdef NULL_PTR_REG_INT
  }
#endif
  return (retVal);
}

__weak uint8_t RI_GetRegCommandParser (MCP_Handle_t * pHandle, uint16_t txSyncFreeSpace)
{
  uint8_t retVal = MCP_CMD_NOK;
#ifdef NULL_PTR_REG_INT
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    uint16_t * dataElementID;
    uint8_t * rxData = pHandle->rxBuffer;
    uint8_t * txData = pHandle->txBuffer;
    uint16_t size = 0;
    uint16_t rxLength = pHandle->rxLength;
    int16_t freeSpaceS16 = (int16_t) txSyncFreeSpace;

    pHandle->txLength = 0;

    while (rxLength > 0U)
    {
      dataElementID = (uint16_t *) rxData;
      rxLength = rxLength-MCP_ID_SIZE;
      rxData = rxData+MCP_ID_SIZE; // Shift buffer to the next MCP_ID
      retVal = RI_GetReg (*dataElementID,txData, &size, freeSpaceS16);
      if (retVal == MCP_CMD_OK )
      {
        txData = txData+size;
        pHandle->txLength += size;
        freeSpaceS16 = freeSpaceS16-size;
      }
      else
      {
        rxLength = 0;
      }
    }
#ifdef NULL_PTR_REG_INT
  }
#endif
  return (retVal);
}

uint8_t RI_SetReg (uint16_t dataID, uint8_t * data, uint16_t *size, int16_t dataAvailable)
{
  uint8_t retVal = MCP_CMD_OK;
#ifdef NULL_PTR_REG_INT
  if ((MC_NULL == data) || (MC_NULL == size))
  {
    retVal = MCP_CMD_NOK;
  }
  else
  {
#endif
    uint16_t regID = dataID & REG_MASK;
    uint8_t motorID;
    uint8_t typeID;

    typeID = (uint8_t)dataID & TYPE_MASK;
    motorID = 0U;
    MCI_Handle_t *pMCIN = &Mci[motorID];

    switch (typeID)
    { //cstat !MISRAC2012-Rule-16.1
      case TYPE_DATA_8BIT:
      {
        switch (regID)
        {
          case MC_REG_STATUS:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }

          case MC_REG_CONTROL_MODE:
          {
            uint8_t regdata8 = *data;
            if ((uint8_t)MCM_TORQUE_MODE == regdata8)
            {
              MCI_ExecTorqueRamp(pMCIN, MCI_GetTeref(pMCIN), 0);
            }
            else
            {
              /* Nothing to do */
            }

            if ((uint8_t)MCM_SPEED_MODE == regdata8)
            {
              MCI_ExecSpeedRamp(pMCIN, MCI_GetMecSpeedRefUnit(pMCIN), 0);
            }
            else
            {
              /* Nothing to do */
            }

            break;
          }

          default:
          {
            retVal = MCP_ERROR_UNKNOWN_REG;
            break;
          }
        }
        *size = 1;
        break;
      }

      case TYPE_DATA_16BIT:
      {
        uint16_t regdata16 = *(uint16_t *)data; //cstat !MISRAC2012-Rule-11.3
        switch (regID)
        {
          case MC_REG_SPEED_KP:
          {
            PID_SetKP(pPIDSpeed[motorID], (int16_t)regdata16);
            break;
          }

          case MC_REG_SPEED_KI:
          {
            PID_SetKI(pPIDSpeed[motorID], (int16_t)regdata16);
            break;
          }

          case MC_REG_SPEED_KD:
          {
            PID_SetKD(pPIDSpeed[motorID], (int16_t)regdata16);
            break;
          }
          case MC_REG_I_Q_KP:
          {
            PID_SetKP(pPIDIq[motorID], (int16_t)regdata16);
            break;
          }

          case MC_REG_I_Q_KI:
          {
            PID_SetKI(pPIDIq[motorID], (int16_t)regdata16);
            break;
          }

          case MC_REG_I_Q_KD:
          {
            PID_SetKD(pPIDIq[motorID], (int16_t)regdata16);
            break;
          }

          case MC_REG_I_D_KP:
          {
            PID_SetKP(pPIDId[motorID], (int16_t)regdata16);
            break;
          }

          case MC_REG_I_D_KI:
          {
            PID_SetKI(pPIDId[motorID], (int16_t)regdata16);
            break;
          }

          case MC_REG_I_D_KD:
          {
            PID_SetKD(pPIDId[motorID], (int16_t)regdata16);
            break;
          }

          case MC_REG_BUS_VOLTAGE:
          case MC_REG_HEATS_TEMP:
          case MC_REG_MOTOR_POWER:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }
          case MC_REG_I_A:
          case MC_REG_I_B:
          case MC_REG_I_ALPHA_MEAS:
          case MC_REG_I_BETA_MEAS:
          case MC_REG_I_Q_MEAS:
          case MC_REG_I_D_MEAS:

          case MC_REG_FLUXWK_BUS_MEAS:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }
          case MC_REG_I_Q_REF:
          {
            qd_t currComp;
            currComp = MCI_GetIqdref(pMCIN);
            currComp.q = (int16_t)regdata16;
            MCI_SetCurrentReferences(pMCIN,currComp);
            break;
          }

          case MC_REG_I_D_REF:
          {
            qd_t currComp;
            currComp = MCI_GetIqdref(pMCIN);
            currComp.d = (int16_t)regdata16;
            MCI_SetCurrentReferences(pMCIN,currComp);
            break;
          }
          case MC_REG_V_Q:
          case MC_REG_V_D:
          case MC_REG_V_ALPHA:
          case MC_REG_V_BETA:
          case MC_REG_ENCODER_EL_ANGLE:
          case MC_REG_ENCODER_SPEED:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }
          case MC_REG_DAC_USER1:
          case MC_REG_DAC_USER2:
            break;

          case MC_REG_SPEED_KP_DIV:
          {
            PID_SetKPDivisorPOW2(pPIDSpeed[motorID], regdata16);
            break;
          }

          case MC_REG_SPEED_KI_DIV:
          {
            PID_SetKIDivisorPOW2(pPIDSpeed[motorID], regdata16);
            break;
          }

          case MC_REG_SPEED_KD_DIV:
          {
            PID_SetKDDivisorPOW2(pPIDSpeed[motorID], regdata16);
            break;
          }

          case MC_REG_I_D_KP_DIV:
          {
            PID_SetKPDivisorPOW2(pPIDId[motorID], regdata16);
            break;
          }

          case MC_REG_I_D_KI_DIV:
          {
            PID_SetKIDivisorPOW2(pPIDId[motorID], regdata16);
            break;
          }

          case MC_REG_I_D_KD_DIV:
          {
            PID_SetKDDivisorPOW2(pPIDId[motorID], regdata16);
            break;
          }

          case MC_REG_I_Q_KP_DIV:
          {
            PID_SetKPDivisorPOW2(pPIDIq[motorID], regdata16);
            break;
          }

          case MC_REG_I_Q_KI_DIV:
          {
            PID_SetKIDivisorPOW2(pPIDIq[motorID], regdata16);
            break;
          }

          case MC_REG_I_Q_KD_DIV:
          {
            PID_SetKDDivisorPOW2(pPIDIq[motorID], regdata16);
            break;
          }

          default:
          {
            retVal = MCP_ERROR_UNKNOWN_REG;
            break;
          }
        }
        *size = 2;
        break;
      }

      case TYPE_DATA_32BIT:
      {
        uint32_t regdata32 = (((uint32_t)(*(uint16_t *)&data[2])) << 16) | *(uint16_t *)data;

        switch (regID)
        {

          case MC_REG_FAULTS_FLAGS:
          case MC_REG_SPEED_MEAS:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }

          case MC_REG_SPEED_REF:
          {
            MCI_ExecSpeedRamp(pMCIN,((((int16_t)regdata32) * ((int16_t)SPEED_UNIT)) / (int16_t)U_RPM), 0);
            break;
          }

          default:
          {
            retVal = MCP_ERROR_UNKNOWN_REG;
            break;
          }
        }
        *size = 4;
        break;
      }
      case TYPE_DATA_STRING:
      {
        const char_t *charData = (const char_t *)data;
        char_t *dummy = (char_t *)data ;
        retVal = MCP_ERROR_RO_REG;
        /* Used to compute String length stored in RXBUFF even if Reg does not exist*/
        /* It allows to jump to the next command in the buffer */
        (void)RI_MovString (charData, dummy, size, dataAvailable);
        break;
      }

      case TYPE_DATA_RAW:
      {
        uint16_t rawSize = *(uint16_t *) data; //cstat !MISRAC2012-Rule-11.3
        /* The size consumed by the structure is the structure size + 2 bytes used to store the size*/
        *size = rawSize + 2U;
        uint8_t *rawData = data; /* rawData points to the first data (after size extraction) */
        rawData++;
        rawData++;

        if (*size > dataAvailable )
        { /* The decoded size of the raw structure can not match with transmitted buffer, error in buffer construction*/
          *size = 0;
          retVal = MCP_ERROR_BAD_RAW_FORMAT; /* this error stop the parsing of the CMD buffer */
        }
        else
        {
          switch (regID)
          {
            case MC_REG_GLOBAL_CONFIG:
            case MC_REG_MOTOR_CONFIG:
            case MC_REG_APPLICATION_CONFIG:
            case MC_REG_FOCFW_CONFIG:
            {
              retVal = MCP_ERROR_RO_REG;
              break;
            }
            case MC_REG_SPEED_RAMP:
            {
              int32_t rpm;
              uint16_t duration;

              /* 32 bits access are splited into 2x16 bits access */
              rpm = (((int32_t)(*(int16_t *)&rawData[2])) << 16) | *(uint16_t *)rawData; //cstat !MISRAC2012-Rule-11.3
              duration = *(uint16_t *)&rawData[4]; //cstat !MISRAC2012-Rule-11.3
              MCI_ExecSpeedRamp(pMCIN, (int16_t)((rpm * SPEED_UNIT) / U_RPM), duration);
              break;
            }
            case MC_REG_TORQUE_RAMP:
            {
              uint32_t torque;
              uint16_t duration;

              /* 32 bits access are splited into 2x16 bits access */
              //cstat !MISRAC2012-Rule-11.3
              torque = ((uint32_t)(*(int16_t *)&rawData[2]))<<16 | *(uint16_t *)rawData;
              duration = *(uint16_t *)&rawData[4]; //cstat !MISRAC2012-Rule-11.3
              MCI_ExecTorqueRamp(pMCIN, (int16_t)torque, duration);
              break;
            }

            case MC_REG_CURRENT_REF:
            {
              qd_t currComp;
              currComp.q = *((int16_t *) rawData); //cstat !MISRAC2012-Rule-11.3
              currComp.d = *((int16_t *) &rawData[2]); //cstat !MISRAC2012-Rule-11.3
              MCI_SetCurrentReferences(pMCIN, currComp);
              break;
            }

            default:
            {
              retVal = MCP_ERROR_UNKNOWN_REG;
              break;
            }
          }
        }
        break;
      }

      default:
      {
        retVal = MCP_ERROR_BAD_DATA_TYPE;
        *size =0; /* From this point we are not able anymore to decode the RX buffer*/
        break;
      }
    }
#ifdef NULL_PTR_REG_INT
  }
#endif
  return (retVal);
}

uint8_t RI_GetReg (uint16_t dataID, uint8_t * data, uint16_t *size, int16_t freeSpace)
{
  uint8_t retVal = MCP_CMD_OK;
#ifdef NULL_PTR_REG_INT
  if ((MC_NULL == data) || (MC_NULL == size))
  {
    retVal = MCP_CMD_NOK;
  }
  else
  {
#endif
    uint16_t regID = dataID & REG_MASK;
    uint8_t typeID = ((uint8_t)dataID) & TYPE_MASK;
    BusVoltageSensor_Handle_t* BusVoltageSensor[NBR_OF_MOTORS]={ &BusVoltageSensor_M1._Super};
    uint8_t motorID = 0U;

    MCI_Handle_t *pMCIN = &Mci[motorID];
    switch (typeID)
    {
      case TYPE_DATA_8BIT:
      {
        if (freeSpace > 0U)
        {
          switch (regID)
          {
            case MC_REG_STATUS:
            {
              *data = (uint8_t)MCI_GetSTMState(pMCIN);
              break;
            }

            case MC_REG_CONTROL_MODE:
            {
              *data = (uint8_t)MCI_GetControlMode(pMCIN);
              break;
            }

            default:
            {
              retVal = MCP_ERROR_UNKNOWN_REG;
              break;
            }
          }
          *size = 1;
        }
        else
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
        break;
      }
      case TYPE_DATA_16BIT:
      {
        uint16_t *regdataU16 = (uint16_t *)data; //cstat !MISRAC2012-Rule-11.3
        int16_t *regdata16 = (int16_t *) data; //cstat !MISRAC2012-Rule-11.3

        if (freeSpace >= 2U)
        {
          switch (regID)
          {
            case MC_REG_SPEED_KP:
            {
              *regdata16 = PID_GetKP(pPIDSpeed[motorID]);
              break;
            }

            case MC_REG_SPEED_KI:
            {
              *regdata16 = PID_GetKI(pPIDSpeed[motorID]);
              break;
            }

            case MC_REG_SPEED_KD:
            {
              *regdata16 = PID_GetKD(pPIDSpeed[motorID]);
              break;
            }

        case MC_REG_I_Q_KP:
            {
              *regdata16 = PID_GetKP(pPIDIq[motorID]);
              break;
            }

        case MC_REG_I_Q_KI:
            {
              *regdata16 = PID_GetKI(pPIDIq[motorID]);
              break;
            }

        case MC_REG_I_Q_KD:
            {
              *regdata16 = PID_GetKD(pPIDIq[motorID]);
              break;
            }

        case MC_REG_I_D_KP:
            {
              *regdata16 = PID_GetKP(pPIDId[motorID]);
              break;
            }

        case MC_REG_I_D_KI:
            {
              *regdata16 = PID_GetKI(pPIDId[motorID]);
              break;
            }

        case MC_REG_I_D_KD:
            {
              *regdata16 = PID_GetKD(pPIDId[motorID]);
              break;
            }
            case MC_REG_BUS_VOLTAGE:
            {
              *regdataU16 = VBS_GetAvBusVoltage_V(BusVoltageSensor[motorID]);
              break;
            }

            case MC_REG_HEATS_TEMP:
            {
              *regdata16 = NTC_GetAvTemp_C(pTemperatureSensor[motorID]);
              break;
            }

            case MC_REG_I_A:
            {
              *regdata16 = MCI_GetIab(pMCIN).a;
              break;
            }

            case MC_REG_I_B:
            {
              *regdata16 = MCI_GetIab(pMCIN).b;
              break;
            }

            case MC_REG_I_ALPHA_MEAS:
            {
              *regdata16 = MCI_GetIalphabeta(pMCIN).alpha;
              break;
            }

            case MC_REG_I_BETA_MEAS:
            {
              *regdata16 = MCI_GetIalphabeta(pMCIN).beta;
              break;
            }

            case MC_REG_I_Q_MEAS:
            {
              *regdata16 = MCI_GetIqd(pMCIN).q;
              break;
            }

            case MC_REG_I_D_MEAS:
            {
              *regdata16 = MCI_GetIqd(pMCIN).d;
              break;
            }

            case MC_REG_I_Q_REF:
            {
              *regdata16 = MCI_GetIqdref(pMCIN).q;
              break;
            }

            case MC_REG_I_D_REF:
            {
              *regdata16 = MCI_GetIqdref(pMCIN).d;
              break;
            }
            case MC_REG_V_Q:
            {
              *regdata16 = MCI_GetVqd(pMCIN).q;
              break;
            }

            case MC_REG_V_D:
            {
              *regdata16 = MCI_GetVqd(pMCIN).d;
              break;
            }

            case MC_REG_V_ALPHA:
            {
              *regdata16 = MCI_GetValphabeta(pMCIN).alpha;
              break;
            }

            case MC_REG_V_BETA:
            {
              *regdata16 = MCI_GetValphabeta(pMCIN).beta;
              break;
            }

            case MC_REG_ENCODER_EL_ANGLE:
            {
              *regdata16 = SPD_GetElAngle ((SpeednPosFdbk_Handle_t*) pEncoder[motorID]); //cstat !MISRAC2012-Rule-11.3
              break;
            }

            case MC_REG_ENCODER_SPEED:
            {
              *regdata16 = SPD_GetS16Speed ((SpeednPosFdbk_Handle_t*) pEncoder[motorID]); //cstat !MISRAC2012-Rule-11.3
              break;
            }

            case MC_REG_DAC_USER1:
            case MC_REG_DAC_USER2:
              break;

            case MC_REG_SPEED_KP_DIV:
            {
              *regdataU16 = (uint16_t)PID_GetKPDivisorPOW2(pPIDSpeed[motorID]);
              break;
            }

            case MC_REG_SPEED_KI_DIV:
            {
              *regdataU16 = (uint16_t)PID_GetKIDivisorPOW2(pPIDSpeed[motorID]);
              break;
            }

            case MC_REG_SPEED_KD_DIV:
            {
              *regdataU16 = PID_GetKDDivisorPOW2(pPIDSpeed[motorID]);
              break;
            }
            case MC_REG_I_D_KP_DIV:
            {
              *regdataU16 = PID_GetKPDivisorPOW2(pPIDId[motorID]);
              break;
            }

            case MC_REG_I_D_KI_DIV:
            {
              *regdataU16 = PID_GetKIDivisorPOW2(pPIDId[motorID]);
              break;
            }

            case MC_REG_I_D_KD_DIV:
            {
              *regdataU16 = PID_GetKDDivisorPOW2(pPIDId[motorID]);
              break;
            }

            case MC_REG_I_Q_KP_DIV:
            {
              *regdataU16 = PID_GetKPDivisorPOW2(pPIDIq[motorID]);
              break;
            }

            case MC_REG_I_Q_KI_DIV:
            {
              *regdataU16 = PID_GetKIDivisorPOW2(pPIDIq[motorID]);
              break;
            }

            case MC_REG_I_Q_KD_DIV:
            {
              *regdataU16 = PID_GetKDDivisorPOW2(pPIDIq[motorID]);
              break;
            }

            default:
            {
              retVal = MCP_ERROR_UNKNOWN_REG;
              break;
            }
          }
          *size = 2;
        }
        else
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
        break;
      }

      case TYPE_DATA_32BIT:
      {
        uint32_t *regdataU32 = (uint32_t *)data; //cstat !MISRAC2012-Rule-11.3
        int32_t *regdata32 = (int32_t *)data; //cstat !MISRAC2012-Rule-11.3

        if (freeSpace >= 4U)
        {
          switch (regID)
          {

            case MC_REG_FAULTS_FLAGS:
            {
              *regdataU32 = MCI_GetFaultState(pMCIN);
              break;
            }

            case MC_REG_SPEED_MEAS:
            {
              *regdata32 = (((int32_t)MCI_GetAvrgMecSpeedUnit(pMCIN) * U_RPM) / SPEED_UNIT);
              break;
            }

            case MC_REG_SPEED_REF:
            {
              *regdata32 = (((int32_t)MCI_GetMecSpeedRefUnit(pMCIN) * U_RPM) / SPEED_UNIT);
              break;
            }
            case MC_REG_MOTOR_POWER:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = PQD_GetAvrgElMotorPowerW(pMPM[M1]);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }
            default:
            {
              retVal = MCP_ERROR_UNKNOWN_REG;
              break;
            }
          }
          *size = 4;
        }
        else
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
        break;
      }

      case TYPE_DATA_STRING:
      {
        char_t *charData = (char_t *)data;
        switch (regID)
        {
          case MC_REG_FW_NAME:
            retVal = RI_MovString (FIRMWARE_NAME ,charData, size, freeSpace);
            break;

          case MC_REG_CTRL_STAGE_NAME:
          {
            retVal = RI_MovString (CTL_BOARD ,charData, size, freeSpace);
            break;
          }

          case MC_REG_PWR_STAGE_NAME:
          {
            retVal = RI_MovString (PWR_BOARD_NAME[motorID] ,charData, size, freeSpace);
            break;
          }

          case MC_REG_MOTOR_NAME:
          {
            retVal = RI_MovString (MotorConfig_reg[motorID]->name ,charData, size, freeSpace);
            break;
          }
          default:
          {
            retVal = MCP_ERROR_UNKNOWN_REG;
            *size= 0 ; /* */
            break;
          }
        }
        break;
      }

      case TYPE_DATA_RAW:
      {
        /* First 2 bytes of the answer is reserved to the size */
        uint16_t *rawSize = (uint16_t *)data; //cstat !MISRAC2012-Rule-11.3
        uint8_t * rawData = data;
        rawData++;
        rawData++;

        switch (regID)
        {
          case MC_REG_GLOBAL_CONFIG:
          {
            *rawSize = (uint16_t)sizeof(GlobalConfig_reg_t);
            if (((*rawSize) + 2U) > freeSpace)
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else
            {
              (void)memcpy(rawData, &globalConfig_reg, sizeof(GlobalConfig_reg_t));
            }
            break;
          }

          case MC_REG_MOTOR_CONFIG:
          {
            *rawSize = (uint16_t)sizeof(MotorConfig_reg_t);
            if (((*rawSize) + 2U) > freeSpace)
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else
            {
              MotorConfig_reg_t const *pMotorConfig_reg = MotorConfig_reg[motorID];
              (void)memcpy(rawData, (uint8_t *)pMotorConfig_reg, sizeof(MotorConfig_reg_t));
            }
            break;
          }
          case MC_REG_APPLICATION_CONFIG:
          {
            *rawSize = sizeof(ApplicationConfig_reg_t);
            if ((*rawSize) +2  > freeSpace)
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else
            {
              memcpy(rawData, ApplicationConfig_reg[motorID], sizeof(ApplicationConfig_reg_t));
            }
            break;
          }
          case MC_REG_FOCFW_CONFIG:
          {
            *rawSize = (uint16_t)sizeof(FOCFwConfig_reg_t);
            if (((*rawSize) + 2U) > freeSpace)
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else
            {
              FOCFwConfig_reg_t const *pFOCConfig_reg = FOCConfig_reg[motorID];
              (void)memcpy(rawData, (uint8_t *)pFOCConfig_reg, sizeof(FOCFwConfig_reg_t));
            }
            break;
          }
          case MC_REG_SPEED_RAMP:
          {
            uint16_t *rpm16p = (uint16_t *)rawData; //cstat !MISRAC2012-Rule-11.3
            uint16_t *duration = (uint16_t *)&rawData[4]; //cstat !MISRAC2012-Rule-11.3
            int32_t rpm32 = ((int32_t)(MCI_GetLastRampFinalSpeed(pMCIN) * U_RPM) / (int32_t)SPEED_UNIT);
            *rpm16p = (uint16_t) rpm32;
            *(rpm16p+1) = (uint16_t)(rpm32>>16);
            *duration = MCI_GetLastRampFinalDuration(pMCIN);
            *rawSize = 6;
            break;
          }
          case MC_REG_TORQUE_RAMP:
          {
            int16_t *torque = (int16_t *)rawData; //cstat !MISRAC2012-Rule-11.3
            uint16_t *duration = (uint16_t *)&rawData[2]; //cstat !MISRAC2012-Rule-11.3

            *rawSize = 4;
            *torque = MCI_GetLastRampFinalTorque(pMCIN);
            *duration = MCI_GetLastRampFinalDuration(pMCIN) ;
            break;
          }

          case MC_REG_CURRENT_REF:
          {
            uint16_t *iqref = (uint16_t *)rawData; //cstat !MISRAC2012-Rule-11.3
            uint16_t *idref = (uint16_t *)&rawData[2]; //cstat !MISRAC2012-Rule-11.3

            *rawSize = 4;
            *iqref = (uint16_t)MCI_GetIqdref(pMCIN).q;
            *idref = (uint16_t)MCI_GetIqdref(pMCIN).d;
            break;
    }

          case MC_REG_ASYNC_UARTA:
          case MC_REG_ASYNC_UARTB:
          case MC_REG_ASYNC_STLNK:
          default:
          {
            retVal = MCP_ERROR_UNKNOWN_REG;
            break;
          }
        }

        /* Size of the answer is size of the data + 2 bytes containing data size*/
        *size = (*rawSize) + 2U;
        break;
      }

      default:
      {
        retVal = MCP_ERROR_BAD_DATA_TYPE;
        break;
      }
    }
#ifdef NULL_PTR_REG_INT
  }
#endif
  return (retVal);
}
uint8_t RI_MovString(const char_t *srcString, char_t *destString, uint16_t *size, int16_t maxSize)
{
  uint8_t retVal = MCP_CMD_OK;

  const char_t *tempsrcString = srcString;
  char_t *tempdestString = destString;
  *size= 1U ; /* /0 is the min String size */

  while ((*tempsrcString != (char_t)0) && (*size < maxSize))
  {
    *tempdestString = *tempsrcString;
    tempdestString++;
    tempsrcString++;
    *size = *size + 1U;
  }

  if (*tempsrcString != (char_t)0)
  { /* Last string char must be 0 */
    retVal = MCP_ERROR_STRING_FORMAT;
  }
  else
  {
    *tempdestString = (int8_t)0;
  }

  return (retVal);
}
uint8_t RI_GetIDSize(uint16_t dataID)
{
  uint8_t typeID = ((uint8_t)dataID) & TYPE_MASK;
  uint8_t result;
  switch (typeID)
  {
    case TYPE_DATA_8BIT:
    {
      result = 1;
      break;
    }

    case TYPE_DATA_16BIT:
    {
      result = 2;
      break;
    }

    case TYPE_DATA_32BIT:
    {
      result = 4;
      break;
    }

    default:
    {
      result=0;
      break;
    }
  }

  return (result);
}

__weak uint8_t RI_GetPtrReg(uint16_t dataID, void **dataPtr)
{
  uint8_t retVal = MCP_CMD_OK;
  static uint16_t nullData16=0;

#ifdef NULL_PTR_REG_INT
  if (MC_NULL == dataPtr)
  {
    retVal = MCP_CMD_NOK;
  }
  else
  {
#endif

    uint8_t vmotorID = 0U;

    MCI_Handle_t *pMCIN = &Mci[vmotorID];
    uint16_t regID = dataID & REG_MASK;
    uint8_t typeID = ((uint8_t)dataID) & TYPE_MASK;

    switch (typeID)
    {
      case TYPE_DATA_16BIT:
      {
        switch (regID)
        {
          case MC_REG_I_A:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iab.a);
             break;
          }

          case MC_REG_I_B:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iab.b);
            break;
          }

          case MC_REG_I_ALPHA_MEAS:
          {
            *dataPtr = &(pMCIN->pFOCVars->Ialphabeta.alpha);
            break;
          }

          case MC_REG_I_BETA_MEAS:
          {
            *dataPtr = &(pMCIN->pFOCVars->Ialphabeta.beta);
            break;
          }

          case MC_REG_I_Q_MEAS:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iqd.q);
            break;
          }

          case MC_REG_I_D_MEAS:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iqd.d);
            break;
          }

          case MC_REG_I_Q_REF:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iqdref.q);
            break;
          }

          case MC_REG_I_D_REF:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iqdref.d);
            break;
          }
          case MC_REG_V_Q:
          {
            *dataPtr = &(pMCIN->pFOCVars->Vqd.q);
            break;
          }

          case MC_REG_V_D:
          {
            *dataPtr = &(pMCIN->pFOCVars->Vqd.d);
            break;
          }

          case MC_REG_V_ALPHA:
          {
            *dataPtr = &(pMCIN->pFOCVars->Valphabeta.alpha);
            break;
          }

          case MC_REG_V_BETA:
          {
            *dataPtr = &(pMCIN->pFOCVars->Valphabeta.beta);
            break;
          }

          case MC_REG_ENCODER_SPEED:
          {
            *dataPtr = &(pEncoder[vmotorID]->_Super.hAvrMecSpeedUnit);
            break;
          }

          case MC_REG_ENCODER_EL_ANGLE:
          {
            *dataPtr = &(pEncoder[vmotorID]->_Super.hElAngle);
            break;
          }

          default:
          {
            *dataPtr = &nullData16;
            retVal = MCP_ERROR_UNKNOWN_REG;
            break;
          }
        }
        break;
      }

      default:
      {
        *dataPtr = &nullData16;
        retVal = MCP_ERROR_UNKNOWN_REG;
        break;
      }
    }
#ifdef NULL_PTR_REG_INT
  }
#endif
  return (retVal);
}

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
