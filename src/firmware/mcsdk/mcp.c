/**
  ******************************************************************************
  * @file    mcp.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the MCP protocol
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

#include "mc_type.h"
#include "mcp.h"
#include "register_interface.h"
#include "mc_config.h"
#include "mcp_config.h"
#include "mc_api.h"

void MCP_ReceivedPacket(MCP_Handle_t *pHandle)
{
  uint16_t *packetHeader;
  uint16_t command;
  uint8_t motorID;
  uint8_t MCPResponse;
  uint8_t userCommand=0;
  int16_t txSyncFreeSpace;
#ifdef NULL_PTR_CHECK_MCP
  if ((MC_NULL == pHandle) || (0U == pHandle->rxLength))
  {
    /* Nothing to do, txBuffer and txLength have not been modified */
  }
  else /* Length is 0, this is a request to send back the last packet */
  {
#endif
    packetHeader = (uint16_t *)pHandle->rxBuffer; //cstat !MISRAC2012-Rule-11.3
    command = (uint16_t)(*packetHeader & CMD_MASK);
    if ((command & MCP_USER_CMD_MASK) == MCP_USER_CMD)
    {
      userCommand = (command >> 3) & 0x1f;
      command = MCP_USER_CMD;    	
    }

    motorID = (uint8_t)((*packetHeader - 1U) & MOTOR_MASK);

    
    MCI_Handle_t *pMCI = &Mci[motorID];
    /* Removing MCP Header from RxBuffer*/
    pHandle->rxLength = pHandle->rxLength-MCP_HEADER_SIZE;
    pHandle->rxBuffer = pHandle->rxBuffer+MCP_HEADER_SIZE;
    /* Commands requiering payload response must be aware of space available for the payload*/
    txSyncFreeSpace = pHandle->pTransportLayer->txSyncMaxPayload-1; /* Last byte is reserved for MCP response*/
    /* Initialization of the tx length, command which send back data has to increment the txLength
     * (case of Read register) */
    pHandle->txLength = 0;

    switch (command)
    {
      case GET_MCP_VERSION:
        pHandle->txLength = 4;
        *pHandle->txBuffer = (uint32_t) MCP_VERSION;
        MCPResponse = MCP_CMD_OK;
      break;
      case SET_DATA_ELEMENT:
      {
        MCPResponse = RI_SetRegCommandParser(pHandle, txSyncFreeSpace);
        break;
      }

      case GET_DATA_ELEMENT:
      {
        MCPResponse = RI_GetRegCommandParser(pHandle, txSyncFreeSpace);
        break;
      }

      case START_MOTOR:
      {
        MCPResponse = (MCI_StartMotor(pMCI)) ? MCP_CMD_OK : MCP_CMD_NOK;
        break;
      }

      case STOP_MOTOR: /* Todo: Check the pertinance of return value*/
      {
        (void)MCI_StopMotor(pMCI);
        MCPResponse = MCP_CMD_OK;
        break;
      }

      case STOP_RAMP:
      {
        if (RUN == MCI_GetSTMState(pMCI))
        {
          MCI_StopRamp(pMCI);
        }
        MCPResponse = MCP_CMD_OK;
        break;
      }

      case START_STOP:
      {
        /* Queries the STM and a command start or stop depending on the state. */
        if (IDLE == MCI_GetSTMState(pMCI))
        {
          MCPResponse = (MCI_StartMotor(pMCI)) ? MCP_CMD_OK : MCP_CMD_NOK;
        }
        else
        {
          (void)MCI_StopMotor(pMCI);
          MCPResponse = MCP_CMD_OK;
        }
        break;
      }

      case FAULT_ACK:
      {
        (void)MCI_FaultAcknowledged(pMCI);
        MCPResponse = MCP_CMD_OK;
        break;
      }

      case IQDREF_CLEAR:
      {
        MCI_Clear_Iqdref(pMCI);
        MCPResponse = MCP_CMD_OK;
        break;
      }
      case PFC_ENABLE:
      case PFC_DISABLE:
      case PFC_FAULT_ACK:
      {
        MCPResponse = MCP_CMD_UNKNOWN;
        break;
      }      
      case PROFILER_CMD:
      {
  	    MCPResponse = MC_ProfilerCommand(pHandle->rxLength, pHandle->rxBuffer, txSyncFreeSpace, &pHandle->txLength, pHandle->txBuffer);
        break;
      }
      
      case MCP_USER_CMD:
    	  if (userCommand < MCP_USER_CALLBACK_MAX && MCP_UserCallBack[userCommand] != NULL)
    	  {
    	    MCPResponse = MCP_UserCallBack[userCommand](pHandle->rxLength, pHandle->rxBuffer, txSyncFreeSpace, &pHandle->txLength, pHandle->txBuffer);
    	  }
    	  else
    	  {
          MCPResponse = MCP_ERROR_CALLBACK_NOT_REGISTRED;
    	  }
        break;
      default :
      {
        MCPResponse = MCP_CMD_UNKNOWN;
        break;
      }
    }
    pHandle->txBuffer[pHandle->txLength] = MCPResponse;
    pHandle->txLength++;
#ifdef NULL_PTR_CHECK_MCP
  }
#endif
}

uint8_t MCP_RegisterCallBack (uint8_t callBackID, MCP_user_cb_t fctCB)
{
	uint8_t result;

  if (callBackID < MCP_USER_CALLBACK_MAX)
  {
	  MCP_UserCallBack[callBackID] = fctCB;
	  result = MCP_CMD_OK;
  }
  else
  {
	  result = MCP_CMD_NOK;
  }
  return result;
}

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
