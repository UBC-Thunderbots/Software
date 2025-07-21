  /******************************************************************************
  * @file    mcpa.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the datalog feature
  *          of the MCP protocol
  *
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "string.h"
#include "mcp.h"
#include "register_interface.h"
#include "mcpa.h"

uint32_t GLOBAL_TIMESTAMP = 0U;
static void MCPA_stopDataLog(MCPA_Handle_t *pHandle);

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MCP
  * @{
  */

/**
  * @brief  Allocates and fills buffer with asynchronous data to be sent to controller
  *
  * @param  *pHandle Pointer to the MCPA Handle
  */
void MCPA_dataLog(MCPA_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_MCPA
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    uint32_t *logValue;
    uint16_t *logValue16;
    uint8_t i;

    if (pHandle->HFIndex == pHandle->HFRateBuff) /*  */
    {
      pHandle->HFIndex = 0U;
      if (0U == pHandle->bufferIndex)
      {
        /* New buffer allocation */
        if (0U == pHandle->pTransportLayer->fGetBuffer (pHandle->pTransportLayer,
                                                       (void **) &pHandle->currentBuffer, //cstat !MISRAC2012-Rule-11.3
                                                       MCTL_ASYNC))
        {
          /* Nothing to do, try next HF Task to get an Async buffer */
#ifdef MCP_DEBUG_METRICS
          pHandle->bufferMissed++;
#endif
        }
        else
        {
          logValue = (uint32_t *)pHandle->currentBuffer; //cstat !MISRAC2012-Rule-11.3
          *logValue = GLOBAL_TIMESTAMP; /* 32 first bits is used to store Timestamp */
          pHandle->bufferIndex = 4U;
          pHandle->MFIndex = 0U; /* Restart the motif from scratch at each buffer */
          /* Check if configuration has changed for this new buffer */
          if (pHandle->Mark == pHandle->MarkBuff)
          {
            /* Nothing to do */
          }
          else
          {
            pHandle->MarkBuff            = pHandle->Mark;
            pHandle->HFNumBuff           = pHandle->HFNum;
            pHandle->MFNumBuff           = pHandle->MFNum;
            pHandle->HFRateBuff          = pHandle->HFRate;
            pHandle->MFRateBuff          = pHandle->MFRate;
            pHandle->bufferTxTriggerBuff = pHandle->bufferTxTrigger;

            /* We store pointer here, so 4 bytes */
            (void)memcpy(pHandle->dataPtrTableBuff, pHandle->dataPtrTable,
                         ((uint32_t)pHandle->HFNum + (uint32_t)pHandle->MFNum) * 4U); /* We store pointer here,
                                                                                         so 4 bytes */
            (void)memcpy(pHandle->dataSizeTableBuff, pHandle->dataSizeTable,
                         (uint32_t)pHandle->HFNum + (uint32_t)pHandle->MFNum); /* 1 size byte per ID */
          }
        }
      }
      else
      {
        /* Nothing to do */
      }

      /* */
      if ((pHandle->bufferIndex > 0U)  && (pHandle->bufferIndex <= pHandle->bufferTxTriggerBuff))
      {
        logValue16 = (uint16_t *)&pHandle->currentBuffer[pHandle->bufferIndex]; //cstat !MISRAC2012-Rule-11.3
        for (i = 0U; i < pHandle->HFNumBuff; i++)
        {
          *logValue16 = *((uint16_t *) pHandle->dataPtrTableBuff[i]) ; //cstat !MISRAC2012-Rule-11.5
          logValue16++;
          pHandle->bufferIndex = pHandle->bufferIndex + 2U;
        }
        /* MFRateBuff=254 means we dump MF data once per buffer */
        /* MFRateBuff=255 means we do not dump MF data */
        if (pHandle->MFRateBuff < 254U)
        {
          if (pHandle->MFIndex == pHandle->MFRateBuff)
          {
            pHandle->MFIndex = 0U;
            for (i = pHandle->HFNumBuff; i < (pHandle->MFNumBuff + pHandle->HFNumBuff); i++)
            {
              /* Dump MF data */
              logValue = (uint32_t *)&pHandle->currentBuffer[pHandle->bufferIndex]; //cstat !MISRAC2012-Rule-11.3
              *logValue = *((uint32_t *)pHandle->dataPtrTableBuff[i]); //cstat !MISRAC2012-Rule-11.5

#ifdef NOT_IMPLEMENTED /* Code not implemented. */
              switch (pHandle->dataSizeTableBuff[i])
              {
                case 1:
                {
                  logValue8 = (uint8_t *)&pHandle->currentBuffer[pHandle->bufferIndex];
                  *logValue8 = *((uint8_t *)pHandle->dataPtrTableBuff[i]);
                  break;
                }
                case 2:
                {
                  logValue16 = (uint16_t *)&pHandle->currentBuffer[pHandle->bufferIndex];
                  *logValue16 = *((uint16_t *)pHandle->dataPtrTableBuff[i]);
                  break;
                }
                case 4:
                {
                  logValue32 = (uint32_t *)&pHandle->currentBuffer[pHandle->bufferIndex];
                  *logValue32 = *((uint32_t *)pHandle->dataPtrTableBuff[i]);
                  break;
                }
                default:
                  break;
              }
#endif
              pHandle->bufferIndex = pHandle->bufferIndex+pHandle->dataSizeTableBuff[i];
            }
          }
          else
          {
            pHandle->MFIndex ++;
          }
        }
        else
        {
          /* Nothing to do */
        }
      }
      else
      {
        /* Nothing to do */
      }
      if (pHandle->bufferIndex > pHandle->bufferTxTriggerBuff)
      {
        if (pHandle->MFRateBuff == 254U) /* MFRateBuff = 254 means we dump MF data once per buffer */
        {
          for (i = pHandle->HFNumBuff; i < (pHandle->MFNumBuff + pHandle->HFNumBuff); i++)
          {
            logValue = (uint32_t *)&pHandle->currentBuffer[pHandle->bufferIndex]; //cstat !MISRAC2012-Rule-11.3
            *logValue = *((uint32_t *)pHandle->dataPtrTableBuff[i]); //cstat !MISRAC2012-Rule-11.5
            pHandle->bufferIndex = pHandle->bufferIndex + pHandle->dataSizeTableBuff[i];
          }
        }
        else
        {
          /* Nothing to do */
        }
        /* Buffer is ready to be send */
        logValue16 = (uint16_t *)&pHandle->currentBuffer[pHandle->bufferIndex]; //cstat !MISRAC2012-Rule-11.3
        *logValue16 = pHandle->MarkBuff; /* MarkBuff is actually 8 bits, but we add also 8 bits of the ASYNCID=0 after
                                            the MARK. */
        pHandle->pTransportLayer->fSendPacket(pHandle->pTransportLayer, pHandle->currentBuffer,
                                              pHandle->bufferIndex + 2U, MCTL_ASYNC);
        pHandle->bufferIndex = 0U;
      }
      else
      {
        /* Nothing to do */
      }
    }
    else
    {
      /* Nothing to log just waiting next call to MCPA_datalog */
      pHandle->HFIndex++;
    }
#ifdef NULL_PTR_CHECK_MCPA
  }
#endif
}

/**
  * @brief  Sends asynchronous data to controller when the buffer is full
  *
  * @param  *pHandle Pointer to the MCPA Handle
  */
void MCPA_flushDataLog (MCPA_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_MCPA
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    uint32_t *logValue;
    uint16_t *logValue16;
    uint8_t i;

    if (pHandle->bufferIndex > 0U)
    {  /* If buffer is allocated, we must send it */
      if (pHandle->MFRateBuff == 254U) /* In case of flush, we must respect the packet format to allow
                                          proper decoding */
      {
        for (i = pHandle->HFNumBuff; i < (pHandle->MFNumBuff + pHandle->HFNumBuff); i++)
        {
         logValue = (uint32_t *)&pHandle->currentBuffer[pHandle->bufferIndex]; //cstat !MISRAC2012-Rule-11.3
         *logValue = *((uint32_t *)pHandle->dataPtrTableBuff[i]); //cstat !MISRAC2012-Rule-11.5
         pHandle->bufferIndex = pHandle->bufferIndex+pHandle->dataSizeTableBuff[i];
        }
      }
      else
      {
        /* Nothing to do */
      }
      logValue16 = (uint16_t *)&pHandle->currentBuffer[pHandle->bufferIndex]; //cstat !MISRAC2012-Rule-11.3
      *logValue16 = pHandle->MarkBuff; /* MarkBuff is actually 8 bits, but we add also 8 bits of the ASYNCID=0 after
                                          the MARK */
      pHandle->pTransportLayer->fSendPacket (pHandle->pTransportLayer, pHandle->currentBuffer,
                                             pHandle->bufferIndex + 2U, MCTL_ASYNC);
      pHandle->bufferIndex = 0U;
    }
    else
    {
      /* Nothing to do */
    }
#ifdef NULL_PTR_CHECK_MCPA
  }
#endif
}

/**
  * @brief  Stops the asynchronous communication
  *
  * @param  *pHandle Pointer to the MCPA Handle
  */
void MCPA_stopDataLog(MCPA_Handle_t *pHandle)
{
  uint16_t *logValue16;

  pHandle->Mark = 0U;
  if (pHandle->bufferIndex > 0U)
  { /* If buffer is allocated, we must send it */
    logValue16 = (uint16_t *)&pHandle->currentBuffer[pHandle->bufferIndex]; //cstat !MISRAC2012-Rule-11.3
    *logValue16 = pHandle->MarkBuff; /* MarkBuff is actually 8 bits, but we add also 8 bits of the ASYNCID=0 after
                                        the MARK */
    pHandle->pTransportLayer->fSendPacket (pHandle->pTransportLayer, pHandle->currentBuffer,
                                           pHandle->bufferIndex + 2U, MCTL_ASYNC);
  }
  else
  {
    /* Nothing to do */
  }
  pHandle->bufferIndex = 0U;
  pHandle->MarkBuff    = 0U;
  pHandle->HFIndex     = 0U;
  pHandle->HFRateBuff  = 0U; /* We do not want to miss any sample at the restart */
}

/**
  * @brief  Stores the asynchronous configuration stating all the register to be continuously sent to controller
  *
  * @param  *pHandle Pointer to the MCPA Handle
  * @param  *cfgdata Configuration of the Async communication
  */
uint8_t MCPA_cfgLog(MCPA_Handle_t *pHandle, uint8_t *cfgdata)
{
  uint8_t result = MCP_CMD_OK;

#ifdef NULL_PTR_CHECK_MCPA
  if (MC_NULL == pHandle)
  {
    result = MCP_CMD_NOK;
  }
  else
  {
#endif
    uint8_t i;
    uint16_t logSize = 0U; /* Max size of a log per iteration (HF+MF) */
    uint16_t newID, buffSize;
    uint8_t *pCfgData = cfgdata;

    buffSize = *((uint16_t *)pCfgData); //cstat !MISRAC2012-Rule-11.3

    if (buffSize == 0U)
    { 
      /* Switch Off condition */
      MCPA_stopDataLog(pHandle);
    }
    else if (buffSize > pHandle->pTransportLayer->txAsyncMaxPayload)
    {
      result = MCP_ERROR_NO_TXASYNC_SPACE;
    }
    else
    {
      pHandle->HFRate = *((uint8_t *)&pCfgData[2]);
      pHandle->HFNum  = *((uint8_t *)&pCfgData[3]);
      pHandle->MFRate = *((uint8_t *)&pCfgData[4]);
      pHandle->MFNum  = *((uint8_t *)&pCfgData[5]);
      pCfgData = &pCfgData[6]; /* Start of the HF IDs */

      if ((pHandle->HFNum + pHandle->MFNum) <= pHandle->nbrOfDataLog)
      {
        for (i = 0; i < (pHandle->HFNum + pHandle->MFNum); i++)
        {
          newID = *((uint16_t *)pCfgData); //cstat !MISRAC2012-Rule-11.3
          (void)RI_GetPtrReg(newID, &pHandle->dataPtrTable[i]);
          /* HF Data are fixed to 2 bytes */
          pHandle->dataSizeTable[i] = (i < pHandle->HFNum ) ? 2U : RI_GetIDSize(newID);
          pCfgData++; /* Point to the next UID */
          pCfgData++;
          logSize = logSize+pHandle->dataSizeTable[i];
        }

        /* Smallest packet must be able to contain logSize Markbyte AsyncID and TimeStamp */
        if (buffSize < (logSize + 2U + 4U))
        {
          result = MCP_ERROR_NO_TXASYNC_SPACE;
        }
        else
        {
          pHandle->bufferTxTrigger = buffSize-logSize - 2U; /* 2 is required to add the last Mark byte and NUL
                                                               ASYNCID */
          pHandle->Mark = *((uint8_t *)pCfgData);
          if (0U == pHandle->Mark)
          {  /* Switch Off condition */
            MCPA_stopDataLog(pHandle);
          }
          else
          {
            /* Nothing to do */
          }
        }
      }
      else
      {
        result = MCP_ERROR_BAD_RAW_FORMAT;
      }
    }
#ifdef NULL_PTR_CHECK_MCPA
  }
#endif
  return (result);
}

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
  
