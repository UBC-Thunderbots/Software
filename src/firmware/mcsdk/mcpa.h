/**
  ******************************************************************************
  * @file    mcpa.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the Datalog
  *          of the Motor Control SDK.
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
  *
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MCPA_H
#define MCPA_H

#include "mcptl.h"
extern uint32_t GLOBAL_TIMESTAMP;



typedef struct
{
  MCTL_Handle_t *pTransportLayer;
  void ** dataPtrTable;
  void ** dataPtrTableBuff;
  uint8_t *dataSizeTable;
  uint8_t *dataSizeTableBuff;  
  uint8_t *currentBuffer;
  uint16_t bufferIndex;
  uint16_t bufferTxTrigger;
  uint16_t bufferTxTriggerBuff;
#ifdef MCP_DEBUG_METRICS
  uint16_t bufferMissed;
#endif
  uint8_t nbrOfDataLog;
  uint8_t HFIndex;
  uint8_t MFIndex;
  uint8_t HFRate;
  uint8_t HFRateBuff;
  uint8_t HFNum;
  uint8_t HFNumBuff;
  uint8_t MFRate;
  uint8_t MFRateBuff;
  uint8_t MFNum;
  uint8_t MFNumBuff;
  uint8_t Mark;
  uint8_t MarkBuff;
} MCPA_Handle_t; /* MCP Async handle type*/


void MCPA_dataLog(MCPA_Handle_t *pHandle);
uint8_t MCPA_cfgLog(MCPA_Handle_t *pHandle, uint8_t *cfgdata);
void MCPA_flushDataLog (MCPA_Handle_t *pHandle);

#endif /* MCPA_H */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
