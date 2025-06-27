/**
  ******************************************************************************
  * @file    mcptl.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware definitions of the Motor control protocol transport layer
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

#ifndef MC_TRANSPORT_LAYER
#define MC_TRANSPORT_LAYER

#define MCTL_SYNC  ( uint8_t )0xAU
#define MCTL_ASYNC ( uint8_t )0x9U

#define MCTL_SYNC_NOT_EXPECTED 1


typedef struct MCTL_Handle MCTL_Handle_t; //cstat !MISRAC2012-Rule-2.4
typedef bool (* MCTL_GetBuf)(MCTL_Handle_t *pHandle, void **buffer, uint8_t syncAsync);
typedef uint8_t (* MCTL_SendPacket)(MCTL_Handle_t *pHandle, void *txBuffer, uint16_t txDataLength, uint8_t syncAsync);
typedef uint8_t *(* MCTL_RXpacketProcess)(MCTL_Handle_t *pHandle, uint16_t *packetLength);

typedef enum
{
  available = 0,
  writeLock = 1,
  pending = 2,
  readLock = 3,
} buff_access_t;

typedef struct
{
  uint8_t *buffer;
  uint16_t length;
  buff_access_t state;
#ifdef MCP_DEBUG_METRICS
  /* Debug metrics */
  uint16_t SentNumber;
  uint16_t PendingNumber;
  uint16_t RequestedNumber;
  /* End of Debug metrics */
#endif
} MCTL_Buff_t;

struct MCTL_Handle
{
  MCTL_GetBuf fGetBuffer;
  MCTL_SendPacket fSendPacket;
  MCTL_RXpacketProcess fRXPacketProcess;
  uint16_t txSyncMaxPayload;
  uint16_t txAsyncMaxPayload;
  bool MCP_PacketAvailable; /* Packet available for Motor control protocol*/
} ;

bool MCTL_decodeCRCData(MCTL_Handle_t *pHandle);

#endif

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
