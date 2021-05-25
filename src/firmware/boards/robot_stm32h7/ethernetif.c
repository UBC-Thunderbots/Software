/**
 ******************************************************************************
 * File Name          : ethernetif.c
 * Description        : This file provides code for the configuration
 *                      of the ethernetif.c MiddleWare.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "ethernetif.h"

#include "lwip/ethip6.h"
#include "lwip/opt.h"
#include "lwip/timeouts.h"
#include "main.h"
#include "netif/etharp.h"
#include "netif/ethernet.h"
/* USER CODE BEGIN Include for User BSP */
#include "lan8742.h"

/* USER CODE END Include for User BSP */
#include <string.h>

#include "cmsis_os.h"
#include "lwip/tcpip.h"
#include "queue.h"
#include "semphr.h"

/* Within 'USER CODE' section, code will be kept by default at each generation */
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/* Private define ------------------------------------------------------------*/
/* The time to block waiting for input. */
#define TIME_WAITING_FOR_INPUT (portMAX_DELAY)
/* Stack size of the interface thread */
#define INTERFACE_THREAD_STACK_SIZE (1024)
/* Network interface name */
#define IFNAME0 's'
#define IFNAME1 't'


/* ETH_RX_BUFFER_SIZE
 *  - ETH_RX_BUFFER_SIZE should be a multiple of the cache line size. For STM32H7, cache
 * lines are 32-bytes.
 *  - The implementation is zero-copy. Packets are received directly to buffers. Buffers
 * chain to receive packets larger than ETH_RX_BUFFER_SIZE.
 *  - Is the length of the receive packets known? To conserve memory, ETH_RX_BUFFER_SIZE
 * may be reduced to the expected maximum packet length.
 *
 * ETH_RX_BUFFER_COUNT
 *  - ETH_RX_BUFFER_COUNT must be greater than or equal to ETH_RX_DESC_CNT.
 *  - Is IP reassembly (lwipopts.h IP_REASSEMBLY) necessary? If it is, ETH_RX_BUFFER_COUNT
 * should be much greater than ETH_RX_DESC_CNT because lwIP holds the buffers of
 * fragmented packets until they're reassembled or timed out.
 *  - If packets may arrive faster than they can be processed, ETH_RX_BUFFER_COUNT should
 * increase with their burstiness.
 */
#define ETH_RX_BUFFER_COUNT                                                              \
    125 /* This app buffers receive packets of its primary service                       \
         * protocol for processing later. */
#define ETH_RX_BUFFERS_ARE_CACHED                                                        \
    0 /* To conserve memory, this app positions its Rx Buffers                           \
       * (".RxArraySection" section) in a not-cacheable MPU region. */

#define ETH_TX_BUFFER_MAX                                                                \
    ((ETH_TX_DESC_CNT)*2)           /* HAL_ETH_Transmit(_IT) may attach two              \
                                     * buffers per descriptor. */
#define ETH_TX_BUFFERS_ARE_CACHED 1 /* Tx buffers are in normal, cached memory. */
#define ETH_TX_QUEUE_ENABLE                                                              \
    1 /* Queue to reduce tx-blocking. Only enable if you know the app                    \
       * won't change pbufs after transmit. */
#define ETH_TX_QUEUE_SIZE                                                                \
    ((ETH_TX_DESC_CNT)-2) /* Dimension the tx queue slightly smaller than                \
                           * the Tx Descriptor Count because, if the descriptors are     \
                           * dimensioned well, it's less cycles wait for a previous      \
                           * transmit to complete than for HAL_ETH_Transmit_IT to fail   \
                           * because there aren't enough descriptors and to have to wait \
                           * and re-try. */

#define ETH_DMA_TRANSMIT_TIMEOUT (20U)

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* Private variables ---------------------------------------------------------*/
/*
@Note: This interface is implemented to operate in zero-copy mode only:
        - Rx buffers are allocated statically and passed directly to the LwIP stack
          they will return back to DMA after been processed by the stack.
        - Tx Buffers will be allocated from LwIP stack memory heap,
          then passed to ETH HAL driver.

@Notes:
  1.a. ETH DMA Rx descriptors must be contiguous, the default count is 4,
       to customize it please redefine ETH_RX_DESC_CNT in ETH GUI (Rx Descriptor Length)
       so that updated value will be generated in stm32xxxx_hal_conf.h
  1.b. ETH DMA Tx descriptors must be contiguous, the default count is 4,
       to customize it please redefine ETH_TX_DESC_CNT in ETH GUI (Tx Descriptor Length)
       so that updated value will be generated in stm32xxxx_hal_conf.h

  2.a. Rx Buffers number: ETH_RX_BUFFER_COUNT must be greater than ETH_RX_DESC_CNT.
  2.b. Rx Buffers must have the same size: ETH_RX_BUFFER_SIZE, this value must
       passed to ETH DMA in the init field (heth.Init.RxBuffLen)
*/


/* RxBuff_t is an Rx Buffer container for the ETH driver to zero-copy receive into.
 * With lwIP, it contains a pbuf_custom and the buffer.
 * In production code, D-cache would be enabled. So the Rx Buffer memory is either
 * cached or in an MPU region configured not-cacheable or write-through.
 *
 * The Rx DMA writes direct to physical memory.
 * The EthIfRxBuff array interleaves pbuf_customs and buffers in memory.
 *
 * If the Rx Buffer memory is cached:
 *  - After Rx DMA has written a buffer, the buffer must be cache-invalidated before the
 *    core reads it, but without invalidating the pbuf_customs part. So
 *    - The EthIfRxBuff definition is cache-line aligned (perhaps in the link command
 * file),
 *    - The buffer offsets are cache-line aligned, and
 *    - The buffer lengths are rounded up to a multiple of cache-line size.
 *  - Cortex M7 cache lines are 32-bytes.
 *  - The memory size is:
 *      (ETH_RX_BUFFER_COUNT * (((sizeof(struct pbuf_custom) + 31) & ~31) +
 *        ((ETH_RX_BUFFER_SIZE + 31) & ~31)))
 *
 * If the Rx Buffer memory is not-cacheable or write-through:
 *  - The memory alignments may be relaxed, resulting a smaller Rx Buffer memory
 * footprint.
 *  - The memory size is:
 *      (ETH_RX_BUFFER_COUNT * (((sizeof(struct pbuf_custom) + 3) & ~3) +
 *        ((ETH_RX_BUFFER_SIZE + 3) & ~3)))
 *  - The MPU region size should be the lowest power of two greater than or equal the
 *    section's size, and its start address should be a multiple of its size.
 */
typedef struct
{
    struct pbuf_custom pbuf_custom;
#if ETH_RX_BUFFERS_ARE_CACHED
    uint8_t buff[(ETH_RX_BUFFER_SIZE + 31) & ~31] __ALIGNED(32);
#else
    uint8_t buff[(ETH_RX_BUFFER_SIZE + 3) & ~3] __ALIGNED(4);
#endif
} RxBuff_t;

#if defined(__ICCARM__) /*!< IAR Compiler */

#pragma location = 0x24000100
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location = 0x24000e38
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location = 0x24002000
RxBuff_t EthIfRxBuff[ETH_RX_BUFFER_COUNT]; /* Ethernet Rx Buffers */

#elif defined(__CC_ARM) /* MDK ARM Compiler */

__attribute__((at(0x24000100)))
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x24000e38)))
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x24002000)))
RxBuff_t EthIfRxBuff[ETH_RX_BUFFER_COUNT]; /* Ethernet Rx Buffers */

#elif defined(__GNUC__) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT]
    __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT]
    __attribute__((section(".TxDecripSection"))); /* Ethernet Tx DMA Descriptors */
RxBuff_t EthIfRxBuff[ETH_RX_BUFFER_COUNT]
    __attribute__((aligned(32), section(".RxArraySection"))); /* Ethernet Rx Buffers */

#endif


/* LwIP Memory pool descriptor for EthIfRxBuff.
 * This does the same as LWIP_MEMPOOL_DECLARE, except EthIfRxBuff's defined
 * separately for special section placement. */
#if MEMP_STATS
static struct stats_mem memp_stats_EthIfRxBuff;
#endif
static struct memp *memp_tab_EthIfRxBuff;
static const struct memp_desc memp_EthIfRxBuff = {
#if defined(LWIP_DEBUG) || MEMP_OVERFLOW_CHECK || LWIP_STATS_DISPLAY
    "RxBuff Pool",
#endif /* LWIP_DEBUG || MEMP_OVERFLOW_CHECK || LWIP_STATS_DISPLAY */
#if MEMP_STATS
    &memp_stats_EthIfRxBuff,
#endif
    LWIP_MEM_ALIGN_SIZE(sizeof(RxBuff_t)),
    ETH_RX_BUFFER_COUNT,
    (uint8_t *)EthIfRxBuff,
    &memp_tab_EthIfRxBuff};

/* Flag when RxBuffAlloc exhausts the Rx Buffer Pool, so the next RxPktDiscard will
 * repopulate the buffers of the Rx DMA Descriptors. */
static uint8_t RxBuffEmpty;


#if ETH_TX_QUEUE_ENABLE
/* Queue for packet transmit. */
typedef struct TxQueue_t
{
    struct TxQueue_t *next;
    struct pbuf *p;
} TxQueue_t;

static TxQueue_t TxQueue[ETH_TX_QUEUE_SIZE];
static TxQueue_t *TxQueueFree, *TxQueueHead, *TxQueueTail;
#endif /* ETH_TX_QUEUE_ENABLE */

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

TaskHandle_t EthIfThread;     /* Handle of the interface thread */
QueueHandle_t TxPktSemaphore; /* Semaphore to signal transmit packet complete */

/* Global Ethernet handle */
ETH_HandleTypeDef heth;
ETH_TxPacketConfig TxConfig;
ETH_BufferTypeDef Txbuffer[ETH_TX_BUFFER_MAX];

int32_t ETH_PHY_IO_Init(void);
int32_t ETH_PHY_IO_DeInit(void);
int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal);
int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal);
int32_t ETH_PHY_IO_GetTick(void);

lan8742_Object_t LAN8742;
lan8742_IOCtx_t LAN8742_IOCtx = {ETH_PHY_IO_Init, ETH_PHY_IO_DeInit, ETH_PHY_IO_WriteReg,
                                 ETH_PHY_IO_ReadReg, ETH_PHY_IO_GetTick};


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN Private function prototypes for User BSP */

/* USER CODE END Private function prototypes for User BSP */

/* USER CODE BEGIN 3 */

/* USER CODE END 3 */

/* Private functions ---------------------------------------------------------*/
void Error_Handler(void);

void HAL_ETH_MspInit(ETH_HandleTypeDef *ethHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (ethHandle->Instance == ETH)
    {
        /* USER CODE BEGIN ETH_MspInit 0 */

        /* USER CODE END ETH_MspInit 0 */
        /* Enable Peripheral clock */
        __HAL_RCC_ETH1MAC_CLK_ENABLE();
        __HAL_RCC_ETH1TX_CLK_ENABLE();
        __HAL_RCC_ETH1RX_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**ETH GPIO Configuration
        PC1     ------> ETH_MDC
        PA1     ------> ETH_REF_CLK
        PA2     ------> ETH_MDIO
        PA7     ------> ETH_CRS_DV
        PC4     ------> ETH_RXD0
        PC5     ------> ETH_RXD1
        PB11     ------> ETH_TX_EN
        PB12     ------> ETH_TXD0
        PB13     ------> ETH_TXD1
        */
        GPIO_InitStruct.Pin       = RMII_MDC_Pin | RMII_RXD0_Pin | RMII_RXD1_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = RMII_REF_CLK_Pin | RMII_MDIO_Pin | RMII_CRS_DV_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = RMII_TX_EN_Pin | RMII_TXD0_Pin | RMII_TXD1_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* Peripheral interrupt init */
        HAL_NVIC_SetPriority(ETH_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(ETH_IRQn);
        HAL_NVIC_SetPriority(ETH_WKUP_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(ETH_WKUP_IRQn);
        /* USER CODE BEGIN ETH_MspInit 1 */

        /* USER CODE END ETH_MspInit 1 */
    }
}

void HAL_ETH_MspDeInit(ETH_HandleTypeDef *ethHandle)
{
    if (ethHandle->Instance == ETH)
    {
        /* USER CODE BEGIN ETH_MspDeInit 0 */

        /* USER CODE END ETH_MspDeInit 0 */
        /* Disable Peripheral clock */
        __HAL_RCC_ETH1MAC_CLK_DISABLE();
        __HAL_RCC_ETH1TX_CLK_DISABLE();
        __HAL_RCC_ETH1RX_CLK_DISABLE();

        /**ETH GPIO Configuration
        PC1     ------> ETH_MDC
        PA1     ------> ETH_REF_CLK
        PA2     ------> ETH_MDIO
        PA7     ------> ETH_CRS_DV
        PC4     ------> ETH_RXD0
        PC5     ------> ETH_RXD1
        PB11     ------> ETH_TX_EN
        PB12     ------> ETH_TXD0
        PB13     ------> ETH_TXD1
        */
        HAL_GPIO_DeInit(GPIOC, RMII_MDC_Pin | RMII_RXD0_Pin | RMII_RXD1_Pin);

        HAL_GPIO_DeInit(GPIOA, RMII_REF_CLK_Pin | RMII_MDIO_Pin | RMII_CRS_DV_Pin);

        HAL_GPIO_DeInit(GPIOB, RMII_TX_EN_Pin | RMII_TXD0_Pin | RMII_TXD1_Pin);

        /* Peripheral interrupt Deinit*/
        HAL_NVIC_DisableIRQ(ETH_IRQn);

        HAL_NVIC_DisableIRQ(ETH_WKUP_IRQn);

        /* USER CODE BEGIN ETH_MspDeInit 1 */

        /* USER CODE END ETH_MspDeInit 1 */
    }
}

/**
 * @brief  Ethernet Rx Transfer completed callback
 * @param  heth: ETH handle
 * @retval None
 */
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
    portBASE_TYPE taskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(EthIfThread, &taskWoken);
    portYIELD_FROM_ISR(taskWoken);
}

/* USER CODE BEGIN 4 */

#if ETH_TX_QUEUE_ENABLE
/**
 * @brief  Ethernet Tx Transfer completed callback
 * @param  heth: ETH handle
 * @retval None
 */
void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *heth)
{
    portBASE_TYPE taskWoken = pdFALSE;
    xSemaphoreGiveFromISR(TxPktSemaphore, &taskWoken);
    portYIELD_FROM_ISR(taskWoken);
}
#endif /* ETH_TX_QUEUE_ENABLE */

/**
 * @brief  Ethernet DMA transfer error callback
 * @param  heth: ETH handle
 * @retval None
 */
void HAL_ETH_DMAErrorCallback(ETH_HandleTypeDef *heth)
{
    /* If a receive buffer unavailable error occurred, signal the
     * ethernetif_input task to call HAL_ETH_GetRxDataBuffer to rebuild the
     * Rx Descriptors and restart receive. */
    if (heth->DMAErrorCode & ETH_DMACSR_RBU)
    {
        portBASE_TYPE taskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(EthIfThread, &taskWoken);
        portYIELD_FROM_ISR(taskWoken);
    }
}


/**
 * @brief  Custom free for the Rx Buffer Pool.
 * @param  p Pointer to the packet buffer struct.
 * @retval None
 */
static void RxPoolCustomFree(struct pbuf *p)
{
    LWIP_MEMPOOL_FREE(EthIfRxBuff, p);

    /* If the Rx Buffer Pool was exhausted, signal the ethernetif_input task to
     * call HAL_ETH_GetRxDataBuffer to rebuild the Rx descriptors. */
    if (RxBuffEmpty)
    {
        xTaskNotifyGive(EthIfThread);
        RxBuffEmpty = 0;
    }
}

/**
 * @brief  Callback function given to the ETH driver at init to allocate a
 *         buffer.
 * @retval Pointer to the buffer, or NULL if no buffers are available.
 */
static uint8_t *RxBuffAlloc(void)
{
    struct pbuf_custom *p = LWIP_MEMPOOL_ALLOC(EthIfRxBuff);
    uint8_t *buff;

    if (p)
    {
        /* Get the buff from the struct pbuf address. */
        buff = (uint8_t *)p + offsetof(RxBuff_t, buff);

        /* Initialize the struct pbuf.
         * This must be performed whenever a buffer's allocated because it may be
         * changed by lwIP or the app, e.g., pbuf_free decrements ref. */
        pbuf_alloced_custom(PBUF_RAW, 0, PBUF_REF, p, buff, ETH_RX_BUFFER_SIZE);
    }
    else
    {
        /* Rx Buffer Pool is exhausted. */
        RxBuffEmpty = 1;
        buff        = NULL;
    }

    return buff;
}

/**
 * @brief  Callback function given to the ETH driver at init to free a buffer.
 * @param  buff Pointer to the buffer, or NULL if no buffers are available.
 * @retval None
 */
static void RxBuffFree(uint8_t *buff)
{
    struct pbuf *p;

    /* Get the struct pbuf from the buff address. */
    p = (struct pbuf *)(buff - offsetof(RxBuff_t, buff));
    RxPoolCustomFree(p);
}

/**
 * @brief  Callback function given to the ETH driver at init to chain buffers
 *         as a packet.
 * @param  ppPktFirst Double-pointer to the first packet buffer struct of the
 *         packet.
 * @param  ppPktLast Double-pointer to the last packet buffer struct of the
 *         packet.
 * @param  buff Pointer to the buffer.
 * @param  buffLength length of the buffer.
 * @retval None
 */
static void RxPktAssemble(void **ppPktFirst, void **ppPktLast, uint8_t *buff,
                          uint16_t buffLength)
{
    struct pbuf **ppFirst = (struct pbuf **)ppPktFirst;
    struct pbuf **ppLast  = (struct pbuf **)ppPktLast;
    struct pbuf *p;

    /* Get the struct pbuf from the buff address. */
    p          = (struct pbuf *)(buff - offsetof(RxBuff_t, buff));
    p->next    = NULL;
    p->tot_len = 0;
    p->len     = buffLength;

    /* Chain the buffer. */
    if (!*ppFirst)
    {
        /* The first buffer of the packet. */
        *ppFirst = p;
    }
    else
    {
        /* Chain the buffer to the end of the packet. */
        (*ppLast)->next = p;
    }
    *ppLast = p;

    /* Update the total length of all the buffers of the chain. Each pbuf in the chain
     * should have its tot_len
     * set to its own length, plus the length of all the following pbufs in the chain. */
    for (p = *ppFirst; p != NULL; p = p->next)
    {
        p->tot_len += buffLength;
    }

    /* Invalidating cache isn't necessary if the rx buffers are in a not-cacheable or
     * write-through MPU region. */
#if ETH_RX_BUFFERS_ARE_CACHED
    /* Invalidate data cache because Rx DMA's writing to physical memory makes it stale.
     */
    SCB_InvalidateDCache_by_Addr((uint32_t *)buff, buffLength);
#endif
}

/**
 * @brief  Callback function given to the ETH driver at init to discard a
 *         packet.
 * @param  pPkt Pointer to the packet buffer struct.
 * @retval None
 */
static void RxPktDiscard(void *pPkt)
{
    struct pbuf *p = (struct pbuf *)pPkt;
    struct pbuf *nextp;

    while (p)
    {
        nextp = p->next;
        RxPoolCustomFree(p);
        p = nextp;
    }
}

/**
 * @brief  Initialize the Rx Buffer Pool.
 * @retval None
 */
static void RxPoolInit(void)
{
    int k;

    /* Initialize the custom_free_function of each pbuf_custom here to save
     * cycles later. */
    for (k = 0; k < ETH_RX_BUFFER_COUNT; k++)
    {
        EthIfRxBuff[k].pbuf_custom.custom_free_function = RxPoolCustomFree;
    }

    /* Initialize the Rx Buffer Pool. */
    LWIP_MEMPOOL_INIT(EthIfRxBuff);
}


/* USER CODE END 4 */

/*******************************************************************************
                       LL Driver Interface ( LwIP stack --> ETH)
*******************************************************************************/
/**
 * @brief In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void low_level_init(struct netif *netif)
{
    HAL_StatusTypeDef hal_eth_init_status;
    /* USER CODE BEGIN low_level_init Variables Intialization for User BSP */
    osThreadAttr_t attributes;
    ETH_MACConfigTypeDef MACConf;
    int32_t PHYLinkState;
    uint32_t duplex, speed = 0;
    /* USER CODE END low_level_init Variables Intialization for User BSP */
    /* Start ETH HAL Init */

    uint8_t MACAddr[6];
    heth.Instance            = ETH;
    MACAddr[0]               = 0x00;
    MACAddr[1]               = 0x80;
    MACAddr[2]               = 0xE1;
    MACAddr[3]               = 0x00;
    MACAddr[4]               = 0x00;
    MACAddr[5]               = 0x00;
    heth.Init.MACAddr        = &MACAddr[0];
    heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
    heth.Init.TxDesc         = DMATxDscrTab;
    heth.Init.RxDesc         = DMARxDscrTab;
    heth.Init.RxBuffLen      = ETH_RX_BUFFER_SIZE;
    heth.Init.RxBuffAlloc    = RxBuffAlloc;
    heth.Init.RxBuffFree     = RxBuffFree;
    heth.Init.RxPktAssemble  = RxPktAssemble;
    heth.Init.RxPktDiscard   = RxPktDiscard;

    /* USER CODE BEGIN MACADDRESS */

    /* USER CODE END MACADDRESS */

    /* Initialize the Rx Buffer Pool.
     * This must occur prior to HAL_ETH_Init because HAL_ETH_Init uses the
     * buffers to build the Rx DMA Descriptors. */
    RxPoolInit();

    hal_eth_init_status = HAL_ETH_Init(&heth);

    /* Configuration for HAL_ETH_Transmit(_IT). */
    TxConfig.Attributes   = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
    TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
    TxConfig.CRCPadCtrl   = ETH_CRC_PAD_INSERT;
    TxConfig.TxBuffer     = Txbuffer;

#if ETH_TX_QUEUE_ENABLE
    /* Initialize the Tx Packet Queue. */
    for (int i = 0; i < ETH_TX_QUEUE_SIZE; i++)
    {
        TxQueue[i].next = TxQueueFree;
        TxQueueFree     = &TxQueue[i];
    }

    /* Create a counting semaphore for signalling transmit packet complete
     * Use raw FreeRTOS because CMSIS-RTOS API can't create a semaphore with zero count */
    TxPktSemaphore = xSemaphoreCreateCounting(ETH_TX_QUEUE_SIZE, 0);
#endif /* ETH_TX_QUEUE_ENABLE */

    /* End ETH HAL Init */

#if LWIP_ARP || LWIP_ETHERNET

    /* set MAC hardware address length */
    netif->hwaddr_len = ETH_HWADDR_LEN;

    /* set MAC hardware address */
    netif->hwaddr[0] = heth.Init.MACAddr[0];
    netif->hwaddr[1] = heth.Init.MACAddr[1];
    netif->hwaddr[2] = heth.Init.MACAddr[2];
    netif->hwaddr[3] = heth.Init.MACAddr[3];
    netif->hwaddr[4] = heth.Init.MACAddr[4];
    netif->hwaddr[5] = heth.Init.MACAddr[5];

    /* maximum transfer unit */
    netif->mtu = ETH_MAX_PAYLOAD;

/* Accept broadcast address and ARP traffic */
/* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
#if LWIP_ARP
    netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
    netif->flags |= NETIF_FLAG_IGMP;
#else
    netif->flags |= NETIF_FLAG_BROADCAST;
#endif /* LWIP_ARP */

    /* Create the task that handles receiving packets from the ETH_MAC */
    memset(&attributes, 0x0, sizeof(osThreadAttr_t));
    attributes.name       = "EthIf";
    attributes.stack_size = INTERFACE_THREAD_STACK_SIZE;
    attributes.priority   = osPriorityRealtime;
    EthIfThread           = osThreadNew(ethernetif_input, netif, &attributes);
    /* USER CODE BEGIN low_level_init Code 1 for User BSP */
    /* Set PHY IO functions */
    LAN8742_RegisterBusIO(&LAN8742, &LAN8742_IOCtx);

    /* Initialize the LAN8742 ETH PHY */
    LAN8742_Init(&LAN8742);

    /* USER CODE END low_level_init Code 1 for User BSP */

    if (hal_eth_init_status == HAL_OK)
    {
        /* USER CODE BEGIN low_level_init Code 2 for User BSP */
        PHYLinkState = LAN8742_GetLinkState(&LAN8742);

        /* Get link state */
        if (PHYLinkState <= LAN8742_STATUS_LINK_DOWN)
        {
            netif_set_link_down(netif);
            netif_set_down(netif);
        }
        else
        {
            switch (PHYLinkState)
            {
                case LAN8742_STATUS_100MBITS_FULLDUPLEX:
                    duplex = ETH_FULLDUPLEX_MODE;
                    speed  = ETH_SPEED_100M;
                    break;
                case LAN8742_STATUS_100MBITS_HALFDUPLEX:
                    duplex = ETH_HALFDUPLEX_MODE;
                    speed  = ETH_SPEED_100M;
                    break;
                case LAN8742_STATUS_10MBITS_FULLDUPLEX:
                    duplex = ETH_FULLDUPLEX_MODE;
                    speed  = ETH_SPEED_10M;
                    break;
                case LAN8742_STATUS_10MBITS_HALFDUPLEX:
                    duplex = ETH_HALFDUPLEX_MODE;
                    speed  = ETH_SPEED_10M;
                    break;
                default:
                    duplex = ETH_FULLDUPLEX_MODE;
                    speed  = ETH_SPEED_100M;
                    break;
            }

            /* Get MAC Config MAC */
            HAL_ETH_GetMACConfig(&heth, &MACConf);
            MACConf.DuplexMode = duplex;
            MACConf.Speed      = speed;
            HAL_ETH_SetMACConfig(&heth, &MACConf);

            HAL_ETH_Start_IT(&heth);
            netif_set_up(netif);
            netif_set_link_up(netif);

            /* USER CODE BEGIN PHY_POST_CONFIG */

            /* USER CODE END PHY_POST_CONFIG */
        }
        /* USER CODE END low_level_init Code 2 for User BSP */
    }
    else
    {
        Error_Handler();
    }
#endif /* LWIP_ARP || LWIP_ETHERNET */

    /* USER CODE BEGIN LOW_LEVEL_INIT */

    /* USER CODE END LOW_LEVEL_INIT */
}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet was sent, or ERR_IF if the packet was unable to be sent
 *
 * @note ERR_OK means the packet was sent (but not necessarily transmit complete),
 * and ERR_IF means the packet has more chained buffers than the interface supports.
 */
static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
    struct pbuf *q;
    int i;

    /* Prepare TxConfig for HAL_ETH_Transmit_IT. */
    q = p;
    for (i = 0;; i++)
    {
        if (i >= ETH_TX_BUFFER_MAX)
            return ERR_IF;

        Txbuffer[i].buffer = q->payload;
        Txbuffer[i].len    = q->len;

        /* Cleaning cache isn't necessary if the tx buffers are in a not-cacheable MPU
         * region. */
#if ETH_TX_BUFFERS_ARE_CACHED
        uint8_t *dataStart = q->payload;
        uint8_t *lineStart = (uint8_t *)((uint32_t)dataStart & ~31);
        SCB_CleanDCache_by_Addr((uint32_t *)lineStart, q->len + (dataStart - lineStart));
#endif

        q = q->next;
        if (!q)
        {
            Txbuffer[i].next = NULL;
            break;
        }

        Txbuffer[i].next = &Txbuffer[i + 1];
    }
    TxConfig.Length = p->tot_len;

#if ETH_TX_QUEUE_ENABLE
    TxQueue_t *txQ;
    TickType_t blockTime;
    HAL_StatusTypeDef halResult;

    /* Loop until the transmit's started. */
    while (p)
    {
        txQ = TxQueueFree;
        if (txQ)
        {
            /* Queue available.
             * Try to transmit.
             * NOTE performance could be improved if HAL_ETH_Transmit_IT did this
             * queuing as then we wouldn't "try" to transmit as we'd know how many
             * descriptors are available and wouldn't start until we know there's
             * enough. */
            halResult = HAL_ETH_Transmit_IT(&heth, &TxConfig);
            if (halResult == HAL_OK)
            {
                /* Transmit's started.
                 * Queue the packet so its buffers may be freed in a later call of
                 * low_level_output, after its transmit's complete. */
                TxQueueFree = txQ->next;
                txQ->next   = 0;
                txQ->p      = p;
                if (!TxQueueHead)
                {
                    TxQueueHead = txQ;
                }
                else
                {
                    TxQueueTail->next = txQ;
                }
                TxQueueTail = txQ;

                /* Prevent lwIP freeing the packet before its transmit's complete. */
                pbuf_ref(p);

                p         = NULL;
                blockTime = 0;
            }
            else
            {
                /* HAL_ETH_Transmit_IT error status means no Tx Descriptors available.
                 * Wait for a previous transmit to complete, and try again. */
                blockTime = portMAX_DELAY;
            }
        }
        else
        {
            /* Queue full.
             * Wait for a previous transmit to complete, and try again. */
            blockTime = portMAX_DELAY;
        }

        /* Wait for previous transmits to complete.
         * Free their packet buffers.
         * Only block if we couldn't start transmit. */
        while (xSemaphoreTake(TxPktSemaphore, blockTime) == pdPASS)
        {
            txQ         = TxQueueHead;
            TxQueueHead = txQ->next;

            pbuf_free(txQ->p);
            txQ->next   = TxQueueFree;
            TxQueueFree = txQ;

            /* If we blocked, most likely we only need to free one queue position to start
             * transmit.
             * For performance, leave freeing any others to after the transmit's started.
             */
            if (blockTime)
                break;
        }
    }

#else
    HAL_ETH_Transmit(&heth, &TxConfig, ETH_DMA_TRANSMIT_TIMEOUT);

#endif
    return ERR_OK;
}

/**
 * This task should be signalled when a receive packet is ready to be read
 * from the interface.
 *
 * @param argument the lwip network interface structure for this ethernetif
 */
void ethernetif_input(void *argument)
{
    struct pbuf *p;
    struct netif *netif = (struct netif *)argument;
    uint32_t errorCode;
    err_t result;

    while (1)
    {
        if (ulTaskNotifyTake(/* clear count */ pdTRUE, TIME_WAITING_FOR_INPUT) > 0)
        {
            /* Loop while any there's any receive packet ready to avoid a receive
             * buffer unavailable error from stalling receive. */
            while (HAL_ETH_GetRxDataBuffer(&heth, (void **)&p) == HAL_OK)
            {
                /* Received a packet. */
                /* Discard if errored. */
                HAL_ETH_GetRxDataErrorCode(&heth, &errorCode);
                if (errorCode)
                {
                    RxPktDiscard(p);
                }
                else
                {
                    result = netif->input(p, netif);
                    if (result != ERR_OK)
                    {
                        RxPktDiscard(p);
                    }
                }
            }
        }
    }
}

#if !LWIP_ARP
/**
 * This function has to be completed by user in case of ARP OFF.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if ...
 */
static err_t low_level_output_arp_off(struct netif *netif, struct pbuf *q,
                                      const ip4_addr_t *ipaddr)
{
    err_t errval;
    errval = ERR_OK;

    /* USER CODE BEGIN 5 */

    /* USER CODE END 5 */

    return errval;
}
#endif /* LWIP_ARP */

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t ethernetif_init(struct netif *netif)
{
    LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
    /* Initialize interface hostname */
    netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;
    /* We directly use etharp_output() here to save a function call.
     * You can instead declare your own function an call etharp_output()
     * from it if you have to do some checks before sending (e.g. if link
     * is available...) */

#if LWIP_IPV4
#if LWIP_ARP || LWIP_ETHERNET
#if LWIP_ARP
    netif->output = etharp_output;
#else
    /* The user should write ist own code in low_level_output_arp_off function */
    netif->output = low_level_output_arp_off;
#endif /* LWIP_ARP */
#endif /* LWIP_ARP || LWIP_ETHERNET */
#endif /* LWIP_IPV4 */

#if LWIP_IPV6
    netif->output_ip6 = ethip6_output;
#endif /* LWIP_IPV6 */

    netif->linkoutput = low_level_output;

    /* initialize the hardware */
    low_level_init(netif);

    return ERR_OK;
}


/* USER CODE BEGIN 6 */

/**
 * @brief  Returns the current time in milliseconds
 *         when LWIP_TIMERS == 1 and NO_SYS == 1
 * @param  None
 * @retval Current Time value
 */
u32_t sys_jiffies(void)
{
    return HAL_GetTick();
}

/**
 * @brief  Returns the current time in milliseconds
 *         when LWIP_TIMERS == 1 and NO_SYS == 1
 * @param  None
 * @retval Current Time value
 */
u32_t sys_now(void)
{
    return HAL_GetTick();
}

/* USER CODE END 6 */

/*******************************************************************************
                       PHI IO Functions
*******************************************************************************/
/**
 * @brief  Initializes the MDIO interface GPIO and clocks.
 * @param  None
 * @retval 0 if OK, -1 if ERROR
 */
int32_t ETH_PHY_IO_Init(void)
{
    /* We assume that MDIO GPIO configuration is already done
       in the ETH_MspInit() else it should be done here
    */

    /* Configure the MDIO Clock */
    HAL_ETH_SetMDIOClockRange(&heth);

    return 0;
}

/**
 * @brief  De-Initializes the MDIO interface .
 * @param  None
 * @retval 0 if OK, -1 if ERROR
 */
int32_t ETH_PHY_IO_DeInit(void)
{
    return 0;
}

/**
 * @brief  Read a PHY register through the MDIO interface.
 * @param  DevAddr: PHY port address
 * @param  RegAddr: PHY register address
 * @param  pRegVal: pointer to hold the register value
 * @retval 0 if OK -1 if Error
 */
int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal)
{
    if (HAL_ETH_ReadPHYRegister(&heth, DevAddr, RegAddr, pRegVal) != HAL_OK)
    {
        return -1;
    }

    return 0;
}

/**
 * @brief  Write a value to a PHY register through the MDIO interface.
 * @param  DevAddr: PHY port address
 * @param  RegAddr: PHY register address
 * @param  RegVal: Value to be written
 * @retval 0 if OK -1 if Error
 */
int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal)
{
    if (HAL_ETH_WritePHYRegister(&heth, DevAddr, RegAddr, RegVal) != HAL_OK)
    {
        return -1;
    }

    return 0;
}

/**
 * @brief  Get the time in millisecons used for internal PHY driver process.
 * @retval Time value
 */
int32_t ETH_PHY_IO_GetTick(void)
{
    return HAL_GetTick();
}

/**
 * @brief  Check the ETH link state then update ETH driver and netif link accordingly.
 * @param  argument: netif
 * @retval None
 */
void ethernet_link_thread(void *argument)
{
    ETH_MACConfigTypeDef MACConf;
    uint32_t PHYLinkState;
    uint32_t linkchanged = 0, speed = 0, duplex = 0;

    struct netif *netif = (struct netif *)argument;
    /* USER CODE BEGIN ETH link init */

    /* USER CODE END ETH link init */

    for (;;)
    {
        PHYLinkState = LAN8742_GetLinkState(&LAN8742);

        if (netif_is_link_up(netif) && (PHYLinkState <= LAN8742_STATUS_LINK_DOWN))
        {
            HAL_ETH_Stop_IT(&heth);
            netif_set_down(netif);
            netif_set_link_down(netif);
        }
        else if (!netif_is_link_up(netif) && (PHYLinkState > LAN8742_STATUS_LINK_DOWN))
        {
            switch (PHYLinkState)
            {
                case LAN8742_STATUS_100MBITS_FULLDUPLEX:
                    duplex      = ETH_FULLDUPLEX_MODE;
                    speed       = ETH_SPEED_100M;
                    linkchanged = 1;
                    break;
                case LAN8742_STATUS_100MBITS_HALFDUPLEX:
                    duplex      = ETH_HALFDUPLEX_MODE;
                    speed       = ETH_SPEED_100M;
                    linkchanged = 1;
                    break;
                case LAN8742_STATUS_10MBITS_FULLDUPLEX:
                    duplex      = ETH_FULLDUPLEX_MODE;
                    speed       = ETH_SPEED_10M;
                    linkchanged = 1;
                    break;
                case LAN8742_STATUS_10MBITS_HALFDUPLEX:
                    duplex      = ETH_HALFDUPLEX_MODE;
                    speed       = ETH_SPEED_10M;
                    linkchanged = 1;
                    break;
                default:
                    break;
            }

            if (linkchanged)
            {
                /* Get MAC Config MAC */
                HAL_ETH_GetMACConfig(&heth, &MACConf);
                MACConf.DuplexMode = duplex;
                MACConf.Speed      = speed;
                HAL_ETH_SetMACConfig(&heth, &MACConf);

                HAL_ETH_Start_IT(&heth);
                netif_set_up(netif);
                netif_set_link_up(netif);
            }
        }

        /* USER CODE BEGIN ETH link Thread core code for User BSP */

        /* USER CODE END ETH link Thread core code for User BSP */

        osDelay(100);
    }
}

/* USER CODE BEGIN 8 */

/* USER CODE END 8 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
