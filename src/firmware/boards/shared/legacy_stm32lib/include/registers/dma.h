/**
 * \ingroup REG
 * \defgroup REGDMA DMA controller
 * @{
 */
#ifndef STM32LIB_REGISTERS_DMA_H
#define STM32LIB_REGISTERS_DMA_H

#include <stdint.h>

#define DMA1_BASE 0x40026000U
#define DMA2_BASE 0x40026400U

typedef struct
{
    unsigned FEIF0 : 1;
    unsigned : 1;
    unsigned DMEIF0 : 1;
    unsigned TEIF0 : 1;
    unsigned HTIF0 : 1;
    unsigned TCIF0 : 1;
    unsigned FEIF1 : 1;
    unsigned : 1;
    unsigned DMEIF1 : 1;
    unsigned TEIF1 : 1;
    unsigned HTIF1 : 1;
    unsigned TCIF1 : 1;
    unsigned : 4;
    unsigned FEIF2 : 1;
    unsigned : 1;
    unsigned DMEIF2 : 1;
    unsigned TEIF2 : 1;
    unsigned HTIF2 : 1;
    unsigned TCIF2 : 1;
    unsigned FEIF3 : 1;
    unsigned : 1;
    unsigned DMEIF3 : 1;
    unsigned TEIF3 : 1;
    unsigned HTIF3 : 1;
    unsigned TCIF3 : 1;
    unsigned : 4;
} DMA_LISR_t;
_Static_assert(sizeof(DMA_LISR_t) == 4U, "DMA_LISR_t is wrong size");

typedef struct
{
    unsigned FEIF4 : 1;
    unsigned : 1;
    unsigned DMEIF4 : 1;
    unsigned TEIF4 : 1;
    unsigned HTIF4 : 1;
    unsigned TCIF4 : 1;
    unsigned FEIF5 : 1;
    unsigned : 1;
    unsigned DMEIF5 : 1;
    unsigned TEIF5 : 1;
    unsigned HTIF5 : 1;
    unsigned TCIF5 : 1;
    unsigned : 4;
    unsigned FEIF6 : 1;
    unsigned : 1;
    unsigned DMEIF6 : 1;
    unsigned TEIF6 : 1;
    unsigned HTIF6 : 1;
    unsigned TCIF6 : 1;
    unsigned FEIF7 : 1;
    unsigned : 1;
    unsigned DMEIF7 : 1;
    unsigned TEIF7 : 1;
    unsigned HTIF7 : 1;
    unsigned TCIF7 : 1;
    unsigned : 4;
} DMA_HISR_t;
_Static_assert(sizeof(DMA_HISR_t) == 4U, "DMA_HISR_t is wrong size");

typedef struct
{
    unsigned CFEIF0 : 1;
    unsigned : 1;
    unsigned CDMEIF0 : 1;
    unsigned CTEIF0 : 1;
    unsigned CHTIF0 : 1;
    unsigned CTCIF0 : 1;
    unsigned CFEIF1 : 1;
    unsigned : 1;
    unsigned CDMEIF1 : 1;
    unsigned CTEIF1 : 1;
    unsigned CHTIF1 : 1;
    unsigned CTCIF1 : 1;
    unsigned : 4;
    unsigned CFEIF2 : 1;
    unsigned : 1;
    unsigned CDMEIF2 : 1;
    unsigned CTEIF2 : 1;
    unsigned CHTIF2 : 1;
    unsigned CTCIF2 : 1;
    unsigned CFEIF3 : 1;
    unsigned : 1;
    unsigned CDMEIF3 : 1;
    unsigned CTEIF3 : 1;
    unsigned CHTIF3 : 1;
    unsigned CTCIF3 : 1;
    unsigned : 4;
} DMA_LIFCR_t;
_Static_assert(sizeof(DMA_LIFCR_t) == 4U, "DMA_LIFCR_t is wrong size");

typedef struct
{
    unsigned CFEIF4 : 1;
    unsigned : 1;
    unsigned CDMEIF4 : 1;
    unsigned CTEIF4 : 1;
    unsigned CHTIF4 : 1;
    unsigned CTCIF4 : 1;
    unsigned CFEIF5 : 1;
    unsigned : 1;
    unsigned CDMEIF5 : 1;
    unsigned CTEIF5 : 1;
    unsigned CHTIF5 : 1;
    unsigned CTCIF5 : 1;
    unsigned : 4;
    unsigned CFEIF6 : 1;
    unsigned : 1;
    unsigned CDMEIF6 : 1;
    unsigned CTEIF6 : 1;
    unsigned CHTIF6 : 1;
    unsigned CTCIF6 : 1;
    unsigned CFEIF7 : 1;
    unsigned : 1;
    unsigned CDMEIF7 : 1;
    unsigned CTEIF7 : 1;
    unsigned CHTIF7 : 1;
    unsigned CTCIF7 : 1;
    unsigned : 4;
} DMA_HIFCR_t;
_Static_assert(sizeof(DMA_HIFCR_t) == 4U, "DMA_HIFCR_t is wrong size");

typedef enum
{
    DMA_DIR_P2M = 0U,
    DMA_DIR_M2P = 1U,
    DMA_DIR_M2M = 2U,
} dma_dir_t;

typedef enum
{
    DMA_DSIZE_BYTE      = 0U,
    DMA_DSIZE_HALF_WORD = 1U,
    DMA_DSIZE_WORD      = 2U,
} dma_dsize_t;

typedef enum
{
    DMA_BURST_SINGLE = 0U,
    DMA_BURST_INCR4  = 1U,
    DMA_BURST_INCR8  = 2U,
    DMA_BURST_INCR16 = 3U,
} dma_burst_t;

typedef struct
{
    unsigned EN : 1;
    unsigned DMEIE : 1;
    unsigned TEIE : 1;
    unsigned HTIE : 1;
    unsigned TCIE : 1;
    unsigned PFCTRL : 1;
    unsigned DIR : 2;
    unsigned CIRC : 1;
    unsigned PINC : 1;
    unsigned MINC : 1;
    unsigned PSIZE : 2;
    unsigned MSIZE : 2;
    unsigned PINCOS : 1;
    unsigned PL : 2;
    unsigned DBM : 1;
    unsigned CT : 1;
    unsigned : 1;
    unsigned PBURST : 2;
    unsigned MBURST : 2;
    unsigned CHSEL : 3;
    unsigned : 4;
} DMA_SxCR_t;
_Static_assert(sizeof(DMA_SxCR_t) == 4U, "DMA_SxCR_t is wrong size");

typedef enum
{
    DMA_FIFO_THRESHOLD_QUARTER  = 0U,
    DMA_FIFO_THRESHOLD_HALF     = 1U,
    DMA_FIFO_THRESHOLD_3QUARTER = 2U,
    DMA_FIFO_THRESHOLD_FULL     = 3U,
} dma_fifo_threshold_t;

typedef enum
{
    DMA_FIFO_STATUS_LT_QUARTER  = 0U,
    DMA_FIFO_STATUS_LT_HALF     = 1U,
    DMA_FIFO_STATUS_LT_3QUARTER = 2U,
    DMA_FIFO_STATUS_LT_FULL     = 3U,
    DMA_FIFO_STATUS_EMPTY       = 4U,
    DMA_FIFO_STATUS_FULL        = 5U,
} dma_fifo_status_t;

typedef struct
{
    unsigned FTH : 2;
    unsigned DMDIS : 1;
    unsigned FS : 3;
    unsigned : 1;
    unsigned FEIE : 1;
    unsigned : 24;
} DMA_SxFCR_t;
_Static_assert(sizeof(DMA_SxFCR_t) == 4U, "DMA_SxFCR_t is wrong size");

typedef struct
{
    DMA_SxCR_t CR;
    uint32_t NDTR;
    volatile void *PAR;
    volatile void *M0AR;
    volatile void *M1AR;
    DMA_SxFCR_t FCR;
} dma_stream_t;
_Static_assert(sizeof(dma_stream_t) == 0x18U, "dma_stream_t is wrong size");
#endif

typedef struct
{
    DMA_LISR_t LISR;
    DMA_HISR_t HISR;
    DMA_LIFCR_t LIFCR;
    DMA_HIFCR_t HIFCR;
    dma_stream_t streams[8U];
} DMA_t;
_Static_assert(sizeof(DMA_t) == 0xD0U, "DMA_t is wrong size");

extern volatile DMA_t DMA1;
extern volatile DMA_t DMA2;


/**
 * @}
 */
