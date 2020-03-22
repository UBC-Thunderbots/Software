/**
 * \defgroup DMA Direct Memory Access Functions
 *
 * \brief These functions handle some tasks related to the DMA controllers.
 *
 * DMA streams are allocated as follows:
 *
 * <table>
 * <tr><th>Controller</th><th>Stream</th><th>Channel</th><th>Function</th></tr>
 * <tr><td>1</td><td>0</td><td>&nbsp;</td><td>&nbsp;</td></tr>
 * <tr><td>1</td><td>1</td><td>&nbsp;</td><td>&nbsp;</td></tr>
 * <tr><td>1</td><td>2</td><td>&nbsp;</td><td>&nbsp;</td></tr>
 * <tr><td>1</td><td>3</td><td>&nbsp;</td><td>&nbsp;</td></tr>
 * <tr><td>1</td><td>4</td><td>&nbsp;</td><td>&nbsp;</td></tr>
 * <tr><td>1</td><td>5</td><td>&nbsp;</td><td>&nbsp;</td></tr>
 * <tr><td>1</td><td>6</td><td>&nbsp;</td><td>&nbsp;</td></tr>
 * <tr><td>1</td><td>7</td><td>&nbsp;</td><td>&nbsp;</td></tr>
 * <tr><td>2</td><td>0</td><td>3</td><td>SPI 1 receive</td></tr>
 * <tr><td>2</td><td>1</td><td>&nbsp;</td><td>&nbsp;</td></tr>
 * <tr><td>2</td><td>2</td><td>1</td><td>ADC 2</td></tr>
 * <tr><td>2</td><td>3</td><td>3</td><td>SPI 1 transmit</td></tr>
 * <tr><td>2</td><td>4</td><td>0</td><td>ADC 1</td></tr>
 * <tr><td>2</td><td>5</td><td>&nbsp;</td><td>&nbsp;</td></tr>
 * <tr><td>2</td><td>6</td><td>&nbsp;</td><td>&nbsp;</td></tr>
 * <tr><td>2</td><td>7</td><td>&nbsp;</td><td>&nbsp;</td></tr>
 * </table>
 *
 * @{
 */

#include "dma.h"

#include <rcc.h>
#include <stdint.h>
#include <stdlib.h>

#define SRAM_BASE 0x20000000U
#define SRAM_SIZE (128U * 1024U)
#define FLASH_BASE 0x08000000U
#define FLASH_SIZE (1024U * 1024U)
#define DMA_ALIGN_BITS 15U

/**
 * \brief Initializes the DMA engine(s).
 */
void dma_init(void)
{
    rcc_enable_reset(AHB1, DMA2);
}

/**
 * \brief Checks whether a block of memory can be used with a DMA engine.
 *
 * \param[in] pointer the address of the start of the block
 * \param[in] length the length of the block, in bytes
 * \retval true if the entire memory block is DMA-capable
 * \retval false if any byte of the block is not DMA-capable
 */
bool dma_check(const void *pointer, size_t length)
{
    uintptr_t pointeri = (uintptr_t)pointer;
    uintptr_t lasti    = pointeri + length - 1U;

    if (SRAM_BASE <= pointeri && pointeri < (SRAM_BASE + SRAM_SIZE) &&
        SRAM_BASE <= lasti && lasti < (SRAM_BASE + SRAM_SIZE))
    {
        return true;
    }
    else if (FLASH_BASE <= pointeri && pointeri < (FLASH_BASE + FLASH_SIZE) &&
             FLASH_BASE <= lasti && lasti < (FLASH_BASE + FLASH_SIZE))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * \brief Allocates a new DMA-capable memory block.
 *
 * \param[in] length the number of bytes of memory needed
 * \return a handle to the new block, or null on failure
 */
dma_memory_handle_t dma_alloc(size_t length)
{
    return malloc(length + DMA_ALIGN_BITS);
}

/**
 * \brief Frees a DMA-capable memory block.
 *
 * \param[in] handle the handle of the block to free
 */
void dma_free(dma_memory_handle_t handle)
{
    free(handle);
}

/**
 * \brief Retrieves the actual buffer address from a DMA-capable memory block.
 *
 * \param[in] handle the handle of the block
 *
 * \return the data area of the block
 */
void *dma_get_buffer(dma_memory_handle_t handle)
{
    return (void *)(((uintptr_t)handle + DMA_ALIGN_BITS) & ~DMA_ALIGN_BITS);
}

/**
 * @}
 */
