#include <crc32.h>
#include <rcc.h>
#include <registers/crc.h>

/**
 * \brief Initializes the CRC32 module.
 */
void crc32_init(void)
{
    rcc_enable_reset(AHB1, CRC);
}

/**
 * \brief Computes the CRC32 of a block of data.
 *
 * The CRC is computed as though the data will be transmitted MSb-first.
 *
 * \warning This function is not reentrant.
 *
 * \param[in] data the data block to process
 * \param[in] length the number of bytes
 * \param[in] initial the initial CRC to accumulate onto
 * \return the CRC32 value
 */
uint32_t crc32_be(const void *data, size_t length, uint32_t initial)
{
    // The algorithm in this function was inspired by code posted here:
    //
    // https://my.st.com/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/Flat.aspx?RootFolder=https%3a%2f%2fmy%2est%2ecom%2fpublic%2fSTe2ecommunities%2fmcu%2fLists%2fcortex%5fmx%5fstm32%2fSTM32F4%20CRC%20%2d%20Preload%20CRC%2dDR&FolderCTID=0x01200200770978C69A1141439FE559EB459D7580009C4E14902C3CDE46A77F0FFD06506F5B&currentviews=1185
    //
    // However, that code was designed for LSb-first operation.
    // This code implements MSb-first operation.
    const CRC_CR_t reset_cr = {.RESET = 1};
    uint32_t acc            = ~initial;
    const uint8_t *bptr     = data;

    // Consume single bytes until the data pointer is word-aligned.
    while (length && ((uintptr_t)bptr & 3U))
    {
        CRC.CR = reset_cr;
        asm volatile("dmb");
        acc    = acc ^ ((uint32_t)*bptr << 24U);
        CRC.DR = (acc >> 24U) | 0xFFFFFF00U;
        acc    = (acc << 8U) ^ CRC.DR ^ 0xFFU;
        --length;
        ++bptr;
    }

    // Consume as many whole words as we can.
    const uint32_t *wptr = (const uint32_t *)bptr;
    if (length >= 4U)
    {
        CRC.CR = reset_cr;
        asm volatile("dmb");
        CRC.DR = acc ^ __builtin_bswap32(*wptr);
        ++wptr;
        length -= 4U;
        while (length >= 4U)
        {
            CRC.DR = __builtin_bswap32(*wptr);
            ++wptr;
            length -= 4U;
        }
        acc = ~CRC.DR;
    }

    // Consume any remaining single bytes.
    bptr = (const uint8_t *)wptr;
    while (length)
    {
        CRC.CR = reset_cr;
        asm volatile("dmb");
        acc    = acc ^ ((uint32_t)*bptr << 24U);
        CRC.DR = (acc >> 24U) | 0xFFFFFF00U;
        acc    = (acc << 8U) ^ CRC.DR ^ 0xFFU;
        --length;
        ++bptr;
    }

    return ~acc;
}
