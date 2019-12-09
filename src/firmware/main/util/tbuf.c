/**
 * \defgroup TBUF Triple Buffer Management Functions
 *
 * These functions provide management of triple buffers used in producer-consumer
 * relationships. A triple buffer provides the following semantics: \li Given exactly one
 * producer, the producer can always, without blocking, find a buffer into which to
 * produce a new element. \li Given exactly one consumer, the consumer can always, without
 * blocking, find a buffer containing a completed element. \li The buffer found by the
 * consumer is always the most recent buffer submitted as complete by the producer.
 *
 * This module provides the control-plane functionality for deciding which buffer should
 * be used by which task when. The actual data buffers must be provided by the
 * application—there must be three buffers, indexed from 0 to 2.
 *
 * @{
 */
#include "tbuf.h"

#include <stdbool.h>

// Byte 0 is a spare buffer.
// Byte 1 is another spare buffer.
// Byte 2 is the next buffer to read.
// 0xFF means no buffer in that slot.

/**
 * \brief Locks a buffer for the producer to write into.
 *
 * \param[in] tbuf the control block
 *
 * \return the index of the locked buffer
 */
unsigned int tbuf_write_get(tbuf_t *tbuf)
{
    for (;;)
    {
        unsigned int old = __atomic_load_n(tbuf, __ATOMIC_RELAXED);
        unsigned int new;
        unsigned int ret;
        // Replace whichever spare slot has a buffer with no buffer, and return the buffer
        // from that slot.
        if ((old & 0xFFU) != 0xFFU)
        {
            ret = old & 0xFFU;
            new = old | 0xFFU;
        }
        else
        {
            ret = (old >> 8U) & 0xFFU;
            new = old | 0xFF00U;
        }
        if (__atomic_compare_exchange_n(tbuf, &old, new, false, __ATOMIC_RELAXED,
                                        __ATOMIC_RELAXED))
        {
            __atomic_signal_fence(__ATOMIC_ACQUIRE);
            return ret;
        }
    }
}

/**
 * \brief Releases a buffer that the producer has finished writing into.
 *
 * \param[in] tbuf the control block
 * \param[in] written the index of the written buffer
 */
void tbuf_write_put(tbuf_t *tbuf, unsigned int written)
{
    __atomic_signal_fence(__ATOMIC_RELEASE);
    for (;;)
    {
        unsigned int old = __atomic_load_n(tbuf, __ATOMIC_RELAXED);
        unsigned int new = old;
        if ((old >> 16U) != 0xFFU)
        {
            // There is a next buffer to read, so move that buffer into a spare slot
            // first.
            unsigned int demoted = old >> 16U;
            if ((old & 0xFFU) == 0xFFU)
            {
                new = (old & ~0xFFU) | demoted;
            }
            else
            {
                new = (old & ~0xFF00U) | (demoted << 8U);
            }
        }
        // Write the new buffer into the next to read slot.
        new = (new & ~0xFF0000U) | (written << 16U);
        if (__atomic_compare_exchange_n(tbuf, &old, new, false, __ATOMIC_RELAXED,
                                        __ATOMIC_RELAXED))
        {
            return;
        }
    }
}

/**
 * \brief Locks a buffer for the consumer to read from.
 *
 * \param[in] tbuf the control block
 *
 * \return the index of the buffer to read from, which is the buffer most recently passed
 * to \ref tbuf_write_put
 */
unsigned int tbuf_read_get(tbuf_t *tbuf)
{
    unsigned int ret = __atomic_fetch_or(tbuf, 0xFF0000U, __ATOMIC_RELAXED) >> 16U;
    __atomic_signal_fence(__ATOMIC_ACQUIRE);
    return ret;
}

/**
 * \brief Releases a buffer that the consumer has finished reading from.
 *
 * \param[in] tbuf the control block
 * \param[in] released the index of the buffer to release
 */
void tbuf_read_put(tbuf_t *tbuf, unsigned int released)
{
    __atomic_signal_fence(__ATOMIC_RELEASE);
    for (;;)
    {
        unsigned int old = __atomic_load_n(tbuf, __ATOMIC_RELAXED);
        unsigned int new;
        if ((old >> 16U) == 0xFFU)
        {
            // No new buffer has been provided as next to read, so the buffer being
            // released still has the newest completed data.
            new = (old & ~0xFF0000U) | (released << 16U);
        }
        else
        {
            // Some other buffer now contains newer data than what we are releasing, and
            // has arrived in the next to read slot. Put the released buffer into a spare
            // slot.
            if ((old & 0xFFU) == 0xFFU)
            {
                new = (old & ~0xFFU) | released;
            }
            else
            {
                new = (old & ~0xFF00U) | (released << 8U);
            }
        }
        if (__atomic_compare_exchange_n(tbuf, &old, new, false, __ATOMIC_RELAXED,
                                        __ATOMIC_RELAXED))
        {
            return;
        }
    }
}

/**
 * @}
 */
