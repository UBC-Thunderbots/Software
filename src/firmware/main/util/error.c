/**
 * \defgroup ERROR Error Reporting Functions
 *
 * These functions allow reporting system errors to the host.
 *
 * \{
 */
#include "util/error.h"

#include <FreeRTOS.h>
#include <string.h>
#include <task.h>

/**
 * \brief The number of bits in a word.
 */
#define ERROR_WORD_BITS (sizeof(unsigned int) * CHAR_BIT)

/**
 * \brief The number of unsigned integers needed to store the level-triggered
 * errors as a bitmask.
 */
#define ERROR_LT_WORDS ((ERROR_LT_COUNT + ERROR_WORD_BITS - 1) / ERROR_WORD_BITS)

/**
 * \brief The number of unsigned integers needed to store the edge-triggered
 * errors as a bitmask.
 */
#define ERROR_ET_WORDS ((ERROR_ET_COUNT + ERROR_WORD_BITS - 1) / ERROR_WORD_BITS)

/**
 * \brief The current live values of the level-triggered errors.
 */
static unsigned int error_lt_state[ERROR_LT_WORDS];

/**
 * \brief The latched values of all the errors.
 */
static unsigned int error_latched[ERROR_CONSUMER_COUNT][ERROR_LT_WORDS + ERROR_ET_WORDS];

/**
 * \brief The mask of errors most recently packed for reporting.
 */
static unsigned int error_packed[ERROR_CONSUMER_COUNT][ERROR_LT_WORDS + ERROR_ET_WORDS];

/**
 * \brief Checks whether a level-triggered error is currently active.
 *
 * This function may be called from any thread at any time.
 *
 * \param[in] error the error to check
 * \retval true the error is currently active
 * \retval false the error is currently inactive
 */
bool error_lt_get(error_lt_t error)
{
    unsigned int word = error / ERROR_WORD_BITS;
    unsigned int bit  = error % ERROR_WORD_BITS;
    return __atomic_load_n(&error_lt_state[word], __ATOMIC_RELAXED) & (1U << bit);
}

/**
 * \brief Sets the state of a level-triggered error.
 *
 * This function may be called from any thread at any time.
 *
 * \param[in] error the error to set
 * \param[in] active whether the error is currently active
 */
void error_lt_set(error_lt_t error, bool active)
{
    unsigned int word = error / ERROR_WORD_BITS;
    unsigned int bit  = error % ERROR_WORD_BITS;
    if (active)
    {
        // Set the current state and also the latch.
        taskENTER_CRITICAL();
        error_lt_state[word] |= 1U << bit;
        for (unsigned int consumer = 0; consumer != ERROR_CONSUMER_COUNT; ++consumer)
        {
            error_latched[consumer][word] |= 1U << bit;
        }
        taskEXIT_CRITICAL();
    }
    else
    {
        // Clear the current state. Do not clear the latch; it will be cleared
        // after a successful report, thus ensuring that even if the error was
        // only active for a short time, it will be reported.
        __atomic_fetch_and(&error_lt_state[word], ~(1U << bit), __ATOMIC_RELAXED);
    }
}

/**
 * \brief Marks an occurrence of an edge-triggered error.
 *
 * This function may be called from any thread at any time.
 *
 * \param[in] error the error to mark
 */
void error_et_fire(error_et_t error)
{
    // Just set the latch, ensuring the error will be reported at least once.
    unsigned int word = error / ERROR_WORD_BITS;
    unsigned int bit  = error % ERROR_WORD_BITS;
    for (unsigned int consumer = 0; consumer != ERROR_CONSUMER_COUNT; ++consumer)
    {
        __atomic_fetch_or(&error_latched[consumer][ERROR_LT_WORDS + word], 1U << bit,
                          __ATOMIC_RELAXED);
    }
}

/**
 * \brief Checks whether any errors are latched for reporting.
 *
 * \param[in] consumer the log reporting consumer
 * \retval true at least one error would be reported
 * \retval false no errors are ready to report
 */
bool error_any_latched(error_consumer_t consumer)
{
    bool any = false;
    for (size_t i = 0; i != ERROR_LT_WORDS + ERROR_ET_WORDS; ++i)
    {
        any |= __atomic_load_n(&error_latched[consumer][i], __ATOMIC_RELAXED);
    }
    return any;
}

/**
 * \brief Prepares an error report bitmask.
 *
 * Per-consumer, only one thread may call any of @ref error_pre_report and @ref
 * error_post_report at a time, and @ref error_post_report must be called after
 * @ref error_pre_report before a second call to @ref error_pre_report.
 *
 * \param[in] consumer the log reporting consumer
 * \param[out] buffer a block of @ref ERROR_BYTES bytes to be filled with the
 * error bitmask
 */
void error_pre_report(error_consumer_t consumer, void *buffer)
{
    // Copy the latches into the set of errors that will be reported.
    for (size_t i = 0; i != ERROR_LT_WORDS + ERROR_ET_WORDS; ++i)
    {
        error_packed[consumer][i] =
            __atomic_load_n(&error_latched[consumer][i], __ATOMIC_RELAXED);
    }

    // Zero out the buffer.
    memset(buffer, 0, ERROR_BYTES);

    // Pack the errors to report, removing the alignment gap between
    // level-triggered and edge-triggered errors.
    unsigned char *dptr = buffer;
    for (unsigned int i = 0; i != ERROR_LT_COUNT; ++i)
    {
        unsigned int word = i / ERROR_WORD_BITS;
        unsigned int bit  = i % ERROR_WORD_BITS;
        if (error_packed[consumer][word] & (1U << bit))
        {
            dptr[i / CHAR_BIT] |= 1U << (i % CHAR_BIT);
        }
        else
        {
            dptr[i / CHAR_BIT] &= ~(1U << (i % CHAR_BIT));
        }
    }
    for (unsigned int i = 0; i != ERROR_ET_COUNT; ++i)
    {
        unsigned int word = ERROR_LT_WORDS + i / ERROR_WORD_BITS;
        unsigned int bit  = i % ERROR_WORD_BITS;
        if (error_packed[consumer][word] & (1U << bit))
        {
            dptr[(i + ERROR_LT_COUNT) / CHAR_BIT] |= 1U
                                                     << ((i + ERROR_LT_COUNT) % CHAR_BIT);
        }
        else
        {
            dptr[(i + ERROR_LT_COUNT) / CHAR_BIT] &=
                ~(1U << ((i + ERROR_LT_COUNT) % CHAR_BIT));
        }
    }
}

/**
 * \brief Completes an error report.
 *
 * Per-consumer, only one thread may call any of @ref error_pre_report and @ref
 * error_post_report at a time, and @ref error_post_report must be called after
 * @ref error_pre_report before a second call to @ref error_pre_report.
 *
 * \param[in] consumer the log reporting consumer
 * \param[in] ok whether the error report was successfully delivered
 */
void error_post_report(error_consumer_t consumer, bool ok)
{
    // Only clear latches on successful report.
    if (ok)
    {
        // For level-triggered errors, only clear the latch for a reported
        // error if the error is currently inactive. If the error is still
        // active, leave the latch set.
        taskENTER_CRITICAL();
        for (size_t i = 0; i != ERROR_LT_WORDS; ++i)
        {
            error_latched[consumer][i] =
                (error_latched[consumer][i] & ~error_packed[consumer][i]) |
                error_lt_state[i];
        }
        taskEXIT_CRITICAL();

        // For edge-triggered errors, clear the latch for all reported errors.
        for (size_t i = ERROR_LT_WORDS; i != ERROR_LT_WORDS + ERROR_ET_WORDS; ++i)
        {
            __atomic_fetch_and(&error_latched[consumer][i], ~error_packed[consumer][i],
                               __ATOMIC_RELAXED);
        }
    }
}
/**
 * \}
 */
