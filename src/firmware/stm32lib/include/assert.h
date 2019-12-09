#ifndef STM32LIB_ASSERT_H
#define STM32LIB_ASSERT_H

/**
 * \brief Checks a condition and calls the standard library’s \c abort if false.
 *
 * \param cond the condition to check, which is not evaluated if the NDEBUG macro is
 * defined
 */
#ifdef NDEBUG
#define assert(cond)
#else
#include <stdlib.h>
#define assert(cond)                                                                     \
    do                                                                                   \
    {                                                                                    \
        if (!(cond))                                                                     \
        {                                                                                \
            volatile unsigned int aborted_line_number __attribute__((unused)) =          \
                __LINE__;                                                                \
            abort();                                                                     \
        }                                                                                \
    } while (0)
#endif

#endif
