/**
 * \defgroup UNUSED Unused parameter marking macro
 * @{
 */

#ifndef STM32LIB_UNUSED_H
#define STM32LIB_UNUSED_H

/**
 * \brief Marks a function parameter as unused.
 *
 * This macro should be wrapped around the parameter name in the function definition’s
 * parameter list.
 *
 * \param x the name of the parameter
 *
 * \ingroup UNUSED
 */
#ifdef __GNUC__
#define UNUSED(x) UNUSED_##x __attribute__((unused))
#else
#define UNUSED(x) x
#endif

#endif

/**
 * @}
 */
