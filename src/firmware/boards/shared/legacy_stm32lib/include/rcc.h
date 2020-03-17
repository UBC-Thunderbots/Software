/**
 * \defgroup RCC Reset and clock control utility macros
 * @{
 */

#ifndef STM32LIB_RCC_H
#define STM32LIB_RCC_H

#include <registers/rcc.h>

/**
 * \brief Enables a module.
 *
 * \param bus the bus the module is attached to, one of AHB1, AHB2, AHB3, APB1, or APB2
 * (based on the name of the RCC registers controlling it)
 *
 * \param module the name of the module (the prefix of the control bits in the controlling
 * RCC registers)
 */
#define rcc_enable(bus, module)                                                          \
    do                                                                                   \
    {                                                                                    \
        RCC.bus##ENR.module##EN = 1;                                                     \
        asm volatile("dsb");                                                             \
        asm volatile("nop");                                                             \
    } while (0)

/**
 * \brief Resets a module.
 *
 * The module should already be enabled.
 *
 * \param bus the bus the module is attached to, one of AHB1, AHB2, AHB3, APB1, or APB2
 * (based on the name of the RCC registers controlling it)
 *
 * \param module the name of the module (the prefix of the control bits in the controlling
 * RCC registers)
 */
#define rcc_reset(bus, module)                                                           \
    do                                                                                   \
    {                                                                                    \
        RCC.bus##RSTR.module##RST = 1;                                                   \
        asm volatile("dsb");                                                             \
        asm volatile("nop");                                                             \
        RCC.bus##RSTR.module##RST = 0;                                                   \
        asm volatile("dsb");                                                             \
        asm volatile("nop");                                                             \
    } while (0)

/**
 * \brief Enables and resets a module.
 *
 * \param bus the bus the module is attached to, one of AHB1, AHB2, AHB3, APB1, or APB2
 * (based on the name of the RCC registers controlling it)
 *
 * \param module the name of the module (the prefix of the control bits in the controlling
 * RCC registers)
 */
#define rcc_enable_reset(bus, module)                                                    \
    do                                                                                   \
    {                                                                                    \
        RCC.bus##ENR.module##EN = 0;                                                     \
        asm volatile("dsb");                                                             \
        asm volatile("nop");                                                             \
        RCC.bus##RSTR.module##RST = 1;                                                   \
        asm volatile("dsb");                                                             \
        asm volatile("nop");                                                             \
        asm volatile("nop");                                                             \
        asm volatile("nop");                                                             \
        asm volatile("nop");                                                             \
        RCC.bus##RSTR.module##RST = 0;                                                   \
        asm volatile("dsb");                                                             \
        asm volatile("nop");                                                             \
        RCC.bus##ENR.module##EN = 1;                                                     \
        asm volatile("dsb");                                                             \
        asm volatile("nop");                                                             \
    } while (0)

/**
 * \brief Disables a module.
 *
 * \param bus the bus the module is attached to, one of AHB1, AHB2, AHB3, APB1, or APB2
 * (based on the name of the RCC registers controlling it)
 *
 * \param module the name of the module (the prefix of the control bits in the controlling
 * RCC registers)
 */
#define rcc_disable(bus, module)                                                         \
    do                                                                                   \
    {                                                                                    \
        asm volatile("dsb");                                                             \
        asm volatile("nop");                                                             \
        RCC.bus##ENR.module##EN = 0;                                                     \
    } while (0)

#endif

/**
 * @}
 */
