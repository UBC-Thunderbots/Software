/**
 * \defgroup GPIO General-purpose I/O port utility functions and macros
 *
 * An I/O pin is represented by two values, a port and a bit number.
 * For example, pin PB12 is bit 12 of port B.
 * The macros in this function represent a port as one of the memory mapped structure
 * instances given names in the registers header file. For example, PB12 would be
 * represented as <code>GPIOB, 12</code>.
 *
 * @{
 */

#include <assert.h>
#include <gpio.h>
#include <rcc.h>

/**
 * \brief Initializes all I/O ports.
 *
 * Once this function returns, all I/O pins are in their specified states and all ports
 * are enabled in the RCC.
 *
 * \param[in] specs the specifications of the pins, indexed first by port and then by pin
 * \param[in] length the number of ports to initialize
 */
void gpio_init(const gpio_init_pin_t (*specs)[16U], size_t length)
{
    rcc_enable_reset(AHB1, GPIOA);
    rcc_enable_reset(AHB1, GPIOB);
    rcc_enable_reset(AHB1, GPIOC);
    rcc_enable_reset(AHB1, GPIOD);
    rcc_enable_reset(AHB1, GPIOE);
    rcc_enable_reset(AHB1, GPIOF);
    rcc_enable_reset(AHB1, GPIOG);
    rcc_enable_reset(AHB1, GPIOH);
    rcc_enable_reset(AHB1, GPIOI);
    for (unsigned int port = 0U; port < length; ++port)
    {
        uint32_t moder = 0U, otyper = 0U, ospeedr = 0U, pupdr = 0U, odr = 0U,
                 afr[2U] = {0U, 0U}, lock = 0U;
        for (unsigned int pin = 0U; pin < 16U; ++pin)
        {
            moder |= specs[port][pin].mode << (pin * 2U);
            otyper |= specs[port][pin].otype << pin;
            ospeedr |= specs[port][pin].ospeed << (pin * 2U);
            pupdr |= specs[port][pin].pupd << (pin * 2U);
            odr |= specs[port][pin].od << pin;
            afr[pin / 8U] |= specs[port][pin].af << ((pin % 8U) * 4U);
            if (!specs[port][pin].unlock)
            {
                lock |= 1U << pin;
            }
        }

        GPIO[port].ODR     = odr;
        GPIO[port].OSPEEDR = ospeedr;
        GPIO[port].PUPDR   = pupdr;
        GPIO[port].AFRL    = afr[0U];
        GPIO[port].AFRH    = afr[1U];
        GPIO[port].OTYPER  = otyper;
        GPIO[port].MODER   = moder;
        if (lock)
        {
            GPIO[port].LCKR = lock;
            GPIO[port].LCKR = lock | 0x10000U;
            GPIO[port].LCKR = lock;
            GPIO[port].LCKR = lock | 0x10000U;
            (void)GPIO[port].LCKR;
            assert(GPIO[port].LCKR & 0x10000U);
        }
    }
}

/**
 * @}
 */
