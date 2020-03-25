#ifndef MRF_H
#define MRF_H

/**
 * \file
 *
 * \brief Communicates with an MRF24J40 radio.
 */

#include <stdbool.h>
#include <stdint.h>

#include "pins.h"

/**
 * \brief The possible short register addresses.
 */
typedef enum
{
    MRF_REG_SHORT_RXMCR,
    MRF_REG_SHORT_PANIDL,
    MRF_REG_SHORT_PANIDH,
    MRF_REG_SHORT_SADRL,
    MRF_REG_SHORT_SADRH,
    MRF_REG_SHORT_EADR0,
    MRF_REG_SHORT_EADR1,
    MRF_REG_SHORT_EADR2,
    MRF_REG_SHORT_EADR3,
    MRF_REG_SHORT_EADR4,
    MRF_REG_SHORT_EADR5,
    MRF_REG_SHORT_EADR6,
    MRF_REG_SHORT_EADR7,
    MRF_REG_SHORT_RXFLUSH,
    MRF_REG_SHORT_ORDER = 0x10,
    MRF_REG_SHORT_TXMCR,
    MRF_REG_SHORT_ACKTMOUT,
    MRF_REG_SHORT_ESLOTG1,
    MRF_REG_SHORT_SYMTICKL,
    MRF_REG_SHORT_SYMTICKH,
    MRF_REG_SHORT_PACON0,
    MRF_REG_SHORT_PACON1,
    MRF_REG_SHORT_PACON2,
    MRF_REG_SHORT_TXBCON0 = 0x1A,
    MRF_REG_SHORT_TXNCON,
    MRF_REG_SHORT_TXG1CON,
    MRF_REG_SHORT_TXG2CON,
    MRF_REG_SHORT_ESLOTG23,
    MRF_REG_SHORT_ESLOTG45,
    MRF_REG_SHORT_ESLOTG67,
    MRF_REG_SHORT_TXPEND,
    MRF_REG_SHORT_WAKECON,
    MRF_REG_SHORT_FRMOFFSET,
    MRF_REG_SHORT_TXSTAT,
    MRF_REG_SHORT_TXBCON1,
    MRF_REG_SHORT_GATECLK,
    MRF_REG_SHORT_TXTIME,
    MRF_REG_SHORT_HSYMTMRL,
    MRF_REG_SHORT_HSYMTMRH,
    MRF_REG_SHORT_SOFTRST,
    MRF_REG_SHORT_SECCON0 = 0x2C,
    MRF_REG_SHORT_SECCON1,
    MRF_REG_SHORT_TXSTBL,
    MRF_REG_SHORT_RXSR = 0x30,
    MRF_REG_SHORT_INTSTAT,
    MRF_REG_SHORT_INTCON,
    MRF_REG_SHORT_GPIO,
    MRF_REG_SHORT_TRISGPIO,
    MRF_REG_SHORT_SLPACK,
    MRF_REG_SHORT_RFCTL,
    MRF_REG_SHORT_SECCR2,
    MRF_REG_SHORT_BBREG0,
    MRF_REG_SHORT_BBREG1,
    MRF_REG_SHORT_BBREG2,
    MRF_REG_SHORT_BBREG3,
    MRF_REG_SHORT_BBREG4,
    MRF_REG_SHORT_BBREG6 = 0x3E,
    MRF_REG_SHORT_CCAEDTH,
} mrf_reg_short_t;

/**
 * \brief The possible long register addresses.
 */
typedef enum
{
    MRF_REG_LONG_TXNFIFO    = 0x000,
    MRF_REG_LONG_TXBFIFO    = 0x080,
    MRF_REG_LONG_TXGTS1FIFO = 0x100,
    MRF_REG_LONG_TXGTS2FIFO = 0x180,
    MRF_REG_LONG_RFCON0     = 0x200,
    MRF_REG_LONG_RFCON1,
    MRF_REG_LONG_RFCON2,
    MRF_REG_LONG_RFCON3,
    MRF_REG_LONG_RFCON5 = 0x205,
    MRF_REG_LONG_RFCON6,
    MRF_REG_LONG_RFCON7,
    MRF_REG_LONG_RFCON8,
    MRF_REG_LONG_SLPCAL0,
    MRF_REG_LONG_SLPCAL1,
    MRF_REG_LONG_SLPCAL2,
    MRF_REG_LONG_RFSTATE = 0x20F,
    MRF_REG_LONG_RSSI,
    MRF_REG_LONG_SLPCON0,
    MRF_REG_LONG_SLPCON1   = 0x220,
    MRF_REG_LONG_WAKETIMEL = 0x222,
    MRF_REG_LONG_WAKETIMEH,
    MRF_REG_LONG_REMCNTL,
    MRF_REG_LONG_REMCNTH,
    MRF_REG_LONG_MAINCNT0,
    MRF_REG_LONG_MAINCNT1,
    MRF_REG_LONG_MAINCNT2,
    MRF_REG_LONG_MAINCNT3,
    MRF_REG_LONG_TESTMODE = 0x22F,
    MRF_REG_LONG_ASSOEADR0,
    MRF_REG_LONG_ASSOEADR1,
    MRF_REG_LONG_ASSOEADR2,
    MRF_REG_LONG_ASSOEADR3,
    MRF_REG_LONG_ASSOEADR4,
    MRF_REG_LONG_ASSOEADR5,
    MRF_REG_LONG_ASSOEADR6,
    MRF_REG_LONG_ASSOEADR7,
    MRF_REG_LONG_ASSOSADR0,
    MRF_REG_LONG_ASSOSADR1,
    MRF_REG_LONG_UPNONCE0 = 0x240,
    MRF_REG_LONG_UPNONCE1,
    MRF_REG_LONG_UPNONCE2,
    MRF_REG_LONG_UPNONCE3,
    MRF_REG_LONG_UPNONCE4,
    MRF_REG_LONG_UPNONCE5,
    MRF_REG_LONG_UPNONCE6,
    MRF_REG_LONG_UPNONCE7,
    MRF_REG_LONG_UPNONCE8,
    MRF_REG_LONG_UPNONCE9,
    MRF_REG_LONG_UPNONCE10,
    MRF_REG_LONG_UPNONCE11,
    MRF_REG_LONG_UPNONCE12,
    MRF_REG_LONG_KEYFIFO = 0x280,
    MRF_REG_LONG_RXFIFO  = 0x300,
} mrf_reg_long_t;

/**
 * \brief Early-initializes the IPC objects needed by the MRF subsystem.
 */
void mrf_init_once(void);

/**
 * \brief Initializes the interface to the radio and places the radio in reset.
 */
void mrf_init(void);

/**
 * \brief Resets the radio and deinitializes the radio module.
 */
void mrf_deinit(void);

/**
 * \brief Releases the radio from reset.
 */
#define mrf_release_reset() gpio_set(PIN_MRF_NRESET)

/**
 * \brief Checks the radio’s interrupt line.
 *
 * \return \c true if the interrupt line is high, or \c false if low
 */
#define mrf_get_interrupt() gpio_get_input(PIN_MRF_INT)

/**
 * \brief Enables taking an interrupt on rising edge of the MRF interrupt pin.
 *
 * \param[in] isr the service routine that will handle the interrupt
 */
void mrf_enable_interrupt(void (*isr)(void));

/**
 * \brief Disables taking an interrupt on rising edge of the MRF interrupt pin.
 */
void mrf_disable_interrupt(void);

/**
 * \brief Reads a short-address register.
 *
 * \param reg the register to read
 *
 * \return the register’s value
 */
uint8_t mrf_read_short(mrf_reg_short_t reg);

/**
 * \brief Writes a short-address register.
 *
 * \param reg the register to write
 *
 * \param value the value to write
 */
void mrf_write_short(mrf_reg_short_t reg, uint8_t value);

/**
 * \brief Reads a long-address register.
 *
 * \param reg the register to read
 *
 * \return the register’s value
 */
uint8_t mrf_read_long(mrf_reg_long_t reg);

/**
 * \brief Writes a long-address register.
 *
 * \param reg the register to write
 *
 * \param value the value to write
 */
void mrf_write_long(mrf_reg_long_t reg, uint8_t value);

/**
 * \brief Performs common initialization of the radio based on the configuration
 * parameters.
 */
void mrf_common_init(void);

/**
 * \brief Sets the analogue path on the MRF to consume minimum power and not allow any
 * communication.
 */
void mrf_analogue_off(void);

/**
 * \brief Sets the analogue path on the MRF to allow reception but not transmission.
 */
void mrf_analogue_rx(void);

/**
 * \brief Sets the analogue path on the MRF to allow both transmission and reception.
 */
void mrf_analogue_txrx(void);

/**
 * \brief Handles DMA2 stream 0 interrupts.
 *
 * This function should be registered in the interrupt vector table at position 56.
 */
void dma2_stream0_isr(void);

/**
 * \brief Handles EXTI 10 through 15 interrupts.
 *
 * This function should be registered in the interrupt vector table at position 40.
 */
void exti10_15_isr(void);

#endif
