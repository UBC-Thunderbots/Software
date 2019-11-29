/**
 * \ingroup REG
 * \defgroup REGI2C Inter-integrated circuit interface
 * @{
 */

#ifndef STM32LIB_REGISTERS_I2C_H
#define STM32LIB_REGISTERS_I2C_H

#include <stdint.h>

typedef struct
{
    unsigned PE : 1;
    unsigned SMBUS : 1;
    unsigned : 1;
    unsigned SMBTYPE : 1;
    unsigned ENARP : 1;
    unsigned ENPEC : 1;
    unsigned ENGC : 1;
    unsigned NOSTRETCH : 1;
    unsigned START : 1;
    unsigned STOP : 1;
    unsigned ACK : 1;
    unsigned POS : 1;
    unsigned PEC : 1;
    unsigned ALERT : 1;
    unsigned : 1;
    unsigned SWRST : 1;
    unsigned : 16;
} I2C_CR1_t;
_Static_assert(sizeof(I2C_CR1_t) == 4U, "I2C_CR1_t is wrong size");

typedef struct
{
    unsigned FREQ : 6;
    unsigned : 2;
    unsigned ITERREN : 1;
    unsigned ITEVTEN : 1;
    unsigned ITBUFEN : 1;
    unsigned DMAEN : 1;
    unsigned LAST : 1;
    unsigned : 19;
} I2C_CR2_t;
_Static_assert(sizeof(I2C_CR2_t) == 4U, "I2C_CR2_t is wrong size");

typedef struct
{
    unsigned ADD0 : 1;
    unsigned ADD : 9;
    unsigned : 4;
    unsigned defaultval : 1;
    unsigned ADDMODE : 1;
    unsigned : 16;
} I2C_OAR1_t;
_Static_assert(sizeof(I2C_OAR1_t) == 4U, "I2C_OAR1_t is wrong size");

typedef struct
{
    unsigned ENDUAL : 1;
    unsigned ADD2 : 7;
    unsigned : 8;
    unsigned : 16;
} I2C_OAR2_t;
_Static_assert(sizeof(I2C_OAR2_t) == 4U, "I2C_OAR2_t is wrong size");

typedef struct
{
    unsigned DR : 8;
    unsigned : 24;
} I2C_DR_t;
_Static_assert(sizeof(I2C_DR_t) == 4U, "I2C_DR_t is wrong size");

typedef struct
{
    unsigned SB : 1;
    unsigned ADDR : 1;
    unsigned BTF : 1;
    unsigned ADD10 : 1;
    unsigned STOPF : 1;
    unsigned : 1;
    unsigned RxNE : 1;
    unsigned TxE : 1;
    unsigned BERR : 1;
    unsigned ARLO : 1;
    unsigned AF : 1;
    unsigned OVR : 1;
    unsigned PECERR : 1;
    unsigned : 1;
    unsigned TIMEOUT : 1;
    unsigned SMBALERT : 1;
    unsigned : 16;
} I2C_SR1_t;
_Static_assert(sizeof(I2C_SR1_t) == 4U, "I2C_SR1_t is wrong size");

typedef struct
{
    unsigned MSL : 1;
    unsigned BUSY : 1;
    unsigned TRA : 1;
    unsigned : 1;
    unsigned GENCALL : 1;
    unsigned SMBDEFAULT : 1;
    unsigned SMBHOST : 1;
    unsigned DUALF : 1;
    unsigned PEC : 8;
    unsigned : 16;
} I2C_SR2_t;
_Static_assert(sizeof(I2C_SR2_t) == 4U, "I2C_SR2_t is wrong size");

typedef struct
{
    unsigned CCR : 12;
    unsigned : 2;
    unsigned DUTY : 1;
    unsigned FS : 1;
    unsigned : 16;
} I2C_CCR_t;
_Static_assert(sizeof(I2C_CCR_t) == 4U, "I2C_CCR_t is wrong size");

typedef struct
{
    unsigned TRISE : 6;
    unsigned : 26;
} I2C_TRISE_t;
_Static_assert(sizeof(I2C_TRISE_t) == 4U, "I2C_TRISE_t is wrong size");

// Filter register is only available on STM32F42 and F43.
/*
typedef struct {
    unsigned DNF : 4;
    unsigned ANOFF : 1;
    unsigned : 27;
} I2C_FLTR_t;
_Static_assert(sizeof(I2C_FLTR_t) == 4U, "I2C_FLTR_t is wrong size");
*/

typedef struct
{
    I2C_CR1_t CR1;
    I2C_CR2_t CR2;
    I2C_OAR1_t OAR1;
    I2C_OAR2_t OAR2;
    I2C_DR_t DR;
    I2C_SR1_t SR1;
    I2C_SR2_t SR2;
    I2C_CCR_t CCR;
    I2C_TRISE_t TRISE;
    // I2C_FLTR_t FLTR;
} I2C_t;
_Static_assert(sizeof(I2C_t) == 36U, "I2C_t is wrong size");

extern volatile I2C_t I2C1;
extern volatile I2C_t I2C2;

#endif;
