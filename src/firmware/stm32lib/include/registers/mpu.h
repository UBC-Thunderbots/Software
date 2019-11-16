/**
 * \ingroup REG
 * \defgroup REGMPU Memory protection unit
 * @{
 */
#ifndef STM32LIB_REGISTERS_MPU_H
#define STM32LIB_REGISTERS_MPU_H

typedef struct
{
    unsigned SEPARATE : 1;
    unsigned : 7;
    unsigned DREGION : 8;
    unsigned IREGION : 8;
    unsigned : 8;
} MPU_TYPE_t;
_Static_assert(sizeof(MPU_TYPE_t) == 4U, "MPU_TYPE_t is wrong size");

typedef struct
{
    unsigned ENABLE : 1;
    unsigned HFNMIENA : 1;
    unsigned PRIVDEFENA : 1;
    unsigned : 29;
} MPU_CTRL_t;
_Static_assert(sizeof(MPU_CTRL_t) == 4U, "MPU_CTRL_t is wrong size");

typedef struct
{
    unsigned REGION : 8;
    unsigned : 24;
} MPU_RNR_t;
_Static_assert(sizeof(MPU_RNR_t) == 4U, "MPU_RNR_t is wrong size");

typedef struct
{
    unsigned REGION : 4;
    unsigned VALID : 1;
    unsigned ADDR : 27;
} MPU_RBAR_t;
_Static_assert(sizeof(MPU_RBAR_t) == 4U, "MPU_RBAR_t is wrong size");

typedef struct
{
    unsigned ENABLE : 1;
    unsigned SIZE : 5;
    unsigned : 2;
    unsigned SRD : 8;
    unsigned B : 1;
    unsigned C : 1;
    unsigned S : 1;
    unsigned TEX : 3;
    unsigned : 2;
    unsigned AP : 3;
    unsigned : 1;
    unsigned XN : 1;
    unsigned : 3;
} MPU_RASR_t;
_Static_assert(sizeof(MPU_RASR_t) == 4U, "MPU_RASR_t is wrong size");

typedef struct
{
    MPU_TYPE_t TYPE;
    MPU_CTRL_t CTRL;
    MPU_RNR_t RNR;
    MPU_RBAR_t RBAR;
    MPU_RASR_t RASR;
} MPU_t;
_Static_assert(sizeof(MPU_t) == 20U, "MPU_t is wrong size");

extern volatile MPU_t MPU;

#endif

/**
 * @}
 */
