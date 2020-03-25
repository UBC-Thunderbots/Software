/**
 * \ingroup REG
 * \defgroup REGDCMI Digital camera interface
 * @{
 */

#ifndef STM32LIB_REGISTERS_DCMI_H
#define STM32LIB_REGISTERS_DCMI_H

#include <stdint.h>

typedef struct
{
    unsigned CAPTURE : 1;
    unsigned CM : 1;
    unsigned CROP : 1;
    unsigned JPEG : 1;
    unsigned ESS : 1;
    unsigned PCKPOL : 1;
    unsigned HSPOL : 1;
    unsigned VSPOL : 1;
    unsigned FCRC : 2;
    unsigned EDM : 2;
    unsigned : 2;
    unsigned ENABLE : 1;
    unsigned : 17;
} DCMI_CR_t;
_Static_assert(sizeof(DCMI_CR_t) == 4U, "DCMI_CR_t is wrong size");

typedef struct
{
    unsigned HSYNC : 1;
    unsigned VSYNC : 1;
    unsigned FNE : 1;
    unsigned : 29;
} DCMI_SR_t;
_Static_assert(sizeof(DCMI_SR_t) == 4U, "DCMI_SR_t is wrong size");

typedef struct
{
    unsigned FRAME_RIS : 1;
    unsigned OVR_RIS : 1;
    unsigned ERR_RIS : 1;
    unsigned VSYNC_RIS : 1;
    unsigned LINE_RIS : 1;
    unsigned : 27;
} DCMI_RIS_t;
_Static_assert(sizeof(DCMI_RIS_t) == 4U, "DCMI_RIS_t is wrong size");

typedef struct
{
    unsigned FRAME_IE : 1;
    unsigned OVR_IE : 1;
    unsigned ERR_IE : 1;
    unsigned VSYNC_IE : 1;
    unsigned LINE_IE : 1;
    unsigned : 27;
} DCMI_IER_t;
_Static_assert(sizeof(DCMI_IER_t) == 4U, "DCMI_IER_t is wrong size");

typedef struct
{
    unsigned FRAME_MIS : 1;
    unsigned OVR_MIS : 1;
    unsigned ERR_MIS : 1;
    unsigned VSYNC_MIS : 1;
    unsigned LINE_MIS : 1;
    unsigned : 27;
} DCMI_MIS_t;
_Static_assert(sizeof(DCMI_MIS_t) == 4U, "DCMI_MIS_t is wrong size");

typedef struct
{
    unsigned FRAME_ISC : 1;
    unsigned OVR_ISC : 1;
    unsigned ERR_ISC : 1;
    unsigned VSYNC_ISC : 1;
    unsigned LINE_ISC : 1;
    unsigned : 27;
} DCMI_ICR_t;
_Static_assert(sizeof(DCMI_ICR_t) == 4U, "DCMI_ICR_t is wrong size");

typedef struct
{
    unsigned FSC : 8;
    unsigned LSC : 8;
    unsigned LEC : 8;
    unsigned FEC : 8;
} DCMI_ESCR_t;
_Static_assert(sizeof(DCMI_ESCR_t) == 4U, "DCMI_ESCR_t is wrong size");

typedef struct
{
    unsigned FSU : 8;
    unsigned LSU : 8;
    unsigned LEU : 8;
    unsigned FEU : 8;
} DCMI_ESUR_t;
_Static_assert(sizeof(DCMI_ESUR_t) == 4U, "DCMI_ESUR_t is wrong size");

typedef struct
{
    unsigned HOFFCNT : 14;
    unsigned : 2;
    unsigned VST : 13;
    unsigned : 3;
} DCMI_CWSTRT_t;
_Static_assert(sizeof(DCMI_CWSTRT_t) == 4U, "DCMI_CWSTRT_t is wrong size");

typedef struct
{
    unsigned CAPCNT : 14;
    unsigned : 2;
    unsigned VLINE : 14;
    unsigned : 2;
} DCMI_CWSIZE_t;
_Static_assert(sizeof(DCMI_CWSIZE_t) == 4U, "DCMI_CWSIZE_t is wrong size");

typedef struct
{
    DCMI_CR_t CR;
    DCMI_SR_t SR;
    DCMI_RIS_t RIS;
    DCMI_IER_t IER;
    DCMI_MIS_t MIS;
    DCMI_ICR_t ICR;
    DCMI_ESCR_t ESCR;
    DCMI_ESUR_t ESUR;
    DCMI_CWSTRT_t CWSTRT;
    DCMI_CWSIZE_t CWSIZE;
    uint32_t DR;
} DCMI_t;
_Static_assert(sizeof(DCMI_t) == 44U, "DCMI_t is wrong size");

extern volatile DCMI_t DCMI;


#endif
