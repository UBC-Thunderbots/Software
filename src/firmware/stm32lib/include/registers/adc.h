/**
 * \ingroup REG
 * \defgroup REGADC Analogue-to-digital converter
 * @{
 */
#ifndef STM32LIB_REGISTERS_ADC_H
#define STM32LIB_REGISTERS_ADC_H

#include <stdint.h>

typedef struct
{
    unsigned AWD : 1;
    unsigned EOC : 1;
    unsigned JEOC : 1;
    unsigned JSTRT : 1;
    unsigned STRT : 1;
    unsigned OVR : 1;
    unsigned : 26;
} ADC_SR_t;
_Static_assert(sizeof(ADC_SR_t) == 4U, "ADC_SR_t is wrong size");

typedef struct
{
    unsigned AWDCH : 5;
    unsigned EOCIE : 1;
    unsigned AWDIE : 1;
    unsigned JEOCIE : 1;
    unsigned SCAN : 1;
    unsigned AWDSGL : 1;
    unsigned JAUTO : 1;
    unsigned DISCEN : 1;
    unsigned JDISCEN : 1;
    unsigned DISCNUM : 3;
    unsigned : 6;
    unsigned JAWDEN : 1;
    unsigned AWDEN : 1;
    unsigned RES : 2;
    unsigned OVRIE : 1;
    unsigned : 5;
} ADC_CR1_t;
_Static_assert(sizeof(ADC_CR1_t) == 4U, "ADC_CR1_t is wrong size");

typedef struct
{
    unsigned ADON : 1;
    unsigned CONT : 1;
    unsigned : 6;
    unsigned DMA : 1;
    unsigned DDS : 1;
    unsigned EOCS : 1;
    unsigned ALIGN : 1;
    unsigned : 4;
    unsigned JEXTSEL : 4;
    unsigned JEXTEN : 2;
    unsigned JSWSTART : 1;
    unsigned : 1;
    unsigned EXTSEL : 4;
    unsigned EXTEN : 2;
    unsigned SWSTART : 1;
    unsigned : 1;
} ADC_CR2_t;
_Static_assert(sizeof(ADC_CR2_t) == 4U, "ADC_CR2_t is wrong size");

typedef struct
{
    unsigned SMP10 : 3;
    unsigned SMP11 : 3;
    unsigned SMP12 : 3;
    unsigned SMP13 : 3;
    unsigned SMP14 : 3;
    unsigned SMP15 : 3;
    unsigned SMP16 : 3;
    unsigned SMP17 : 3;
    unsigned SMP18 : 3;
    unsigned : 5;
} ADC_SMPR1_t;
_Static_assert(sizeof(ADC_SMPR1_t) == 4U, "ADC_SMPR1_t is wrong size");

typedef struct
{
    unsigned SMP0 : 3;
    unsigned SMP1 : 3;
    unsigned SMP2 : 3;
    unsigned SMP3 : 3;
    unsigned SMP4 : 3;
    unsigned SMP5 : 3;
    unsigned SMP6 : 3;
    unsigned SMP7 : 3;
    unsigned SMP8 : 3;
    unsigned SMP9 : 3;
    unsigned : 2;
} ADC_SMPR2_t;
_Static_assert(sizeof(ADC_SMPR2_t) == 4U, "ADC_SMPR2_t is wrong size");

typedef struct
{
    unsigned JOFFSET : 12;
    unsigned : 20;
} ADC_JOFR_t;
_Static_assert(sizeof(ADC_JOFR_t) == 4U, "ADC_JOFR_t is wrong size");

typedef struct
{
    unsigned SQ13 : 5;
    unsigned SQ14 : 5;
    unsigned SQ15 : 5;
    unsigned SQ16 : 5;
    unsigned L : 4;
    unsigned : 8;
} ADC_SQR1_t;
_Static_assert(sizeof(ADC_SQR1_t) == 4U, "ADC_SQR1_t is wrong size");

typedef struct
{
    unsigned SQ7 : 5;
    unsigned SQ8 : 5;
    unsigned SQ9 : 5;
    unsigned SQ10 : 5;
    unsigned SQ11 : 5;
    unsigned SQ12 : 5;
    unsigned : 2;
} ADC_SQR2_t;
_Static_assert(sizeof(ADC_SQR2_t) == 4U, "ADC_SQR2_t is wrong size");

typedef struct
{
    unsigned SQ1 : 5;
    unsigned SQ2 : 5;
    unsigned SQ3 : 5;
    unsigned SQ4 : 5;
    unsigned SQ5 : 5;
    unsigned SQ6 : 5;
    unsigned : 2;
} ADC_SQR3_t;
_Static_assert(sizeof(ADC_SQR3_t) == 4U, "ADC_SQR3_t is wrong size");

typedef struct
{
    unsigned JSQ1 : 5;
    unsigned JSQ2 : 5;
    unsigned JSQ3 : 5;
    unsigned JSQ4 : 5;
    unsigned JL : 2;
    unsigned : 10;
} ADC_JSQR_t;
_Static_assert(sizeof(ADC_JSQR_t) == 4U, "ADC_JSQR_t is wrong size");

typedef struct
{
    unsigned DATA : 16;
    unsigned : 16;
} ADC_DR_t;
_Static_assert(sizeof(ADC_DR_t) == 4U, "ADC_DR_t is wrong size");

typedef struct
{
    ADC_SR_t SR;
    ADC_CR1_t CR1;
    ADC_CR2_t CR2;
    ADC_SMPR1_t SMPR1;
    ADC_SMPR2_t SMPR2;
    ADC_JOFR_t JOFR[4];
    unsigned int HTR;
    unsigned int LTR;
    ADC_SQR1_t SQR1;
    ADC_SQR2_t SQR2;
    ADC_SQR3_t SQR3;
    ADC_JSQR_t JSQR;
    ADC_DR_t JDR[4];
    ADC_DR_t DR;
    uint32_t pad[0x100 / 4 - 20];
} ADC_t;
_Static_assert(sizeof(ADC_t) == 0x100U, "ADC_t is wrong size");

typedef ADC_t ADCs_t[3];

extern volatile ADCs_t ADC;
#define ADC1 (ADC[0])
#define ADC2 (ADC[1])
#define ADC3 (ADC[2])

typedef struct
{
    unsigned AWD1 : 1;
    unsigned EOC1 : 1;
    unsigned JEOC1 : 1;
    unsigned JSTRT1 : 1;
    unsigned STRT1 : 1;
    unsigned OVR1 : 1;
    unsigned : 2;
    unsigned AWD2 : 1;
    unsigned EOC2 : 1;
    unsigned JEOC2 : 1;
    unsigned JSTRT2 : 1;
    unsigned STRT2 : 1;
    unsigned OVR2 : 1;
    unsigned : 2;
    unsigned AWD3 : 1;
    unsigned EOC3 : 1;
    unsigned JEOC3 : 1;
    unsigned JSTRT3 : 1;
    unsigned STRT3 : 1;
    unsigned OVR3 : 1;
    unsigned : 10;
} ADC_CSR_t;
_Static_assert(sizeof(ADC_CSR_t) == 4U, "ADC_CSR_t is wrong size");

typedef struct
{
    unsigned MULTI : 5;
    unsigned : 3;
    unsigned DELAY : 4;
    unsigned : 1;
    unsigned DDS : 1;
    unsigned DMA : 2;
    unsigned ADCPRE : 2;
    unsigned : 4;
    unsigned VBATE : 1;
    unsigned TSVREFE : 1;
    unsigned : 8;
} ADC_CCR_t;
_Static_assert(sizeof(ADC_CCR_t) == 4U, "ADC_CCR_t is wrong size");

typedef struct
{
    unsigned DATA1 : 16;
    unsigned DATA2 : 16;
} ADC_CDR_t;
_Static_assert(sizeof(ADC_CDR_t) == 4U, "ADC_CDR_t is wrong size");

typedef struct
{
    ADC_CSR_t CSR;
    ADC_CCR_t CCR;
    ADC_CDR_t CDR;
} ADC_Common_t;
_Static_assert(sizeof(ADC_Common_t) == 12U, "ADC_Common_t is wrong size");

extern volatile ADC_Common_t ADC_Common;

#endif

/**
 * @}
 */
