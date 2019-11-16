/**
 * \ingroup REG
 * \defgroup REGTMR Timer
 * @{
 */
#ifndef STM32LIB_REGISTERS_TIMER_H
#define STM32LIB_REGISTERS_TIMER_H

#include <stdint.h>

typedef struct
{
    unsigned CEN : 1;
    unsigned UDIS : 1;
    unsigned URS : 1;
    unsigned OPM : 1;
    unsigned : 3;
    unsigned ARPE : 1;
    unsigned : 24;
} TIM_basic_CR1_t;
_Static_assert(sizeof(TIM_basic_CR1_t) == 4U, "TIM_basic_CR1_t is wrong size");

typedef struct
{
    unsigned : 4;
    unsigned MMS : 3;
    unsigned : 25;
} TIM_basic_CR2_t;
_Static_assert(sizeof(TIM_basic_CR2_t) == 4U, "TIM_basic_CR2_t is wrong size");

typedef struct
{
    unsigned UIE : 1;
    unsigned : 7;
    unsigned UDE : 1;
    unsigned : 23;
} TIM_basic_DIER_t;
_Static_assert(sizeof(TIM_basic_DIER_t) == 4U, "TIM_basic_DIER_t is wrong size");

typedef struct
{
    unsigned UIF : 1;
    unsigned : 31;
} TIM_basic_SR_t;
_Static_assert(sizeof(TIM_basic_SR_t) == 4U, "TIM_basic_SR_t is wrong size");

typedef struct
{
    unsigned UG : 1;
    unsigned : 31;
} TIM_basic_EGR_t;
_Static_assert(sizeof(TIM_basic_EGR_t) == 4U, "TIM_basic_EGR_t is wrong size");

typedef struct
{
    TIM_basic_CR1_t CR1;
    TIM_basic_CR2_t CR2;
    uint32_t reserved1;
    TIM_basic_DIER_t DIER;
    TIM_basic_SR_t SR;
    TIM_basic_EGR_t EGR;
    uint32_t reserved2[3];
    uint32_t CNT;
    uint32_t PSC;
    uint32_t ARR;
} TIM_basic_t;
_Static_assert(sizeof(TIM_basic_t) == 0x30U, "TIM_basic_t is wrong size");

typedef struct
{
    unsigned CEN : 1;
    unsigned UDIS : 1;
    unsigned URS : 1;
    unsigned OPM : 1;
    unsigned : 3;
    unsigned ARPE : 1;
    unsigned CKD : 2;
    unsigned : 22;
} TIM9_12_CR1_t;
_Static_assert(sizeof(TIM9_12_CR1_t) == 4U, "TIM9_12_CR1_t is wrong size");

typedef struct
{
    unsigned CEN : 1;
    unsigned UDIS : 1;
    unsigned URS : 1;
    unsigned : 4;
    unsigned ARPE : 1;
    unsigned CKD : 2;
    unsigned : 22;
} TIM10_14_CR1_t;
_Static_assert(sizeof(TIM10_14_CR1_t) == 4U, "TIM10_14_CR1_t is wrong size");

typedef struct
{
    unsigned : 4;
    unsigned MMS : 3;
    unsigned : 25;
} TIM9_12_CR2_t;
_Static_assert(sizeof(TIM9_12_CR2_t) == 4U, "TIM9_12_CR2_t is wrong size");

typedef struct
{
    unsigned SMS : 3;
    unsigned : 1;
    unsigned TS : 3;
    unsigned MSM : 1;
    unsigned : 24;
} TIM9_12_SMCR_t;
_Static_assert(sizeof(TIM9_12_SMCR_t) == 4U, "TIM9_12_SMCR_t is wrong size");

typedef struct
{
    unsigned UIE : 1;
    unsigned CC1IE : 1;
    unsigned CC2IE : 1;
    unsigned : 3;
    unsigned TIE : 1;
    unsigned : 25;
} TIM9_12_DIER_t;
_Static_assert(sizeof(TIM9_12_DIER_t) == 4U, "TIM9_12_DIER_t is wrong size");

typedef struct
{
    unsigned UIE : 1;
    unsigned CC1IE : 1;
    unsigned : 30;
} TIM10_14_DIER_t;
_Static_assert(sizeof(TIM10_14_DIER_t) == 4U, "TIM10_14_DIER_t is wrong size");

typedef struct
{
    unsigned UIF : 1;
    unsigned CC1IF : 1;
    unsigned CC2IF : 1;
    unsigned : 3;
    unsigned TIF : 1;
    unsigned : 2;
    unsigned CC1OF : 1;
    unsigned CC2OF : 1;
    unsigned : 21;
} TIM9_12_SR_t;
_Static_assert(sizeof(TIM9_12_SR_t) == 4U, "TIM9_12_SR_t is wrong size");

typedef struct
{
    unsigned UIF : 1;
    unsigned CC1IF : 1;
    unsigned : 7;
    unsigned CC1OF : 1;
    unsigned : 22;
} TIM10_14_SR_t;
_Static_assert(sizeof(TIM10_14_SR_t) == 4U, "TIM10_14_SR_t is wrong size");

typedef struct
{
    unsigned UG : 1;
    unsigned CC1G : 1;
    unsigned CC2G : 1;
    unsigned : 3;
    unsigned TG : 1;
    unsigned : 25;
} TIM9_12_EGR_t;
_Static_assert(sizeof(TIM9_12_EGR_t) == 4U, "TIM9_12_EGR_t is wrong size");

typedef struct
{
    unsigned UG : 1;
    unsigned CC1G : 1;
    unsigned : 30;
} TIM10_14_EGR_t;
_Static_assert(sizeof(TIM10_14_EGR_t) == 4U, "TIM10_14_EGR_t is wrong size");

typedef struct
{
    unsigned CC1S : 2;
    unsigned OC1FE : 1;
    unsigned OC1PE : 1;
    unsigned OC1M : 3;
    unsigned : 1;
    unsigned CC2S : 2;
    unsigned OC2FE : 1;
    unsigned OC2PE : 1;
    unsigned OC2M : 3;
    unsigned : 17;
} TIM9_12_CCMR1_output_t;
_Static_assert(sizeof(TIM9_12_CCMR1_output_t) == 4U,
               "TIM9_12_CCMR1_output_t is wrong size");

typedef struct
{
    unsigned CC1S : 2;
    unsigned OC1FE : 1;
    unsigned OC1PE : 1;
    unsigned OC1M : 3;
    unsigned : 25;
} TIM10_14_CCMR1_output_t;
_Static_assert(sizeof(TIM10_14_CCMR1_output_t) == 4U,
               "TIM10_14_CCMR1_output_t is wrong size");

typedef struct
{
    unsigned CC1S : 2;
    unsigned IC1PSC : 2;
    unsigned IC1F : 4;
    unsigned CC2S : 2;
    unsigned IC2PSC : 2;
    unsigned IC2F : 4;
    unsigned : 16;
} TIM9_12_CCMR1_input_t;
_Static_assert(sizeof(TIM9_12_CCMR1_input_t) == 4U,
               "TIM9_12_CCMR1_input_t is wrong size");

typedef struct
{
    unsigned CC1S : 2;
    unsigned IC1PSC : 2;
    unsigned IC1F : 4;
    unsigned : 24;
} TIM10_14_CCMR1_input_t;
_Static_assert(sizeof(TIM10_14_CCMR1_input_t) == 4U,
               "TIM10_14_CCMR1_input_t is wrong size");

typedef union {
    TIM9_12_CCMR1_output_t O;
    TIM9_12_CCMR1_input_t I;
} TIM9_12_CCMR1_t;
_Static_assert(sizeof(TIM9_12_CCMR1_t) == 4U, "TIM9_12_CCMR1_t is wrong size");

typedef union {
    TIM10_14_CCMR1_output_t O;
    TIM10_14_CCMR1_input_t I;
} TIM10_14_CCMR1_t;
_Static_assert(sizeof(TIM10_14_CCMR1_t) == 4U, "TIM10_14_CCMR1_t is wrong size");

typedef struct
{
    unsigned CC1E : 1;
    unsigned CC1P : 1;
    unsigned : 1;
    unsigned CC1NP : 1;
    unsigned CC2E : 1;
    unsigned CC2P : 1;
    unsigned : 1;
    unsigned CC2NP : 1;
    unsigned : 24;
} TIM9_12_CCER_t;
_Static_assert(sizeof(TIM9_12_CCER_t) == 4U, "TIM9_12_CCER_t is wrong size");

typedef struct
{
    unsigned CC1E : 1;
    unsigned CC1P : 1;
    unsigned : 1;
    unsigned CC1NP : 1;
    unsigned : 28;
} TIM10_14_CCER_t;
_Static_assert(sizeof(TIM10_14_CCER_t) == 4U, "TIM10_14_CCER_t is wrong size");

typedef struct
{
    unsigned TI1_RMP : 2;
    unsigned : 30;
} TIM11_OR_t;
_Static_assert(sizeof(TIM11_OR_t) == 4U, "TIM11_OR_t is wrong size");

typedef struct
{
    TIM10_14_CR1_t CR1;
    uint32_t reserved1[2];
    TIM10_14_DIER_t DIER;
    TIM10_14_SR_t SR;
    TIM10_14_EGR_t EGR;
    TIM10_14_CCMR1_t CCMR1;
    uint32_t reserved2;
    TIM10_14_CCER_t CCER;
    uint32_t CNT;
    uint32_t PSC;
    uint32_t ARR;
    uint32_t reserved3;
    uint32_t CCR1;
    uint32_t reserved4[6];
    TIM11_OR_t OR;
} TIM11_t;
_Static_assert(sizeof(TIM11_t) == 0x54U, "TIM11_t is wrong size");

typedef struct
{
    TIM9_12_CR1_t CR1;
    TIM9_12_CR2_t CR2;
    TIM9_12_SMCR_t SMCR;
    TIM9_12_DIER_t DIER;
    TIM9_12_SR_t SR;
    TIM9_12_EGR_t EGR;
    TIM9_12_CCMR1_t CCMR1;
    uint32_t reserved1;
    TIM9_12_CCER_t CCER;
    uint32_t CNT;
    uint32_t PSC;
    uint32_t ARR;
    uint32_t reserved2;
    uint32_t CCR1;
    uint32_t CCR2;
} TIM9_12_t;
_Static_assert(sizeof(TIM9_12_t) == 0x3CU, "TIM9_12_t is wrong size");

typedef struct
{
    TIM10_14_CR1_t CR1;
    uint32_t reserved1[2];
    TIM10_14_DIER_t DIER;
    TIM10_14_SR_t SR;
    TIM10_14_EGR_t EGR;
    TIM10_14_CCMR1_t CCMR1;
    uint32_t reserved2;
    TIM10_14_CCER_t CCER;
    uint32_t CNT;
    uint32_t PSC;
    uint32_t ARR;
    uint32_t reserved3;
    uint32_t CCR1;
} TIM10_14_t;
_Static_assert(sizeof(TIM10_14_t) == 0x38U, "TIM10_14_t is wrong size");

typedef struct
{
    unsigned CEN : 1;
    unsigned UDIS : 1;
    unsigned URS : 1;
    unsigned OPM : 1;
    unsigned DIR : 1;
    unsigned CMS : 2;
    unsigned ARPE : 1;
    unsigned CKD : 2;
    unsigned : 22;
} TIM2_5_CR1_t;
_Static_assert(sizeof(TIM2_5_CR1_t) == 4U, "TIM2_5_CR1_t is wrong size");

typedef struct
{
    unsigned : 3;
    unsigned CCDS : 1;
    unsigned MMS : 3;
    unsigned TI1S : 1;
    unsigned : 24;
} TIM2_5_CR2_t;
_Static_assert(sizeof(TIM2_5_CR2_t) == 4U, "TIM2_5_CR2_t is wrong size");

typedef struct
{
    unsigned SMS : 3;
    unsigned : 1;
    unsigned TS : 3;
    unsigned MSM : 1;
    unsigned ETF : 4;
    unsigned ETPS : 2;
    unsigned ECE : 1;
    unsigned ETP : 1;
    unsigned : 16;
} TIM2_5_SMCR_t;
_Static_assert(sizeof(TIM2_5_SMCR_t) == 4U, "TIM2_5_SMCR_t is wrong size");

typedef struct
{
    unsigned UIE : 1;
    unsigned CC1IE : 1;
    unsigned CC2IE : 1;
    unsigned CC3IE : 1;
    unsigned CC4IE : 1;
    unsigned : 1;
    unsigned TIE : 1;
    unsigned : 1;
    unsigned UDE : 1;
    unsigned CC1DE : 1;
    unsigned CC2DE : 1;
    unsigned CC3DE : 1;
    unsigned CC4DE : 1;
    unsigned : 1;
    unsigned TDE : 1;
    unsigned : 17;
} TIM2_5_DIER_t;
_Static_assert(sizeof(TIM2_5_DIER_t) == 4U, "TIM2_5_DIER_t is wrong size");

typedef struct
{
    unsigned UIF : 1;
    unsigned CC1IF : 1;
    unsigned CC2IF : 1;
    unsigned CC3IF : 1;
    unsigned CC4IF : 1;
    unsigned : 1;
    unsigned TIF : 1;
    unsigned : 2;
    unsigned CC1OF : 1;
    unsigned CC2OF : 1;
    unsigned CC3OF : 1;
    unsigned CC4OF : 1;
    unsigned : 19;
} TIM2_5_SR_t;
_Static_assert(sizeof(TIM2_5_SR_t) == 4U, "TIM2_5_SR_t is wrong size");

typedef struct
{
    unsigned UG : 1;
    unsigned CC1G : 1;
    unsigned CC2G : 1;
    unsigned CC3G : 1;
    unsigned CC4G : 1;
    unsigned : 1;
    unsigned TG : 1;
    unsigned : 25;
} TIM2_5_EGR_t;
_Static_assert(sizeof(TIM2_5_EGR_t) == 4U, "TIM2_5_EGR_t is wrong size");

typedef struct
{
    unsigned CC1S : 2;
    unsigned OC1FE : 1;
    unsigned OC1PE : 1;
    unsigned OC1M : 3;
    unsigned OC1CE : 1;
    unsigned CC2S : 2;
    unsigned OC2FE : 1;
    unsigned OC2PE : 1;
    unsigned OC2M : 3;
    unsigned OC2CE : 1;
    unsigned : 16;
} TIM2_5_CCMR1_output_t;
_Static_assert(sizeof(TIM2_5_CCMR1_output_t) == 4U,
               "TIM2_5_CCMR1_output_t is wrong size");

typedef struct
{
    unsigned CC1S : 2;
    unsigned IC1PSC : 2;
    unsigned IC1F : 4;
    unsigned CC2S : 2;
    unsigned IC2PSC : 2;
    unsigned IC2F : 4;
    unsigned : 16;
} TIM2_5_CCMR1_input_t;
_Static_assert(sizeof(TIM2_5_CCMR1_input_t) == 4U, "TIM2_5_CCMR1_input_t is wrong size");

typedef union {
    TIM2_5_CCMR1_output_t O;
    TIM2_5_CCMR1_input_t I;
} TIM2_5_CCMR1_t;
_Static_assert(sizeof(TIM2_5_CCMR1_t) == 4U, "TIM2_5_CCMR1_t is wrong size");

typedef struct
{
    unsigned CC3S : 2;
    unsigned OC3FE : 1;
    unsigned OC3PE : 1;
    unsigned OC3M : 3;
    unsigned OC3CE : 1;
    unsigned CC4S : 2;
    unsigned OC4FE : 1;
    unsigned OC4PE : 1;
    unsigned OC4M : 3;
    unsigned OC4CE : 1;
    unsigned : 16;
} TIM2_5_CCMR2_output_t;
_Static_assert(sizeof(TIM2_5_CCMR2_output_t) == 4U,
               "TIM2_5_CCMR2_output_t is wrong size");

typedef struct
{
    unsigned CC3S : 2;
    unsigned IC3PSC : 2;
    unsigned IC3F : 4;
    unsigned CC4S : 2;
    unsigned IC4PSC : 2;
    unsigned IC4F : 4;
    unsigned : 16;
} TIM2_5_CCMR2_input_t;
_Static_assert(sizeof(TIM2_5_CCMR2_input_t) == 4U, "TIM2_5_CCMR2_input_t is wrong size");

typedef union {
    TIM2_5_CCMR2_output_t O;
    TIM2_5_CCMR2_input_t I;
} TIM2_5_CCMR2_t;
_Static_assert(sizeof(TIM2_5_CCMR2_t) == 4U, "TIM2_5_CCMR2_t is wrong size");

typedef struct
{
    unsigned CC1E : 1;
    unsigned CC1P : 1;
    unsigned : 1;
    unsigned CC1NP : 1;
    unsigned CC2E : 1;
    unsigned CC2P : 1;
    unsigned : 1;
    unsigned CC2NP : 1;
    unsigned CC3E : 1;
    unsigned CC3P : 1;
    unsigned : 1;
    unsigned CC3NP : 1;
    unsigned CC4E : 1;
    unsigned CC4P : 1;
    unsigned : 1;
    unsigned CC4NP : 1;
    unsigned : 16;
} TIM2_5_CCER_t;
_Static_assert(sizeof(TIM2_5_CCER_t) == 4U, "TIM2_5_CCER_t is wrong size");

typedef struct
{
    unsigned DBA : 5;
    unsigned : 3;
    unsigned DBL : 5;
    unsigned : 19;
} TIM2_5_DCR_t;
_Static_assert(sizeof(TIM2_5_DCR_t) == 4U, "TIM2_5_DCR_t is wrong size");

typedef struct
{
    unsigned : 10;
    unsigned ITR1_RMP : 2;
    unsigned : 20;
} TIM2_OR_t;
_Static_assert(sizeof(TIM2_OR_t) == 4U, "TIM2_OR_t is wrong size");

typedef struct
{
    unsigned : 6;
    unsigned TI4_RMP : 2;
    unsigned : 24;
} TIM5_OR_t;
_Static_assert(sizeof(TIM5_OR_t) == 4U, "TIM5_OR_t is wrong size");

typedef struct
{
    TIM2_5_CR1_t CR1;
    TIM2_5_CR2_t CR2;
    TIM2_5_SMCR_t SMCR;
    TIM2_5_DIER_t DIER;
    TIM2_5_SR_t SR;
    TIM2_5_EGR_t EGR;
    TIM2_5_CCMR1_t CCMR1;
    TIM2_5_CCMR2_t CCMR2;
    TIM2_5_CCER_t CCER;
    uint32_t CNT;
    uint32_t PSC;
    uint32_t ARR;
    uint32_t reserved1;
    uint32_t CCR1;
    uint32_t CCR2;
    uint32_t CCR3;
    uint32_t CCR4;
    uint32_t reserved2;
    TIM2_5_DCR_t DCR;
    uint32_t DMAR;
    TIM2_OR_t OR;
} TIM2_t;
_Static_assert(sizeof(TIM2_t) == 0x54U, "TIM2_t is wrong size");

typedef struct
{
    TIM2_5_CR1_t CR1;
    TIM2_5_CR2_t CR2;
    TIM2_5_SMCR_t SMCR;
    TIM2_5_DIER_t DIER;
    TIM2_5_SR_t SR;
    TIM2_5_EGR_t EGR;
    TIM2_5_CCMR1_t CCMR1;
    TIM2_5_CCMR2_t CCMR2;
    TIM2_5_CCER_t CCER;
    uint32_t CNT;
    uint32_t PSC;
    uint32_t ARR;
    uint32_t reserved1;
    uint32_t CCR1;
    uint32_t CCR2;
    uint32_t CCR3;
    uint32_t CCR4;
    uint32_t reserved2;
    TIM2_5_DCR_t DCR;
    uint32_t DMAR;
    TIM5_OR_t OR;
} TIM5_t;
_Static_assert(sizeof(TIM5_t) == 0x54U, "TIM5_t is wrong size");

typedef struct
{
    TIM2_5_CR1_t CR1;
    TIM2_5_CR2_t CR2;
    TIM2_5_SMCR_t SMCR;
    TIM2_5_DIER_t DIER;
    TIM2_5_SR_t SR;
    TIM2_5_EGR_t EGR;
    TIM2_5_CCMR1_t CCMR1;
    TIM2_5_CCMR2_t CCMR2;
    TIM2_5_CCER_t CCER;
    uint32_t CNT;
    uint32_t PSC;
    uint32_t ARR;
    uint32_t reserved1;
    uint32_t CCR1;
    uint32_t CCR2;
    uint32_t CCR3;
    uint32_t CCR4;
    uint32_t reserved2;
    TIM2_5_DCR_t DCR;
    uint32_t DMAR;
} TIM2_5_t;
_Static_assert(sizeof(TIM2_5_t) == 0x50U, "TIM2_5_t is wrong size");

typedef TIM2_5_CR1_t TIM_act_CR1_t;

typedef struct
{
    unsigned CCPC : 1;
    unsigned : 1;
    unsigned CCUS : 1;
    unsigned CCDS : 1;
    unsigned MMS : 3;
    unsigned TI1S : 1;
    unsigned OIS1 : 1;
    unsigned OIS1N : 1;
    unsigned OIS2 : 1;
    unsigned OIS2N : 1;
    unsigned OIS3 : 1;
    unsigned OIS3N : 1;
    unsigned OIS4 : 1;
    unsigned : 17;
} TIM_act_CR2_t;
_Static_assert(sizeof(TIM_act_CR2_t) == 4U, "TIM_act_CR2_t is wrong size");

typedef TIM2_5_SMCR_t TIM_act_SMCR_t;

typedef struct
{
    unsigned UIE : 1;
    unsigned CC1IE : 1;
    unsigned CC2IE : 1;
    unsigned CC3IE : 1;
    unsigned CC4IE : 1;
    unsigned COMIE : 1;
    unsigned TIE : 1;
    unsigned BIE : 1;
    unsigned UDE : 1;
    unsigned CC1DE : 1;
    unsigned CC2DE : 1;
    unsigned CC3DE : 1;
    unsigned CC4DE : 1;
    unsigned COMDE : 1;
    unsigned TDE : 1;
    unsigned : 17;
} TIM_act_DIER_t;
_Static_assert(sizeof(TIM_act_DIER_t) == 4U, "TIM_act_DIER_t is wrong size");

typedef struct
{
    unsigned UIF : 1;
    unsigned CC1IF : 1;
    unsigned CC2IF : 1;
    unsigned CC3IF : 1;
    unsigned CC4IF : 1;
    unsigned COMIF : 1;
    unsigned TIF : 1;
    unsigned BIF : 1;
    unsigned : 1;
    unsigned CC1OF : 1;
    unsigned CC2OF : 1;
    unsigned CC3OF : 1;
    unsigned CC4OF : 1;
    unsigned : 19;
} TIM_act_SR_t;
_Static_assert(sizeof(TIM_act_SR_t) == 4U, "TIM_act_SR_t is wrong size");

typedef struct
{
    unsigned UG : 1;
    unsigned CC1G : 1;
    unsigned CC2G : 1;
    unsigned CC3G : 1;
    unsigned CC4G : 1;
    unsigned COMG : 1;
    unsigned TG : 1;
    unsigned BG : 1;
    unsigned : 24;
} TIM_act_EGR_t;
_Static_assert(sizeof(TIM_act_EGR_t) == 4U, "TIM_act_EGR_t is wrong size");

typedef TIM2_5_CCMR1_t TIM_act_CCMR1_t;
typedef TIM2_5_CCMR2_t TIM_act_CCMR2_t;

typedef struct
{
    unsigned CC1E : 1;
    unsigned CC1P : 1;
    unsigned CC1NE : 1;
    unsigned CC1NP : 1;
    unsigned CC2E : 1;
    unsigned CC2P : 1;
    unsigned CC2NE : 1;
    unsigned CC2NP : 1;
    unsigned CC3E : 1;
    unsigned CC3P : 1;
    unsigned CC3NE : 1;
    unsigned CC3NP : 1;
    unsigned CC4E : 1;
    unsigned CC4P : 1;
    unsigned : 18;
} TIM_act_CCER_t;
_Static_assert(sizeof(TIM_act_CCER_t) == 4U, "TIM_act_CCER_t is wrong size");

typedef struct
{
    unsigned DTG : 8;
    unsigned LOCK : 2;
    unsigned OSSI : 1;
    unsigned OSSR : 1;
    unsigned BKE : 1;
    unsigned BKP : 1;
    unsigned AOE : 1;
    unsigned MOE : 1;
    unsigned : 16;
} TIM_act_BDTR_t;
_Static_assert(sizeof(TIM_act_BDTR_t) == 4U, "TIM_act_BDTR_t is wrong size");

typedef TIM2_5_DCR_t TIM_act_DCR_t;

typedef struct
{
    TIM_act_CR1_t CR1;
    TIM_act_CR2_t CR2;
    TIM_act_SMCR_t SMCR;
    TIM_act_DIER_t DIER;
    TIM_act_SR_t SR;
    TIM_act_EGR_t EGR;
    TIM_act_CCMR1_t CCMR1;
    TIM_act_CCMR2_t CCMR2;
    TIM_act_CCER_t CCER;
    uint32_t CNT;
    uint32_t PSC;
    uint32_t ARR;
    uint32_t RCR;
    uint32_t CCR1;
    uint32_t CCR2;
    uint32_t CCR3;
    uint32_t CCR4;
    TIM_act_BDTR_t BDTR;
    TIM_act_DCR_t DCR;
    uint32_t DMAR;
} TIM_act_t;
_Static_assert(sizeof(TIM_act_t) == 0x50U, "TIM_act_t is wrong size");

extern volatile TIM_act_t TIM1;
extern volatile TIM2_t TIM2;
extern volatile TIM2_5_t TIM3;
extern volatile TIM2_5_t TIM4;
extern volatile TIM5_t TIM5;
extern volatile TIM_basic_t TIM6;
extern volatile TIM_basic_t TIM7;
extern volatile TIM_act_t TIM8;
extern volatile TIM9_12_t TIM9;
extern volatile TIM10_14_t TIM10;
extern volatile TIM11_t TIM11;
extern volatile TIM9_12_t TIM12;
extern volatile TIM10_14_t TIM13;
extern volatile TIM10_14_t TIM14;

#endif

/**
 * @}
 */
