/**
 * \ingroup REG
 * \defgroup REGRCC Reset and clock control
 * @{
 */
#ifndef STM32LIB_REGISTERS_RCC_H
#define STM32LIB_REGISTERS_RCC_H

typedef struct
{
    unsigned HSION : 1;
    unsigned HSIRDY : 1;
    unsigned : 1;
    unsigned HSITRIM : 5;
    unsigned HSICAL : 8;
    unsigned HSEON : 1;
    unsigned HSERDY : 1;
    unsigned HSEBYP : 1;
    unsigned CSSON : 1;
    unsigned : 4;
    unsigned PLLON : 1;
    unsigned PLLRDY : 1;
    unsigned PLLI2SON : 1;
    unsigned PLLI2SRDY : 1;
    unsigned : 4;
} RCC_CR_t;
_Static_assert(sizeof(RCC_CR_t) == 4U, "RCC_CR_t is wrong size");

typedef struct
{
    unsigned PLLM : 6;
    unsigned PLLN : 9;
    unsigned : 1;
    unsigned PLLP : 2;
    unsigned : 4;
    unsigned PLLSRC : 1;
    unsigned : 1;
    unsigned PLLQ : 4;
    unsigned : 4;
} RCC_PLLCFGR_t;
_Static_assert(sizeof(RCC_PLLCFGR_t) == 4U, "RCC_PLLCFGR_t is wrong size");

typedef struct
{
    unsigned SW : 2;
    unsigned SWS : 2;
    unsigned HPRE : 4;
    unsigned : 2;
    unsigned PPRE1 : 3;
    unsigned PPRE2 : 3;
    unsigned RTCPRE : 5;
    unsigned MCO1 : 2;
    unsigned I2SSRC : 1;
    unsigned MCO1PRE : 3;
    unsigned MCO2PRE : 3;
    unsigned MCO2 : 2;
} RCC_CFGR_t;
_Static_assert(sizeof(RCC_CFGR_t) == 4U, "RCC_CFGR_t is wrong size");

typedef struct
{
    unsigned LSIRDYF : 1;
    unsigned LSERDYF : 1;
    unsigned HSIRDYF : 1;
    unsigned HSERDYF : 1;
    unsigned PLLRDYF : 1;
    unsigned PLLI2SRDYF : 1;
    unsigned : 1;
    unsigned CSSF : 1;
    unsigned LSIRDYIE : 1;
    unsigned LSERDYIE : 1;
    unsigned HSIRDYIE : 1;
    unsigned HSERDYIE : 1;
    unsigned PLLRDYIE : 1;
    unsigned PLLI2SRDYIE : 1;
    unsigned : 2;
    unsigned LSIRDYC : 1;
    unsigned LSERDYC : 1;
    unsigned HSIRDYC : 1;
    unsigned HSERDYC : 1;
    unsigned PLLRDYC : 1;
    unsigned PLLI2SRDYC : 1;
    unsigned : 1;
    unsigned CSSC : 1;
    unsigned : 8;
} RCC_CIR_t;
_Static_assert(sizeof(RCC_CIR_t) == 4U, "RCC_CIR_t is wrong size");

typedef struct
{
    unsigned GPIOARST : 1;
    unsigned GPIOBRST : 1;
    unsigned GPIOCRST : 1;
    unsigned GPIODRST : 1;
    unsigned GPIOERST : 1;
    unsigned GPIOFRST : 1;
    unsigned GPIOGRST : 1;
    unsigned GPIOHRST : 1;
    unsigned GPIOIRST : 1;
    unsigned : 3;
    unsigned CRCRST : 1;
    unsigned : 8;
    unsigned DMA1RST : 1;
    unsigned DMA2RST : 1;
    unsigned : 2;
    unsigned ETHMACRST : 1;
    unsigned : 3;
    unsigned OTGHSRST : 1;
    unsigned : 2;
} RCC_AHB1RSTR_t;
_Static_assert(sizeof(RCC_AHB1RSTR_t) == 4U, "RCC_AHB1RSTR_t is wrong size");

typedef struct
{
    unsigned DCMIRST : 1;
    unsigned : 3;
    unsigned CRYPRST : 1;
    unsigned HASHRST : 1;
    unsigned RNGRST : 1;
    unsigned OTGFSRST : 1;
    unsigned : 24;
} RCC_AHB2RSTR_t;
_Static_assert(sizeof(RCC_AHB2RSTR_t) == 4U, "RCC_AHB2RSTR_t is wrong size");

typedef struct
{
    unsigned FSMCRST : 1;
    unsigned : 31;
} RCC_AHB3RSTR_t;
_Static_assert(sizeof(RCC_AHB3RSTR_t) == 4U, "RCC_AHB3RSTR_t is wrong size");

typedef struct
{
    unsigned TIM2RST : 1;
    unsigned TIM3RST : 1;
    unsigned TIM4RST : 1;
    unsigned TIM5RST : 1;
    unsigned TIM6RST : 1;
    unsigned TIM7RST : 1;
    unsigned TIM12RST : 1;
    unsigned TIM13RST : 1;
    unsigned TIM14RST : 1;
    unsigned : 2;
    unsigned WWDGRST : 1;
    unsigned : 2;
    unsigned SPI2RST : 1;
    unsigned SPI3RST : 1;
    unsigned : 1;
    unsigned UART2RST : 1;
    unsigned UART3RST : 1;
    unsigned UART4RST : 1;
    unsigned UART5RST : 1;
    unsigned I2C1RST : 1;
    unsigned I2C2RST : 1;
    unsigned I2C3RST : 1;
    unsigned : 1;
    unsigned CAN1RST : 1;
    unsigned CAN2RST : 1;
    unsigned : 1;
    unsigned PWRRST : 1;
    unsigned DACRST : 1;
    unsigned UART7RST : 1;
    unsigned UART8RST : 1;
} RCC_APB1RSTR_t;
_Static_assert(sizeof(RCC_APB1RSTR_t) == 4U, "RCC_APB1RSTR_t is wrong size");

typedef struct
{
    unsigned TIM1RST : 1;
    unsigned TIM8RST : 1;
    unsigned : 2;
    unsigned USART1RST : 1;
    unsigned USART6RST : 1;
    unsigned : 2;
    unsigned ADCRST : 1;
    unsigned : 2;
    unsigned SDIORST : 1;
    unsigned SPI1RST : 1;
    unsigned SPI4RST : 1;
    unsigned SYSCFGRST : 1;
    unsigned : 1;
    unsigned TIM9RST : 1;
    unsigned TIM10RST : 1;
    unsigned TIM11RST : 1;
    unsigned : 1;
    unsigned SPI5RST : 1;
    unsigned SPI6RST : 1;
    unsigned : 10;
} RCC_APB2RSTR_t;
_Static_assert(sizeof(RCC_APB2RSTR_t) == 4U, "RCC_APB2RSTR_t is wrong size");

typedef struct
{
    unsigned GPIOAEN : 1;
    unsigned GPIOBEN : 1;
    unsigned GPIOCEN : 1;
    unsigned GPIODEN : 1;
    unsigned GPIOEEN : 1;
    unsigned GPIOFEN : 1;
    unsigned GPIOGEN : 1;
    unsigned GPIOHEN : 1;
    unsigned GPIOIEN : 1;
    unsigned : 3;
    unsigned CRCEN : 1;
    unsigned : 5;
    unsigned BKPSRAMEN : 1;
    unsigned : 1;
    unsigned CCMDATARAMEN : 1;
    unsigned DMA1EN : 1;
    unsigned DMA2EN : 1;
    unsigned : 2;
    unsigned ETHMACEN : 1;
    unsigned ETHMACTXEN : 1;
    unsigned ETHMACRXEN : 1;
    unsigned ETHMACPTPEN : 1;
    unsigned OTGHSEN : 1;
    unsigned OTGHSULPIEN : 1;
    unsigned : 1;
} RCC_AHB1ENR_t;
_Static_assert(sizeof(RCC_AHB1ENR_t) == 4U, "RCC_AHB1ENR_t is wrong size");

typedef struct
{
    unsigned DCMIEN : 1;
    unsigned : 3;
    unsigned CRYPEN : 1;
    unsigned HASHEN : 1;
    unsigned RNGEN : 1;
    unsigned OTGFSEN : 1;
    unsigned : 24;
} RCC_AHB2ENR_t;
_Static_assert(sizeof(RCC_AHB2ENR_t) == 4U, "RCC_AHB2ENR_t is wrong size");

typedef struct
{
    unsigned FSMCEN : 1;
    unsigned : 31;
} RCC_AHB3ENR_t;
_Static_assert(sizeof(RCC_AHB3ENR_t) == 4U, "RCC_AHB3ENR_t is wrong size");

typedef struct
{
    unsigned TIM2EN : 1;
    unsigned TIM3EN : 1;
    unsigned TIM4EN : 1;
    unsigned TIM5EN : 1;
    unsigned TIM6EN : 1;
    unsigned TIM7EN : 1;
    unsigned TIM12EN : 1;
    unsigned TIM13EN : 1;
    unsigned TIM14EN : 1;
    unsigned : 2;
    unsigned WWDGEN : 1;
    unsigned : 2;
    unsigned SPI2EN : 1;
    unsigned SPI3EN : 1;
    unsigned : 1;
    unsigned UART2EN : 1;
    unsigned UART3EN : 1;
    unsigned UART4EN : 1;
    unsigned UART5EN : 1;
    unsigned I2C1EN : 1;
    unsigned I2C2EN : 1;
    unsigned I2C3EN : 1;
    unsigned : 1;
    unsigned CAN1EN : 1;
    unsigned CAN2EN : 1;
    unsigned : 1;
    unsigned PWREN : 1;
    unsigned DACEN : 1;
    unsigned UART7EN : 1;
    unsigned UART8EN : 1;
} RCC_APB1ENR_t;
_Static_assert(sizeof(RCC_APB1ENR_t) == 4U, "RCC_APB1ENR_t is wrong size");

typedef struct
{
    unsigned TIM1EN : 1;
    unsigned TIM8EN : 1;
    unsigned : 2;
    unsigned USART1EN : 1;
    unsigned USART6EN : 1;
    unsigned : 2;
    unsigned ADC1EN : 1;
    unsigned ADC2EN : 1;
    unsigned ADC3EN : 1;
    unsigned SDIOEN : 1;
    unsigned SPI1EN : 1;
    unsigned SPI4EN : 1;
    unsigned SYSCFGEN : 1;
    unsigned : 1;
    unsigned TIM9EN : 1;
    unsigned TIM10EN : 1;
    unsigned TIM11EN : 1;
    unsigned : 1;
    unsigned SPI5EN : 1;
    unsigned SPI6EN : 1;
    unsigned : 10;
} RCC_APB2ENR_t;
_Static_assert(sizeof(RCC_APB2ENR_t) == 4U, "RCC_APB2ENR_t is wrong size");

typedef struct
{
    unsigned GPIOALPEN : 1;
    unsigned GPIOBLPEN : 1;
    unsigned GPIOCLPEN : 1;
    unsigned GPIODLPEN : 1;
    unsigned GPIOELPEN : 1;
    unsigned GPIOFLPEN : 1;
    unsigned GPIOGLPEN : 1;
    unsigned GPIOHLPEN : 1;
    unsigned GPIOILPEN : 1;
    unsigned : 3;
    unsigned CRCLPEN : 1;
    unsigned : 2;
    unsigned FLITFLPEN : 1;
    unsigned SRAM1LPEN : 1;
    unsigned SRAM2LPEN : 1;
    unsigned BKPSRAMLPEN : 1;
    unsigned SRAM3LPEN : 1;
    unsigned : 1;
    unsigned DMA1LPEN : 1;
    unsigned DMA2LPEN : 1;
    unsigned : 2;
    unsigned ETHMACLPEN : 1;
    unsigned ETHTXLPEN : 1;
    unsigned ETHRXLPEN : 1;
    unsigned ETHPTPLPEN : 1;
    unsigned OTGHSLPEN : 1;
    unsigned OTGHSULPILPEN : 1;
    unsigned : 1;
} RCC_AHB1LPENR_t;
_Static_assert(sizeof(RCC_AHB1LPENR_t) == 4U, "RCC_AHB1LPENR_t is wrong size");

typedef struct
{
    unsigned DCMILPEN : 1;
    unsigned : 3;
    unsigned CRYPLPEN : 1;
    unsigned HASHLPEN : 1;
    unsigned RNGLPEN : 1;
    unsigned OTGFSLPEN : 1;
    unsigned : 24;
} RCC_AHB2LPENR_t;
_Static_assert(sizeof(RCC_AHB2LPENR_t) == 4U, "RCC_AHB2LPENR_t is wrong size");

typedef struct
{
    unsigned FSMCLPEN : 1;
    unsigned : 31;
} RCC_AHB3LPENR_t;
_Static_assert(sizeof(RCC_AHB3LPENR_t) == 4U, "RCC_AHB3LPENR_t is wrong size");

typedef struct
{
    unsigned TIM2LPEN : 1;
    unsigned TIM3LPEN : 1;
    unsigned TIM4LPEN : 1;
    unsigned TIM5LPEN : 1;
    unsigned TIM6LPEN : 1;
    unsigned TIM7LPEN : 1;
    unsigned TIM12LPEN : 1;
    unsigned TIM13LPEN : 1;
    unsigned TIM14LPEN : 1;
    unsigned : 2;
    unsigned WWDGLPEN : 1;
    unsigned : 2;
    unsigned SPI2LPEN : 1;
    unsigned SPI3LPEN : 1;
    unsigned : 1;
    unsigned UART2LPEN : 1;
    unsigned UART3LPEN : 1;
    unsigned UART4LPEN : 1;
    unsigned UART5LPEN : 1;
    unsigned I2C1LPEN : 1;
    unsigned I2C2LPEN : 1;
    unsigned I2C3LPEN : 1;
    unsigned : 1;
    unsigned CAN1LPEN : 1;
    unsigned CAN2LPEN : 1;
    unsigned : 1;
    unsigned PWRLPEN : 1;
    unsigned DACLPEN : 1;
    unsigned UART7LPEN : 1;
    unsigned UART8LPEN : 1;
} RCC_APB1LPENR_t;
_Static_assert(sizeof(RCC_APB1LPENR_t) == 4U, "RCC_APB1LPENR_t is wrong size");

typedef struct
{
    unsigned TIM1LPEN : 1;
    unsigned TIM8LPEN : 1;
    unsigned : 2;
    unsigned USART1LPEN : 1;
    unsigned USART6LPEN : 1;
    unsigned : 2;
    unsigned ADC1LPEN : 1;
    unsigned ADC2LPEN : 1;
    unsigned ADC3LPEN : 1;
    unsigned SDIOLPEN : 1;
    unsigned SPI1LPEN : 1;
    unsigned SPI4LPEN : 1;
    unsigned SYSCFGLPEN : 1;
    unsigned : 1;
    unsigned TIM9LPEN : 1;
    unsigned TIM10LPEN : 1;
    unsigned TIM11LPEN : 1;
    unsigned : 1;
    unsigned SPI5LPEN : 1;
    unsigned SPI6LPEN : 1;
    unsigned : 10;
} RCC_APB2LPENR_t;
_Static_assert(sizeof(RCC_APB2LPENR_t) == 4U, "RCC_APB2LPENR_t is wrong size");

typedef struct
{
    unsigned LSEON : 1;
    unsigned LSERDY : 1;
    unsigned LSEBYP : 1;
    unsigned : 5;
    unsigned RTCSEL : 2;
    unsigned : 5;
    unsigned RTCEN : 1;
    unsigned BDRST : 1;
    unsigned : 15;
} RCC_BDCR_t;
_Static_assert(sizeof(RCC_BDCR_t) == 4U, "RCC_BDCR_t is wrong size");

typedef struct
{
    unsigned LSION : 1;
    unsigned LSIRDY : 1;
    unsigned : 22;
    unsigned RMVF : 1;
    unsigned BORRSTF : 1;
    unsigned PINRSTF : 1;
    unsigned PORRSTF : 1;
    unsigned SFTRSTF : 1;
    unsigned IWDGRSTF : 1;
    unsigned WWDGRSTF : 1;
    unsigned LPWRRSTF : 1;
} RCC_CSR_t;
_Static_assert(sizeof(RCC_CSR_t) == 4U, "RCC_CSR_t is wrong size");

typedef struct
{
    unsigned MODPER : 13;
    unsigned INCSTEP : 15;
    unsigned : 2;
    unsigned SPREADSEL : 1;
    unsigned SSCGEN : 1;
} RCC_SSCGR_t;
_Static_assert(sizeof(RCC_SSCGR_t) == 4U, "RCC_SSCGR_t is wrong size");

typedef struct
{
    unsigned : 6;
    unsigned PLLI2SN : 9;
    unsigned : 13;
    unsigned PLLI2SR : 3;
    unsigned : 1;
} RCC_PLLI2SCFGR_t;
_Static_assert(sizeof(RCC_PLLI2SCFGR_t) == 4U, "RCC_PLLI2SCFGR_t is wrong size");

typedef struct
{
    unsigned : 6;
    unsigned PLLSAIN : 9;
    unsigned : 9;
    unsigned PLLSAIQ : 4;
    unsigned PLLSAIR : 3;
    unsigned : 1;
} RCC_PLLSAICFGR_t;
_Static_assert(sizeof(RCC_PLLSAICFGR_t) == 4U, "RCC_PLLSAICFGR_t is wrong size");

typedef struct
{
    unsigned : 24;
    unsigned TIMPRE : 1;
    unsigned : 7;
} RCC_DCKCFGR_t;
_Static_assert(sizeof(RCC_DCKCFGR_t) == 4U, "RCC_DCKCFGR_t is wrong size");

typedef struct
{
    RCC_CR_t CR;
    RCC_PLLCFGR_t PLLCFGR;
    RCC_CFGR_t CFGR;
    RCC_CIR_t CIR;
    RCC_AHB1RSTR_t AHB1RSTR;
    RCC_AHB2RSTR_t AHB2RSTR;
    RCC_AHB3RSTR_t AHB3RSTR;
    unsigned int pad1;
    RCC_APB1RSTR_t APB1RSTR;
    RCC_APB2RSTR_t APB2RSTR;
    unsigned int pad2[2U];
    RCC_AHB1ENR_t AHB1ENR;
    RCC_AHB2ENR_t AHB2ENR;
    RCC_AHB3ENR_t AHB3ENR;
    unsigned int pad3;
    RCC_APB1ENR_t APB1ENR;
    RCC_APB2ENR_t APB2ENR;
    unsigned int pad4[2U];
    RCC_AHB1LPENR_t AHB1LPENR;
    RCC_AHB2LPENR_t AHB2LPENR;
    RCC_AHB3LPENR_t AHB3LPENR;
    unsigned int pad5;
    RCC_APB1LPENR_t APB1LPENR;
    RCC_APB2LPENR_t APB2LPENR;
    unsigned int pad6[2U];
    RCC_BDCR_t BDCR;
    RCC_CSR_t CSR;
    unsigned int pad7[2U];
    RCC_SSCGR_t SSCGR;
    RCC_PLLI2SCFGR_t PLLI2SCFGR;
    RCC_PLLSAICFGR_t PLLSAICFGR;
    RCC_DCKCFGR_t DCKCFGR;
} RCC_t;
_Static_assert(sizeof(RCC_t) == 0x90U, "RCC_t is wrong size");

extern volatile RCC_t RCC;

#endif

/**
 * @}
 */
