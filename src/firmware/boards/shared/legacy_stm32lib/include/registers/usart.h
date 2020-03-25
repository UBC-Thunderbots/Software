/**
 * \ingroup REG
 * \defgroup REGUSART Universal synchronous asynchronous receiver transmitter
 * @{
 */
#ifndef STM32LIB_INCLUDE_REGISTERS_USART_H
#define STM32LIB_INCLUDE_REGISTERS_USART_H

typedef struct
{
    unsigned PE : 1;
    unsigned FE : 1;
    unsigned NF : 1;
    unsigned ORE : 1;
    unsigned IDLE : 1;
    unsigned RXNE : 1;
    unsigned TC : 1;
    unsigned TXE : 1;
    unsigned LBD : 1;
    unsigned CTS : 1;
    unsigned : 22;
} USART_SR_t;
_Static_assert(sizeof(USART_SR_t) == 4U, "USART_SR_t is wrong size");

typedef struct
{
    unsigned PE : 1;
    unsigned FE : 1;
    unsigned NF : 1;
    unsigned ORE : 1;
    unsigned IDLE : 1;
    unsigned RXNE : 1;
    unsigned TC : 1;
    unsigned TXE : 1;
    unsigned LBD : 1;
    unsigned : 23;
} UART_SR_t;
_Static_assert(sizeof(UART_SR_t) == 4U, "UART_SR_t is wrong size");

typedef struct
{
    unsigned DIV_Fraction : 4;
    unsigned DIV_Mantissa : 12;
    unsigned : 16;
} USART_BRR_t;
_Static_assert(sizeof(USART_BRR_t) == 4U, "USART_BRR_t is wrong size");

typedef struct
{
    unsigned SBK : 1;
    unsigned RWU : 1;
    unsigned RE : 1;
    unsigned TE : 1;
    unsigned IDLEIE : 1;
    unsigned RXNEIE : 1;
    unsigned TCIE : 1;
    unsigned TXEIE : 1;
    unsigned PEIE : 1;
    unsigned PS : 1;
    unsigned PCE : 1;
    unsigned WAKE : 1;
    unsigned M : 1;
    unsigned UE : 1;
    unsigned : 1;
    unsigned OVER8 : 1;
    unsigned : 16;
} USART_CR1_t;
_Static_assert(sizeof(USART_CR1_t) == 4U, "USART_CR1_t is wrong size");

typedef struct
{
    unsigned ADD : 4;
    unsigned : 1;
    unsigned LBDL : 1;
    unsigned LBDIE : 1;
    unsigned : 1;
    unsigned LBCL : 1;
    unsigned CPHA : 1;
    unsigned CPOL : 1;
    unsigned CLKEN : 1;
    unsigned STOP : 2;
    unsigned LINEN : 1;
    unsigned : 17;
} USART_CR2_t;
_Static_assert(sizeof(USART_CR2_t) == 4U, "USART_CR2_t is wrong size");

typedef struct
{
    unsigned ADD : 4;
    unsigned : 1;
    unsigned LBDL : 1;
    unsigned LBDIE : 1;
    unsigned : 5;
    unsigned STOP : 2;
    unsigned LINEN : 1;
    unsigned : 17;
} UART_CR2_t;
_Static_assert(sizeof(UART_CR2_t) == 4U, "UART_CR2_t is wrong size");

typedef struct
{
    unsigned EIE : 1;
    unsigned IREN : 1;
    unsigned IRLP : 1;
    unsigned HDSEL : 1;
    unsigned NACK : 1;
    unsigned SCEN : 1;
    unsigned DMAR : 1;
    unsigned DMAT : 1;
    unsigned RTSE : 1;
    unsigned CTSE : 1;
    unsigned CTSIE : 1;
    unsigned ONEBIT : 1;
    unsigned : 20;
} USART_CR3_t;
_Static_assert(sizeof(USART_CR3_t) == 4U, "USART_CR3_t is wrong size");

typedef struct
{
    unsigned EIE : 1;
    unsigned IREN : 1;
    unsigned IRLP : 1;
    unsigned HDSEL : 1;
    unsigned : 2;
    unsigned DMAR : 1;
    unsigned DMAT : 1;
    unsigned : 3;
    unsigned ONEBIT : 1;
    unsigned : 20;
} UART_CR3_t;
_Static_assert(sizeof(UART_CR3_t) == 4U, "UART_CR3_t is wrong size");

typedef struct
{
    unsigned PSC : 8;
    unsigned GT : 8;
    unsigned : 16;
} USART_GTPR_t;
_Static_assert(sizeof(USART_GTPR_t) == 4U, "USART_GTPR_t is wrong size");

typedef struct
{
    USART_SR_t SR;
    unsigned int DR;
    USART_BRR_t BRR;
    USART_CR1_t CR1;
    USART_CR2_t CR2;
    USART_CR3_t CR3;
    USART_GTPR_t GTPR;
} USART_t;
_Static_assert(sizeof(USART_t) == 0x1CU, "USART_t is wrong size");

typedef struct
{
    UART_SR_t SR;
    unsigned int DR;
    USART_BRR_t BRR;
    USART_CR1_t CR1;
    UART_CR2_t CR2;
    UART_CR3_t CR3;
} UART_t;
_Static_assert(sizeof(UART_t) == 0x18U, "UART_t is wrong size");

extern volatile USART_t USART1;
extern volatile USART_t USART2;
extern volatile USART_t USART3;
extern volatile UART_t UART4;
extern volatile UART_t UART5;
extern volatile USART_t USART6;
extern volatile USART_t USART7;
extern volatile USART_t USART8;

#endif

/**
 * @}
 */
