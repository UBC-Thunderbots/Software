/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32h7xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_it.h"

#include "FreeRTOS.h"
#include "main.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>

#include "firmware/app/logger/logger.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ETH_HandleTypeDef heth;
extern DMA_HandleTypeDef hdma_adc3;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

    /* USER CODE END NonMaskableInt_IRQn 0 */
    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

    /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
    /* USER CODE BEGIN HardFault_IRQn 0 */
    TLOG_FATAL(" ------------- HARD FAULT -------------- ");

    if (SCB->CFSR & SCB_CFSR_IACCVIOL_Msk)
    {
        TLOG_FATAL("Instruction access violation flag");
    }
    if (SCB->CFSR & SCB_CFSR_MUNSTKERR_Msk)
    {
        TLOG_FATAL("Memory manager fault on unstacking for a return from exception");
    }
    if (SCB->CFSR & SCB_CFSR_MSTKERR_Msk)
    {
        TLOG_FATAL("Memory manager fault on stacking for exception entry.");
    }
    if (SCB->CFSR & SCB_CFSR_MMARVALID_Msk)
    {
        TLOG_FATAL("Memory Management Fault Address Register (MMAR) valid flag");
    }
    if (SCB->CFSR & SCB_CFSR_IBUSERR_Msk)
    {
        TLOG_FATAL("Instruction bus error");
    };
    if (SCB->CFSR & SCB_CFSR_PRECISERR_Msk)
    {
        TLOG_FATAL("Precise data bus error");
    };
    if (SCB->CFSR & SCB_CFSR_IMPRECISERR_Msk)
    {
        TLOG_FATAL("Precise data bus error");
    };
    if (SCB->CFSR & SCB_CFSR_UNSTKERR_Msk)
    {
        TLOG_FATAL("Bus fault on unstacking for a return from exception");
    };
    if (SCB->CFSR & SCB_CFSR_STKERR_Msk)
    {
        TLOG_FATAL("Bus fault on stacking for exception entry");
    };
    if (SCB->CFSR & SCB_CFSR_LSPERR_Msk)
    {
        TLOG_FATAL("Bus fault on floating-point lazy state preservation");
    };
    if (SCB->CFSR & SCB_CFSR_BFARVALID_Msk)
    {
        TLOG_FATAL(
            "Bus Fault Address Register (BFAR) is valid, so probably an invalid mem access");
        TLOG_FATAL("BFAR value: %x", SCB->BFAR);
    };
    if (SCB->CFSR & SCB_CFSR_UNDEFINSTR_Msk)
    {
        TLOG_FATAL("Undefined instruction usage fault");
    };
    if (SCB->CFSR & SCB_CFSR_INVSTATE_Msk)
    {
        TLOG_FATAL("Invalid state usage fault");
    };
    if (SCB->CFSR & SCB_CFSR_INVPC_Msk)
    {
        TLOG_FATAL("Invalid PC load usage fault");
    };
    if (SCB->CFSR & SCB_CFSR_NOCP_Msk)
    {
        TLOG_FATAL("No coprocessor usage fault.");
    };
    if (SCB->CFSR & SCB_CFSR_UNALIGNED_Msk)
    {
        TLOG_FATAL("Unaligned access usage fault");
    };
    if (SCB->CFSR & SCB_CFSR_DIVBYZERO_Msk)
    {
        TLOG_FATAL("Divide by zero usage fault");
    };

    /* USER CODE END HardFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
    /* USER CODE BEGIN MemoryManagement_IRQn 0 */

    /* USER CODE END MemoryManagement_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
        /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
    /* USER CODE BEGIN BusFault_IRQn 0 */

    /* USER CODE END BusFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_BusFault_IRQn 0 */
        /* USER CODE END W1_BusFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
    /* USER CODE BEGIN UsageFault_IRQn 0 */

    /* USER CODE END UsageFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
        /* USER CODE END W1_UsageFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
    /* USER CODE BEGIN DebugMonitor_IRQn 0 */

    /* USER CODE END DebugMonitor_IRQn 0 */
    /* USER CODE BEGIN DebugMonitor_IRQn 1 */

    /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
    /* USER CODE BEGIN SysTick_IRQn 0 */

    /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState == 1)
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
#endif /* INCLUDE_xTaskGetSchedulerState */
        xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1)
    }
#endif /* INCLUDE_xTaskGetSchedulerState */
       /* USER CODE BEGIN SysTick_IRQn 1 */

    /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles DMA1 stream0 global interrupt.
 */
void DMA1_Stream0_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

    /* USER CODE END DMA1_Stream0_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_uart4_rx);
    /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

    /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
 * @brief This function handles DMA1 stream1 global interrupt.
 */
void DMA1_Stream1_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

    /* USER CODE END DMA1_Stream1_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_adc3);
    /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

    /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
 * @brief This function handles USART3 global interrupt.
 */
void USART3_IRQHandler(void)
{
    /* USER CODE BEGIN USART3_IRQn 0 */

    /* USER CODE END USART3_IRQn 0 */
    HAL_UART_IRQHandler(&huart3);
    /* USER CODE BEGIN USART3_IRQn 1 */

    /* USER CODE END USART3_IRQn 1 */
}

/**
 * @brief This function handles UART4 global interrupt.
 */
void UART4_IRQHandler(void)
{
    /* USER CODE BEGIN UART4_IRQn 0 */
    if (RESET != __HAL_UART_GET_FLAG(
                     &huart4, UART_FLAG_IDLE))  // Judging whether it is idle interruption
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart4);  // Clear idle interrupt sign (otherwise it
                                             // will continue to enter the interrupt)
        io_ublox_odinw262_communicator_handleIdleLine();
    }

    /* USER CODE END UART4_IRQn 0 */
    HAL_UART_IRQHandler(&huart4);
    /* USER CODE BEGIN UART4_IRQn 1 */

    /* USER CODE END UART4_IRQn 1 */
}

/**
 * @brief This function handles Ethernet global interrupt.
 */
void ETH_IRQHandler(void)
{
    /* USER CODE BEGIN ETH_IRQn 0 */

    /* USER CODE END ETH_IRQn 0 */
    HAL_ETH_IRQHandler(&heth);
    /* USER CODE BEGIN ETH_IRQn 1 */

    /* USER CODE END ETH_IRQn 1 */
}

/**
 * @brief This function handles Ethernet wake-up interrupt through EXTI line 86.
 */
void ETH_WKUP_IRQHandler(void)
{
    /* USER CODE BEGIN ETH_WKUP_IRQn 0 */

    /* USER CODE END ETH_WKUP_IRQn 0 */
    HAL_ETH_IRQHandler(&heth);
    /* USER CODE BEGIN ETH_WKUP_IRQn 1 */

    /* USER CODE END ETH_WKUP_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
