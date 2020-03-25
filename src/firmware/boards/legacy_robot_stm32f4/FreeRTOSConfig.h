#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "priority.h"

#define configUSE_PREEMPTION 1
#define configUSE_IDLE_HOOK 1
#define configUSE_TICK_HOOK 0
#define configCPU_CLOCK_HZ 168000000
#define configTICK_RATE_HZ 200
#define configMAX_PRIORITIES (PRIO_TASK_MAX + 1U)
#define configMINIMAL_STACK_SIZE ((unsigned short)136)
#define configMAX_TASK_NAME_LEN (10)
#define configUSE_TRACE_FACILITY 1
#define configUSE_16_BIT_TICKS 0
#define configIDLE_SHOULD_YIELD 1
#define configUSE_MUTEXES 1
#define configQUEUE_REGISTRY_SIZE 0
#define configCHECK_FOR_STACK_OVERFLOW 0
#define configUSE_RECURSIVE_MUTEXES 1
#define configUSE_MALLOC_FAILED_HOOK 1
#define configUSE_APPLICATION_TASK_TAG 0
#define configUSE_COUNTING_SEMAPHORES 1
#define configGENERATE_RUN_TIME_STATS 0
#define configENABLE_BACKWARD_COMPATIBILITY 0
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configSUPPORT_DYNAMIC_ALLOCATION 0
#define configSUPPORT_STATIC_ALLOCATION 1

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 0
#define configMAX_CO_ROUTINE_PRIORITIES (2)

/* Software timer definitions. */
#define configUSE_TIMERS 0

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet 1
#define INCLUDE_uxTaskPriorityGet 1
#define INCLUDE_vTaskDelete 0
#define INCLUDE_vTaskCleanUpResources 1
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_vTaskDelayUntil 1
#define INCLUDE_vTaskDelay 1
#define INCLUDE_xTaskAbortDelay 0
#define INCLUDE_xTimerPendFunctionCall 0

#define configKERNEL_INTERRUPT_PRIORITY EXCEPTION_MKPRIO(7U, 1U)
#define configMAX_SYSCALL_INTERRUPT_PRIORITY EXCEPTION_MKPRIO(2U, 0U)

#include <assert.h>
#define configASSERT assert

#endif /* FREERTOS_CONFIG_H */
