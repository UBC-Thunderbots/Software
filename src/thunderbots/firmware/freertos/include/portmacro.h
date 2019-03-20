#ifndef PORTMACRO_H
#define PORTMACRO_H

#include <exception.h>
#include <limits.h>
#include <stddef.h>
#include <stdlib.h>
#include <registers/nvic.h>
#include <registers/scb.h>

// The type of words in a stack.
typedef unsigned long StackType_t;

// The general-purpose type for an ordinary integer.
typedef long BaseType_t;
typedef unsigned long UBaseType_t;

// The type of a tick counter.
typedef unsigned long TickType_t;

// The infinity value for a tick count.
#define portMAX_DELAY ULONG_MAX

// The stack grows down.
#define portSTACK_GROWTH -1

// Tick counters are atomic.
#define portTICK_TYPE_IS_ATOMIC 1

// The tick interval, in milliseconds.
#define portTICK_PERIOD_MS (1000UL / configTICK_RATE_HZ)

// Stacks will be 32-byte aligned.
#define portBYTE_ALIGNMENT 32

// The type of a task function.
#define portTASK_FUNCTION_PROTO(fn, params) void fn(void *params)
#define portTASK_FUNCTION portTASK_FUNCTION_PROTO

// Functions related to yielding tasks.
inline void portYIELD(void) {
	// Do not allow the compiler to sink any prior non-volatile-qualified
	// memory writes below the yield, as code should reasonably be able to
	// expect that an explicit yield happens in program order. Furthermore, the
	// yield might be due to a semaphore give releasing a higher-priority task
	// which must have release semantics.
	__atomic_signal_fence(__ATOMIC_RELEASE);

	// In a task, yielding is done by pending the PendSV interrupt, which will
	// be taken immediately.
	ICSR_t tmp = { .PENDSVSET = 1 };
	SCB.ICSR = tmp;

	// ARMv7-M Architecture Reference Manual A3.7.3 (Memory barriers),
	// Synchronization requirements for System Control Space updates, states
	// that a DSB followed by an ISB is necessary to ensure subsequent
	// instructions are executed in a manner that reflects the effect an access
	// to the system control space that performs a context-altering operation.
	asm volatile("dsb\n\tisb" ::: "memory");

	// Do not allow the compiler to hoist any subsequent non-volatile-qualified
	// memory reads or writes above the yield. The yield could be due to a
	// semaphore take, and neither reads not writes should be hoisted above
	// such an action.
	__atomic_signal_fence(__ATOMIC_SEQ_CST);
}
#define portYIELD_WITHIN_API portYIELD
inline void portYIELD_FROM_ISR(void) {
	// In an ISR, yielding is done by pending the PendSV interrupt, which will
	// be taken on exception return to thread mode.
	ICSR_t tmp = { .PENDSVSET = 1 };
	SCB.ICSR = tmp;

	// ARMv7-M Architecture Reference Manual A3.7.3 (Memory barriers),
	// Synchronization requirements for System Control Space updates, states
	// that a DSB followed by an ISB is necessary to ensure subsequent
	// instructions are executed in a manner that reflects the effect an access
	// to the system control space that performs a context-altering operation.
	// We do not care when, exactly, the interrupt is pended between now and
	// the exception return from the ISR, so we can omit the ISB and rely on
	// the InstructionSynchronizationBarrier executed as part of the exception
	// return procedure. However, exception return does not imply a
	// DataSynchronizationBarrier, so the explicit DSB is still needed.
	//
	// That having been said, every ISR must have a DSB immediately before the
	// exception return due to Cortex-M4 erratum 838869. Thus, we can assume
	// the presence of a DSB before the exception return, and need not insert
	// one here.
	//
	// There is no need for a compiler barrier here because this is an ISR. Any
	// memory reads or writes will definitely be complete by the time the ISR
	// returns (and hence before any newly unblocked task starts running),
	// while any preempting ISR cannot assume anything about the state of the
	// system anyway.
}

// These functions enable and disable interrupts from ISR or non-ISR code.
inline void portENABLE_INTERRUPTS(void) {
	// Do not allow the compiler to sink non-volatile-qualified memory accesses
	// below this point.
	__atomic_signal_fence(__ATOMIC_RELEASE);

	// Enable interrupts.
	asm volatile("msr basepri, %[zero]" :: [zero] "r" (0UL));

	// No barrier is needed here as it is OK for interrupts not to be enabled
	// immediately.
}
inline void portDISABLE_INTERRUPTS(void) {
	// Disable interrupts.
	asm volatile("msr basepri, %[newpri]" :: [newpri] "r" (configMAX_SYSCALL_INTERRUPT_PRIORITY));

	// ARMv7-M Architecture Reference Manual B5.2.3 (MSR instruction),
	// Visibility of changes, states that an MSR that increases execution
	// priority serializes the change to the instruction stream, so no ISB is
	// needed here.

	// Do not permit the compiler to hoist non-volatile-qualified memory
	// accesses above this point.
	__atomic_signal_fence(__ATOMIC_ACQUIRE);
}

// These functions are basically nestable variants of the functions above, which just maintain a counter and call those functions.
void portENTER_CRITICAL(void);
void portEXIT_CRITICAL(void);
void portRELAX_CRITICAL_ISR_CHECK(void);

// These functions enable and disable interrupts from an ISR.
// We do not need to save a prior value of BASEPRI.
// Exception entry does not modify BASEPRI, so it will normally be zero.
// Actual execution priority is determined from the lesser of the values implied by BASEPRI, PRIMASK, FAULTMASK, and the active exception set.
// So, ISR entry just modifies the active exception set and that modifies the execution priority of the CPU and prevents inappropriate preemption.
// What this means for us is that we can disable unwanted interrupts by setting BASEPRI to an appropriate value, then re-enable them by setting it to zero.
// We won’t reenter ourself, because the active exception set takes care of that (and is, in fact, the usual mechanism for doing so).
// Therefore no old mask value needs to be saved at all!
inline unsigned long portSET_INTERRUPT_MASK_FROM_ISR(void) {
	portDISABLE_INTERRUPTS();
	return 0UL;
}
inline void portCLEAR_INTERRUPT_MASK_FROM_ISR(unsigned long old __attribute__((unused))) {
	portENABLE_INTERRUPTS();
}
#define portSET_INTERRUPT_MASK_FROM_ISR portSET_INTERRUPT_MASK_FROM_ISR
#define portCLEAR_INTERRUPT_MASK_FROM_ISR portCLEAR_INTERRUPT_MASK_FROM_ISR

// These functions turn a specific hardware interrupt on and off.
inline void portENABLE_HW_INTERRUPT(unsigned int irq) {
	// Do not allow the compiler to sink non-volatile-qualified memory accesses
	// below this point.
	__atomic_signal_fence(__ATOMIC_RELEASE);

	// Enable the interrupt.
	NVIC.ISER[irq / 32U] = 1U << (irq % 32U);

	// No barrier is needed here as it is OK for the interrupt not to be
	// enabled immediately.
}
inline void portDISABLE_HW_INTERRUPT(unsigned int irq) {
	// Disable the interrupt.
	NVIC.ICER[irq / 32U] = 1U << (irq % 32U);

	// ARMv7-M Architecture Reference Manual A3.7.3 (Memory barriers),
	// Synchronization requirements for System Control Space updates, states
	// that a DSB followed by an ISB is necessary to ensure subsequent
	// instructions are executed in a manner that reflects the effect an access
	// to the system control space that performs a context-altering operation.
	asm volatile("dsb\n\tisb");

	// Do not allow the compiler to hoist non-volatile-qualified memory
	// accesses above this point.
	__atomic_signal_fence(__ATOMIC_ACQUIRE);
}

// Interrupt handlers which the application must wire into the vector table.
void vPortSVCHandler(void);
void vPortPendSVHandler(void);
void vPortSysTickHandler(void);

// Initializes the stack guard based on the bottom of the task stack.
#define portSETUP_TCB(tcb) \
do { \
	*tcb->pxTopOfStack = (unsigned long) tcb->pxStack; \
} while (0)

// Functionality used by tasks.c to optimize task selection if requested.
// If not otherwise specified, we will enable optimized task selection.
#ifndef configUSE_PORT_OPTIMISED_TASK_SELECTION
#define configUSE_PORT_OPTIMIZED_TASK_SELECTION 1
#endif
#if configUSE_PORT_OPTIMISED_TASK_SELECTION == 1
#if configMAX_PRIORITIES > 32
#error Cannot use optimized task selection with more than 32 priorities!
#endif
#define portRECORD_READY_PRIORITY(priority, ready_priorities) ((ready_priorities) |= (1UL << (priority)))
#define portRESET_READY_PRIORITY(priority, ready_priorities) ((ready_priorities) &= ~(1UL << (priority)))
#define portGET_HIGHEST_PRIORITY(top_priority, ready_priorities) ((top_priority) = (31U - __builtin_clz((ready_priorities))))
#endif

#if configASSERT_DEFINED == 1
// Checks if the CPU is currently running at a legal interrupt level to call FreeRTOS functions.
void vPortAssertIfInterruptPriorityInvalid(void);
#define portASSERT_IF_INTERRUPT_PRIORITY_INVALID vPortAssertIfInterruptPriorityInvalid
#endif

#endif
