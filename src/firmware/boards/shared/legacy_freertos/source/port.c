// Stack frame format, FPU not in use:
//
// Hardware:
// xPSR
// Return address
// LR
// R12
// R3
// R2
// R1
// R0
//
// Software:
// R11
// R10
// R9 ← Here and below may be covered by stack guard.
// R8
// R7
// R6
// R5
// R4
// Exception return code
// Stack guard RBAR
//
//
//
// Stack frame format, FPU in use:
//
// Hardware:
// FPSCR
// S15
// S14
// S13
// S12
// S11
// S10
// S9
// S8
// S7
// S6
// S5
// S4
// S3
// S2
// S1
// S0
// xPSR
// Return address
// LR
// R12
// R3
// R2
// R1
// R0
//
// Software:
// S31
// S30
// S29
// S28
// S27
// S26
// S25
// S24
// S23
// S22
// S21
// S20
// S19
// S18
// S17
// S16
// R11
// R10
// R9 ← Here and below may be covered by stack guard.
// R8
// R7
// R6
// R5
// R4
// Exception return code
// Stack guard RBAR

#include <assert.h>
#include <FreeRTOS.h>
#include <exception.h>
#include <inttypes.h>
#include <semphr.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <task.h>
#include <registers/mpu.h>
#include <registers/nvic.h>
#include <registers/systick.h>

// Layout of the CCM itself.
#define CCM_BASE 0x10000000U
#define CCM_SIZE 65536U

// Nesting level of critical sections.
static unsigned int critical_section_nesting = 0xAAAAAAAAU;

// Current task control block pointer.
extern unsigned long *pxCurrentTCB;

// Whether to allow portENTER_CRITICAL from an ISR.
static bool relax_critical_isr_check = false;



// MPU regions shared by all tasks.
static const struct {
	uint32_t address;
	MPU_RASR_t rasr;
} COMMON_MPU_REGIONS[] = {
	// 0x08000000–0x080FFFFF (length 1 MiB): Flash memory (normal, read-only, write-through cache, executable)
	{ 0x08000000, { .XN = 0, .AP = 0b111, .TEX = 0b000, .S = 0, .C = 1, .B = 0, .SRD = 0, .SIZE = 19, .ENABLE = 1 } },

	// 0x10000000–0x1000FFFF (length 64 kiB): CCM (stack) (normal, read-write, write-back write-allocate cache, not executable)
	{ 0x10000000, { .XN = 1, .AP = 0b011, .TEX = 0b001, .S = 0, .C = 1, .B = 1, .SRD = 0, .SIZE = 15, .ENABLE = 1 } },

	// 0x1FFF0000–0x1FFF7FFF (length 32 kiB): System memory including U_ID and F_SIZE (normal, read-only, write-through cache, not executable)
	{ 0x1FFF0000, { .XN = 1, .AP = 0b111, .TEX = 0b000, .S = 0, .C = 1, .B = 0, .SRD = 0, .SIZE = 14, .ENABLE = 1 } },

	// 0x20000000–0x2001FFFF (length 128 kiB): SRAM (normal, read-write, write-back write-allocate cache, not executable)
	{ 0x20000000, { .XN = 1, .AP = 0b011, .TEX = 0b001, .S = 1, .C = 1, .B = 1, .SRD = 0, .SIZE = 16, .ENABLE = 1 } },

	// 0x40000000–0x4007FFFF (length 512 kiB): Peripherals (device, read-write, not executable) using subregions:
	// Subregion 0 (0x40000000–0x4000FFFF): Enabled (contains APB1)
	// Subregion 1 (0x40010000–0x4001FFFF): Enabled (contains APB2)
	// Subregion 2 (0x40020000–0x4002FFFF): Enabled (contains AHB1)
	// Subregion 3 (0x40030000–0x4003FFFF): Disabled
	// Subregion 4 (0x40040000–0x4004FFFF): Disabled
	// Subregion 5 (0x40050000–0x4005FFFF): Disabled
	// Subregion 6 (0x40060000–0x4006FFFF): Disabled
	// Subregion 7 (0x40070000–0x4007FFFF): Disabled
	{ 0x40000000, { .XN = 1, .AP = 0b011, .TEX = 0b000, .S = 0, .C = 0, .B = 1, .SRD = 0b11111000, .SIZE = 18, .ENABLE = 1 } },

	// 0x50000000–0x5007FFFF (length 512 kiB): Peripherals (device, read-write, not executable) using subregions:
	// Subregion 0 (0x50000000–0x5000FFFF): Enabled (contains AHB2)
	// Subregion 1 (0x50010000–0x5001FFFF): Enabled (contains AHB2)
	// Subregion 2 (0x50020000–0x5002FFFF): Enabled (contains AHB2)
	// Subregion 3 (0x50030000–0x5003FFFF): Enabled (contains AHB2)
	// Subregion 4 (0x50040000–0x5004FFFF): Enabled (contains AHB2)
	// Subregion 5 (0x50050000–0x5005FFFF): Enabled (contains AHB2)
	// Subregion 6 (0x50060000–0x5006FFFF): Enabled (contains AHB2)
	// Subregion 7 (0x50070000–0x5007FFFF): Disabled
	{ 0x50000000, { .XN = 1, .AP = 0b011, .TEX = 0b000, .S = 0, .C = 0, .B = 1, .SRD = 0b10000000, .SIZE = 18, .ENABLE = 1 } },
};

#define STACK_GUARD_MPU_REGION (sizeof(COMMON_MPU_REGIONS) / sizeof(*COMMON_MPU_REGIONS))

static const MPU_RASR_t ZERO_RASR = { .ENABLE = 0 };


unsigned long *pxPortInitialiseStack(unsigned long *tos, TaskFunction_t code, void *params) {
	// xTaskGenericCreate subtracts one word from TOS, then rounds down to
	// alignment. In our case, this means it subtracts 32 bytes. ARM CPUs in
	// particular have predecrement stack pointers, so that’s pointless. Fix
	// it, to avoid wasting space!
	tos += portBYTE_ALIGNMENT / sizeof(unsigned long);

	// Push a hardware frame.
	*--tos = 0x01000000UL; // xPSR
	*--tos = (unsigned long) code; // Return address
	*--tos = 0; // LR
	*--tos = 0; // R12
	*--tos = 0; // R3
	*--tos = 0; // R2
	*--tos = 0; // R1
	*--tos = (unsigned long) params; // R0

	// Push a software frame.
	*--tos = 0; // R11
	*--tos = 0; // R10
	*--tos = 0; // R9
	*--tos = 0; // R8
	*--tos = 0; // R7
	*--tos = 0; // R6
	*--tos = 0; // R5
	*--tos = 0; // R4
	*--tos = 0xFFFFFFFDUL; // Exception return code
	*--tos = 0; // Stack guard RBAR (filled in portSETUP_TCB)

	return tos;
}



void __malloc_lock(void) {
	vTaskSuspendAll();
}

void __malloc_unlock(void) {
	xTaskResumeAll();
}



long xPortStartScheduler(void) {
	// Set SVCall, PendSV, and systick priorities.
	// SVCall is used to start the scheduler, but is invoked following portDISABLE_INTERRUPTS in vTaskStartScheduler.
	// In ARM, even synchronous interrupts obey the masking registers; in order not to be escalated to hard fault, SVCall must be of an unmasked priority.
	// We never use it for anythign else, so just give it priority above maximum syscall interrupt priority.
	// The others, PendSV and systick, are the normal kernel workhorse interrupts.
	SCB.SHPR2.PRI_11 = EXCEPTION_MKPRIO(EXCEPTION_GROUP_PRIO(configMAX_SYSCALL_INTERRUPT_PRIORITY) - 1U, (1U << EXCEPTION_SUB_PRIO_BITS) - 1U);
	SCB.SHPR3.PRI_15 = configKERNEL_INTERRUPT_PRIORITY;
	SCB.SHPR3.PRI_14 = configKERNEL_INTERRUPT_PRIORITY;

	// ARMv7-M Architecture Reference Manual A3.7.3 (Memory barriers),
	// Synchronization requirements for System Control Space updates, states
	// that a DSB followed by an ISB is necessary to ensure subsequent
	// instructions are executed in a manner that reflects the effect an access
	// to the system control space that performs a context-altering operation.
	asm volatile("dsb\n\tisb");

	// Ensure the compiler cannot sink any non-volatile-qualified memory writes
	// below this point, as control flow leaves the function in a way that is
	// not visible to the compiler and might thus defeat the writes altogether.
	__atomic_signal_fence(__ATOMIC_RELEASE);

	// We are currently running in process mode on the main stack.
	// What we want to be doing is running a task in process mode on the task stack, with the main stack pointer reset to its initial value ready to handle interrupts.
	// What pxPortInitializeStack builds on the process stack is an exception return frame, as would be built by a PendSV.
	// The easiest way to take advantage of that stack structure to get the first task running is… to perform an exception return!
	// We can’t use PendSV, as that expects to be able to save the current context somewhere first, before loading the new context.
	// It also won’t fix up MSP to clean up the main stack.
	// Use SVCall instead, and let the SVCall handler get everything ready, clean up MSP, and then do an exception return into the first task.
	asm volatile("svc 0");
	__builtin_unreachable();
}



void portENTER_CRITICAL(void) {
	portDISABLE_INTERRUPTS();
	++critical_section_nesting;
	
	/* This is not the interrupt safe version of the enter critical function so
	assert() if it is being called from an interrupt context.  Only API 
	functions that end in "FromISR" can be used in an interrupt.  Only assert if
	the critical nesting count is 1 to protect against recursive calls if the
	assert function also uses a critical section. */
	if (critical_section_nesting == 1U && !relax_critical_isr_check) {
		unsigned int xpsr;
		asm volatile("mrs %[xpsr], xpsr\n\t" : [xpsr] "=r" (xpsr));
		configASSERT(!(xpsr & 0xFFU));
	}
}

void portEXIT_CRITICAL(void) {
	if (!--critical_section_nesting) {
		portENABLE_INTERRUPTS();
	}
}

void portRELAX_CRITICAL_ISR_CHECK(void) {
	relax_critical_isr_check = true;
}


void vPortEndScheduler( void ) {
  // So. This should really have been defined as a function under
  // `freertos/vendor/portable`. However, some _genius_ decided that it was OK
  // to write their *OWN* implementation of `portable.h` and `port.c`. We have
  // no idea why they did this or how it works. When we tried to build this
  // with bazel it complained about this function not being defined. We tried
  // to define it in the header, but that complained about multiple
  // implementations of this function existing. Why? WHO KNOWS. WE SURE AS ALL
  // HELL DON'T. If you're reading this in the hopes of some magical
  // enlightenment, you're SOL, may god have mercy on your soul.
  //
  // - Gareth and Mathew, 2019
}

void vPortSVCHandler(void) {
	// These variables must be held in exactly these registers, as they are used below in the inline assembly block.
	register unsigned long tos asm("r0");
	register unsigned long init_stack asm("r1");

	// Initialize critical section nesting count.
	critical_section_nesting = 0U;

	// Enable system timer.
	SYSTICK.RVR = configCPU_CLOCK_HZ / configTICK_RATE_HZ - 1U;
	SYSTICK.CVR = 0;
	{
		SYST_CSR_t tmp = { .ENABLE = 1, .TICKINT = 1, .CLKSOURCE = 1 };
		SYSTICK.CSR = tmp;
	}

	// Enable FPU and set up automatic lazy state preservation.
	//
	// Here’s how it will go down:
	//
	// In a task not using the FPU, CONTROL.FPCA will be zero, and on interrupt entry, a basic frame will be stacked.
	// If the ISR uses the FPU, CONTROL.FPCA will be set during that usage, which will also prepare a default context from FPDSCR for the ISR.
	// When the ISR returns, the exception return code will cause CONTROL.FPCA to go back to zero for the task as expected.
	// If the ISR doesn’t use the FPU, CONTROL.FPCA will remain zero throughout, and the exception return will work the same way.
	//
	// In a task using the FPU, on first access, CONTROL.FPCA will be set and a default context will be prepared from FPDSCR.
	// From that moment onward, because CONTROL.FPCA=1 and FPCCR.ASPEN=1, interrupts in that task will always stack extended frames.
	// However, because LSPEN=1, the extended frames will not be immediately populated; rather, on interrupt entry, LSPACT will be set.
	// If the ISR doesn’t use the FPU, LSPACT will remain set throughout.
	// When the ISR returns, the exception return code will indicate an extended frame, but LSPACT=1 will elide restoration of registers.
	// If the ISR does use the FPU, then on first access, the frame will be populated and LSPACT will be cleared, allowing activity to proceed.
	// In that case, on exception return, LSPACT=0 will trigger restoration of registers.
	//
	// Thus, time is spent saving and restoring the FP registers only in the case that both the task *and* the ISR actually use them.
	//
	// This explains ISRs, but what happens during a task switch?
	//
	// When a task not using the FPU takes a PendSV, CONTROL.FPCA will be zero, and on interrupt entry, a basic hardware frame will be stacked.
	// The PendSV ISR will observe that the link register indicates a basic frame, and will not touch the FP registers, stacking only a basic software frame.
	//
	// When a task not using the FPU is being resumed, the PendSV ISR will first load the link register from the basic software frame.
	// Observing that the link register indicates a basic hardware frame, it will conclude that the software frame is also basic and will not touch the FP registers.
	// The exception return will unstack the basic hardware frame and leave CONTROL.FPCA=0.
	//
	// When a task using the FPU takes a PendSV, CONTROL.FPCA will be one, and on interrupt entry, an extended hardware frame will be stacked (though not populated).
	// Because the extended hardware frame is not populated, LSPACT will be set.
	// The PendSV ISR will observe that the link register indicates an extended frame, and will stack the callee-saved FP registers into an extended software frame.
	// The VSTM instruction used to do this is itself an FP instruction, which thus causes the extended hardware frame to be populated and LSPACT to be cleared.
	// Thus, by the time the stack switch occurs, all FP registers will have been placed on the stack (half in the hardware frame and half in the software frame), and LSPACT=0.
	//
	// When a task using the FPU is being resumed, the PendSV ISR will first load the link register from the basic software frame at the beginning of the extended software frame.
	// The PendSV ISR will observe that the link register indicates an extended frame, and will reload the callee-saved FP registers from the extended software frame.
	// The subsequent exception return will unstack the extended hardware frame (restoring the remaining FP registers, since LSPACT=0) and leave CONTROL.FPCA=1.
	{
		FPCCR_t fpccr = { .LSPEN = 1, .ASPEN = 1 };
		FP.CCR = fpccr;
		CPACR_t cpacr = CPACR;
		cpacr.CP10 = cpacr.CP11 = 3;
		CPACR = cpacr;
	}

	// Configure common MPU regions.
	for (size_t i = 0; i < sizeof(COMMON_MPU_REGIONS) / sizeof(*COMMON_MPU_REGIONS); ++i) {
		MPU_RNR_t rnr = { .REGION = i };
		MPU.RNR = rnr;
		MPU_RBAR_t rbar = { .REGION = 0, .VALID = 0, .ADDR = COMMON_MPU_REGIONS[i].address >> 5 };
		MPU.RBAR = rbar;
		MPU.RASR = COMMON_MPU_REGIONS[i].rasr;
	}

	// Leave MPU_RNR set to the CCM stack guard region number. Set up the RASR.
	// Point RBAR at zero for now, until we can load a proper value from a task
	// stack.
	{
		MPU_RNR_t rnr = { .REGION = STACK_GUARD_MPU_REGION };
		MPU.RNR = rnr;
		MPU_RBAR_t rbar = { .ADDR = 0, .VALID = 0, .REGION = 0 };
		MPU.RBAR = rbar;
		MPU_RASR_t rasr = { .XN = 1, .AP = 0, .SRD = 0, .SIZE = 4, .ENABLE = 1 };
		MPU.RASR = rasr;
	}

	// Enable the MPU.
	{
		MPU_CTRL_t ctrl = {
			.PRIVDEFENA = 0,
			.HFNMIENA = 0,
			.ENABLE = 1,
		};
		MPU.CTRL = ctrl;
	}

	// Ensure the compiler cannot sink any non-volatile-qualified memory writes
	// below this point, as control flow leaves the function in a way that is
	// not visible to the compiler and might thus defeat the writes altogether.
	__atomic_signal_fence(__ATOMIC_RELEASE);

	// No need to handle an extended software frame here.
	// Because the scheduler is just starting, no tasks have run yet.
	// Every task is always created, initially, with only basic frames.
	// We must also reset the MSP to the start of the main stack (see explanation in xPortStartScheduler).
	tos = *pxCurrentTCB;
	init_stack = *(const unsigned long *) 0x08000000;
	asm volatile(
			// Fix up MSP.
			"msr msp, r1\n\t"
			// ARMv7-M Architecture Reference Manual B1.4.6 (Special-purpose
			// register updates and the memory order model) states that “Except
			// for writes to the CONTROL register, any change to a
			// special-purpose register by a … MSR instruction is guaranteed …
			// to be visible to all instructions that appear in program order
			// after that CPS or MSR instruction.” Consequently, the write to
			// PSP is guaranteed to be visible by the time the exception return
			// procedure begins.
			// Restore software frame from process stack.
			"ldr r1, =0xE000ED9C\n\t" // R1 = &MPU_RBAR
			"ldmia r0!, {r2, lr}\n\t"
			"ldmia r0!, {r4-r11}\n\t"
			"str r2, [r1]\n\t"
			// Cortex-M4 Devices Generic User Guide 2.2.4 (Software ordering of
			// memory accesses), MPU programming, states that one must “use a
			// DSB followed by an ISB instruction or exception return to ensure
			// that the new MPU configuration is used by subsequent
			// instructions.” We do not care whether the new RBAR value is in
			// force until after we return to thread mode via the exception
			// return, so we can omit the ISB and rely on the
			// InstructionSynchronizationBarrier executed as part of the
			// exception return procedure. However, exception return does not
			// imply a DataSynchronizationBarrier, so the explicit DSB is still
			// needed.
			//
			// Cortex-M4 erratum 838869 also requires a DSB here.
			"dsb\n\t"
			// Set PSP and return.
			"msr psp, r0\n\t"
			// ARMv7-M Architecture Reference Manual B1.4.6 (Special-purpose
			// register updates and the memory order model) states that “Except
			// for writes to the CONTROL register, any change to a
			// special-purpose register by a … MSR instruction is guaranteed …
			// to be visible to all instructions that appear in program order
			// after that CPS or MSR instruction.” Consequently, the write to
			// PSP is guaranteed to be visible by the time the exception return
			// procedure begins.
			"bx lr\n\t"
			:
			// Note that these variables do not need placeholders in the text; they are fixed in r0 and r1 respectively by their asm constraints above.
			: "r" (tos), "r" (init_stack));
	__builtin_unreachable();
}

void vPortPendSVHandler(void) __attribute__((naked));
void vPortPendSVHandler(void) {
	asm volatile(
			// Make software frame on process stack.
			// See the explanation in vPortSVCHandlerImpl for how we deal with floating point registers.
			"mrs r0, psp\n\t"
			"ldr r1, =0xE000ED9C\n\t" // R1 = &MPU_RBAR
			"tst lr, #16\n\t"
			"it eq\n\t"
			"vstmdbeq r0!, {s16-s31}\n\t"
			"stmdb r0!, {r10-r11}\n\t"
			"ldr r2, [r1]\n\t" // Stack guard RBAR → R2
			"mov r3, #0\n\t"
			"str r3, [r1]\n\t" // Disable stack guard
			// Cortex-M4 Devices Generic User Guide 2.2.4 (Software ordering of
			// memory accesses), MPU programming, states that one must “use a
			// DSB followed by an ISB instruction or exception return to ensure
			// that the new MPU configuration is used by subsequent
			// instructions.”
			"dsb\n\t"
			"isb\n\t"
			"stmdb r0!, {r4-r9}\n\t"
			"stmdb r0!, {r2, lr}\n\t"
			// Write new top of stack pointer into TCB.
			"ldr r1, =pxCurrentTCB\n\t"
			"ldr r1, [r1]\n\t"
			"str r0, [r1]\n\t"
			// Disable interrupts.
			"mov r0, %[newbasepri]\n\t"
			"msr basepri, r0\n\t"
			// ARMv7-M Architecture Reference Manual B5.2.3 (MSR instruction),
			// Visibility of changes, states that an MSR that increases
			// execution priority serializes the change to the instruction
			// stream, so no ISB is needed here.
			// Ask scheduler to pick a new task.
			"bl vTaskSwitchContext\n\t"
			// Enable interrupts.
			"mov r0, #0\n\t"
			"msr basepri, r0\n\t"
			// ARMv7-M Architecture Reference Manual B5.2.3 (MSR instruction),
			// Visibility of changes, states that an MSR that decreases
			// execution priority may only result in the new priority being
			// visible after an ISB or exception return. We will be doing an
			// exception return soon, and we don’t care if the new priority is
			// visible before that or not, so no ISB is needed here.
			// Read the new top of stack pointer from TCB.
			"ldr r0, =pxCurrentTCB\n\t"
			"ldr r1, =0xE000ED9C\n\t" // R1 = &MPU_RBAR
			"ldr r0, [r0]\n\t"
			"ldr r0, [r0]\n\t"
			// Restore software frame from process stack.
			"ldmia r0!, {r2, lr}\n\t" // R2 = Stack guard RBAR
			"ldmia r0!, {r4-r11}\n\t"
			"tst lr, #16\n\t"
			"it eq\n\t"
			"vldmiaeq r0!, {s16-s31}\n\t"
			// Enable stack guard.
			"str r2, [r1]\n\t"
			// Cortex-M4 Devices Generic User Guide 2.2.4 (Software ordering of
			// memory accesses), MPU programming, states that one must “use a
			// DSB followed by an ISB instruction or exception return to ensure
			// that the new MPU configuration is used by subsequent
			// instructions.” We do not care whether the new RBAR value is in
			// force until after we return to thread mode via the exception
			// return, so we can omit the ISB and rely on the
			// InstructionSynchronizationBarrier executed as part of the
			// exception return procedure. However, exception return does not
			// imply a DataSynchronizationBarrier, so the explicit DSB is still
			// needed.
			//
			// Cortex-M4 erratum 838869 also requires a DSB here.
			"dsb\n\t"
			// Set PSP and return.
			"msr psp, r0\n\t"
			// ARMv7-M Architecture Reference Manual B1.4.6 (Special-purpose
			// register updates and the memory order model) states that “Except
			// for writes to the CONTROL register, any change to a
			// special-purpose register by a … MSR instruction is guaranteed …
			// to be visible to all instructions that appear in program order
			// after that CPS or MSR instruction.” Consequently, the write to
			// PSP is guaranteed to be visible by the time the exception return
			// procedure begins.
			"bx lr\n\t"
			:
			: [pxCurrentTCB] "i" (&pxCurrentTCB), [newbasepri] "i" (configMAX_SYSCALL_INTERRUPT_PRIORITY));
}

void vPortSysTickHandler(void) {
	portSET_INTERRUPT_MASK_FROM_ISR();
	if(xTaskIncrementTick()) {
		portYIELD_FROM_ISR();
	}
	portCLEAR_INTERRUPT_MASK_FROM_ISR(0U);
	EXCEPTION_RETURN_BARRIER();
}

#if configASSERT_DEFINED == 1
void vPortAssertIfInterruptPriorityInvalid(void) {
	unsigned long current_interrupt;
	asm("mrs %[current_interrupt], ipsr" : [current_interrupt] "=r" (current_interrupt));
	if (current_interrupt >= 16U) {
		// This is a normal hardware interrupt, not a fault or special CPU interrupt.
		// Check the priority.
		current_interrupt -= 16U;
		configASSERT((NVIC.IPR[current_interrupt / 4U] >> (current_interrupt % 4U)) >= configMAX_SYSCALL_INTERRUPT_PRIORITY);
	}
}
#endif



extern inline void portYIELD(void);
extern inline void portYIELD_FROM_ISR(void);
extern inline void portENABLE_INTERRUPTS(void);
extern inline void portDISABLE_INTERRUPTS(void);
extern inline unsigned long portSET_INTERRUPT_MASK_FROM_ISR(void);
extern inline void portCLEAR_INTERRUPT_MASK_FROM_ISR(unsigned long old);
extern inline void portENABLE_HW_INTERRUPT(unsigned int irq);
extern inline void portDISABLE_HW_INTERRUPT(unsigned int irq);
