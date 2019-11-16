/**
 * \defgroup EXC Exception handling functions
 *
 * \brief These functions handle crashes.
 *
 * When running code crashes, four things happen:
 * \li An application-provided early callback is invoked, if provided; this can be used to
 * ensure the system is safe. \li A core dump is generated and handed off to a core dump
 * writer, if provided. \li An application-provided late callback is invoked, if provided;
 * this can be used to display something to the user announcing the crash. \li If the late
 * callback returns, the system is locked up forever.
 *
 * @{
 */
#include <exception.h>
#include <nvic.h>
#include <sleep.h>
#include <string.h>
#if STM32LIB_USE_FREERTOS
#include <FreeRTOS.h>
#include <task.h>
#endif
#include <registers/mpu.h>
#include <registers/scb.h>

/**
 * \cond INTERNAL
 */
#if STM32LIB_USE_FREERTOS && configUSE_TRACE_FACILITY
#define EXCEPTION_MAX_THREADS 32
#else
#define EXCEPTION_MAX_THREADS 1
#endif
static const exception_core_writer_t *core_writer = 0;
static const exception_app_cbs_t *app_cbs         = 0;
/**
 * \endcond
 */

/**
 * \brief Initializes the exception handling subsystem.
 *
 * \param[in] cw the core dump writer module, or null to not write core dumps
 * \param[in] acbs the application-specific callbacks invoked when a crash
 * occurs, or null to omit
 * \param[in] prios the hardware interrupt priorities, indexed by NVIC
 * interrupt number, one byte per interrupt
 * \param[in] prioCount the number of bytes in \p prios
 */
void exception_init(const exception_core_writer_t *cw, const exception_app_cbs_t *acbs,
                    const uint8_t *prios)
{
    // Set the interrupt system to set priorities as having the upper three bits for group
    // priorities and the rest as subpriorities.
    {
        AIRCR_t tmp  = SCB.AIRCR;
        tmp.VECTKEY  = 0x05FA;
        tmp.PRIGROUP = 7U - EXCEPTION_GROUP_PRIO_BITS;
        SCB.AIRCR    = tmp;
    }

    // CPU exceptions (UsageFault, BusFault, MemManage, DebugMonitor) will be
    // priority 0.0 and thus preempt everything else. Hardware interrupts will
    // have the priorities provided in the priority table.
    for (size_t i = 0; i < NVIC_IRQ_COUNT; i += 4)
    {
        uint32_t u32 = 0;
        memcpy(&u32, prios + i, 4);
        NVIC.IPR[i / 4] = u32;
    }

    // FreeRTOS sets the PendSV and SysTick exceptions’ priorities itself, so
    // there is no need to do so here. Non-FreeRTOS firmware doesn’t use these
    // exceptions.

    // Enable Usage, Bus, and MemManage faults to be taken as such rather than
    // escalating to HardFaults.
    {
        SHCSR_t tmp     = SCB.SHCSR;
        tmp.USGFAULTENA = 1;
        tmp.BUSFAULTENA = 1;
        tmp.MEMFAULTENA = 1;
        SCB.SHCSR       = tmp;
    }

    // Enable DebugMonitor faults to be taken as such rather than escalating to
    // HardFaults.
    DEBUG.DEMCR.MON_EN = 1;

    // Enable trap on divide by zero.
    SCB.CCR.DIV_0_TRP = 1;

    // Register the core dump writer and application callbacks.
    core_writer = cw;
    app_cbs     = acbs;
}

/**
 * \cond INTERNAL
 */
typedef struct __attribute__((packed))
{
    char ei_mag[4];
    uint8_t ei_class;
    uint8_t ei_data;
    uint8_t ei_version;
    uint8_t ei_osabi;
    uint8_t ei_abiversion;
    uint8_t ei_pad[7];
    uint16_t e_type;
    uint16_t e_machine;
    uint32_t e_version;
    uint32_t e_entry;
    uint32_t e_phoff;
    uint32_t e_shoff;
    uint32_t e_flags;
    uint16_t e_ehsize;
    uint16_t e_phentsize;
    uint16_t e_phnum;
    uint16_t e_shentsize;
    uint16_t e_shnum;
    uint16_t e_shstrndx;
} elf_file_header_t;

typedef struct __attribute__((packed))
{
    uint32_t p_type;
    uint32_t p_offset;
    uint32_t p_vaddr;
    uint32_t p_paddr;
    uint32_t p_filesz;
    uint32_t p_memsz;
    uint32_t p_flags;
    uint32_t p_align;
} elf_program_header_t;

typedef struct __attribute__((packed))
{
    uint32_t n_namesz;
    uint32_t n_descsz;
    uint32_t n_type;
} elf_note_header_t;

typedef struct __attribute__((packed))
{
    uint32_t si_signo;
    uint32_t si_code;
    uint32_t si_errno;
    uint32_t pr_cursig;
    uint32_t pr_sigpend;
    uint32_t pr_sighold;
    uint32_t pr_pid;
    uint32_t pr_ppid;
    uint32_t pr_pgrp;
    uint32_t pr_sid;
    uint64_t pr_utime;
    uint64_t pr_stime;
    uint64_t pr_cutime;
    uint64_t pr_cstime;
    uint32_t pr_gpregs[13];
    uint32_t pr_reg_sp;
    uint32_t pr_reg_lr;
    uint32_t pr_reg_pc;
    uint32_t pr_reg_xpsr;
    uint32_t pr_reg_orig_r0;
    uint32_t pr_fpvalid;
} elf_note_prstatus_t;

typedef struct __attribute__((packed))
{
    uint32_t si_signum;
    uint32_t si_errno;
    uint32_t si_code;
    uint32_t si_addr;
    uint32_t padding[28];
} elf_note_pr_siginfo_t;

typedef struct __attribute__((packed))
{
    uint32_t sregs[32];
    uint32_t padding[32];
    uint32_t fpscr;
} elf_note_arm_vfp_t;

typedef struct __attribute__((packed))
{
    elf_file_header_t file_header;
    elf_program_header_t note_pheader;
    elf_program_header_t ccm_pheader;
    elf_program_header_t ram_pheader;
} elf_headers_t;

typedef struct __attribute__((packed))
{
    elf_note_header_t prstatus_nheader;
    char prstatus_name[8];
    elf_note_prstatus_t prstatus;

    elf_note_header_t arm_vfp_nheader;
    char arm_vfp_name[8];
    elf_note_arm_vfp_t arm_vfp;
} elf_thread_notes_t;

typedef struct __attribute__((packed))
{
    elf_note_header_t pr_siginfo_nheader;
    char pr_siginfo_name[8];
    elf_note_pr_siginfo_t pr_siginfo;

    elf_thread_notes_t threads[EXCEPTION_MAX_THREADS];
} elf_notes_t;

static const elf_headers_t ELF_HEADERS = {
    .file_header =
        {
            .ei_mag        = {0x7F, 'E', 'L', 'F'},
            .ei_class      = 1,  // ELFCLASS32
            .ei_data       = 1,  // ELFDATA2LSB
            .ei_version    = 1,  // EV_CURRENT
            .ei_osabi      = 0,  // ELFOSABI_NONE
            .ei_abiversion = 0,
            .ei_pad        = {0},

            .e_type    = 4,   // ET_CORE
            .e_machine = 40,  // EM_ARM
            .e_version = 1,   // EV_CURRENT
            .e_entry   = 0,
            .e_phoff   = sizeof(elf_file_header_t),
            .e_shoff   = 0,
            .e_flags   = 0x05400400,  // EF_ARM_EABI_VER5 | EF_ARM_LE8 | EF_ARM_VFP_FLOAT
            .e_ehsize  = sizeof(elf_file_header_t),
            .e_phentsize = sizeof(elf_program_header_t),
            .e_phnum     = 3,
            .e_shentsize = 0,
            .e_shnum     = 0,
            .e_shstrndx  = 0,
        },

    .note_pheader =
        {
            .p_type   = 4,  // PT_NOTE
            .p_offset = sizeof(elf_headers_t),
            .p_vaddr  = 0,
            .p_paddr  = 0,
            .p_filesz = sizeof(elf_notes_t),
            .p_memsz  = 0,
            .p_flags  = 0,
            .p_align  = 0,
        },

    .ccm_pheader =
        {
            .p_type   = 1,  // PT_LOAD
            .p_offset = sizeof(elf_headers_t) + sizeof(elf_notes_t),
            .p_vaddr  = 0x10000000,
            .p_paddr  = 0x10000000,
            .p_filesz = 64 * 1024,
            .p_memsz  = 64 * 1024,
            .p_flags  = 6,  // PF_R | PF_W
            .p_align  = 4,
        },

    .ram_pheader =
        {
            .p_type   = 1,  // PT_LOAD
            .p_offset = sizeof(elf_headers_t) + sizeof(elf_notes_t) + 64 * 1024,
            .p_vaddr  = 0x20000000,
            .p_paddr  = 0x20000000,
            .p_filesz = 128 * 1024,
            .p_memsz  = 128 * 1024,
            .p_flags  = 6,  // PF_R | PF_W
            .p_align  = 4,
        },
};

static elf_notes_t elf_notes = {
    .pr_siginfo_nheader =
        {
            .n_namesz = 5,
            .n_descsz = sizeof(elf_note_pr_siginfo_t),
            .n_type   = 0x53494749,  // IGIS
        },
    .pr_siginfo_name = "CORE",
};

typedef struct __attribute__((packed))
{
    uint32_t xpsr;
    uint32_t msp;
    uint32_t psp;
    uint32_t primask;
    uint32_t basepri;
    uint32_t faultmask;
    uint32_t control;
    uint32_t gpregs[13];
    uint32_t lr;
} exception_isr_frame_t;

#if STM32LIB_USE_FREERTOS && configUSE_TRACE_FACILITY
typedef struct __attribute__((packed))
{
    unsigned long stack_guard_rbar;
    unsigned long lr;
    unsigned long r4;
    unsigned long r5;
    unsigned long r6;
    unsigned long r7;
    unsigned long r8;
    unsigned long r9;
    unsigned long r10;
    unsigned long r11;
} exception_rtos_basic_frame_t;

typedef struct __attribute__((packed))
{
    exception_rtos_basic_frame_t basic;
    uint32_t fpregs[16];
} exception_rtos_extended_frame_t;
#endif

typedef struct __attribute__((packed))
{
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t xpsr;
} exception_hw_basic_frame_t;

typedef struct __attribute__((packed))
{
    exception_hw_basic_frame_t basic;
    uint32_t fpregs[16];
    uint32_t fpscr;
} exception_hw_extended_frame_t;

static void exception_fill_siginfo(unsigned int cause)
{
    // Fill out the signal info based on the exception number and fault status
    // registers.
    switch (cause)
    {
        case 3:  // Hard fault
            elf_notes.pr_siginfo.si_signum =
                9;  // SIGKILL (not really any good answer here)
            elf_notes.pr_siginfo.si_code = 0x80;  // SI_KERNEL
            break;

        case 4:                                   // Memory manage fault
            elf_notes.pr_siginfo.si_signum = 11;  // SIGSEGV
            if (SCB.CFSR.MMARVALID)
            {
                elf_notes.pr_siginfo.si_addr = (uint32_t)SCB.MMFAR;
            }
            // We don’t bother figuring out whether we had SEGV_MAPERR or
            // SEGV_ACCERR. So, leave si_code at zero.
            break;

        case 5:                                  // Bus fault
            elf_notes.pr_siginfo.si_signum = 7;  // SIGBUS
            if (SCB.CFSR.BFARVALID)
            {
                elf_notes.pr_siginfo.si_addr = (uint32_t)SCB.BFAR;
            }
            elf_notes.pr_siginfo.si_code = 3;  // BUS_OBJERR
            break;

        case 6:  // Usage fault
            if (SCB.CFSR.DIVBYZERO)
            {
                elf_notes.pr_siginfo.si_signum = 8;  // SIGFPE
                elf_notes.pr_siginfo.si_code   = 1;  // FPE_INTDIV
            }
            else if (SCB.CFSR.UNALIGNED)
            {
                elf_notes.pr_siginfo.si_signum = 7;  // SIGBUS
                elf_notes.pr_siginfo.si_code   = 1;  // BUS_ADRALN
            }
            else if (SCB.CFSR.NOCP)
            {
                elf_notes.pr_siginfo.si_signum = 4;  // SIGILL
                elf_notes.pr_siginfo.si_code   = 7;  // ILL_COPROC
            }
            else if (SCB.CFSR.INVPC)
            {
                elf_notes.pr_siginfo.si_signum = 4;  // SIGILL
                elf_notes.pr_siginfo.si_code   = 2;  // ILL_ILLOPN
            }
            else if (SCB.CFSR.INVSTATE)
            {
                elf_notes.pr_siginfo.si_signum = 4;  // SIGILL
                elf_notes.pr_siginfo.si_code   = 2;  // ILL_ILLOPN
            }
            else if (SCB.CFSR.UNDEFINSTR)
            {
                elf_notes.pr_siginfo.si_signum = 4;  // SIGILL
                elf_notes.pr_siginfo.si_code   = 1;  // ILL_ILLOPC
            }
            break;

        case 12:                                 // Debug monitor fault
            elf_notes.pr_siginfo.si_signum = 6;  // SIGABRT
            elf_notes.pr_siginfo.si_code =
                -6;  // Sent by tkill (what abort()/assert() do)
            break;
    }
}

static void exception_fill_thread_isr(const exception_isr_frame_t *swframe,
                                      elf_thread_notes_t *thread)
{
    // Fill headers.
    thread->prstatus_nheader.n_type = 1;  // NT_PRSTATUS
    strcpy(thread->prstatus_name, "CORE");
    thread->arm_vfp_nheader.n_type = 0x400;
    strcpy(thread->arm_vfp_name, "LINUX");

    // Copy signal info over to prstatus.
    thread->prstatus.si_signo  = elf_notes.pr_siginfo.si_signum;
    thread->prstatus.si_code   = elf_notes.pr_siginfo.si_code;
    thread->prstatus.si_errno  = elf_notes.pr_siginfo.si_code;
    thread->prstatus.pr_cursig = elf_notes.pr_siginfo.si_signum;

    // General purpose registers always come from the software frame. The
    // hardware frame may or may not exist, and if it does, it only has a few
    // GP regs. The software frame *always* exists and has *all* the GP regs,
    // so it’s much easier.
    memcpy(&thread->prstatus.pr_gpregs, &swframe->gpregs, 13 * sizeof(uint32_t));

    const exception_hw_basic_frame_t *hwbframe;
    const exception_hw_extended_frame_t *hweframe;
    {
        // Find the hardware frame, if there is one, and initialize ELF note SP
        // to point at it.
        const void *hwframeptr;
        if (swframe->lr & 4U)
        {
            // We were running on the process stack. The hardware frame may or
            // may not be present, as there might have been an error while
            // stacking the frame. If it is present, then it was the last thing
            // pushed on the process stack and is thus still pointed to by PSP.
            if (SCB.CFSR.MSTKERR || SCB.CFSR.STKERR)
            {
                hwframeptr = 0;
            }
            else
            {
                hwframeptr = (const void *)swframe->psp;
            }
            thread->prstatus.pr_reg_sp = swframe->psp;
        }
        else
        {
            // Hardware frame lives on the main stack. The main stack is
            // assumed not to have stacking errors, since we’re using it right
            // now. The hardware frame was pushed just before entry to the
            // fault ISR, immediately after which the software frame was
            // pushed. So, the hardware frame lives immediately following the
            // software frame in memory.
            hwframeptr                 = swframe + 1;
            thread->prstatus.pr_reg_sp = (uint32_t)hwframeptr;
        }

        // Classify hardware frame into basic or extended. Advance SP over
        // frame.
        if (swframe->lr & 16U)
        {
            hwbframe = hwframeptr;
            hweframe = 0;
            thread->prstatus.pr_reg_sp += sizeof(exception_hw_basic_frame_t);
        }
        else
        {
            hweframe = hwframeptr;
            hwbframe = &hweframe->basic;
            thread->prstatus.pr_reg_sp += sizeof(exception_hw_extended_frame_t);
        }

        // Check for stack realignment.
        if (hwbframe && hwbframe->xpsr & (1U << 9U))
        {
            // Stack pointer was adjusted by 4 to achieve 8-byte alignment.
            thread->prstatus.pr_reg_sp += 4U;
        }
    }

    // Classify thread mode as PID 1 and handler mode as PID 0.
    if (swframe->lr & 8U)
    {
        thread->prstatus.pr_pid = 1;
    }
    else
    {
        thread->prstatus.pr_pid = 0;
    }

    // Fill the rest of prstatus as best we can.
    if (hwbframe)
    {
        thread->prstatus.pr_reg_lr   = hwbframe->lr;
        thread->prstatus.pr_reg_pc   = hwbframe->pc;
        thread->prstatus.pr_reg_xpsr = hwbframe->xpsr;
    }
    else
    {
        thread->prstatus.pr_reg_xpsr = swframe->xpsr;
    }

    // Sort out the floating point registers.
    if (CPACR.CP11 != 0 && CPACR.CP10 != 0 && hwbframe)
    {
        if (hweframe)
        {
            // The hardware pushed an extended frame.
            if (FP.CCR.LSPEN)
            {
                // Lazy state preservation has applied here. There is no data
                // in the frame yet. Execute an arbitrary floating-point
                // instruction to force the state out to RAM.
                asm volatile("vmov.f32 s0, #1.0" ::: "s0");
            }
            // The first 16 registers and FPSCR are in the stack frame. The
            // second 16 registers are not preserved anywhere yet and can be
            // grabbed directly.
            memcpy(&thread->arm_vfp.sregs, &hweframe->fpregs, sizeof(hweframe->fpregs));
            asm volatile(
                "vstm %[dest], {s16-s31}" ::[dest] "r"(&thread->arm_vfp.sregs[16])
                : "memory");
            thread->arm_vfp.fpscr = hweframe->fpscr;
        }
        else
        {
            // The hardware pushed a basic frame.
            if (FP.CCR.ASPEN)
            {
                // Automatic preservation is enabled. If we had been using FP,
                // the hardware would have pushed an extended frame. That it
                // pushed a basic frame means no FP was happening. Wipe the
                // note.
                thread->arm_vfp_nheader.n_type = 0;
                memset(thread->arm_vfp_name, 0, sizeof(thread->arm_vfp_name));
            }
            else
            {
                // Automatic preservation is disabled. We really don’t know
                // whether any FP was happening or not. The hardware will
                // always push a basic frame in this state. Just to be safe,
                // write out all the FP registers from their current values. If
                // the application wasn’t using FP, the data can be ignored.
                asm volatile("vstm %[dest], {s0-s31}" ::[dest] "r"(&thread->arm_vfp.sregs)
                             : "memory");
                asm("vmrs %[dest], fpscr" : [dest] "=r"(thread->arm_vfp.fpscr));
            }
        }
    }
    else
    {
        // The coprocessor is disabled or no hardware frame was pushed (due to
        // a fault during stacking). If no hardware frame was pushed, we can’t
        // differentiate between whether the frame that wasn’t pushed would
        // have been basic or extended. If the frame would have been extended,
        // then the bottom 16 FP registers (and FPSCR) would have had
        // indeterminate values after pushing. Therefore, it’s not safe to just
        // copy the current values of the FP registers from the register file.
        // Wipe the note.
        thread->arm_vfp_nheader.n_type = 0;
        memset(thread->arm_vfp_name, 0, sizeof(thread->arm_vfp_name));
    }

    // Do some final tidying up.
    thread->prstatus.pr_reg_orig_r0 = thread->prstatus.pr_gpregs[0];
}

#if STM32LIB_USE_FREERTOS && configUSE_TRACE_FACILITY
static void exception_fill_thread_rtos(const exception_rtos_basic_frame_t *swbframe,
                                       uint32_t pid, elf_thread_notes_t *thread)
{
    // Fill headers.
    thread->prstatus_nheader.n_type = 1;  // NT_PRSTATUS
    strcpy(thread->prstatus_name, "CORE");
    thread->arm_vfp_nheader.n_type = 0x400;
    strcpy(thread->arm_vfp_name, "LINUX");

    // Copy signal info over to prstatus.
    thread->prstatus.si_signo  = elf_notes.pr_siginfo.si_signum;
    thread->prstatus.si_code   = elf_notes.pr_siginfo.si_code;
    thread->prstatus.si_errno  = elf_notes.pr_siginfo.si_code;
    thread->prstatus.pr_cursig = elf_notes.pr_siginfo.si_signum;

    // Determine if the frames are basic or extended.
    bool extended = !(swbframe->lr & 16U);

    // Get a software extended frame if there is one.
    const exception_rtos_extended_frame_t *sweframe =
        extended ? (const exception_rtos_extended_frame_t *)swbframe : 0;

    // Get the hardware frames and populate the SP register in the ELF note to
    // point just past the hardware frame.
    const exception_hw_basic_frame_t *hwbframe;
    const exception_hw_extended_frame_t *hweframe;
    if (extended)
    {
        hweframe = (const exception_hw_extended_frame_t *)(sweframe + 1);
        hwbframe = &hweframe->basic;
        thread->prstatus.pr_reg_sp = (uint32_t)(hweframe + 1);
    }
    else
    {
        hweframe                   = 0;
        hwbframe                   = (const exception_hw_basic_frame_t *)(swbframe + 1);
        thread->prstatus.pr_reg_sp = (uint32_t)(hwbframe + 1);
    }

    // Check for stack realignment.
    if (hwbframe->xpsr & (1U << 9U))
    {
        // Stack pointer was adjusted by 4 to achieve 8-byte alignment.
        thread->prstatus.pr_reg_sp += 4U;
    }

    // Grab general purpose registers from a mix of the software and hardware
    // frames.
    thread->prstatus.pr_gpregs[0]  = hwbframe->r0;
    thread->prstatus.pr_gpregs[1]  = hwbframe->r1;
    thread->prstatus.pr_gpregs[2]  = hwbframe->r2;
    thread->prstatus.pr_gpregs[3]  = hwbframe->r3;
    thread->prstatus.pr_gpregs[4]  = swbframe->r4;
    thread->prstatus.pr_gpregs[5]  = swbframe->r5;
    thread->prstatus.pr_gpregs[6]  = swbframe->r6;
    thread->prstatus.pr_gpregs[7]  = swbframe->r7;
    thread->prstatus.pr_gpregs[8]  = swbframe->r8;
    thread->prstatus.pr_gpregs[9]  = swbframe->r9;
    thread->prstatus.pr_gpregs[10] = swbframe->r10;
    thread->prstatus.pr_gpregs[11] = swbframe->r11;
    thread->prstatus.pr_gpregs[12] = hwbframe->r12;

    // Grab a process ID.
    thread->prstatus.pr_pid = pid;

    // Fill the rest of prstatus.
    thread->prstatus.pr_reg_lr   = hwbframe->lr;
    thread->prstatus.pr_reg_pc   = hwbframe->pc;
    thread->prstatus.pr_reg_xpsr = hwbframe->xpsr;

    // Sort out the floating point registers.
    if (extended)
    {
        // This thread was using the FPU. The first 16 registers and FPSCR are
        // in the hardware frame. The second 16 registers are in the software
        // frame.
        memcpy(thread->arm_vfp.sregs, hweframe->fpregs, sizeof(hweframe->fpregs));
        memcpy(thread->arm_vfp.sregs + 16, sweframe->fpregs, sizeof(sweframe->fpregs));
        thread->arm_vfp.fpscr = hweframe->fpscr;
    }
    else
    {
        // This thread is not using the FPU. Wipe the note.
        thread->arm_vfp_nheader.n_type = 0;
        memset(thread->arm_vfp_name, 0, sizeof(thread->arm_vfp_name));
    }

    // Do some final tidying up.
    thread->prstatus.pr_reg_orig_r0 = thread->prstatus.pr_gpregs[0];
}
#endif

static void exception_fill_core_notes(const exception_isr_frame_t *swframe,
                                      unsigned int cause)
{
    // Fill note name sizes and data sizes, but not names or types. The sizes
    // must be correct so that a note reader can walk from note to note and the
    // notes block can be well-formed. However, the names and types should only
    // be filled in when actually populating a note, so a reader won’t pay
    // attention to the contents of unused space.
    for (size_t i = 0; i != EXCEPTION_MAX_THREADS; ++i)
    {
        elf_notes.threads[i].prstatus_nheader.n_namesz = 5;
        elf_notes.threads[i].prstatus_nheader.n_descsz = sizeof(elf_note_prstatus_t);
        elf_notes.threads[i].arm_vfp_nheader.n_namesz  = 6;
        elf_notes.threads[i].arm_vfp_nheader.n_descsz  = sizeof(elf_note_arm_vfp_t);
    }

    // Fill siginfo structure.
    exception_fill_siginfo(cause);

    // Fill the executing thread.
    exception_fill_thread_isr(swframe, &elf_notes.threads[0]);

    // If using FreeRTOS and this is not a hard fault, fill the other threads
    // as well.
#if STM32LIB_USE_FREERTOS && configUSE_TRACE_FACILITY
    if (cause != 3)
    {
        // Suspend the scheduler. uxTaskGetSystemState also suspends the
        // scheduler while it’s running. However, it then tries to resume the
        // scheduler at the end, as one would expect. Suspending the scheduler
        // is harmless—it just increments a counter—but trying to resume it
        // inside a CPU fault ISR is probably a very, very bad idea. Adding an
        // extra suspend guarantees that it will never actually try to resume.
        //
        // Also, uxTaskGetSystemState uses portENTER_CRITICAL, which asserts
        // that it is not called from an ISR (as it is not intended to be).
        // We’re doing weird stuff here which makes it sort of OK, so relax
        // that check.
        portRELAX_CRITICAL_ISR_CHECK();
        vTaskSuspendAll();
        TaskStatus_t tasks[EXCEPTION_MAX_THREADS];
        unsigned int count   = uxTaskGetSystemState(tasks, EXCEPTION_MAX_THREADS, 0);
        size_t write_index   = 1;
        TaskHandle_t current = xTaskGetCurrentTaskHandle();
        for (unsigned int i = 0; i != count && write_index != EXCEPTION_MAX_THREADS; ++i)
        {
            if (tasks[i].xHandle == current)
            {
                // The current task has already been written out, using the ISR
                // frame. However, if we were running in thread mode, we can
                // update its PID field to align with how we allocate PIDs
                // elsewhere. If it was running in handler mode, leave its PID
                // at zero to signify that fact.
                if (elf_notes.threads[0].prstatus.pr_pid)
                {
                    elf_notes.threads[0].prstatus.pr_pid = (uint32_t)tasks[i].xHandle;
                }
            }
            else
            {
                // Write out the task. We know the top of stack is the first
                // word in the TCB, which is pointed to by the task handle.
                // Yank it out in a bit of an ugly way.
                const exception_rtos_basic_frame_t *const *tcb_pointer =
                    (const exception_rtos_basic_frame_t *const *)tasks[i].xHandle;
                exception_fill_thread_rtos(*tcb_pointer, (uint32_t)tasks[i].xHandle,
                                           &elf_notes.threads[write_index++]);
            }
        }
    }
#endif
}

static void common_fault_isr(const exception_isr_frame_t *sp, unsigned int cause)
    __attribute__((noreturn, used));
static void common_fault_isr(const exception_isr_frame_t *sp, unsigned int cause)
{
    // Call the early application callbacks.
    if (app_cbs && app_cbs->early)
    {
        app_cbs->early();
    }

    // Turn off the memory protection unit.
    MPU.CTRL.ENABLE = 0;
    asm volatile("dsb");
    asm volatile("isb");

    // If we have a core dump writer, then write a core dump.
    bool core_written = false;
    if (core_writer)
    {
        exception_fill_core_notes(sp, cause);
        core_writer->start();
        core_writer->write(&ELF_HEADERS, sizeof(ELF_HEADERS));
        core_writer->write(&elf_notes, sizeof(elf_notes));
        core_writer->write((const void *)0x10000000, 64 * 1024);
        core_writer->write((const void *)0x20000000, 128 * 1024);
        core_written = core_writer->end();
    }

    // Call the late application callback.
    if (app_cbs && app_cbs->late)
    {
        app_cbs->late(core_written);
    }

    // Late application callback return or not provided.
    // Die forever.
    for (;;)
    {
        asm volatile("wfi");
    }
}

#define GEN_FAULT_HANDLER(cause)                                                         \
    asm volatile(                                                                        \
        "push {r0-r12, lr}\n\t"                                                          \
        "mrs r0, xpsr\n\t"                                                               \
        "mrs r1, msp\n\t"                                                                \
        "mrs r2, psp\n\t"                                                                \
        "mrs r3, primask\n\t"                                                            \
        "mrs r4, basepri\n\t"                                                            \
        "mrs r5, faultmask\n\t"                                                          \
        "mrs r6, control\n\t"                                                            \
        "push {r0-r6}\n\t"                                                               \
        "mov r0, sp\n\t"                                                                 \
        "mov r1, %[cause_constant]\n\t"                                                  \
        "b common_fault_isr\n\t"                                                         \
        :                                                                                \
        : [cause_constant] "i"(cause));                                                  \
    __builtin_unreachable();

/**
 * \endcond
 */

/**
 * \brief An interrupt handler that handles hard faults.
 *
 * This function must be inserted as element 3 in the application’s CPU
 * exception vector table.
 */
void exception_hard_fault_isr(void)
{
    GEN_FAULT_HANDLER(3);
}

/**
 * \brief An interrupt handler that handles memory manage faults.
 *
 * This function must be inserted as element 4 in the application’s CPU
 * exception vector table.
 */
void exception_memory_manage_fault_isr(void)
{
    GEN_FAULT_HANDLER(4);
}

/**
 * \brief An interrupt handler that handles bus faults.
 *
 * This function must be inserted as element 5 in the application’s CPU
 * exception vector table.
 */
void exception_bus_fault_isr(void)
{
    GEN_FAULT_HANDLER(5);
}

/**
 * \brief An interrupt handler that handles usage faults.
 *
 * This function must be inserted as element 6 in the application’s CPU
 * exception vector table.
 */
void exception_usage_fault_isr(void)
{
    GEN_FAULT_HANDLER(6);
}

/**
 * \brief An interrupt handler that handles debug faults.
 *
 * This function must be inserted as element 12 in the application’s CPU
 * exception vector table.
 */
void exception_debug_fault_isr(void)
{
    GEN_FAULT_HANDLER(12);
}

/**
 * @}
 */
