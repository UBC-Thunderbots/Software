/**
 * \defgroup INIT Chip initialization functions
 *
 * \{
 */

#include <assert.h>
#include <init.h>
#include <inttypes.h>
#include <rcc.h>
#include <registers/flash.h>
#include <registers/mpu.h>
#include <registers/power.h>
#include <registers/scb.h>
#include <registers/syscfg.h>
#include <registers/systick.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

volatile uint64_t bootload_flag;
#define BOOTLOAD_FLAG_VALUE UINT64_C(0xFE228106195AD2B0)

extern unsigned char linker_data_vma;
extern const unsigned char linker_data_lma;
extern unsigned char linker_data_size;
extern unsigned char linker_pdata_vma;
extern const unsigned char linker_pdata_lma;
extern unsigned char linker_pdata_size;
extern unsigned char linker_ramtext_vma;
extern const unsigned char linker_ramtext_lma;
extern unsigned char linker_ramtext_size;
extern unsigned char linker_bss_vma;
extern unsigned char linker_bss_size;

static const init_specs_t *init_specs_saved;

static unsigned int compute_ahb_prescale(unsigned int sys, unsigned int cpu)
{
    assert(!(sys % cpu));
    switch (sys / cpu)
    {
        case 1:
            return 0;
        case 2:
            return 8;
        case 4:
            return 9;
        case 8:
            return 10;
        case 16:
            return 11;
        case 64:
            return 12;
        case 128:
            return 13;
        case 256:
            return 14;
        case 512:
            return 15;
        default:
            abort();
    }
}

static unsigned int compute_apb_prescale(unsigned int cpu, unsigned int apb)
{
    assert(!(cpu % apb));
    switch (cpu / apb)
    {
        case 1:
            return 0;
        case 2:
            return 4;
        case 4:
            return 5;
        case 8:
            return 6;
        case 16:
            return 7;
        default:
            abort();
    }
}

static unsigned int compute_flash_wait_states(unsigned int cpu)
{
    return (cpu - 1) / 30;
}

/**
 * \brief Initializes the chip.
 *
 * The following initializations are done:
 * \li The bootloader is entered, if requested.
 * \li The system stacks are initialized properly.
 * \li The data section is filled from ROM.
 * \li The BSS section is wiped.
 * \li Interrupt handling is configured.
 * \li The memory protection unit is initialized.
 * \li The HSE oscillator is enabled.
 * \li The PLL is enabled and configured.
 * \li Flash access latency is set.
 * \li The system clock is switched to the PLL.
 * \li CPU caches are cleared and enabled.
 * \li The systick timer is configured to overflow once per microsecond.
 *
 * A call to this function should usually be the first statement in the main
 * function.
 *
 * \param[in] specs the specifications for how to initialize the chip
 * \param[in] specSize the size of \p *specs in bytes
 */
void init_chip(const init_specs_t *specs)
{
    // Check if we’re supposed to go to the bootloader.
    RCC_CSR_t rcc_csr_shadow = RCC.CSR;  // Keep a copy of RCC_CSR
    RCC.CSR.RMVF             = 1;        // Clear reset flags
    RCC.CSR.RMVF             = 0;        // Stop clearing reset flags
    if (rcc_csr_shadow.SFTRSTF && bootload_flag == BOOTLOAD_FLAG_VALUE)
    {
        bootload_flag = 0;
        rcc_enable_reset(APB2, SYSCFG);
        asm volatile("dsb");
        SYSCFG.MEMRMP.MEM_MODE = 1;
        asm volatile("dsb");
        asm volatile("isb");
        rcc_disable(APB2, SYSCFG);
        asm volatile(
            "msr control, %[control]\n\t"
            "isb\n\t"
            "msr msp, %[stack]\n\t"
            "mov pc, %[vector]"
            :
            : [ control ] "r"(0), [ stack ] "r"(*(const volatile uint32_t *)0x1FFF0000),
              [ vector ] "r"(*(const volatile uint32_t *)0x1FFF0004));
        __builtin_unreachable();
    }
    bootload_flag = 0;

    // Copy initialized globals and statics from ROM to RAM.
    memcpy(&linker_data_vma, &linker_data_lma,
           (size_t)&linker_data_size /* Yes, there should be an & here! */);
    memcpy(&linker_ramtext_vma, &linker_ramtext_lma,
           (size_t)&linker_ramtext_size /* Yes, there should be an & here! */);
    if (!rcc_csr_shadow.SFTRSTF)
    {
        memcpy(&linker_pdata_vma, &linker_pdata_lma,
               (size_t)&linker_pdata_size /* Yes, there should be an & here! */);
    }
    // Scrub the BSS section in RAM.
    memset(&linker_bss_vma, 0,
           (size_t)&linker_bss_size /* Yes, there should be an & here! */);

    // Save the specs.
    init_specs_saved = specs;

    // Always 8-byte-align the stack pointer on entry to an interrupt handler (as ARM
    // recommends).
    SCB.CCR.STKALIGN = 1;  // Guarantee 8-byte alignment

    // Set up interrupt handling.
    exception_init(specs->exception_core_writer, &specs->exception_app_cbs,
                   specs->exception_prios);

    if (!specs->flags.freertos)
    {
        // Set up the memory protection unit to catch bad pointer dereferences.
        // The private peripheral bus (0xE0000000 length 1 MiB) always uses the system
        // memory map, so no region is needed for it. We set up the regions first, then
        // enable the MPU.
        static const struct
        {
            uint32_t address;
            MPU_RASR_t rasr;
        } REGIONS[] = {
            // 0x08000000–0x080FFFFF (length 1 MiB): Flash memory (normal, read-only,
            // write-through cache, executable)
            {0x08000000,
             {.XN     = 0,
              .AP     = 0b111,
              .TEX    = 0b000,
              .S      = 0,
              .C      = 1,
              .B      = 0,
              .SRD    = 0,
              .SIZE   = 19,
              .ENABLE = 1}},

            // 0x10000000–0x1000FFFF (length 64 kiB): CCM (stack) (normal, read-write,
            // write-back write-allocate cache, not executable)
            {0x10000000,
             {.XN     = 1,
              .AP     = 0b011,
              .TEX    = 0b001,
              .S      = 0,
              .C      = 1,
              .B      = 1,
              .SRD    = 0,
              .SIZE   = 15,
              .ENABLE = 1}},

            // 0x1FFF0000–0x1FFF7FFF (length 32 kiB): System memory including U_ID and
            // F_SIZE (normal, read-only, write-through cache, not executable)
            {0x1FFF0000,
             {.XN     = 1,
              .AP     = 0b111,
              .TEX    = 0b000,
              .S      = 0,
              .C      = 1,
              .B      = 0,
              .SRD    = 0,
              .SIZE   = 14,
              .ENABLE = 1}},

            // 0x20000000–0x2001FFFF (length 128 kiB): SRAM (normal, read-write,
            // write-back write-allocate cache, not executable)
            {0x20000000,
             {.XN     = 1,
              .AP     = 0b011,
              .TEX    = 0b001,
              .S      = 1,
              .C      = 1,
              .B      = 1,
              .SRD    = 0,
              .SIZE   = 16,
              .ENABLE = 1}},

            // 0x40000000–0x4007FFFF (length 512 kiB): Peripherals (device, read-write,
            // not executable) using subregions:
            // Subregion 0 (0x40000000–0x4000FFFF): Enabled (contains APB1)
            // Subregion 1 (0x40010000–0x4001FFFF): Enabled (contains APB2)
            // Subregion 2 (0x40020000–0x4002FFFF): Enabled (contains AHB1)
            // Subregion 3 (0x40030000–0x4003FFFF): Disabled
            // Subregion 4 (0x40040000–0x4004FFFF): Disabled
            // Subregion 5 (0x40050000–0x4005FFFF): Disabled
            // Subregion 6 (0x40060000–0x4006FFFF): Disabled
            // Subregion 7 (0x40070000–0x4007FFFF): Disabled
            {0x40000000,
             {.XN     = 1,
              .AP     = 0b011,
              .TEX    = 0b010,
              .S      = 0,
              .C      = 0,
              .B      = 0,
              .SRD    = 0b11111000,
              .SIZE   = 18,
              .ENABLE = 1}},

            // 0x50000000–0x5007FFFF (length 512 kiB): Peripherals (device, read-write,
            // not executable) using subregions:
            // Subregion 0 (0x50000000–0x5000FFFF): Enabled (contains AHB2)
            // Subregion 1 (0x50010000–0x5001FFFF): Enabled (contains AHB2)
            // Subregion 2 (0x50020000–0x5002FFFF): Enabled (contains AHB2)
            // Subregion 3 (0x50030000–0x5003FFFF): Enabled (contains AHB2)
            // Subregion 4 (0x50040000–0x5004FFFF): Enabled (contains AHB2)
            // Subregion 5 (0x50050000–0x5005FFFF): Enabled (contains AHB2)
            // Subregion 6 (0x50060000–0x5006FFFF): Enabled (contains AHB2)
            // Subregion 7 (0x50070000–0x5007FFFF): Disabled
            {0x50000000,
             {.XN     = 1,
              .AP     = 0b011,
              .TEX    = 0b010,
              .S      = 0,
              .C      = 0,
              .B      = 0,
              .SRD    = 0b10000000,
              .SIZE   = 18,
              .ENABLE = 1}},
        };
        for (unsigned int i = 0; i < sizeof(REGIONS) / sizeof(*REGIONS); ++i)
        {
            MPU_RNR_t rnr = {.REGION = i};
            MPU.RNR       = rnr;
            MPU.RBAR.ADDR = REGIONS[i].address >> 5;
            MPU.RASR      = REGIONS[i].rasr;
        }
        MPU_CTRL_t tmp = {
            .PRIVDEFENA = 0,  // Background region is disabled even in privileged mode.
            .HFNMIENA   = 0,  // Protection unit disables itself when taking hard faults,
                              // memory faults, and NMIs.
            .ENABLE = 1,      // Enable MPU.
        };
        MPU.CTRL = tmp;
        asm volatile("dsb");
        asm volatile("isb");
    }

    // Enable the SYSCFG module.
    rcc_enable_reset(APB2, SYSCFG);

    // Enable the HSE oscillator.
    {
        RCC_CR_t tmp = {
            .PLLI2SON = 0,   // I²S PLL off.
            .PLLON    = 0,   // Main PLL off.
            .CSSON    = 0,   // Clock security system off.
            .HSEON    = 1,   // HSE oscillator enabled.
            .HSITRIM  = 16,  // HSI oscillator trimmed to midpoint.
            .HSION    = 1,   // HSI oscillator enabled (still using it at this point).
        };
        tmp.HSEBYP = !specs->flags.hse_crystal;
        RCC.CR     = tmp;
    }
    // Wait for the HSE oscillator to be ready.
    while (!RCC.CR.HSERDY)
        ;
    // Configure the PLL.
    {
        RCC_PLLCFGR_t tmp = {
            .PLLSRC = 1,  // Use HSE for PLL input
        };

        // Divide HSE frequency to get 2 MHz input to PLL.
        assert(!(specs->hse_frequency % 2));
        assert(2 <= specs->hse_frequency && specs->hse_frequency <= 50);
        tmp.PLLM = specs->hse_frequency / 2;

        // Multiply 2 MHz input to get PLL output frequency.
        assert(!(specs->pll_frequency % 2));
        assert(192 <= specs->pll_frequency && specs->pll_frequency <= 432);
        tmp.PLLN = specs->pll_frequency / 2;

        // Divide PLL output to get system freqency.
        assert(specs->sys_frequency <= 168);
        assert(!(specs->pll_frequency % specs->sys_frequency));
        switch (specs->pll_frequency / specs->sys_frequency)
        {
            case 2:
                tmp.PLLP = 0;
                break;
            case 4:
                tmp.PLLP = 1;
                break;
            case 6:
                tmp.PLLP = 2;
                break;
            case 8:
                tmp.PLLP = 3;
                break;
            default:
                abort();
                break;
        }

        // Divide PLL output to get 48 MHz USB/SDIO/RNG clock.
        if ((specs->flags).uses_usb)
        {
            assert(!(specs->pll_frequency % 48));
        }

        tmp.PLLQ = specs->pll_frequency / 48;

        RCC.PLLCFGR = tmp;
    }
    // Enable the PLL.
    RCC.CR.PLLON = 1;  // Enable PLL
    // Wait for the PLL to lock.
    while (!RCC.CR.PLLRDY)
        ;
    // Set up bus frequencies.
    {
        RCC_CFGR_t tmp = {
            .MCO2    = 2,  // MCO2 pin outputs HSE
            .MCO2PRE = 0,  // Divide HSE by 1 to get MCO2 (must be ≤ 100 MHz)
            .MCO1PRE = 0,  // Divide HSE by 1 to get MCO1 (must be ≤ 100 MHz)
            .I2SSRC  = 0,  // I²S module gets clock from PLLI2X
            .MCO1    = 2,  // MCO1 pin outputs HSE
            .RTCPRE  = 0,  // RTC clock disabled
            .SW      = 0,  // Use HSI for SYSCLK for now, until everything else is ready
        };

        // Configure MCO if necessary
        if (specs->mco1_cfg.enable)
        {
            tmp.MCO1    = specs->mco1_cfg.source;
            tmp.MCO1PRE = specs->mco1_cfg.prescalar;
        }
        if (specs->mco2_cfg.enable)
        {
            tmp.MCO2    = specs->mco2_cfg.source;
            tmp.MCO2PRE = specs->mco2_cfg.prescalar;
        }

        // Divide system clock frequency to get CPU frequency.
        assert(specs->cpu_frequency <= 168);
        tmp.HPRE = compute_ahb_prescale(specs->sys_frequency, specs->cpu_frequency);

        // Divide CPU frequency to get APB1 and APB2 frequency.
        assert(specs->apb1_frequency <= 42);
        tmp.PPRE1 = compute_apb_prescale(specs->cpu_frequency, specs->apb1_frequency);
        assert(specs->apb2_frequency <= 84);
        tmp.PPRE2 = compute_apb_prescale(specs->cpu_frequency, specs->apb2_frequency);

        RCC.CFGR = tmp;
    }
    // Wait 16 AHB cycles for the new prescalers to settle.
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    // Set Flash access latency to necessary number of wait states.
    unsigned int flash_wait_states = compute_flash_wait_states(specs->cpu_frequency);
    {
        FLASH_ACR_t tmp = {
            .DCRST  = 0,  // Do not clear data cache at this time.
            .ICRST  = 0,  // Do not clear instruction cache at this time.
            .DCEN   = 0,  // Do not enable data cache at this time.
            .ICEN   = 0,  // Do not enable instruction cache at this time.
            .PRFTEN = 0,  // Do not enable prefetcher at this time.
        };
        tmp.LATENCY = flash_wait_states;
        FLASH.ACR   = tmp;
    }
    // Flash access latency change may not be immediately effective; wait until it’s
    // locked in.
    while (FLASH.ACR.LATENCY != flash_wait_states)
        ;
    // Actually initiate the clock switch.
    RCC.CFGR.SW = 2;  // Use PLL for SYSCLK
    // Wait for the clock switch to complete.
    while (RCC.CFGR.SWS != 2)
        ;
    // Turn off the HSI now that it’s no longer needed.
    RCC.CR.HSION = 0;  // Disable HSI

    // Flush any data in the CPU caches (which are not presently enabled).
    {
        FLASH_ACR_t tmp = FLASH.ACR;
        tmp.DCRST       = 1;  // Reset data cache.
        tmp.ICRST       = 1;  // Reset instruction cache.
        FLASH.ACR       = tmp;
    }
    {
        FLASH_ACR_t tmp = FLASH.ACR;
        tmp.DCRST       = 0;  // Stop resetting data cache.
        tmp.ICRST       = 0;  // Stop resetting instruction cache.
        FLASH.ACR       = tmp;
    }

    // Turn on the caches.
    // There is an errata that says prefetching doesn’t work on some silicon, but it seems
    // harmless to enable the flag even so.
    {
        FLASH_ACR_t tmp = FLASH.ACR;
        tmp.DCEN        = 1;  // Enable data cache
        tmp.ICEN        = 1;  // Enable instruction cache
        tmp.PRFTEN      = 1;  // Enable prefetching
        FLASH.ACR       = tmp;
    }

    if (!specs->flags.freertos)
    {
        // Set SYSTICK to divide by cpu_frequency so it overflows every microsecond.
        SYSTICK.RVR = specs->cpu_frequency - 1;
        // Reset the counter.
        SYSTICK.CVR = 0;
        // Set SYSTICK to run with the core AHB clock.
        {
            SYST_CSR_t tmp = {
                .CLKSOURCE = 1,  // Use core clock
                .ENABLE    = 1,  // Counter is running
            };
            SYSTICK.CSR = tmp;
        }

        // Activate the FPU (in FreeRTOs mode, this will be done on scheduler
        // startup so context saving is handled properly).
        FPCCR_t fpccr = {.LSPEN = 1};
        FP.CCR        = fpccr;
        CPACR_t cpacr = CPACR;
        cpacr.CP10 = cpacr.CP11 = 3;
        CPACR                   = cpacr;
    }

    // If we will be running at most 144 MHz, switch to the lower-power voltage
    // regulator mode.
    if (specs->cpu_frequency <= 144)
    {
        rcc_enable_reset(APB1, PWR);
        PWR_CR.VOS = 2;  // Set regulator scale 2
        rcc_disable(APB1, PWR);
    }

    // If requested, enable the I/O compensation cell.
    if (specs->flags.io_compensation_cell)
    {
        SYSCFG_CMPCR_t cmpcr = {.CMP_PD = 1};
        SYSCFG.CMPCR         = cmpcr;
        while (!SYSCFG.CMPCR.READY)
            ;
    }

    // Done initializing; disable SYSCFG module.
    rcc_disable(APB2, SYSCFG);
}

/**
 * \brief Marks a request to enter the bootloader and reboots the chip.
 *
 * The subsequent call to \ref init_chip will actually enter the bootloader.
 */
void init_bootload(void)
{
    // Mark that we should go to the bootloader on next reboot.
    bootload_flag = BOOTLOAD_FLAG_VALUE;

    // Disable all interrupts.
    asm volatile("cpsid i");

    // Ensure all pending memory writes have finished.
    asm volatile("dsb");

    // Reboot the chip.
    {
        AIRCR_t tmp     = SCB.AIRCR;
        tmp.VECTKEY     = 0x05FA;
        tmp.SYSRESETREQ = 1;
        SCB.AIRCR       = tmp;
    }
    __sync_synchronize();
    for (;;)
        ;
}

/**
 * \brief Returns the initialization specs.
 *
 * \return the initialization specs
 */
const init_specs_t *init_specs(void)
{
    return init_specs_saved;
}

/**
 * \}
 */
