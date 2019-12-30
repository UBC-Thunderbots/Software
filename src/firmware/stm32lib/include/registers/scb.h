/**
 * \ingroup REG
 * \defgroup REGSCB System control block
 * @{
 */
#ifndef STM32LIB_REGISTERS_SCB_H
#define STM32LIB_REGISTERS_SCB_H

typedef struct
{
    unsigned DISMCYCNT : 1;
    unsigned DISDEFWBUF : 1;
    unsigned DISFOLD : 1;
    unsigned : 5;
    unsigned DISFPCA : 1;
    unsigned DISOOFP : 1;
    unsigned : 22;
} ACTLR_t;
_Static_assert(sizeof(ACTLR_t) == 4U, "ACTLR_t is wrong size");

extern volatile ACTLR_t ACTLR;

typedef struct
{
    unsigned REVISION : 4;
    unsigned PARTNO : 12;
    unsigned CONSTANT : 4;
    unsigned VARIANT : 4;
    unsigned IMPLEMENTER : 8;
} CPUID_t;
_Static_assert(sizeof(CPUID_t) == 4U, "CPUID_t is wrong size");

typedef struct
{
    unsigned VECTACTIVE : 9;
    unsigned : 2;
    unsigned RETTOBASE : 1;
    unsigned VECTPENDING : 10;
    unsigned ISRPENDING : 1;
    unsigned : 2;
    unsigned PENDSTCLR : 1;
    unsigned PENDSTSET : 1;
    unsigned PENDSVCLR : 1;
    unsigned PENDSVSET : 1;
    unsigned : 2;
    unsigned NMIPENDSET : 1;
} ICSR_t;
_Static_assert(sizeof(ICSR_t) == 4U, "ICSR_t is wrong size");

typedef struct
{
    unsigned VECTRESET : 1;
    unsigned VECTCLRACTIVE : 1;
    unsigned SYSRESETREQ : 1;
    unsigned : 5;
    unsigned PRIGROUP : 3;
    unsigned : 4;
    unsigned ENDIANNESS : 1;
    unsigned VECTKEY : 16;
} AIRCR_t;
_Static_assert(sizeof(AIRCR_t) == 4U, "AIRCR_t is wrong size");

typedef struct
{
    unsigned : 1;
    unsigned SLEEPONEXIT : 1;
    unsigned SLEEPDEEP : 1;
    unsigned : 1;
    unsigned SEVONPEND : 1;
    unsigned : 27;
} SCR_t;
_Static_assert(sizeof(SCR_t) == 4U, "SCR_t is wrong size");

typedef struct
{
    unsigned NONBASETHRDENA : 1;
    unsigned USERSETMPEND : 1;
    unsigned : 1;
    unsigned UNALIGN_TRP : 1;
    unsigned DIV_0_TRP : 1;
    unsigned : 3;
    unsigned BFHFNMIGN : 1;
    unsigned STKALIGN : 1;
    unsigned : 22;
} CCR_t;
_Static_assert(sizeof(CCR_t) == 4U, "CCR_t is wrong size");

typedef struct
{
    unsigned PRI_4 : 8;
    unsigned PRI_5 : 8;
    unsigned PRI_6 : 8;
    unsigned : 8;
} SHPR1_t;
_Static_assert(sizeof(SHPR1_t) == 4U, "SHPR1_t is wrong size");

typedef struct
{
    unsigned : 24;
    unsigned PRI_11 : 8;
} SHPR2_t;
_Static_assert(sizeof(SHPR2_t) == 4U, "SHPR2_t is wrong size");

typedef struct
{
    unsigned : 16;
    unsigned PRI_14 : 8;
    unsigned PRI_15 : 8;
} SHPR3_t;
_Static_assert(sizeof(SHPR3_t) == 4U, "SHPR3_t is wrong size");

typedef struct
{
    unsigned MEMFAULTACT : 1;
    unsigned BUSFAULTACT : 1;
    unsigned : 1;
    unsigned USGFAULTACT : 1;
    unsigned : 3;
    unsigned SVCALLACT : 1;
    unsigned MONITORACT : 1;
    unsigned : 1;
    unsigned PENDSVACT : 1;
    unsigned SYSTICKACT : 1;
    unsigned USGFAULTPENDED : 1;
    unsigned MEMFAULTPENDED : 1;
    unsigned BUSFAULTPENDED : 1;
    unsigned SVCALLPENDED : 1;
    unsigned MEMFAULTENA : 1;
    unsigned BUSFAULTENA : 1;
    unsigned USGFAULTENA : 1;
    unsigned : 13;
} SHCSR_t;
_Static_assert(sizeof(SHCSR_t) == 4U, "SHCSR_t is wrong size");

typedef struct
{
    unsigned IACCVIOL : 1;
    unsigned DACCVIOL : 1;
    unsigned : 1;
    unsigned MUNSTKERR : 1;
    unsigned MSTKERR : 1;
    unsigned MLSPERR : 1;
    unsigned : 1;
    unsigned MMARVALID : 1;
    unsigned IBUSERR : 1;
    unsigned PRECISERR : 1;
    unsigned IMPRECISERR : 1;
    unsigned UNSTKERR : 1;
    unsigned STKERR : 1;
    unsigned LSPERR : 1;
    unsigned : 1;
    unsigned BFARVALID : 1;
    unsigned UNDEFINSTR : 1;
    unsigned INVSTATE : 1;
    unsigned INVPC : 1;
    unsigned NOCP : 1;
    unsigned : 4;
    unsigned UNALIGNED : 1;
    unsigned DIVBYZERO : 1;
    unsigned : 6;
} CFSR_t;
_Static_assert(sizeof(CFSR_t) == 4U, "CFSR_t is wrong size");

typedef struct
{
    unsigned : 1;
    unsigned VECTTBL : 1;
    unsigned : 28;
    unsigned FORCED : 1;
    unsigned DEBUGEVT : 1;
} HFSR_t;
_Static_assert(sizeof(HFSR_t) == 4U, "HFSR_t is wrong size");

typedef struct
{
    CPUID_t CPUID;
    ICSR_t ICSR;
    const void *VTOR;
    AIRCR_t AIRCR;
    SCR_t SCR;
    CCR_t CCR;
    SHPR1_t SHPR1;
    SHPR2_t SHPR2;
    SHPR3_t SHPR3;
    SHCSR_t SHCSR;
    CFSR_t CFSR;
    HFSR_t HFSR;
    unsigned int pad;
    void *MMFAR;
    void *BFAR;
    uint32_t AFSR;
} SCB_t;


// need the ifndef here so that we can ignore this code when compiling
// the firmware tests
#ifndef FWTEST
_Static_assert(sizeof(SCB_t) == 0xED3CU - 0xED00U + 4U, "SCB_t is wrong size");
#endif

extern volatile SCB_t SCB;

typedef struct
{
    unsigned CP0 : 2;
    unsigned CP1 : 2;
    unsigned CP2 : 2;
    unsigned CP3 : 2;
    unsigned CP4 : 2;
    unsigned CP5 : 2;
    unsigned CP6 : 2;
    unsigned CP7 : 2;
    unsigned : 4;
    unsigned CP10 : 2;
    unsigned CP11 : 2;
    unsigned : 8;
} CPACR_t;

extern volatile CPACR_t CPACR;

typedef struct
{
    unsigned LSPACT : 1;
    unsigned USER : 1;
    unsigned : 1;
    unsigned THREAD : 1;
    unsigned HFRDY : 1;
    unsigned MMRDY : 1;
    unsigned BFRDY : 1;
    unsigned : 1;
    unsigned MONRDY : 1;
    unsigned : 21;
    unsigned LSPEN : 1;
    unsigned ASPEN : 1;
} FPCCR_t;

typedef struct
{
    unsigned : 22;
    unsigned RMode : 2;
    unsigned FZ : 1;
    unsigned DN : 1;
    unsigned AHP : 1;
    unsigned : 5;
} FPDSCR_t;

typedef struct
{
    FPCCR_t CCR;
    void *CAR;
    FPDSCR_t DSCR;
} FP_t;

// need the ifndef here so that we can ignore this code when compiling
// the firmware tests
#ifndef FWTEST
_Static_assert(sizeof(FP_t) == 12U, "FP_t is wrong size");
#endif

extern volatile FP_t FP;

typedef struct
{
    unsigned C_DEBUGEN : 1;
    unsigned C_HALT : 1;
    unsigned C_STEP : 1;
    unsigned C_MASKINTS : 1;
    unsigned : 1;
    unsigned C_SNAPSTALL : 1;
    unsigned : 10;
    unsigned S_REGRDY : 1;
    unsigned S_HALT : 1;
    unsigned S_SLEEP : 1;
    unsigned S_LOCKUP : 1;
    unsigned : 4;
    unsigned S_RETIRE_ST : 1;
    unsigned S_RESET_ST : 1;
    unsigned : 6;
} DHCSR_t;
_Static_assert(sizeof(DHCSR_t) == 4U, "DHCSR_t is wrong size");

typedef struct
{
    unsigned REGSEL : 7;
    unsigned : 9;
    unsigned REGWnR : 1;
    unsigned : 15;
} DCRSR_t;
_Static_assert(sizeof(DCRSR_t) == 4U, "DCRSR_t is wrong size");

typedef struct
{
    unsigned VC_CORERESET : 1;
    unsigned : 3;
    unsigned VC_MMERR : 1;
    unsigned VC_NOCPERR : 1;
    unsigned VC_CHKERR : 1;
    unsigned VC_STATERR : 1;
    unsigned VC_BUSERR : 1;
    unsigned VC_INTERR : 1;
    unsigned VC_HARDERR : 1;
    unsigned : 5;
    unsigned MON_EN : 1;
    unsigned MON_PEND : 1;
    unsigned MON_STEP : 1;
    unsigned MON_REQ : 1;
    unsigned : 4;
    unsigned TRCENA : 1;
    unsigned : 7;
} DEMCR_t;
_Static_assert(sizeof(DEMCR_t) == 4U, "DEMCR_t is wrong size");

typedef struct
{
    DHCSR_t DHCSR;
    DCRSR_t DCRSR;
    uint32_t DCRDR;
    DEMCR_t DEMCR;
} DEBUG_t;
_Static_assert(sizeof(DEBUG_t) == 0xEDFCU - 0xEDF0U + 4U, "DEBUG_t is wrong size");

extern volatile DEBUG_t DEBUG;

#endif

/**
 * @}
 */
