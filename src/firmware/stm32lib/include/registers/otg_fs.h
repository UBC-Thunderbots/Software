/**
 * \ingroup REG
 * \defgroup REGOTGFS USB on-the-go full speed
 * @{
 */
#ifndef STM32LIB_REGISTERS_OTG_FS_H
#define STM32LIB_REGISTERS_OTG_FS_H

#include <stdint.h>

typedef struct
{
    unsigned SRQSCS : 1;
    unsigned SRQ : 1;
    unsigned : 6;
    unsigned HNGSCS : 1;
    unsigned HNPRQ : 1;
    unsigned HSHNPEN : 1;
    unsigned DHNPEN : 1;
    unsigned : 4;
    unsigned CIDSTS : 1;
    unsigned DBCT : 1;
    unsigned ASVLD : 1;
    unsigned BSVLD : 1;
    unsigned : 12;
} OTG_FS_GOTGCTL_t;
_Static_assert(sizeof(OTG_FS_GOTGCTL_t) == 4U, "OTG_FS_GOTGCTL_t is wrong size");

typedef struct
{
    unsigned : 2;
    unsigned SEDET : 1;
    unsigned : 5;
    unsigned SRSSCHG : 1;
    unsigned HNSSCHG : 1;
    unsigned : 7;
    unsigned HNGDET : 1;
    unsigned ADTOCHG : 1;
    unsigned DBCDNE : 1;
    unsigned : 12;
} OTG_FS_GOTGINT_t;
_Static_assert(sizeof(OTG_FS_GOTGINT_t) == 4U, "OTG_FS_GOTGINT_t is wrong size");

typedef struct
{
    unsigned GINTMSK : 1;
    unsigned : 6;
    unsigned TXFELVL : 1;
    unsigned PTXFELVL : 1;
    unsigned : 23;
} OTG_FS_GAHBCFG_t;
_Static_assert(sizeof(OTG_FS_GAHBCFG_t) == 4U, "OTG_FS_GAHBCFG_t is wrong size");

typedef struct
{
    unsigned TOCAL : 3;
    unsigned : 3;
    unsigned PHYSEL : 1;
    unsigned : 1;
    unsigned SRPCAP : 1;
    unsigned HNPCAP : 1;
    unsigned TRDT : 4;
    unsigned : 15;
    unsigned FHMOD : 1;
    unsigned FDMOD : 1;
    unsigned CTXPKT : 1;
} OTG_FS_GUSBCFG_t;
_Static_assert(sizeof(OTG_FS_GUSBCFG_t) == 4U, "OTG_FS_GUSBCFG_t is wrong size");

typedef struct
{
    unsigned CSRST : 1;
    unsigned HSRST : 1;
    unsigned FCRST : 1;
    unsigned : 1;
    unsigned RXFFLSH : 1;
    unsigned TXFFLSH : 1;
    unsigned TXFNUM : 5;
    unsigned : 20;
    unsigned AHBIDL : 1;
} OTG_FS_GRSTCTL_t;
_Static_assert(sizeof(OTG_FS_GRSTCTL_t) == 4U, "OTG_FS_GRSTCTL_t is wrong size");

typedef struct
{
    unsigned CMOD : 1;
    unsigned MMIS : 1;
    unsigned OTGINT : 1;
    unsigned SOF : 1;
    unsigned RXFLVL : 1;
    unsigned NPTXFE : 1;
    unsigned GINAKEFF : 1;
    unsigned GOUTNAKEFF : 1;
    unsigned : 2;
    unsigned ESUSP : 1;
    unsigned USBSUSP : 1;
    unsigned USBRST : 1;
    unsigned ENUMDNE : 1;
    unsigned ISOODRP : 1;
    unsigned EOPF : 1;
    unsigned : 2;
    unsigned IEPINT : 1;
    unsigned OEPINT : 1;
    unsigned IISOIXFR : 1;
    unsigned IPXFR_INCOMPISOOUT : 1;
    unsigned : 2;
    unsigned HPRTINT : 1;
    unsigned HCINT : 1;
    unsigned PTXFE : 1;
    unsigned : 1;
    unsigned CIDSCHG : 1;
    unsigned DISCINT : 1;
    unsigned SRQINT : 1;
    unsigned WKUINT : 1;
} OTG_FS_GINTSTS_t;
_Static_assert(sizeof(OTG_FS_GINTSTS_t) == 4U, "OTG_FS_GINTSTS_t is wrong size");

typedef struct
{
    unsigned : 1;
    unsigned MMISM : 1;
    unsigned OTGINT : 1;
    unsigned SOFM : 1;
    unsigned RXFLVLM : 1;
    unsigned NPTXFEM : 1;
    unsigned GINAKEFFM : 1;
    unsigned GONAKEFFM : 1;
    unsigned : 2;
    unsigned ESUSPM : 1;
    unsigned USBSUSPM : 1;
    unsigned USBRST : 1;
    unsigned ENUMDNEM : 1;
    unsigned ISOODRPM : 1;
    unsigned EOPFM : 1;
    unsigned : 1;
    unsigned EPMISM : 1;
    unsigned IEPINT : 1;
    unsigned OEPINT : 1;
    unsigned IISOIXFRM : 1;
    unsigned IPXFRM_IISOOXFRM : 1;
    unsigned : 2;
    unsigned PRTIM : 1;
    unsigned HCIM : 1;
    unsigned PTXFEM : 1;
    unsigned : 1;
    unsigned CIDSCHGM : 1;
    unsigned DISCINT : 1;
    unsigned SRQIM : 1;
    unsigned WKUIM : 1;
} OTG_FS_GINTMSK_t;
_Static_assert(sizeof(OTG_FS_GINTMSK_t) == 4U, "OTG_FS_GINTMSK_t is wrong size");

typedef struct
{
    unsigned CHNUM : 4;
    unsigned BCNT : 11;
    unsigned DPID : 2;
    unsigned PKTSTS : 4;
    unsigned : 11;
} OTG_FS_GRXSTSR_host_t;
_Static_assert(sizeof(OTG_FS_GRXSTSR_host_t) == 4U,
               "OTG_FS_GRXSTSR_host_t is wrong size");
typedef struct
{
    unsigned EPNUM : 4;
    unsigned BCNT : 11;
    unsigned DPID : 2;
    unsigned PKTSTS : 4;
    unsigned FRMNUM : 4;
    unsigned : 7;
} OTG_FS_GRXSTSR_device_t;
_Static_assert(sizeof(OTG_FS_GRXSTSR_device_t) == 4U,
               "OTG_FS_GRXSTSR_device_t is wrong size");
typedef union {
    OTG_FS_GRXSTSR_host_t host;
    OTG_FS_GRXSTSR_device_t device;
} OTG_FS_GRXSTSR_t;
_Static_assert(sizeof(OTG_FS_GRXSTSR_t) == 4U, "OTG_FS_GRXSTSR_t is wrong size");

typedef struct
{
    unsigned RXFD : 16;
    unsigned : 16;
} OTG_FS_GRXFSIZ_t;
_Static_assert(sizeof(OTG_FS_GRXFSIZ_t) == 4U, "OTG_FS_GRXFSIZ_t is wrong size");

typedef struct
{
    unsigned NPTXFSA : 16;
    unsigned NPTXFD : 16;
} OTG_FS_HNPTXFSIZ_t;
_Static_assert(sizeof(OTG_FS_HNPTXFSIZ_t) == 4U, "OTG_FS_HNPTXFSIZ_t is wrong size");

typedef struct
{
    unsigned TX0FSA : 16;
    unsigned TX0FD : 16;
} OTG_FS_DIEPTXF0_t;
_Static_assert(sizeof(OTG_FS_DIEPTXF0_t) == 4U, "OTG_FS_DIEPTXF0_t is wrong size");

typedef struct
{
    unsigned NPTXFSAV : 16;
    unsigned NTPQXSAV : 8;
    unsigned NPTXQTOP : 7;
    unsigned : 1;
} OTG_FS_HNPTXSTS_t;
_Static_assert(sizeof(OTG_FS_HNPTXSTS_t) == 4U, "OTG_FS_HNPTXSTS_t is wrong size");

typedef struct
{
    unsigned : 16;
    unsigned PWRDWN : 1;
    unsigned : 1;
    unsigned VBUSASEN : 1;
    unsigned VBUSBSEN : 1;
    unsigned SOFOUTEN : 1;
    unsigned NOVBUSSENS : 1;
    unsigned : 10;
} OTG_FS_GCCFG_t;
_Static_assert(sizeof(OTG_FS_GCCFG_t) == 4U, "OTG_FS_GCCFG_t is wrong size");

typedef struct
{
    unsigned PTXSA : 16;
    unsigned PTXFSIZ : 16;
} OTG_FS_HPTXFSIZ_t;
_Static_assert(sizeof(OTG_FS_HPTXFSIZ_t) == 4U, "OTG_FS_HPTXFSIZ_t is wrong size");

typedef struct
{
    unsigned INEPTXSA : 16;
    unsigned INEPTXFD : 16;
} OTG_FS_DIEPTXFx_t;
_Static_assert(sizeof(OTG_FS_DIEPTXFx_t) == 4U, "OTG_FS_DIEPTXFx_t is wrong size");

typedef struct
{
    unsigned DSPD : 2;
    unsigned NZLSOHSK : 1;
    unsigned : 1;
    unsigned DAD : 7;
    unsigned PFIVL : 2;
    unsigned : 19;
} OTG_FS_DCFG_t;
_Static_assert(sizeof(OTG_FS_DCFG_t) == 4U, "OTG_FS_DCFG_t is wrong size");

typedef struct
{
    unsigned RWUSIG : 1;
    unsigned SDIS : 1;
    unsigned GINSTS : 1;
    unsigned GONSTS : 1;
    unsigned TCTL : 3;
    unsigned SGINAK : 1;
    unsigned CGINAK : 1;
    unsigned SGONAK : 1;
    unsigned CGONAK : 1;
    unsigned POPRGDNE : 1;
    unsigned : 20;
} OTG_FS_DCTL_t;
_Static_assert(sizeof(OTG_FS_DCTL_t) == 4U, "OTG_FS_DCTL_t is wrong size");

typedef struct
{
    unsigned SUSPSTS : 1;
    unsigned ENUMSPD : 2;
    unsigned EERR : 1;
    unsigned : 4;
    unsigned FNSOF : 14;
    unsigned : 10;
} OTG_FS_DSTS_t;
_Static_assert(sizeof(OTG_FS_DSTS_t) == 4U, "OTG_FS_DSTS_t is wrong size");

typedef struct
{
    unsigned XFRCM : 1;
    unsigned EPDM : 1;
    unsigned : 1;
    unsigned TOM : 1;
    unsigned ITTXFEMSK : 1;
    unsigned INEPNMM : 1;
    unsigned INEPNEM : 1;
    unsigned : 25;
} OTG_FS_DIEPMSK_t;
_Static_assert(sizeof(OTG_FS_DIEPMSK_t) == 4U, "OTG_FS_DIEPMSK_t is wrong size");

typedef struct
{
    unsigned XFRCM : 1;
    unsigned EPDM : 1;
    unsigned : 1;
    unsigned STUPM : 1;
    unsigned OTEPDM : 1;
    unsigned : 27;
} OTG_FS_DOEPMSK_t;
_Static_assert(sizeof(OTG_FS_DOEPMSK_t) == 4U, "OTG_FS_DOEPMSK_t is wrong size");

typedef struct
{
    unsigned IEPINT : 16;
    unsigned OEPINT : 16;
} OTG_FS_DAINT_t;
_Static_assert(sizeof(OTG_FS_DAINT_t) == 4U, "OTG_FS_DAINT_t is wrong size");

typedef struct
{
    unsigned IEPM : 16;
    unsigned OEPM : 16;
} OTG_FS_DAINTMSK_t;
_Static_assert(sizeof(OTG_FS_DAINTMSK_t) == 4U, "OTG_FS_DAINTMSK_t is wrong size");

typedef struct
{
    unsigned VBUSDT : 16;
    unsigned : 16;
} OTG_FS_DVBUSDIS_t;
_Static_assert(sizeof(OTG_FS_DVBUSDIS_t) == 4U, "OTG_FS_DVBUSDIS_t is wrong size");

typedef struct
{
    unsigned DVBUSP : 12;
    unsigned : 20;
} OTG_FS_DVBUSPULSE_t;
_Static_assert(sizeof(OTG_FS_DVBUSPULSE_t) == 4U, "OTG_FS_DVBUSPULSE_t is wrong size");

typedef struct
{
    unsigned MPSIZ : 2;
    unsigned : 13;
    unsigned USBAEP : 1;
    unsigned : 1;
    unsigned NAKSTS : 1;
    unsigned EPTYP : 2;
    unsigned : 1;
    unsigned STALL : 1;
    unsigned TXFNUM : 4;
    unsigned CNAK : 1;
    unsigned SNAK : 1;
    unsigned : 2;
    unsigned EPDIS : 1;
    unsigned EPENA : 1;
} OTG_FS_DIEPCTL0_t;
_Static_assert(sizeof(OTG_FS_DIEPCTL0_t) == 4U, "OTG_FS_DIEPCTL0_t is wrong size");

typedef struct
{
    unsigned XFRSIZ : 7;
    unsigned : 12;
    unsigned PKTCNT : 2;
    unsigned : 11;
} OTG_FS_DIEPTSIZ0_t;
_Static_assert(sizeof(OTG_FS_DIEPTSIZ0_t) == 4U, "OTG_FS_DIEPTSIZ0_t is wrong size");

typedef struct
{
    unsigned MPSIZ : 11;
    unsigned : 4;
    unsigned USBAEP : 1;
    unsigned EONUM_DPID : 1;
    unsigned NAKSTS : 1;
    unsigned EPTYP : 2;
    unsigned : 1;
    unsigned STALL : 1;
    unsigned TXFNUM : 4;
    unsigned CNAK : 1;
    unsigned SNAK : 1;
    unsigned SD0PID_SEVNFRM : 1;
    unsigned SODDFRM : 1;
    unsigned EPDIS : 1;
    unsigned EPENA : 1;
} OTG_FS_DIEPCTLx_t;
_Static_assert(sizeof(OTG_FS_DIEPCTLx_t) == 4U, "OTG_FS_DIEPCTLx_t is wrong size");
typedef struct
{
    unsigned XFRC : 1;
    unsigned EPDISD : 1;
    unsigned : 1;
    unsigned TOC : 1;
    unsigned ITTXFE : 1;
    unsigned : 1;
    unsigned INEPNE : 1;
    unsigned TXFE : 1;
    unsigned : 24;
} OTG_FS_DIEPINTx_t;
_Static_assert(sizeof(OTG_FS_DIEPINTx_t) == 4U, "OTG_FS_DIEPINTx_t is wrong size");
typedef struct
{
    unsigned XFRSIZ : 19;
    unsigned PKTCNT : 10;
    unsigned MCNT : 2;
    unsigned : 1;
} OTG_FS_DIEPTSIZx_t;
_Static_assert(sizeof(OTG_FS_DIEPTSIZx_t) == 4U, "OTG_FS_DIEPTSIZx_t is wrong size");
typedef struct
{
    unsigned INEPTFSAV : 16;
    unsigned : 16;
} OTG_FS_DTXFSTSx_t;
_Static_assert(sizeof(OTG_FS_DTXFSTSx_t) == 4U, "OTG_FS_DTXFSTSx_t is wrong size");
typedef struct
{
    OTG_FS_DIEPCTLx_t DIEPCTL;
    uint32_t pad1;
    OTG_FS_DIEPINTx_t DIEPINT;
    uint32_t pad2;
    OTG_FS_DIEPTSIZx_t DIEPTSIZ;
    uint32_t pad3;
    OTG_FS_DTXFSTSx_t DTXFSTS;
    uint32_t pad4;
} OTG_FS_DIEPx_wrapper_t;
_Static_assert(sizeof(OTG_FS_DIEPx_wrapper_t) == 0x20U,
               "OTG_FS_DIEPx_wrapper_t is wrong size");

typedef struct
{
    unsigned MPSIZ : 2;
    unsigned : 13;
    unsigned USBAEP : 1;
    unsigned : 1;
    unsigned NAKSTS : 1;
    unsigned EPTYP : 2;
    unsigned SNPM : 1;
    unsigned STALL : 1;
    unsigned : 4;
    unsigned CNAK : 1;
    unsigned SNAK : 1;
    unsigned : 2;
    unsigned EPDIS : 1;
    unsigned EPENA : 1;
} OTG_FS_DOEPCTL0_t;
_Static_assert(sizeof(OTG_FS_DOEPCTL0_t) == 4U, "OTG_FS_DOEPCTL0_t is wrong size");

typedef struct
{
    unsigned XFRSIZ : 7;
    unsigned : 12;
    unsigned PKTCNT : 1;
    unsigned : 9;
    unsigned STUPCNT : 2;
    unsigned : 1;
} OTG_FS_DOEPTSIZ0_t;
_Static_assert(sizeof(OTG_FS_DOEPTSIZ0_t) == 4U, "OTG_FS_DOEPTSIZ0_t is wrong size");

typedef struct
{
    unsigned MPSIZ : 11;
    unsigned : 4;
    unsigned USBAEP : 1;
    unsigned EONUM_DPID : 1;
    unsigned NAKSTS : 1;
    unsigned EPTYP : 2;
    unsigned SNPM : 1;
    unsigned STALL : 1;
    unsigned : 4;
    unsigned CNAK : 1;
    unsigned SNAK : 1;
    unsigned SD0PID_SEVENFRM : 1;
    unsigned SODDFRM_SD1PID : 1;
    unsigned EPDIS : 1;
    unsigned EPENA : 1;
} OTG_FS_DOEPCTLx_t;
_Static_assert(sizeof(OTG_FS_DOEPCTLx_t) == 4U, "OTG_FS_DOEPCTLx_t is wrong size");
typedef struct
{
    unsigned XFRC : 1;
    unsigned EPDISD : 1;
    unsigned : 1;
    unsigned STUP : 1;
    unsigned OTEPDIS : 1;
    unsigned : 1;
    unsigned B2BSTUP : 1;
    unsigned : 25;
} OTG_FS_DOEPINTx_t;
_Static_assert(sizeof(OTG_FS_DOEPINTx_t) == 4U, "OTG_FS_DOEPINTx_t is wrong size");
typedef struct
{
    unsigned XFRSIZ : 19;
    unsigned PKTCNT : 10;
    unsigned RXDPID_STUPCNT : 2;
    unsigned : 1;
} OTG_FS_DOEPTSIZx_t;
_Static_assert(sizeof(OTG_FS_DOEPTSIZx_t) == 4U, "OTG_FS_DOEPTSIZx_t is wrong size");
typedef struct
{
    OTG_FS_DOEPCTLx_t DOEPCTL;
    uint32_t reserved1;
    OTG_FS_DOEPINTx_t DOEPINT;
    uint32_t reserved3;
    OTG_FS_DOEPTSIZx_t DOEPTSIZ;
    uint32_t reserved5;
    uint32_t reserved6;
    uint32_t reserved7;
} OTG_FS_DOEPx_wrapper_t;
_Static_assert(sizeof(OTG_FS_DOEPx_wrapper_t) == 0x20U,
               "OTG_FS_DOEPx_wrapper_t is wrong size");

typedef struct
{
    unsigned STPPCLK : 1;
    unsigned GATEHCLK : 1;
    unsigned : 2;
    unsigned PHYSUSP : 1;
    unsigned : 27;
} OTG_FS_PCGCCTL_t;
_Static_assert(sizeof(OTG_FS_PCGCCTL_t) == 4U, "OTG_FS_PCGCCTL_t is wrong size");

typedef struct
{
    OTG_FS_GOTGCTL_t GOTGCTL;
    OTG_FS_GOTGINT_t GOTGINT;
    OTG_FS_GAHBCFG_t GAHBCFG;
    OTG_FS_GUSBCFG_t GUSBCFG;
    OTG_FS_GRSTCTL_t GRSTCTL;
    OTG_FS_GINTSTS_t GINTSTS;
    OTG_FS_GINTMSK_t GINTMSK;
    OTG_FS_GRXSTSR_t GRXSTSR;
    OTG_FS_GRXSTSR_t GRXSTSP;
    OTG_FS_GRXFSIZ_t GRXFSIZ;
    union {
        OTG_FS_HNPTXFSIZ_t HNPTXFSIZ;
        OTG_FS_DIEPTXF0_t DIEPTXF0;
    };
    OTG_FS_HNPTXSTS_t HNPTXSTS;
    uint32_t pad1[2U];
    OTG_FS_GCCFG_t GCCFG;
    uint32_t CID;
    uint32_t pad2[(0x100U - 0x40U) / 4U];
    OTG_FS_HPTXFSIZ_t HPTXFSIZ;
    OTG_FS_DIEPTXFx_t DIEPTXF[3U];
    uint32_t pad3[(0x400U - 0x110U) / 4U];
    uint32_t host_regs[(0x800U - 0x400U) / 4U];
    OTG_FS_DCFG_t DCFG;
    OTG_FS_DCTL_t DCTL;
    OTG_FS_DSTS_t DSTS;
    uint32_t pad4;
    OTG_FS_DIEPMSK_t DIEPMSK;
    OTG_FS_DOEPMSK_t DOEPMSK;
    OTG_FS_DAINT_t DAINT;
    OTG_FS_DAINTMSK_t DAINTMSK;
    uint32_t pad5[2U];
    OTG_FS_DVBUSDIS_t DVBUSDIS;
    OTG_FS_DVBUSPULSE_t DVBUSPULSE;
    uint32_t pad6;
    uint32_t DIEPEMPMSK;
    uint32_t pad7[(0x900U - 0x838U) / 4U];
    OTG_FS_DIEPCTL0_t DIEPCTL0;
    uint32_t pad8;
    OTG_FS_DIEPINTx_t DIEPINT0;
    uint32_t pad9;
    OTG_FS_DIEPTSIZ0_t DIEPTSIZ0;
    uint32_t pad10;
    OTG_FS_DTXFSTSx_t DTXFSTS0;
    uint32_t pad11;
    OTG_FS_DIEPx_wrapper_t DIEP[3U];
    uint32_t pad12[(0xB00U - 0x980U) / 4U];
    OTG_FS_DOEPCTL0_t DOEPCTL0;
    uint32_t pad13;
    OTG_FS_DOEPINTx_t DOEPINT0;
    uint32_t pad14;
    OTG_FS_DOEPTSIZ0_t DOEPTSIZ0;
    uint32_t pad15[3U];
    OTG_FS_DOEPx_wrapper_t DOEP[3U];
    uint32_t pad16[(0xE00U - 0xB80U) / 4U];
    OTG_FS_PCGCCTL_t PCGCCTL;
} OTG_FS_t;
_Static_assert(sizeof(OTG_FS_t) == 0xE04U, "OTG_FS_t is wrong size");

extern volatile OTG_FS_t OTG_FS;

typedef uint32_t OTG_FS_FIFO_t[0x1000 / 4];
typedef OTG_FS_FIFO_t OTG_FS_FIFOS_t[4];
extern volatile OTG_FS_FIFOS_t OTG_FS_FIFO;

#endif

/**
 * @}
 */
