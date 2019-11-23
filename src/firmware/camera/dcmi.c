#include "dcmi.h"

#include <gpio.h>
#include <nvic.h>
#include <rcc.h>
#include <registers/dcmi.h>
#include <registers/dma.h>
#include <stdbool.h>
#include <stdint.h>

#define FRAME_DONE GPIOA, 3U
#define DIRECT_MODE_ERR GPIOA, 5U
#define RAW_FRAME_DONE GPIOC, 3U


void dma2_stream1_isr(void)
{
    DMA_LISR_t lisr   = DMA2.LISR;
    DMA_LIFCR_t lifcr = {0};
    gpio_reset(DIRECT_MODE_ERR);

    if (lisr.TCIF1 == 1)  // Transfer complete
    {
        gpio_toggle(FRAME_DONE);
        lifcr.CTCIF1 = 1;
    }

    if (lisr.DMEIF1 == 1)  // Direct mode error
    {
        gpio_set(DIRECT_MODE_ERR);
        lifcr.CDMEIF1 = 1;
    }
    DMA2.LIFCR = lifcr;
}

void dcmi_isr(void)
{
    DCMI_RIS_t ris = DCMI.RIS;
    DCMI_ICR_t icr = {0};

    if (ris.FRAME_RIS == 1)
    {
        gpio_toggle(RAW_FRAME_DONE);
        icr.FRAME_ISC = 1;
    }

    if (ris.LINE_RIS == 1)
    {
        //		gpio_toggle(
        icr.LINE_ISC = 1;
    }

    if (ris.OVR_RIS == 1)
    {
        gpio_set(DIRECT_MODE_ERR);
        icr.OVR_ISC = 1;
    }

    DCMI.ICR = icr;
}
/*
void dcmi_line_isr(void)
{
    DCMI_RIS
}
*/

// DCMI NVIC = 0x00000178
// DMA2 NVIC = 0x00000124
bool dcmi_init(void)
{
    rcc_enable_reset(AHB2, DCMI);

    DCMI_CR_t cr   = {.HSPOL  = 0,  // HSYNC active low
                    .VSPOL  = 1,  // VSYNC active high
                    .PCKPOL = 1,  // Capture on rising edge of PCLK
                    .CM     = 1};
    DCMI.CR        = cr;
    DCMI_IER_t ier = {
        .FRAME_IE = 1,
        .OVR_IE   = 1,
    };
    //.LINE_IE = 1};
    DCMI.IER = ier;

    DCMI.CR.ENABLE = 1;

    DCMI_CR_t dummy_read = DCMI.CR;  // Flush APB buffer to ensure order

    NVIC.ISER[NVIC_IRQ_DCMI / 32U] = 1U << (NVIC_IRQ_DCMI % 32U);

    return true;
}

bool dcmi_dma_init(void)
{
    rcc_enable_reset(AHB1, DMA2);

    DMA_SxCR_t cr = {
        .CHSEL = 1,
        //					  .DBM = 1, 	// Double buffer
        .PL    = 0b11,  // Very high priority
        .MSIZE = 0b10,  // 32bit
        .PSIZE = 0b10,  // 32bit
        .MINC  = 1,     // Memory increments automatically
                        //					  .CIRC = 1, 	// Double buffer iff circular mode
                        // enabled
        .DIR   = 0b00,  // Peripheral to memory
        .TCIE  = 1,     // Transfer complete interrupt enabled
        .DMEIE = 1      // Direct mode error interrupt enabled
    };
    //	uint32_t ndtr = 12672;   		// 50688 / 4 = # of items for transfer
    uint32_t ndtr = 25344;  // 50688 / 4 = # of items for transfer

    dma_stream_t stream_cfg = {
        .CR = cr, .NDTR = ndtr, .PAR = &(DCMI.DR), .M0AR = &(img_buffer1[0]),
        //								.M1AR = &(img_buffer2[0])
    };

    DMA2.streams[1].CR   = cr;
    DMA2.streams[1].NDTR = ndtr;
    DMA2.streams[1].PAR  = &(DCMI.DR);
    DMA2.streams[1].M0AR = &(img_buffer1[0]);
    // DMA2.streams[1].M1AR = &(img_buffer2[0]);

    DCMI_CR_t dummy_read = DCMI.CR;  // Flush APB buffer to ensure order

    NVIC.ISER[NVIC_IRQ_DMA2_STREAM1 / 32U] = 1U << (NVIC_IRQ_DMA2_STREAM1 % 32U);

    return true;
}


void dcmi_sync_with_new_frame(void)
{
    DMA2.streams[1].CR.EN = 1;  // Enable DMA
    /*while (DCMI.RIS.FRAME_RIS == 0); // In case the frame interrupt is not asserted yet
    DCMI.ICR.FRAME.FRAME_ICR= 1;     // When it is, clear the status
    while (DCMI.RIS.FRAME_RIS == 0); // Wait until another frame is captured
    DCMI.ICR.FRAME.FRAME_ICR= 1;     // This time we are synchronized. Time to enable
    everything
    */
    DCMI.CR.CAPTURE = 1;
}
