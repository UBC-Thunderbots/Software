/* Define the three memory areas implemented by the STM32F4 series */
MEMORY {
	flash_region (RXAI) : ORIGIN = 0x08000000, LENGTH = 1M
	sram_region (RWXA) : ORIGIN = 0x20000000, LENGTH = 128K
	ccm_region (RWXA) : ORIGIN = 0x10000000, LENGTH = 64K
}

/* Lay out the data from the input object files in memory */
SECTIONS {
	/* The CPU always expects the exception vectors to be at the bottom of memory */
	.exception_vectors 0x08000000 : {
		KEEP(*(.exception_vectors))
	} >flash_region

	/* The CPU expects the interrupt vectors to be positioned directly after the exception vectors */
	.interrupt_vectors 0x08000040 : {
		KEEP(*(.interrupt_vectors))
	} >flash_region

	/* Add program code to the Flash following the vectors */
	.text : {
		*(.text .text.*)
	} >flash_region

	/* Read-only data can also live entirely in Flash */
	.rodata : {
		*(.rodata .rodata.*)
	} >flash_region

	/* Read-write data that is given its initial value normally but is not modified on a soft reset */
	.pdata : {
		*(.pdata .pdata.*)
	} >sram_region AT>flash_region

	/* Read-write non-zero-initialized data must live in RAM but a copy be stored in Flash for initialization */
	.data : {
		*(.data .data.*)
	} >sram_region AT>flash_region

	/* Code that for technical reasons must execute from RAM is also copied from Flash to ROM */
	.ramtext : {
		*(.ramtext .ramtext.*)
	} >sram_region AT>flash_region

	/* Allocate the last 256 kiB (two sectors) of Flash for a core dump */
	.coredump 0x080C0000 (NOLOAD) : {
		*(.coredump)
	} >flash_region

	/* Zero-initialized data can live entirely in SRAM and be wiped on boot */
	.bss : {
		*(.bss .bss.*)
	} >sram_region AT>sram_region

	/* Heap lives after BSS, 8-byte aligned, and fills all of the rest of RAM */
	. = ALIGN(8);
	linker_heap_size = 128K - (. - 0x20000000);
	.heap : ALIGN(8) {
		. = linker_heap_size;
        __heap_start__ = .;
        end = __heap_start__;
        _end = end;
        __end = end;
        KEEP(*(.heap))
        __heap_end__ = .;
        __HeapLimit = __heap_end__;
	} >sram_region AT>sram_region

	/* Stacks will go in CCM. */
	stacks (NOLOAD) : ALIGN(32) {
		*(stacks)
	} >ccm_region AT>ccm_region
	. = ALIGN(32);
}

/* Provide symbols that the startup code uses to find the various sections in memory to initialize the data and wipe the bss */
linker_data_vma = ADDR(.data);
linker_data_lma = LOADADDR(.data);
linker_data_size = SIZEOF(.data);

linker_pdata_vma = ADDR(.pdata);
linker_pdata_lma = LOADADDR(.pdata);
linker_pdata_size = SIZEOF(.pdata);

linker_ramtext_vma = ADDR(.ramtext);
linker_ramtext_lma = LOADADDR(.ramtext);
linker_ramtext_size = SIZEOF(.ramtext);

linker_bss_vma = ADDR(.bss);
linker_bss_size = SIZEOF(.bss);

linker_heap_vma = ADDR(.heap);
linker_heap_vma_end = ADDR(.heap) + SIZEOF(.heap);

/* Provide symbols for CPU registers */
ADC = 0x40012000;
ADC_Common = ADC + 0x300;
CRC = 0x40023000;
DMA1 = 0x40026000;
DMA2 = 0x40026400;
EXTI = 0x40013C00;
FLASH = 0x40023C00;
GPIO = 0x40020000;
U_ID = 0x1FFF7A10;
F_ID = 0x1FFF7A22;
IWDG = 0x40003000;
I2C1 = 0x40005400;
I2C2 = 0x40005800;
MPU = 0xE000ED90;
NVIC = 0xE000E100;
STIR = 0xE000EF00;
OTG_FS = 0x50000000;
OTG_FS_FIFO = 0x50001000;
DCMI = 0x50050000;
PWR_CR = 0x40007000;
RCC = 0x40023800;
RNG = 0x50060800;
ACTLR = 0xE000E008;
SCB = 0xE000ED00;
CPACR = 0xE000ED88;
DEBUG = 0xE000EDF0;
FP = 0xE000EF34;
SDIO = 0x40012C00;
SPI1 = 0x40013000;
SPI2 = 0x40003800;
SPI3 = 0x40003C00;
SYSCFG = 0x40013800;
SYSTICK = 0xE000E010;
TIM1 = 0x40010000;
TIM2 = 0x40000000;
TIM3 = 0x40000400;
TIM4 = 0x40000800;
TIM5 = 0x40000C00;
TIM6 = 0x40001000;
TIM7 = 0x40001400;
TIM8 = 0x40010400;
TIM9 = 0x40014000;
TIM10 = 0x40014400;
TIM11 = 0x40014800;
TIM12 = 0x40001800;
TIM13 = 0x40001C00;
TIM14 = 0x40002000;
USART1 = 0x40011000;
USART2 = 0x40004400;
USART3 = 0x40004800;
UART4 = 0x40004C00;
UART5 = 0x40005000;
USART6 = 0x40011400;
USART7 = 0x40007800;
USART8 = 0x40007C00;
