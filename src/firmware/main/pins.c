#include "pins.h"

const gpio_init_pin_t PINS_INIT[4U][16U] = {
	{
		// PA0 = alternate function TIM5 motor 3 encoder 0
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 2 },
		// PA1 = alternate function TIM5 motor 3 encoder 1
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 2 },
		// PA2 = alternate function TIM9 chicker chip
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 3 },
		// PA3 = GPIO input FPGA DONE
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PA4 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PA5 = alternate function SPI1 SCK ICB clock
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_50, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 5 },
		// PA6 = alternate function SPI1 MISO ICB MISO
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 5 },
		// PA7 = alternate function SPI1 MOSI ICB MOSI
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_50, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 5 },
		// PA8 = GPIO input SD card detect
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PA9 = GPIO input additional function OTG VBUS
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PA10 = GPIO output laser drive
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PA11 = alternate function USB D−
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_50, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 10 },
		// PA12 = alternate function USB D+
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_50, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 10 },
		// PA13 = alternate function serial wire debug data
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PU, .od = 0, .af = 0 },
		// PA14 = alternate function serial wire debug clock
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PA15 = alternate function TIM2 motor 0 encoder 0
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 1 },
	},
	{
		// PB0 = EXTI0 ICB IRQ
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PB1 = GPIO input FPGA INIT_B
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PU, .od = 0, .af = 0 },
		// PB2 = shorted to VSS
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_OD, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PB3 = alternate function TIM2 motor 0 encoder 1
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 1 },
		// PB4 = GPIO output LPS drive 2
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PB5 = GPIO output LPS drive 3
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PB6 = alternate function TIM4 motor 1 encoder 0
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 2 },
		// PB7 = alternate function TIM4 motor 1 encoder 1
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 2 },
		// PB8 = alternate function TIM10 chicker charge
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 3 },
		// PB9 = alternate function TIM11 chicker kick
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 3 },
		// PB10 = GPIO output ICB chip select
		// FPGA toggles this during configuration, so make it undriven, then make it push-pull after configuration for ICB.
		// Include a pull-up resistor so that, after configuration, any further post-bitstream data clocked into the ICB will happen with CS=1 and thus be ignored.
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_OD, .ospeed = GPIO_OSPEED_50, .pupd = GPIO_PUPD_PU, .od = 1, .af = 0, .unlock = true },
		// PB11 = GPIO output FPGA PROGRAM_B
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PB12 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PB13 = GPIO output status LED
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 1, .af = 0 },
		// PB14 = GPIO output link LED
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 1, .af = 0 },
		// PB15 = GPIO output charged LED
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 1, .af = 0 },
	},
	{
		// PC0 = GPIO output LPS drive 1
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PC1 = GPIO output LPS drive 0
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PC2 = analogue ADC123_IN12 battery voltage
		{ .mode = GPIO_MODE_AN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PC3 = analogue ADC123_IN13 capacitor voltage
		{ .mode = GPIO_MODE_AN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PU, .od = 0, .af = 0 },
		// PC4 = analogue ADC12_IN14 break beam voltage
		{ .mode = GPIO_MODE_AN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PC5 = analogue ADC12_IN15 LPS voltage
		{ .mode = GPIO_MODE_AN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PC6 = alternate function TIM3 motor 2 encoder 0
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 2 },
		// PC7 = alternate function TIM3 motor 2 encoder 1
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 2 },
		// PC8 = alternate function SDIO D0
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_25, .pupd = GPIO_PUPD_PU, .od = 0, .af = 12 },
		// PC9 = alternate function SDIO D1
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_25, .pupd = GPIO_PUPD_PU, .od = 0, .af = 12 },
		// PC10 = alternate function SDIO D2
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_25, .pupd = GPIO_PUPD_PU, .od = 0, .af = 12 },
		// PC11 = alternate function SDIO D3
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_25, .pupd = GPIO_PUPD_PU, .od = 0, .af = 12, .unlock = true },
		// PC12 = alternate function SDIO clock
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_25, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 12 },
		// PC13 = GPIO output HV power switch
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PC14 = GPIO output logic power switch
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 1, .af = 0 },
		// PC15 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
	},
	{
		// PD0 = unimplemented on package
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PD1 = unimplemented on package
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PD2 = alternate function SDIO command
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_25, .pupd = GPIO_PUPD_PU, .od = 0, .af = 12 },
		// PD3 = unimplemented on package
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PD4 = unimplemented on package
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PD5 = unimplemented on package
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PD6 = unimplemented on package
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PD7 = unimplemented on package
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PD8 = unimplemented on package
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PD9 = unimplemented on package
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PD10 = unimplemented on package
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PD11 = unimplemented on package
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PD12 = unimplemented on package
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PD13 = unimplemented on package
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PD14 = unimplemented on package
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PD15 = unimplemented on package
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
	}
};

