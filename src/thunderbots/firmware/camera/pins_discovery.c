#include "pins.h"

/*
	***DCMI***	(100)	(64)
	PIXCLK 	= PA6
	HSYNC  	= PA4
	VSYNC  	= PB7
	D0		= PC6
	D1		= PC7
	D2		= PC8
	D3		= PE1		PC9 (remember to switch XCLK)
	D4 		= PE4		PC11
	D5		= PB6
	D6		= PE5		PB8
	D7		= PE6		PB9

	***I2C***
	SDA		= PB9		PB11	(I2C2)
	SCL		= PB8		PB10	(I2C2)

	***MISC***
	XCLK	= PC9		PA8

*/
const gpio_init_pin_t PINS_INIT_100[5U][16U] = {
	{
		// PA0 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PA1 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PA2 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PA3 = FRAME_DONE
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_100, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PA4 = DCMI Horizontal sync
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_100, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 13 },
		// PA5 = DIRECT_MODE_ERR
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PA6 = DCMI Pixel clock
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_100, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 13 },
		// PA7 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PA8 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PA9 = GPIO input additional function OTG VBUS
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PA10 = alternate function USB ID
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_50, .pupd = GPIO_PUPD_PD, .od = 0, .af = 10 },
		// PA11 = alternate function USB D−
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_50, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 10 },
		// PA12 = alternate function USB D+
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_50, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 10 },
		// PA13 = alternate function serial wire debug data
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PU, .od = 0, .af = 0 },
		// PA14 = alternate function serial wire debug clock
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PA15 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
	},
	{
		// PB0 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PB1 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PB2 = shorted to VSS
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_OD, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PB3 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PB4 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PB5 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PB6 = DCMI Data 5
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_100, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 13 },
		// PB7 = DCMI Vertical sync
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_100, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 13 },
		// PB8 = Alternate function I2C SCL
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_OD, .ospeed = GPIO_OSPEED_50, .pupd = GPIO_PUPD_PU, .od = 0, .af = 4 },
		// PB9 = Alternate function I2C SDA
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_OD, .ospeed = GPIO_OSPEED_100, .pupd = GPIO_PUPD_PU, .od = 0, .af = 4 },
		// PB10 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PB11 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PB12 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PB13 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PB14 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PB15 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
	},
	{
		// PC0 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PC1 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PC2 = N/C	
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PC3 = N/C
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PC4 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PC5 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PC6 = DCMI Data 0
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_100, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 13 },
		// PC7 = DCMI Data 1
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_100, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 13 },
		// PC8 = DCMI Data 2
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_100, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 13 },
		// PC9 = XCLK for camera
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_50, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PC10 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PC11 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PC12 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PC13 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PC14 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PC15 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
	},
	{
		// PD0 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PD1 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PD2 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PD3 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PD4 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PD5 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PD6 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PD7 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PD8 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PD9 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PD10 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PD11 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PD12 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PD13 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PD14 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PD15 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
	},
	{
		// PE0 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PE1 = DCMI Data 3
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_100, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 13 },
		// PE2 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PE3 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PE4 = DCMI Data 4
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_100, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 13 },
		// PE5 = DCMI Data 6
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_100, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 13 },
		// PE6 = DCMI Data 7
		{ .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_100, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 13 },
		// PE7 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PE8 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PE9 = SUCCESS_INDICATOR
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PE10 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PE11 = FAIL_INDICATOR
		{ .mode = GPIO_MODE_OUT, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_NONE, .od = 0, .af = 0 },
		// PE12 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PE13 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PE14 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
		// PE15 = N/C
		{ .mode = GPIO_MODE_IN, .otype = GPIO_OTYPE_PP, .ospeed = GPIO_OSPEED_2, .pupd = GPIO_PUPD_PD, .od = 0, .af = 0 },
	}
};
