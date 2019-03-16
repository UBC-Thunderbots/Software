
#include "i2c.h"
#include "rcc.h"
#include <registers/i2c.h>
#include <stdint.h>
#include <stdbool.h>
#include <gpio.h>


#define OV7660_FS 0
//Tpclk = 25ns for APB1 frequency = 40MHz
//T_i2c = 0.000001s => f_i2c = 100kHz; T_i2c/Tpclk = f_pclk/f_i2c = 400
//T_high = T_low => 400 * 2 = 800
#define OV7660_CCR 800

// Camera: max 300ns
// Tpclk1 = 25ns
// TRISE = T_rise or T_fall / Tpclk1 + 1 = 300/25 + 1 = 13
#define OV7660_TRISE 13


bool i2c_init(void) {

	// RCC
	rcc_enable_reset(APB1, I2C1);

	I2C_CR1_t cr1 = { .ACK = 1 };

	I2C1.CR1 = cr1;
	
	// Set up peripheral input clock frequency to 40 MHz so that I2C
	I2C_CR2_t cr2 = {
						.FREQ = 40,
					};
	I2C1.CR2 = cr2;

	// Configure clock control registers
	I2C_CCR_t ccr = {
						.FS = OV7660_FS,
						.DUTY = 0,
						.CCR = OV7660_CCR,
						//.FS = OV7660_FS,		// Fast mode (400kHz)
						//.DUTY = 1,		// Duty cycle: 16:9. (In order to reach 400kHz)
						//// 400kHz -> 2.5us periods. 1600 ns : 900 ns. 
						//// T_high = T_pclk*CCR*9 = 900 ns
						//// T_pclk = 25ns (40MHz)

						//
					};
	I2C1.CCR = ccr;



	// Configure the rise time register
	I2C_TRISE_t trise = {
						.TRISE = OV7660_TRISE,
						};
	I2C1.TRISE = trise;

	I2C_OAR1_t oar1 = { 
							.defaultval = 1,
						};

	I2C1.OAR1 = oar1;
	// Program the CR1 to enable the peripheral and set start bit
	I2C1.CR1.PE = 1;

	return true;
}


// read = 0 for write, read = 1 for read
bool i2c_start_com(uint8_t device, uint8_t read)
{
	volatile int i = 0;
	I2C1.CR1.START = 1;
	while (I2C1.CR1.START);

	while(!I2C1.SR1.SB);

	// Write slave address to DR register, with LSB = 0 for write
	I2C_DR_t DR = {0};
	DR.DR = (device << 1) | read;
	I2C1.DR = DR;

	bool done = false;
	
	// Wait until either ack failure or address is sent (acked)
	while (!done)
	{
		I2C_SR1_t sr1 = I2C1.SR1;
		if (sr1.AF || sr1.ADDR)	// ICCB doesn't care about ack-ing
		{
			done = true;
			// Clear possible errors
			I2C_SR1_t tmp = {0};
			I2C1.SR1 = tmp;
			// Read SR1 followed by reading SR2 to clear ADDR
			I2C_SR2_t sr2 = I2C1.SR2;
			// Check if master is in the right mode (transmitter or receiver)
			//if (sr2.TRA != !read)
			//	return false;
		}
	}
	return true;
}

// Send stop condition for transmission
void i2c_stop_tx(void)
{
	I2C_SR1_t sr1 = I2C1.SR1;
	while (!sr1.TxE && !sr1.BTF)
	{
		sr1 = I2C1.SR1;
	}

	I2C_CR1_t cr1 = I2C1.CR1;
	cr1.STOP = 1;

	I2C1.CR1 = cr1;
	while (I2C1.CR1.STOP == 1);
}

// Send stop condition for receiving
void i2c_stop_rx(void)
{
	I2C_CR1_t cr1 = I2C1.CR1;
	cr1.STOP = 1;

	I2C1.CR1 = cr1;

	while (I2C1.CR1.STOP == 1);
}

// Procedure for sending one byte over I2C
//NOTE: this funtion will never fail because NACK represents success
void i2c_write_data(uint8_t data)
{
	bool done = false;
	I2C_SR1_t sr1;

	I2C_DR_t DR = {0};
	DR.DR = data;
	I2C1.DR = DR;

	while (!done)
	{
		sr1 = I2C1.SR1;
		
		if (sr1.TxE || sr1.BTF || sr1.AF) 
		{
			done = true;
			I2C_SR1_t tmp = {0};
			I2C1.SR1 = tmp;
		}
	}
}

uint8_t i2c_read_data( void )
{
	I2C_SR1_t sr1;

	while(1)
	{
		sr1 = I2C1.SR1;
		if (sr1.RxNE || sr1.BTF)
		{
			return I2C1.DR.DR;
		}	
	}
}

