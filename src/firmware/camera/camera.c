#include "i2c.h"
#include "camera.h"
#include <gpio.h>
#include <stdbool.h>
#include <stdint.h>
#include <rcc.h>
#include "pins.h"

#define CAM_IP 0x21

bool camera_init (cam_setting_t* setting_list, unsigned int list_size)
{

	uint16_t delay = 250;
	volatile uint16_t j;
	if (!i2c_init())
	{
		//ERROR: Failure to initialize I2C interface
		return false;
	}

	// send all the settings
	volatile unsigned int i = 0;

	for (i = 0; i < list_size; i++)
	{
		uint8_t reg_val = camera_read_reg(setting_list[i].reg);
		
		if (setting_list[i].val == 1)
		{
			reg_val = reg_val | (1 << setting_list[i].position);
		}
		else
		{
			reg_val = reg_val & ~(1 << setting_list[i].position);
		}
		camera_write2register(setting_list[i].reg, reg_val);
		setting_list[i].final_reg_val = reg_val;

		for( j = 0; j < delay; j++) {asm volatile("nop");};
	}

	// Check if all the commands went through
	for (i = 0; i < list_size; i++)
	{
		if (camera_read_reg(setting_list[i].reg) != setting_list[i].final_reg_val)
		{
			return false;
		}

		for( j = 0; j < delay; j++) {asm volatile("nop");};
	}
	return true;
}


//NOTE: this function will never fail because NACK is assumed to represent success
bool camera_write2register(uint8_t addr, uint8_t setting)
{
	// Start write 
	if (!i2c_start_com(CAM_IP, 0x0))
	{
		//ERROR: Failure to start I2C transmission to camera
		return false;
	}	
	i2c_write_data(addr);
	i2c_write_data(setting);
	i2c_stop_tx();
	return true;
}


//NOTE: Each read needs a stop condition
uint8_t camera_read_reg(uint8_t reg)
{
	// Write the address to device
	if (!i2c_start_com(CAM_IP, 0x0))
	{
		//ERROR: Failure to start I2C transmission to camera
		return false;
	}
	i2c_write_data(reg);
	i2c_stop_tx();

	// Receive data at address
	if (!i2c_start_com(CAM_IP, 0x1))
	{
		//ERROR: Failure to start receiving from camera
		return false;
	}
//	i2c_write_data(reg);

	uint8_t data = i2c_read_data();
	i2c_stop_rx();

	return data;
}

