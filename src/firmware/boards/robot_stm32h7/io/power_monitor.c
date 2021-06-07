#include "firmware/boards/robot_stm32h7/io/power_monitor.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

typedef struct PowerMonitor
{
    I2C_HandleTypeDef* i2c_handle;
    uint16_t dev_address;
} PowerMonitor_t;

static PowerMonitor_t* _power_monitor;
static bool initialized;

void io_power_monitor_init(I2C_TypeDef* i2c_instance, uint16_t dev_address,
                           uint16_t INA226_config)
{
    I2C_HandleTypeDef* i2c_handle = (I2C_HandleTypeDef*)malloc(sizeof(I2C_HandleTypeDef));

    i2c_handle->Instance             = i2c_instance;
    i2c_handle->Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    i2c_handle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2c_handle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2c_handle->Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    i2c_handle->Init.OwnAddress1     = 0x00;
    i2c_handle->Init.Timing          = 0x80200F73;
    if (HAL_I2C_Init(i2c_handle) != HAL_OK)
    {
        HAL_I2CEx_AnalogFilter_Config(i2c_handle, I2C_ANALOGFILTER_ENABLED);
    }
    INA226_setConfig(i2c_handle, dev_address, INA226_config);

    _power_monitor = (PowerMonitor_t*)malloc(sizeof(PowerMonitor_t));

    _power_monitor->i2c_handle  = i2c_handle;
    _power_monitor->dev_address = dev_address;
    initialized                 = true;
}

float io_power_monitor_getBatteryVoltage(void)
{
    return 22.0f;
}

void io_power_monitor_destroy(void)
{
    assert(initialized);
    free(_power_monitor->i2c_handle);
    _power_monitor->i2c_handle = NULL;
    free(_power_monitor);
    _power_monitor = NULL;
    initialized    = false;
}
