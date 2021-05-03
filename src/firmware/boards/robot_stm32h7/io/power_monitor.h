#pragma once

#include "firmware/boards/robot_stm32h7/io/INA226_power_monitor_driver.h"

/**
 * Initializes the power monitor
 *
 * @param i2c_instance The I2C Instance to use to talk to the power monitor
 * @param dev_address the device address
 * @param INA226_config the INA226 config word
 */
void io_power_monitor_init(I2C_TypeDef *i2c_instance, uint16_t dev_address,
                           uint16_t INA226_config);

/**
 * Gets the battery voltage
 *
 * @param the power monitor to get voltage with
 *
 * @param the current voltage of the battery in volts
 */
float io_power_monitor_getBatteryVoltage(void);

/**
 * Destroy the monitor
 *
 * @param the power monitor to destroy
 */
void io_power_monitor_destroy(void);
