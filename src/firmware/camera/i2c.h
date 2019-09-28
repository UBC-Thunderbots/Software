#include <stdint.h>
#include <stdbool.h>

#ifndef I2C_H
#define I2C_H


bool i2c_init (void);
bool i2c_start_com(uint8_t, uint8_t);
void i2c_stop_tx(void);
void i2c_stop_rx(void);
void i2c_write_data(uint8_t);
uint8_t i2c_read_data(void);


#endif
