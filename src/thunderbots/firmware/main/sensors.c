#include "sensors.h"

extern inline sensors_gyro_data_t sensors_get_gyro(void);
extern inline sensors_accel_data_t sensors_get_accel(void);

sensors_gyro_data_t sensors_gyro_buffer;
sensors_accel_data_t sensors_accel_buffer;
