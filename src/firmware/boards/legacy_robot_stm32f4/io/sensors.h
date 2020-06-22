#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

#include "icb.h"

typedef struct __attribute__((packed))
{
    /**
     * \brief Whether or not the gyro is working.
     *
     * If this has the value 0, there is no gyro data available and the \ref
     * data.failed_chip_id member contains the result of reading the chip ID. If this has
     * the value 1, there is gyro data available in the \ref data.reading member.
     */
    uint8_t status;
    /**
     * \brief The gyro data.
     */
    union __attribute__((packed))
    {
        /**
         * \brief A successful gyro reading, if the gyro is working.
         */
        struct __attribute__((packed))
        {
            int16_t x;
            int16_t y;
            int16_t z;
        } reading;
        /**
         * \brief The result of trying to read the chip ID, if the gyro is not working.
         */
        uint8_t failed_chip_id;
    } data;
} sensors_gyro_data_t;
_Static_assert(sizeof(sensors_gyro_data_t) == 7U, "Gyro data struct not correct size");

typedef struct __attribute__((packed))
{
    /**
     * \brief Whether or not the accelerometer is working.
     *
     * If this has the value 0, there is no gyro data available and the \ref
     * data.failed_chip_id member contains the result of reading the chip ID. If this has
     * the value 1, there is gyro data available in the \ref data.reading member.
     */
    uint8_t status;
    /**
     * \brief The accelerometer data.
     */
    union __attribute__((packed))
    {
        /**
         * \brief A succesful accelerometer reading, if the gyro is working.
         */
        struct __attribute__((packed))
        {
            int16_t x;
            int16_t y;
            int16_t z;
        } reading;
        /**
         * \brief The result of trying to read the chip ID, if the gyro is not working.
         */
        uint8_t failed_chip_id;
    } data;
} sensors_accel_data_t;
_Static_assert(sizeof(sensors_accel_data_t) == 7U, "Accel data struct not correct size");

inline sensors_gyro_data_t sensors_get_gyro(void)
{
    extern sensors_gyro_data_t sensors_gyro_buffer;
    icb_receive(ICB_COMMAND_SENSORS_GET_GYRO, &sensors_gyro_buffer,
                sizeof(sensors_gyro_buffer));
    return sensors_gyro_buffer;
}

inline sensors_accel_data_t sensors_get_accel(void)
{
    extern sensors_accel_data_t sensors_accel_buffer;
    icb_receive(ICB_COMMAND_SENSORS_GET_ACCEL, &sensors_accel_buffer,
                sizeof(sensors_accel_buffer));
    return sensors_accel_buffer;
}

#endif
