#include "imu.h"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>

#include "software/logger/logger.h"

// these functions taken from
// https://git.kernel.org/pub/scm/utils/i2c-tools/i2c-tools.git/tree/lib/smbus.c
__s32 i2c_smbus_access(int file, char read_write, __u8 command, int size,
                       union i2c_smbus_data* data)
{
    struct i2c_smbus_ioctl_data args;
    __s32 err;

    args.read_write = read_write;
    args.command    = command;
    args.size       = size;
    args.data       = data;

    err = ioctl(file, I2C_SMBUS, &args);
    if (err == -1)
        err = -errno;
    return err;
}

__s32 i2c_smbus_read_byte_data(int file, __u8 command)
{
    union i2c_smbus_data data;
    int err;

    err = i2c_smbus_access(file, I2C_SMBUS_READ, command, I2C_SMBUS_BYTE_DATA, &data);
    if (err < 0)
        return err;

    return 0x0FF & data.byte;
}

__s32 i2c_smbus_write_byte_data(int file, __u8 command, __u8 value)
{
    union i2c_smbus_data data;
    data.byte = value;
    return i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_BYTE_DATA, &data);
}

ImuService::ImuService() : initialized_(false)
{
    // Establish connection to the IMU and verify that the who am I pin is correct.
    file_descriptor_ = open(IMU_DEVICE.c_str(), O_RDWR);
    int ret          = ioctl(file_descriptor_, I2C_SLAVE_FORCE, 0x6b);
    if (ret < 0)
    {
        LOG(WARNING)
            << "Failed to initialize the IMU: failed to establish i2c connection.";
        return;
    }
    if (i2c_smbus_read_byte_data(file_descriptor_, WHOAMI_REG) != 106)
    {
        LOG(WARNING) << "Failed to initialize the IMU: WHOAMI register " << static_cast<int>(WHOAMI_REG)
                     << " read incorrectly.";
        return;
    }
    // Attempt to enable gyro and accelerometer, checking that writes are successful
    // See lsm6dsl datasheet for how to set these registers.
    i2c_smbus_write_byte_data(file_descriptor_, ACCEL_CONTROL_REG, 0b01000000);
    if (i2c_smbus_read_byte_data(file_descriptor_, ACCEL_CONTROL_REG) != 0b01000000)
    {  // write unsuccessful
        LOG(WARNING)
            << "Failed to initialize the IMU: writing to accelerometer config register "
            << static_cast<int>(ACCEL_CONTROL_REG) << " unsuccessful";
        return;
    }
    // Set Gyroscope output data rate to 208 Hz, setting FS to 1000 dps (pg 61 of
    // datasheet for lsm6dsl, pg 21)
    i2c_smbus_write_byte_data(file_descriptor_, GYRO_CONTROL_REG, 0b01011000);
    if (i2c_smbus_read_byte_data(file_descriptor_, GYRO_CONTROL_REG) != 0b01011000)
    {  // write unsuccessful
        LOG(WARNING) << "Failed to initialize the IMU: writing to gyroscope config register "
                     << "unsuccessful";
        return;
    }

    // Enable Gyro digital LPF1 and set bandwidth to ~65Hz (FTYPE=000)
    // CTRL4_C: bit 1 (LPF1_SEL_G) = 1
    i2c_smbus_write_byte_data(file_descriptor_, CTRL4_C, 0b00000010);
    // CTRL6_C: bits 2:0 (FTYPE) = 000
    i2c_smbus_write_byte_data(file_descriptor_, CTRL6_C, 0b00000000);

    // Enable Accelerometer digital LPF2
    // CTRL8_XL: bit 7 (LPF2_XL_EN) = 1
    i2c_smbus_write_byte_data(file_descriptor_, CTRL8_XL, 0b10000000);

    initialized_ = true;
    LOG(INFO) << "Initialized IMU! Calibrating...";
    degrees_error_ = 0;
    // get 100 sample average of stationary reading, so all future readings can be
    // corrected
    double sum        = 0;
    int valid_samples = 0;
    for (int i = 0; i < 100; i++)
    {
        auto poll = pollHeadingRate();
        if (poll.has_value())
        {
            sum += poll->toDegrees();
            valid_samples++;
        }
        usleep(50000);
    }

    if (valid_samples > 0)
    {
        degrees_error_ = sum / valid_samples;
        LOG(INFO) << "IMU Calibration complete. Offset: " << degrees_error_ << " dps";
    }
    else
    {
        LOG(WARNING) << "IMU Calibration failed: no valid samples received. Heading "
                        "stability will be poor.";
        initialized_ = false;
    }
}

std::optional<AngularVelocity> ImuService::pollHeadingRate()
{
    if (!initialized_)
    {
        return std::nullopt;
    }

    // Two separate registers for the Gyro output data.
    int least_significant = i2c_smbus_read_byte_data(file_descriptor_, YAW_LEAST_SIG_REG);
    int most_significant  = i2c_smbus_read_byte_data(file_descriptor_, YAW_MOST_SIG_REG);

    if (least_significant < 0 || most_significant < 0)
    {
        return std::nullopt;
    }

    int16_t full_word = (static_cast<uint8_t>(most_significant) << 8) |
                        static_cast<uint8_t>(least_significant);

    double degrees_per_sec = static_cast<double>(full_word) /
                             static_cast<double>(SHRT_MAX) * IMU_FULL_SCALE_DPS;
    return AngularVelocity::fromDegrees(degrees_per_sec - degrees_error_);
}
