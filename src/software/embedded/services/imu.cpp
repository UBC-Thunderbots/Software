//
// Created by mkhl on 2024-11-13.
//

#include "imu.h"
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <fcntl.h>
#include "software/logger/logger.h"

// these functions taken from https://git.kernel.org/pub/scm/utils/i2c-tools/i2c-tools.git/tree/lib/smbus.c
__s32 i2c_smbus_access(int file, char read_write, __u8 command,
                       int size, union i2c_smbus_data *data)
{
    struct i2c_smbus_ioctl_data args;
    __s32 err;

    args.read_write = read_write;
    args.command = command;
    args.size = size;
    args.data = data;

    err = ioctl(file, I2C_SMBUS, &args);
    if (err == -1)
        err = -errno;
    return err;
}

__s32 i2c_smbus_read_byte_data(int file, __u8 command)
{
    union i2c_smbus_data data;
    int err;

    err = i2c_smbus_access(file, I2C_SMBUS_READ, command,
                           I2C_SMBUS_BYTE_DATA, &data);
    if (err < 0)
        return err;

    return 0x0FF & data.byte;
}

__s32 i2c_smbus_write_byte_data(int file, __u8 command, __u8 value) {
    union i2c_smbus_data data;
    data.byte = value;
    return i2c_smbus_access(file, I2C_SMBUS_WRITE, command,
                            I2C_SMBUS_BYTE_DATA, &data);
}

ImuService::ImuService() {
    initialized_ = false;
    // Establish connection to the IMU and verify that the who am I pin is correct.
    file_descriptor_ = open("/dev/i2c-1", O_RDWR);
    LOG(INFO) << "Chimbus!";
    int ret = ioctl(file_descriptor_, I2C_SLAVE_FORCE, 0x6b);
    if (ret < 0) {
        return;
    }
    if (i2c_smbus_read_byte_data(file_descriptor_, 0xf) != 108) {
        return;
    }
    // Attempt to enable gyro and accelerometer, checking that writes are successful
    i2c_smbus_write_byte_data(file_descriptor_, 0x10, 0b01000000);
    if (i2c_smbus_read_byte_data(file_descriptor_, 0x10) != 0b01000000) { // write unsuccessful
        return;
    }
    i2c_smbus_write_byte_data(file_descriptor_, 0x11, 0b01000000);
    if (i2c_smbus_read_byte_data(file_descriptor_, 0x11) != 0b01000000) { // write unsuccessful
        return;
    }
    initialized_ = true;
    LOG(INFO) << "Initialized IMU! Calibrating...";
    degrees_error_ = 0;
    // get 50 sample average of stationary reading, so all future readings can be corrected
    double sum = 0;
    for (int i = 0; i < 100; i++) {
        double poll = pollHeadingRate()->toDegrees();
        sum += poll;
        LOG(INFO) << poll;
        usleep(50000);
    }
    double avg = sum / 100;
    degrees_error_ = avg;
    LOG(INFO) << degrees_error_;



}

std::optional<AngularVelocity> ImuService::pollHeadingRate() {
    if (!initialized_) {
        return std::nullopt;
    }
    int ret = ioctl(file_descriptor_, I2C_SLAVE_FORCE, 0x6b);
    if (ret < 0) {
        return std::nullopt;
    }
    auto least_significant = int16_t(i2c_smbus_read_byte_data(file_descriptor_, 0x26));
    auto most_significant = int16_t(i2c_smbus_read_byte_data(file_descriptor_, 0x27));

    auto foo = (int16_t)(most_significant << 8);
    auto full_word = foo + least_significant;

    double degrees_per_sec = double(full_word) / double(SHRT_MAX) * IMU_FULL_SCALE_DPS;

    measured_samples += 1;
    measured_mean = (measured_mean * (measured_samples - 1) + degrees_per_sec) / measured_samples;

    measured_variance = (measured_variance  * (measured_samples - 1) + pow(degrees_per_sec - measured_mean, 2)) / measured_samples;
    std::cout << measured_variance << " samples: " << measured_samples << std::endl;
    LOG(CSV, "variance.csv") << degrees_per_sec << "\n";
    return AngularVelocity::fromDegrees(degrees_per_sec - degrees_error_);
}