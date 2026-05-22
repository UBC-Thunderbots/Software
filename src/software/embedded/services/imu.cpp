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


ImuService:ImuService(): initialized_(false)
{


}
