#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <climits> // For SHRT_MAX

#include "imu.h"
#include "shared/constants.h"
#include "software/logger/logger.h"

// these functions taken from
// https://git.kernel.org/pub/scm/utils/i2c-tools/i2c-tools.git/tree/lib/smbus.c
static __s32 i2c_smbus_access(int file, char read_write, __u8 command, int size,
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

static __s32 i2c_smbus_read_byte_data(int file, __u8 command)
{
    union i2c_smbus_data data;
    int err;

    err = i2c_smbus_access(file, I2C_SMBUS_READ, command, I2C_SMBUS_BYTE_DATA, &data);
    if (err < 0)
        return err;

    return 0x0FF & data.byte;
}

static __s32 i2c_smbus_write_byte_data(int file, __u8 command, __u8 value)
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
        LOG(WARNING) << "Failed to initialize the IMU: WHOAMI register "
                     << static_cast<int>(WHOAMI_REG) << " read incorrectly.";
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
        LOG(WARNING)
            << "Failed to initialize the IMU: writing to gyroscope config register "
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
        // Fixed: Call the actual implemented function name
        auto poll = pollAngularVelocity();
        if (poll.has_value())
        {
            sum += poll->toDegrees();
            valid_samples++;
        }
        usleep(50000);
    }

    // TODO: More robust calibration
    if (valid_samples > 0)
    {
        degrees_error_ = sum / valid_samples;
        LOG(INFO) << "IMU Calibration complete. Offset: " << degrees_error_ << " dps";
    }
    else
    {
        LOG(WARNING) << "IMU Calibration failed: no valid samples received. Angular "
                        "stability will be poor.";
        initialized_ = false;
    }
}

std::optional<ImuData> ImuService::poll(){

	std::optional<AngularVelocity> angular_velocity = pollAngularVelocity();
	std::optional<AngularAcceleration> angular_acceleration = pollAngularAcceleration(angular_velocity);
	std::optional<Eigen::Vector2d> imu_linear_acceleration = pollLinearAcceleration();

	std::optional<Eigen::Vector2d> linear_acceleration;
	if (angular_velocity && angular_acceleration && imu_linear_acceleration) {
	    linear_acceleration = transformLinearAcceleration(
	        *angular_velocity, *angular_acceleration, *imu_linear_acceleration);
	}


	return ImuData{angular_velocity, angular_acceleration, linear_acceleration};

}
std::optional<int16_t> ImuService::readAndCombineByteData(uint8_t ls_reg, uint8_t ms_reg)
{
    int least_significant = i2c_smbus_read_byte_data(file_descriptor_, ls_reg);
    int most_significant  = i2c_smbus_read_byte_data(file_descriptor_, ms_reg);

    if (least_significant < 0 || most_significant < 0)
    {
        return std::nullopt;
    }

    uint16_t combined = (static_cast<uint8_t>(most_significant) << 8) |
                        static_cast<uint8_t>(least_significant);
                        
    return static_cast<int16_t>(combined);
}

std::optional<AngularVelocity> ImuService::pollAngularVelocity()
{
    if (!initialized_)
    {
        return std::nullopt;
    }

    std::optional<int16_t> opt_full_word = readAndCombineByteData(GYRO_LEAST_SIG_REG, GYRO_MOST_SIG_REG);
    
    if (!opt_full_word.has_value())
    {
        return std::nullopt;
    }

    int16_t full_word = opt_full_word.value();
    
    double degrees_per_sec = static_cast<double>(full_word) /
                             static_cast<double>(SHRT_MAX) * IMU_FULL_SCALE_DPS;
	
    return AngularVelocity::fromRadians((degrees_per_sec - degrees_error_)*M_PI/180);
}


std::optional<AngularAcceleration> ImuService::pollAngularAcceleration(std::optional<AngularVelocity> curr_angular_velocity)
{
    if (!initialized_)
    {
        return std::nullopt;
    }
	
	if(!prev_angular_velocity_.has_value()){
		prev_angular_velocity_ = pollAngularVelocity();
		prev_time_ = std::chrono::steady_clock::now();
		return std::nullopt;
	}

    auto curr_time =  std::chrono::steady_clock::now();

	double dt = std::chrono::duration<double>(curr_time - prev_time_).count();
	if (dt <= 0 || !curr_angular_velocity.has_value()) return std::nullopt;

	double alpha = (curr_angular_velocity->toRadians() - 
                prev_angular_velocity_->toRadians()) / dt;

	prev_angular_velocity_ = curr_angular_velocity;
	prev_time_ = curr_time;

    return AngularAcceleration::fromRadians(alpha);
}
	 
	

std::optional<Eigen::Vector2d> ImuService::pollLinearAcceleration()
{
    if (!initialized_)
    {
        return std::nullopt;
    }

    std::optional<int16_t> opt_x = readAndCombineByteData(ACCEL_X_LEAST_SIG_REG, ACCEL_X_MOST_SIG_REG);
    std::optional<int16_t> opt_y = readAndCombineByteData(ACCEL_Y_LEAST_SIG_REG, ACCEL_Y_MOST_SIG_REG);

    if (!opt_x.has_value() || !opt_y.has_value())
    {
        return std::nullopt;
    }

    int16_t raw_x = opt_x.value();
    int16_t raw_y = opt_y.value();

    double a_x = (static_cast<double>(raw_x) / SHRT_MAX) 
                 * ACCELEROMETER_FULL_SCALE_G 
                 * ACCELERATION_DUE_TO_GRAVITY_METERS_PER_SECOND_SQUARED;
                 
    double a_y = (static_cast<double>(raw_y) / SHRT_MAX) 
                 * ACCELEROMETER_FULL_SCALE_G 
                 * ACCELERATION_DUE_TO_GRAVITY_METERS_PER_SECOND_SQUARED;
	
    return Eigen::Vector2d(a_x, a_y);
}

Eigen::Vector2d ImuService::transformLinearAcceleration(
    AngularVelocity omega, AngularAcceleration alpha, Eigen::Vector2d a_imu)
{
    Eigen::Vector2d r(IMU_OFFSET_X, IMU_OFFSET_Y);

    double w = omega.toRadians();
    double a = alpha.toRadians();

    // tangential: alpha x r → (-alpha*ry, alpha*rx)
    Eigen::Vector2d tangential(-a * r.y(), a * r.x());

    // centripetal: omega^2 * r
    Eigen::Vector2d centripetal = (w * w) * r;

    return a_imu + tangential - centripetal;
}
