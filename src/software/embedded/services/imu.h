#pragma once

#include <Eigen/Dense>
#include <utility>
#include <numbers>
#include <climits>

#include "proto/tbots_software_msgs.pb.h"
#include "software/geom/angular_velocity.h"

/**
 * Handles low level IMU I2C communication, and some minor offset filtering.
 */
class ImuService
{
    public:
		ImuService();
		/*
		 * Polls the latest IMU reading of the angular velocity of the robot on the z axis
		 * @return the current angular velocty of the robot on the z axis
		 */
		std::optional<AngularVelocity> pollHeadingVelocity();
		/*
		 * Polls the latest IMU reading of the linear acceleration of the robot on the z plane 
		 * @return the current linear acceleration of the robot on the z plane		 
		 */
		std::optional<Eigen::Vector2d> pollHeadingAcceleration();
		/*
		 * Polls the latest IMU reading of the linear acceleration of the robot on the z plane 
		 * @return the current linear acceleration of the robot on the z plane		 
		 */
		std::optional<Eigen::Vector2d> pollLinearAcceleration();


		// Variance from datasheet (in rad^2/s^2)
		static constexpr double IMU_VARIANCE = 
			(4.0 * 14.4222 / 1000.0 * std::numbers::pi / 180.0) *
			(4.0 * 14.4222 / 1000.0 * std::numbers::pi / 180.0);

	private:
		std::optional<int16_t> combineBits();
		bool initialized_=true;
		int file_descriptor_=0;
		doubel degrees_error_;

		// Maps the maximum raw reading from 16-bit integer to be 2 times gravity	
		static constexpr double ACCELEROMETER_FULL_SCALE_G = 2.0;
		// Same for gyroscope, 1000 degrees per second 
		static constexpr double IMU_FULL_SCALE_DPS=1000.0;
		
		/// Various i2c registers.
		static const uint8_t WHOAMI_REG        = 0xf;
    	static const uint8_t ACCEL_CONTROL_REG = 0x10;
    	static const uint8_t GYRO_CONTROL_REG  = 0x11;
    	static const uint8_t CTRL4_C           = 0x13;
    	static const uint8_t CTRL6_C           = 0x15;
    	static const uint8_t CTRL8_XL          = 0x17;

		// Deviation from center of mass
		double x_deviation;
		double y_deviation;
		

		// Device path for the IMU
	    inline static const std::string IMU_DEVICE = "/dev/i2c-1";

		// Gyroscope Z-axis (Yaw) Output Data Registers
	    static constexpr uint8_t HEADING_LEAST_SIG_REG = 0x26; // OUTZ_L_G
	    static constexpr uint8_t HEADING_MOST_SIG_REG  = 0x27; // OUTZ_H_G
	
	    // Accelerometer X-axis Output Data Registers
	    static constexpr uint8_t ACCEL_X_LEAST_SIG_REG = 0x28; // OUTX_L_XL
	    static constexpr uint8_t ACCEL_X_MOST_SIG_REG  = 0x29; // OUTX_H_XL
	
	    // Accelerometer Y-axis Output Data Registers
	    static constexpr uint8_t ACCEL_Y_LEAST_SIG_REG = 0x2A; // OUTY_L_XL
	    static constexpr uint8_t ACCEL_Y_MOST_SIG_REG  = 0x2B; // OUTY_H_XL
}
