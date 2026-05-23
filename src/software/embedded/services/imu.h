#pragma once

#include <Eigen/Dense>
#include <utility>
#include <numbers>
#include <climits>

#include "proto/tbots_software_msgs.pb.h"
#include "software/geom/angular_velocity.h"
#include "software/geom/angular_acceleration.h"

/**
 * Handles low level IMU I2C communication, and some minor offset filtering.
 */
class ImuService
{
    public:
		/**
     	* Constructs and initializes a new IMU service object.
     	*
     	* If successfully initialized, will try to do a simple calibration of the IMU.
     	*/
		ImuService();
		/*
		 * Polls the latest IMU reading of the angular velocity of the robot on the z axis
		 * @return the current angular velocty of the robot on the z axis
		 */
		std::optional<AngularVelocity> pollHeadingVelocity();
		/*
		 * Computes angular acceleration from successive angular velocity readings
		 * @return the current angular acceleration of the robot on the z axis
		 */
		std::optional<AngularAcceleration> pollHeadingAcceleration();
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

		/*
		 * Reads byte data from two registers, and combine them into a single value 
		 * @parama ls_reg register of the least significant register
		 * @parama ms_reg register of the most significant register	
		 * @return the combined integer value of the two registers		 
		 */

		std::optional<int16_t> readAndCombineByteData(uint8_t ls_reg, uint8_t ms_reg);

		bool initialized_=false;

		int file_descriptor_=0;

		double degrees_error_;

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
															
		// prev time and angular accleration 
		std::optional<AngularVelocity> prev_angular_velocity_;
		std::chrono::steady_clock::time_point prev_time_;

};
