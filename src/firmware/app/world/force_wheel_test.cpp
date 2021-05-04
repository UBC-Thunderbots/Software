extern "C"
{
#include "firmware/app/world/force_wheel.h"
}

#include <gtest/gtest.h>

class WheelTest : public testing::Test
{
   protected:
    virtual void SetUp(void)
    {
        requested_wheel_force = 0;
        brake_called          = true;
        coast_called          = true;

        ForceWheelConstants_t wheel_constants = {
            .motor_current_per_unit_torque       = 1.1f,
            .motor_phase_resistance              = 1.2f,
            .motor_back_emf_per_rpm              = 1.3f,
            .motor_max_voltage_before_wheel_slip = 1.4f,
            .wheel_radius                        = 1.5f,
            .wheel_rotations_per_motor_rotation  = 0.5f};

        wheel =
            app_force_wheel_create(&(this->request_wheel_force), &(this->get_motor_speed),
                                   brake, coast, wheel_constants);
    }

    virtual void TearDown(void)
    {
        app_force_wheel_destroy(wheel);
    }

    static void request_wheel_force(float force)
    {
        requested_wheel_force = force;
    }

    static float get_motor_speed(void)
    {
        return 17.2f;
    }

    static void brake(void)
    {
        brake_called = true;
    }

    static void coast(void)
    {
        coast_called = true;
    }

    ForceWheel_t* wheel;

    inline static float requested_wheel_force;
    inline static bool brake_called;
    inline static bool coast_called;
};

TEST_F(WheelTest, applyForce)
{
    app_force_wheel_applyForce(wheel, 13);

    EXPECT_EQ(13, requested_wheel_force);
}

TEST_F(WheelTest, getMotorSpeed)
{
    float speed = app_force_wheel_getMotorSpeedRPM(wheel);

    EXPECT_NEAR(17.2, speed, 1e-5);
}

TEST_F(WheelTest, getWheelSpeed)
{
    float speed = app_force_wheel_getWheelSpeedRPM(wheel);

    EXPECT_NEAR(17.2 * 0.5, speed, 1e-5);
}

TEST_F(WheelTest, brake)
{
    app_force_wheel_brake(wheel);
    EXPECT_TRUE(brake_called);
}

TEST_F(WheelTest, coast)
{
    app_force_wheel_coast(wheel);
    EXPECT_TRUE(coast);
}

TEST_F(WheelTest, getWheelConstants)
{
    ForceWheelConstants_t constants = app_force_wheel_getWheelConstants(wheel);

    EXPECT_NEAR(1.1, constants.motor_current_per_unit_torque, 1e-4);
    EXPECT_NEAR(1.2, constants.motor_phase_resistance, 1e-4);
    EXPECT_NEAR(1.3, constants.motor_back_emf_per_rpm, 1e-4);
    EXPECT_NEAR(1.4, constants.motor_max_voltage_before_wheel_slip, 1e-4);
    EXPECT_NEAR(1.5, constants.wheel_radius, 1e-4);
    EXPECT_NEAR(0.5, constants.wheel_rotations_per_motor_rotation, 1e-4);
}
