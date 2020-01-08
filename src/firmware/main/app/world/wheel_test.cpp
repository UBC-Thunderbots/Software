extern "C"
{
#include "app/world/wheel.h"
}

#include <gtest/gtest.h>

class WheelTest : public testing::Test
{
   protected:
    virtual void SetUp()
    {
        requested_wheel_force = 0;

        WheelConstants_t wheel_constants = {
            .motor_current_per_unit_torque             = 1.1,
            .motor_phase_resistance                    = 1.2,
            .motor_back_emf_per_rpm                    = 1.3,
            .motor_max_delta_voltage_before_wheel_slip = 1.4,
            .wheel_radius                              = 1.5,
            .wheel_rotations_per_motor_rotation        = 0.5};

        wheel = app_wheel_create(&(this->request_wheel_force), &(this->get_motor_speed),
                                 wheel_constants);
    }

    virtual void TearDown()
    {
        app_wheel_destroy(wheel);
    }

    static void request_wheel_force(float force)
    {
        requested_wheel_force = force;
    }

    static float get_motor_speed()
    {
        return 17.2;
    }

    Wheel_t* wheel;

    inline static float requested_wheel_force;
};

TEST_F(WheelTest, applyForce)
{
    app_wheel_applyForce(wheel, 13);

    EXPECT_EQ(13, requested_wheel_force);
}

TEST_F(WheelTest, getMotorSpeed){
    float speed = app_wheel_getMotorSpeedRPM(wheel);

    EXPECT_NEAR(17.2, speed, 1e-5);
}

TEST_F(WheelTest, getWheelSpeed){
    float speed = app_wheel_getWheelSpeedRPM(wheel);

    EXPECT_NEAR(17.2*0.5, speed, 1e-5);
}

TEST_F(WheelTest, getWheelConstants){
    WheelConstants_t constants = app_wheel_getWheelConstants(wheel);

    EXPECT_NEAR(1.1, constants.motor_current_per_unit_torque, 1e-4);
    EXPECT_NEAR(1.2, constants.motor_phase_resistance, 1e-4);
    EXPECT_NEAR(1.3, constants.motor_back_emf_per_rpm, 1e-4);
    EXPECT_NEAR(1.4, constants.motor_max_delta_voltage_before_wheel_slip, 1e-4);
    EXPECT_NEAR(1.5, constants.wheel_radius, 1e-4);
    EXPECT_NEAR(0.5, constants.wheel_rotations_per_motor_rotation, 1e-4);
}

