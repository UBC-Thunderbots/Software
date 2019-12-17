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

        wheel = app_wheel_create(&(this->request_wheel_force));
    }

    virtual void TearDown()
    {
        app_wheel_destroy(wheel);
    }

    static void request_wheel_force(float force)
    {
        requested_wheel_force = force;
    }

    Wheel_t* wheel;

    inline static float requested_wheel_force;
};

TEST_F(WheelTest, applyForce)
{
    app_wheel_applyForce(wheel, 13);

    EXPECT_EQ(13, requested_wheel_force);
}
