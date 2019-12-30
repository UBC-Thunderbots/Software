extern "C"
{
#include "app/world/firmware_ball.h"
}

#include <gtest/gtest.h>

class FirmwareBallTest : public testing::Test
{
   protected:
    virtual void SetUp()
    {
        firmware_ball =
            app_firmware_ball_create(&(this->returnSeven), &(this->returnEight),
                                     &(this->returnNine), &(this->returnTen));
    }

    virtual void TearDown()
    {
        app_firmware_ball_destroy(firmware_ball);
    }
    static float returnSeven()
    {
        return 7;
    }

    static float returnEight()
    {
        return 8;
    }

    static float returnNine()
    {
        return 9;
    }

    static float returnTen()
    {
        return 10;
    }

    FirmwareBall_t* firmware_ball;
};

TEST_F(FirmwareBallTest, getPositionX)
{
    EXPECT_EQ(7, app_firmware_ball_getPositionX(firmware_ball));
}

TEST_F(FirmwareBallTest, getPositionY)
{
    EXPECT_EQ(8, app_firmware_ball_getPositionY(firmware_ball));
}

TEST_F(FirmwareBallTest, getVelocityX)
{
    EXPECT_EQ(9, app_firmware_ball_getVelocityX(firmware_ball));
}

TEST_F(FirmwareBallTest, getVelocityY)
{
    EXPECT_EQ(10, app_firmware_ball_getVelocityY(firmware_ball));
}
