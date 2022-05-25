extern "C"
{
#include "firmware/app/world/firmware_ball.h"
}

#include <gtest/gtest.h>

class FirmwareBallTest : public testing::Test
{
   protected:
    virtual void SetUp(void)
    {
        firmware_ball =
            app_firmware_ball_create(&(this->returnSeven), &(this->returnEight),
                                     &(this->returnNine), &(this->returnTen));
    }

    virtual void TearDown(void)
    {
        app_firmware_ball_destroy(firmware_ball);
    }
    static float returnSeven(void)
    {
        return 7;
    }

    static float returnEight(void)
    {
        return 8;
    }

    static float returnNine(void)
    {
        return 9;
    }

    static float returnTen(void)
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
