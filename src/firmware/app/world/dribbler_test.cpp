extern "C"
{
#include "firmware/app/world/dribbler.h"
}

#include <gtest/gtest.h>

class DribblerTest : public testing::Test
{
   protected:
    virtual void SetUp(void)
    {
        requested_rpm = 0;
        coast         = false;

        dribbler = app_dribbler_create(&(this->setRequestedRpm), &(this->enable_coast),
                                       &(this->returnSeven));
    }

    virtual void TearDown(void)
    {
        app_dribbler_destroy(dribbler);
    }

    static void setRequestedRpm(uint32_t rpm)
    {
        requested_rpm = static_cast<float>(rpm);
    }

    static unsigned int returnSeven(void)
    {
        return 7;
    }

    static void enable_coast(void)
    {
        coast = true;
    }

    Dribbler_t* dribbler;

    inline static float requested_rpm;
    inline static bool coast;
};

TEST_F(DribblerTest, setSpeed)
{
    app_dribbler_setSpeed(dribbler, 34);

    EXPECT_EQ(34, requested_rpm);
}

TEST_F(DribblerTest, coast)
{
    app_dribbler_coast(dribbler);
    EXPECT_TRUE(coast);
}

TEST_F(DribblerTest, getTemperature)
{
    EXPECT_EQ(7, app_dribbler_getTemperature(dribbler));
}
