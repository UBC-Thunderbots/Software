extern "C"
{
#include "app/world/dribbler.h"
}

#include <gtest/gtest.h>

class DribblerTest : public testing::Test
{
   protected:
    virtual void SetUp()
    {
        requested_rpm = 0;

        dribbler = app_dribbler_create(&(this->setRequestedRpm), &(this->returnSeven));
    }

    virtual void TearDown()
    {
        app_dribbler_destroy(dribbler);
    }

    static void setRequestedRpm(uint32_t rpm)
    {
        requested_rpm = rpm;
    }

    static unsigned int returnSeven()
    {
        return 7;
    }

    Dribbler_t* dribbler;

    inline static float requested_rpm;
};

TEST_F(DribblerTest, setSpeed)
{
    app_dribbler_setSpeed(dribbler, 34);

    EXPECT_EQ(34, requested_rpm);
}

TEST_F(DribblerTest, getTemperature)
{
    EXPECT_EQ(7, app_dribbler_getTemperature(dribbler));
}
