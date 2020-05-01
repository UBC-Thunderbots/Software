extern "C"
{
#include "firmware/app/world/chicker.h"
}

#include <gtest/gtest.h>

class ChickerTest : public testing::Test
{
   protected:
    virtual void SetUp(void)
    {
        requested_kick_speed             = 0;
        requested_chip_distance          = 0;
        autokick_requested_kick_speed    = 0;
        autochip_requested_chip_distance = 0;
        autokick_disabled                = false;
        autochip_disabled                = false;

        chicker = app_chicker_create(&(this->setKickSpeed), &(this->setChipDistance),
                                     &(this->autoKick), &(this->autoChip),
                                     &(this->disableAutoKick), &(this->disableAutoChip));
    }

    virtual void TearDown(void)
    {
        app_chicker_destroy(chicker);
    }

    static void setKickSpeed(float speed)
    {
        requested_kick_speed = speed;
    }

    static void setChipDistance(float distance)
    {
        requested_chip_distance = distance;
    }

    static void autoKick(float speed)
    {
        autokick_requested_kick_speed = speed;
    }

    static void autoChip(float distance)
    {
        autochip_requested_chip_distance = distance;
    }

    static void disableAutoChip(void)
    {
        autochip_disabled = true;
    }

    static void disableAutoKick(void)
    {
        autokick_disabled = true;
    }

    Chicker_t* chicker;

    inline static float requested_kick_speed;
    inline static float requested_chip_distance;
    inline static float autokick_requested_kick_speed;
    inline static float autochip_requested_chip_distance;
    inline static float autokick_disabled;
    inline static float autochip_disabled;
};

TEST_F(ChickerTest, kick)
{
    app_chicker_kick(chicker, 13);

    EXPECT_EQ(13, requested_kick_speed);
    EXPECT_EQ(0, requested_chip_distance);
    EXPECT_EQ(0, autokick_requested_kick_speed);
    EXPECT_EQ(0, autochip_requested_chip_distance);
    EXPECT_FALSE(autokick_disabled);
    EXPECT_FALSE(autochip_disabled);
}

TEST_F(ChickerTest, chip)
{
    app_chicker_chip(chicker, 13);

    EXPECT_EQ(0, requested_kick_speed);
    EXPECT_EQ(13, requested_chip_distance);
    EXPECT_EQ(0, autokick_requested_kick_speed);
    EXPECT_EQ(0, autochip_requested_chip_distance);
    EXPECT_FALSE(autokick_disabled);
    EXPECT_FALSE(autochip_disabled);
}

TEST_F(ChickerTest, enableAutoKick)
{
    app_chicker_enableAutokick(chicker, 13);

    EXPECT_EQ(0, requested_kick_speed);
    EXPECT_EQ(0, requested_chip_distance);
    EXPECT_EQ(13, autokick_requested_kick_speed);
    EXPECT_EQ(0, autochip_requested_chip_distance);
    EXPECT_FALSE(autokick_disabled);
    EXPECT_FALSE(autochip_disabled);
}

TEST_F(ChickerTest, enableAutoChip)
{
    app_chicker_enableAutochip(chicker, 13);

    EXPECT_EQ(0, requested_kick_speed);
    EXPECT_EQ(0, requested_chip_distance);
    EXPECT_EQ(0, autokick_requested_kick_speed);
    EXPECT_EQ(13, autochip_requested_chip_distance);
    EXPECT_FALSE(autokick_disabled);
    EXPECT_FALSE(autochip_disabled);
}

TEST_F(ChickerTest, distableAutoKick)
{
    app_chicker_disableAutokick(chicker);

    EXPECT_EQ(0, requested_kick_speed);
    EXPECT_EQ(0, requested_chip_distance);
    EXPECT_EQ(0, autokick_requested_kick_speed);
    EXPECT_EQ(0, autochip_requested_chip_distance);
    EXPECT_TRUE(autokick_disabled);
    EXPECT_FALSE(autochip_disabled);
}

TEST_F(ChickerTest, disableAutoChip)
{
    app_chicker_disableAutochip(chicker);

    EXPECT_EQ(0, requested_kick_speed);
    EXPECT_EQ(0, requested_chip_distance);
    EXPECT_EQ(0, autokick_requested_kick_speed);
    EXPECT_EQ(0, autochip_requested_chip_distance);
    EXPECT_FALSE(autokick_disabled);
    EXPECT_TRUE(autochip_disabled);
}
