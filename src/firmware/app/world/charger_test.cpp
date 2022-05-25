extern "C"
{
#include "firmware/app/world/charger.h"
}

#include <gtest/gtest.h>

class ChargerTest : public testing::Test
{
   protected:
    virtual void SetUp(void)
    {
        charge_capacitor = false;
        float_capacitor  = false;

        charger =
            app_charger_create(&(this->chargeCapacitor), &(this->dischargeCapacitor),
                               &(this->floatCapacitor));
    }

    virtual void TearDown(void)
    {
        app_charger_destroy(charger);
    }

    static void chargeCapacitor(void)
    {
        charge_capacitor = true;
        float_capacitor  = false;
    }

    static void dischargeCapacitor(void)
    {
        charge_capacitor = false;
        float_capacitor  = false;
    }

    static void floatCapacitor(void)
    {
        charge_capacitor = false;
        float_capacitor  = true;
    }

    Charger_t* charger;

    inline static bool charge_capacitor;
    inline static bool float_capacitor;
};

TEST_F(ChargerTest, charge_capacitor)
{
    app_charger_charge_capacitor(charger);

    EXPECT_TRUE(charge_capacitor);
    EXPECT_FALSE(float_capacitor);
}

TEST_F(ChargerTest, discharge_capacitor)
{
    app_charger_discharge_capacitor(charger);

    EXPECT_FALSE(charge_capacitor);
    EXPECT_FALSE(float_capacitor);
}

TEST_F(ChargerTest, float_capacitor)
{
    app_charger_float_capacitor(charger);

    EXPECT_FALSE(charge_capacitor);
    EXPECT_TRUE(float_capacitor);
}
