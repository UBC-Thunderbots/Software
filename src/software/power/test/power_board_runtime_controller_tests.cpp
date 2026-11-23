#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <vector>

#include "proto/message_translation/power_frame_msg.hpp"
#include "shared/uart_framing/uart_framing.hpp"
#include "software/power/power_board_runtime.h"
#include "software/power/sim/sim_hal.h"

namespace
{
TbotsProto_PowerPulseControl makeKickControl(uint32_t pulse_width_us)
{
    TbotsProto_PowerPulseControl control = TbotsProto_PowerPulseControl_init_default;
    control.has_chicker                  = true;
    control.chicker.which_chicker_command =
        TbotsProto_PowerPulseControl_ChickerControl_kick_pulse_width_tag;
    control.chicker.chicker_command.kick_pulse_width = pulse_width_us;
    return control;
}

TbotsProto_PowerPulseControl makeAutochipControl(uint32_t pulse_width_us)
{
    TbotsProto_PowerPulseControl control = TbotsProto_PowerPulseControl_init_default;
    control.has_chicker                  = true;
    control.chicker.which_chicker_command =
        TbotsProto_PowerPulseControl_ChickerControl_auto_chip_or_kick_tag;
    control.chicker.chicker_command.auto_chip_or_kick.which_auto_chip_or_kick =
        TbotsProto_PowerPulseControl_AutoChipOrKick_autochip_pulse_width_tag;
    control.chicker.chicker_command.auto_chip_or_kick.auto_chip_or_kick
        .autochip_pulse_width = pulse_width_us;
    return control;
}

class PowerBoardRuntimeControllerTest : public ::testing::Test
{
   protected:
    void sendCommand(const TbotsProto_PowerPulseControl& control)
    {
        sendFrame(createUartFrame(control));
    }

    void sendCommand(const TbotsProto_DribblerControl& control)
    {
        sendFrame(createUartFrame(control));
    }

    void sendFrame(const TbotsProto_PowerFrame& frame)
    {
        for (const uint8_t byte : marshallUartPacket(frame))
        {
            runtime_.receiveByte(byte);
        }
    }

    void tick()
    {
        static_cast<void>(runtime_.tick());
    }

    void setBreakBeamTripped(bool tripped)
    {
        SimHal::State state      = hal_.stateSnapshot();
        state.break_beam_tripped = tripped;
        hal_.setState(state);
    }

    SimHal hal_;
    PowerBoardRuntime runtime_{hal_};
};

TEST_F(PowerBoardRuntimeControllerTest, DribblerRampsToRequestedSpeedAndStopsImmediately)
{
    TbotsProto_DribblerControl control = TbotsProto_DribblerControl_init_default;
    control.dribbler_speed             = 8000;
    sendCommand(control);
    tick();

    const uint8_t initial_pwm = hal_.stateSnapshot().dribbler_pwm;
    ASSERT_GT(initial_pwm, 0);

    for (int i = 0; i < 64; ++i)
    {
        tick();
    }
    const uint8_t settled_pwm = hal_.stateSnapshot().dribbler_pwm;
    EXPECT_GT(settled_pwm, initial_pwm);
    EXPECT_LT(settled_pwm, std::numeric_limits<uint8_t>::max());

    control.dribbler_speed = 0;
    sendCommand(control);
    tick();
    EXPECT_EQ(hal_.stateSnapshot().dribbler_pwm, 0);
}

TEST_F(PowerBoardRuntimeControllerTest, DribblerSpeedIsCappedAtMaximumPwm)
{
    TbotsProto_DribblerControl control = TbotsProto_DribblerControl_init_default;
    control.dribbler_speed             = std::numeric_limits<uint32_t>::max();
    sendCommand(control);

    for (int i = 0; i < 64; ++i)
    {
        tick();
    }

    EXPECT_EQ(hal_.stateSnapshot().dribbler_pwm, std::numeric_limits<uint8_t>::max());
}

TEST_F(PowerBoardRuntimeControllerTest, AutochipRequiresTrippedBreakBeam)
{
    const auto control = makeAutochipControl(1000);

    sendCommand(control);
    tick();
    EXPECT_FALSE(hal_.isChickerPulseActive());

    setBreakBeamTripped(true);
    tick();
    EXPECT_FALSE(hal_.isChickerPulseActive());

    sendCommand(control);
    tick();
    EXPECT_TRUE(hal_.isChickerPulseActive());
    EXPECT_TRUE(hal_.stateSnapshot().chipper_pin_high);
    EXPECT_FALSE(hal_.stateSnapshot().kicker_pin_high);
}

TEST_F(PowerBoardRuntimeControllerTest, ChickerPulseWidthIsLimited)
{
    constexpr uint32_t maximum_pulse_width_us = 20000;
    sendCommand(makeKickControl(maximum_pulse_width_us + 1));
    tick();
    EXPECT_FALSE(hal_.isChickerPulseActive());
    EXPECT_EQ(hal_.stateSnapshot().pulse_count, 0u);

    sendCommand(makeKickControl(maximum_pulse_width_us));
    tick();
    EXPECT_TRUE(hal_.isChickerPulseActive());
    EXPECT_EQ(hal_.stateSnapshot().last_pulse_width_us, maximum_pulse_width_us);
}

TEST_F(PowerBoardRuntimeControllerTest, ChickerCooldownRejectsPulseUntilItExpires)
{
    constexpr uint32_t pulse_width_us = 1000;
    constexpr uint32_t cooldown_us    = 3000000;

    sendCommand(makeKickControl(pulse_width_us));
    tick();
    ASSERT_TRUE(hal_.isChickerPulseActive());

    hal_.advanceMicros(pulse_width_us);
    tick();
    ASSERT_FALSE(hal_.isChickerPulseActive());

    sendCommand(makeKickControl(pulse_width_us));
    tick();
    EXPECT_FALSE(hal_.isChickerPulseActive());
    EXPECT_EQ(hal_.stateSnapshot().pulse_count, 1u);

    hal_.advanceMicros(cooldown_us - pulse_width_us);
    tick();
    sendCommand(makeKickControl(pulse_width_us));
    tick();
    EXPECT_TRUE(hal_.isChickerPulseActive());
    EXPECT_EQ(hal_.stateSnapshot().pulse_count, 2u);
}

TEST_F(PowerBoardRuntimeControllerTest, ChargingRestartsAfterChickerPulseWithToggleDelays)
{
    constexpr uint32_t pulse_width_us  = 1000;
    constexpr uint32_t toggle_delay_us = 5000;

    ASSERT_TRUE(hal_.stateSnapshot().charger_enabled_pin);

    sendCommand(makeKickControl(pulse_width_us));
    tick();
    ASSERT_TRUE(hal_.isChickerPulseActive());

    hal_.advanceMicros(pulse_width_us);
    tick();
    ASSERT_FALSE(hal_.isChickerPulseActive());
    EXPECT_TRUE(hal_.stateSnapshot().charger_enabled_pin);

    hal_.advanceMicros(toggle_delay_us - 1);
    tick();
    EXPECT_TRUE(hal_.stateSnapshot().charger_enabled_pin);

    hal_.advanceMicros(1);
    tick();
    EXPECT_FALSE(hal_.stateSnapshot().charger_enabled_pin);

    hal_.advanceMicros(toggle_delay_us - 1);
    tick();
    EXPECT_FALSE(hal_.stateSnapshot().charger_enabled_pin);

    hal_.advanceMicros(1);
    tick();
    EXPECT_TRUE(hal_.stateSnapshot().charger_enabled_pin);
}

TEST_F(PowerBoardRuntimeControllerTest, ChargingStopsAfterMaximumChargeDuration)
{
    ASSERT_TRUE(hal_.stateSnapshot().charger_enabled_pin);

    hal_.advanceMicros(2000000);
    tick();

    EXPECT_FALSE(hal_.stateSnapshot().charger_enabled_pin);
}
}  // namespace
