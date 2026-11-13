#include <gtest/gtest.h>

#include <cstdint>
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

TbotsProto_PowerPulseControl makeChipControl(uint32_t pulse_width_us)
{
    TbotsProto_PowerPulseControl control = TbotsProto_PowerPulseControl_init_default;
    control.has_chicker                  = true;
    control.chicker.which_chicker_command =
        TbotsProto_PowerPulseControl_ChickerControl_chip_pulse_width_tag;
    control.chicker.chicker_command.chip_pulse_width = pulse_width_us;
    return control;
}

TbotsProto_PowerPulseControl makeAutokickControl(uint32_t pulse_width_us)
{
    TbotsProto_PowerPulseControl control = TbotsProto_PowerPulseControl_init_default;
    control.has_chicker                  = true;
    control.chicker.which_chicker_command =
        TbotsProto_PowerPulseControl_ChickerControl_auto_chip_or_kick_tag;
    control.chicker.chicker_command.auto_chip_or_kick.which_auto_chip_or_kick =
        TbotsProto_PowerPulseControl_AutoChipOrKick_autokick_pulse_width_tag;
    control.chicker.chicker_command.auto_chip_or_kick.auto_chip_or_kick
        .autokick_pulse_width = pulse_width_us;
    return control;
}

class PowerBoardRuntimeTest : public ::testing::Test
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

    TbotsProto_PowerFrame receiveStatus()
    {
        TbotsProto_PowerFrame frame = TbotsProto_PowerFrame_init_default;
        EXPECT_TRUE(unmarshalUartPacket(runtime_.tick(), frame));
        return frame;
    }

    SimHal hal_;
    PowerBoardRuntime runtime_{hal_};
};

TEST_F(PowerBoardRuntimeTest, ReportsHalReadingsInStatusFrame)
{
    hal_.state.battery_voltage    = 24.5f;
    hal_.state.capacitor_voltage  = 170.0f;
    hal_.state.current_draw_amp   = 2.25f;
    hal_.state.break_beam_tripped = true;

    const TbotsProto_PowerFrame first_status = receiveStatus();
    ASSERT_EQ(first_status.which_power_msg, TbotsProto_PowerFrame_power_status_tag);
    EXPECT_FLOAT_EQ(first_status.power_msg.power_status.battery_voltage, 24.5f);
    EXPECT_FLOAT_EQ(first_status.power_msg.power_status.capacitor_voltage, 170.0f);
    EXPECT_FLOAT_EQ(first_status.power_msg.power_status.current_draw, 2.25f);
    EXPECT_TRUE(first_status.power_msg.power_status.breakbeam_tripped);
    EXPECT_EQ(first_status.power_msg.power_status.sequence_num, 0u);

    const TbotsProto_PowerFrame second_status = receiveStatus();
    EXPECT_EQ(second_status.power_msg.power_status.sequence_num, 1u);
}

TEST_F(PowerBoardRuntimeTest, DribblerCommandChangesHalPwm)
{
    TbotsProto_DribblerControl control = TbotsProto_DribblerControl_init_default;
    control.dribbler_speed             = 8000;

    // A partial frame must not actuate the dribbler.
    const auto packet = marshallUartPacket(createUartFrame(control));
    for (std::size_t i = 0; i + 1 < packet.size(); ++i)
    {
        runtime_.receiveByte(packet[i]);
    }
    static_cast<void>(runtime_.tick());
    EXPECT_EQ(hal_.state.dribbler_pwm, 0);

    runtime_.receiveByte(packet.back());
    static_cast<void>(runtime_.tick());
    EXPECT_GT(hal_.state.dribbler_pwm, 0);

    control.dribbler_speed = 0;
    sendCommand(control);
    static_cast<void>(runtime_.tick());
    EXPECT_EQ(hal_.state.dribbler_pwm, 0);
}

TEST_F(PowerBoardRuntimeTest, KickCommandStartsAndExpiresHalPulse)
{
    constexpr uint32_t pulse_width_us = 1000;
    sendCommand(makeKickControl(pulse_width_us));
    static_cast<void>(runtime_.tick());

    EXPECT_TRUE(hal_.state.kicker_pin_high);
    EXPECT_FALSE(hal_.state.chipper_pin_high);
    EXPECT_TRUE(hal_.isChickerPulseActive());

    hal_.advanceMicros(pulse_width_us);
    static_cast<void>(runtime_.tick());

    EXPECT_FALSE(hal_.state.kicker_pin_high);
    EXPECT_FALSE(hal_.isChickerPulseActive());
}

TEST_F(PowerBoardRuntimeTest, ChipCommandStartsAndExpiresHalPulse)
{
    constexpr uint32_t pulse_width_us = 1000;
    sendCommand(makeChipControl(pulse_width_us));
    static_cast<void>(runtime_.tick());

    EXPECT_FALSE(hal_.state.kicker_pin_high);
    EXPECT_TRUE(hal_.state.chipper_pin_high);
    EXPECT_TRUE(hal_.isChickerPulseActive());

    hal_.advanceMicros(pulse_width_us);
    static_cast<void>(runtime_.tick());

    EXPECT_FALSE(hal_.state.chipper_pin_high);
    EXPECT_FALSE(hal_.isChickerPulseActive());
}

TEST_F(PowerBoardRuntimeTest, AutokickRequiresTrippedBreakBeam)
{
    const auto control = makeAutokickControl(1000);

    sendCommand(control);
    static_cast<void>(runtime_.tick());
    EXPECT_FALSE(hal_.isChickerPulseActive());

    hal_.state.break_beam_tripped = true;
    sendCommand(control);
    static_cast<void>(runtime_.tick());
    EXPECT_TRUE(hal_.isChickerPulseActive());
    EXPECT_TRUE(hal_.state.kicker_pin_high);
}
}  // namespace
