#include "proto/message_translation/power_frame_msg.hpp"

#include <gtest/gtest.h>
#include <proto/power_frame_msg.nanopb.h>

TEST(PowerFrameMsgTest, google_to_nanopb)
{
    auto google_control = TbotsProto::PowerControl();
    google_control.mutable_chicker()->set_chip_distance_meters(3);
    auto nanopb_control = createNanoPbPowerPulseControl(google_control, 0, 0, 100);
    EXPECT_EQ(nanopb_control.chicker.which_chicker_command,
              TbotsProto_PowerPulseControl_ChickerControl_chip_pulse_width_tag);
    EXPECT_EQ(nanopb_control.chicker.chicker_command.chip_pulse_width, 100);

    google_control.mutable_chicker()->set_kick_speed_m_per_s(5);
    nanopb_control = createNanoPbPowerPulseControl(google_control, 0.3, 300, 0);
    EXPECT_EQ(nanopb_control.chicker.which_chicker_command,
              TbotsProto_PowerPulseControl_ChickerControl_kick_pulse_width_tag);
    EXPECT_EQ(nanopb_control.chicker.chicker_command.kick_pulse_width,
              static_cast<int>(300 * std::exp(0.3 * 5)));

    google_control.mutable_chicker()
        ->mutable_auto_chip_or_kick()
        ->set_autokick_speed_m_per_s(4);
    nanopb_control = createNanoPbPowerPulseControl(google_control, 0.1, 3, 0);
    EXPECT_EQ(nanopb_control.chicker.which_chicker_command,
              TbotsProto_PowerPulseControl_ChickerControl_auto_chip_or_kick_tag);
    EXPECT_EQ(
        nanopb_control.chicker.chicker_command.auto_chip_or_kick.which_auto_chip_or_kick,
        TbotsProto_PowerPulseControl_AutoChipOrKick_autokick_pulse_width_tag);
    EXPECT_EQ(nanopb_control.chicker.chicker_command.auto_chip_or_kick.auto_chip_or_kick
                  .autokick_pulse_width,
              static_cast<int>(3 * std::exp(4 * 0.1)));

    google_control.mutable_chicker()
        ->mutable_auto_chip_or_kick()
        ->set_autokick_speed_m_per_s(6);
    nanopb_control = createNanoPbPowerPulseControl(google_control, 10000, 10000, 0);
    EXPECT_EQ(nanopb_control.chicker.which_chicker_command,
              TbotsProto_PowerPulseControl_ChickerControl_auto_chip_or_kick_tag);
    EXPECT_EQ(
        nanopb_control.chicker.chicker_command.auto_chip_or_kick.which_auto_chip_or_kick,
        TbotsProto_PowerPulseControl_AutoChipOrKick_autokick_pulse_width_tag);
    EXPECT_EQ(nanopb_control.chicker.chicker_command.auto_chip_or_kick.auto_chip_or_kick
                  .autokick_pulse_width,
              static_cast<int>(MAX_KICK_CONSTANT * std::exp(6 * MAX_KICK_COEFFICIENT)));

    google_control.mutable_chicker()
        ->mutable_auto_chip_or_kick()
        ->set_autochip_distance_meters(2);
    nanopb_control = createNanoPbPowerPulseControl(google_control, 0, 0, 200);
    EXPECT_EQ(nanopb_control.chicker.which_chicker_command,
              TbotsProto_PowerPulseControl_ChickerControl_auto_chip_or_kick_tag);
    EXPECT_EQ(
        nanopb_control.chicker.chicker_command.auto_chip_or_kick.which_auto_chip_or_kick,
        TbotsProto_PowerPulseControl_AutoChipOrKick_autochip_pulse_width_tag);
    EXPECT_EQ(nanopb_control.chicker.chicker_command.auto_chip_or_kick.auto_chip_or_kick
                  .autochip_pulse_width,
              200);
}
