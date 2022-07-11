#include "proto/message_translation/power_frame_msg.hpp"

#include <gtest/gtest.h>
#include <proto/power_frame_msg.nanopb.h>

TEST(PowerFrameMsgTest, google_to_nanopb) {
    auto google_control = TbotsProto::PowerControl();
    google_control.mutable_chicker()->set_chip_distance_meters(3);
    auto nanopb_control = createNanoPbPowerPulseControl(google_control);
    EXPECT_EQ(nanopb_control.chicker.which_chicker_command, TbotsProto_PowerPulseControl_ChickerControl_chip_pulse_width_tag);
    EXPECT_EQ(nanopb_control.chicker.chicker_command.chip_pulse_width, 3);

    google_control.mutable_chicker()->set_kick_speed_m_per_s(5);
    nanopb_control = createNanoPbPowerPulseControl(google_control);
    EXPECT_EQ(nanopb_control.chicker.which_chicker_command, TbotsProto_PowerPulseControl_ChickerControl_kick_pulse_width_tag);
    EXPECT_EQ(nanopb_control.chicker.chicker_command.kick_pulse_width, 5);

    google_control.mutable_chicker()->mutable_auto_chip_or_kick()->set_autokick_speed_m_per_s(4);
    nanopb_control = createNanoPbPowerPulseControl(google_control);
    EXPECT_EQ(nanopb_control.chicker.which_chicker_command, TbotsProto_PowerPulseControl_ChickerControl_auto_chip_or_kick_tag);
    EXPECT_EQ(nanopb_control.chicker.chicker_command.auto_chip_or_kick.which_auto_chip_or_kick, TbotsProto_PowerPulseControl_AutoChipOrKick_autokick_pulse_width_tag);
    EXPECT_EQ(nanopb_control.chicker.chicker_command.auto_chip_or_kick.auto_chip_or_kick.autokick_pulse_width, 4);

    google_control.mutable_chicker()->mutable_auto_chip_or_kick()->set_autochip_distance_meters(2);
    nanopb_control = createNanoPbPowerPulseControl(google_control);
    EXPECT_EQ(nanopb_control.chicker.which_chicker_command, TbotsProto_PowerPulseControl_ChickerControl_auto_chip_or_kick_tag);
    EXPECT_EQ(nanopb_control.chicker.chicker_command.auto_chip_or_kick.which_auto_chip_or_kick, TbotsProto_PowerPulseControl_AutoChipOrKick_autochip_pulse_width_tag);
    EXPECT_EQ(nanopb_control.chicker.chicker_command.auto_chip_or_kick.auto_chip_or_kick.autochip_pulse_width, 2);

}
