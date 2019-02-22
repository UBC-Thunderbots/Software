#include "grsim_communication/grsim_backend.h"

#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>

#include <limits>

#include "proto/grSim_Commands.pb.h"
#include "proto/grSim_Packet.pb.h"

TEST(GrSimBackendTest, create_grsim_packet_zero_vel)
{
    GrSimBackend backend = GrSimBackend("127.0.0.1", 20011);

    grSim_Packet result = backend.createGrSimPacketWithRobotVelocity(
        0, YELLOW, Point(), Angle::zero(), 0.0, false, false);

    // Create the packet we expect.
    grSim_Packet expected;
    expected.mutable_commands()->set_isteamyellow(true);
    expected.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = expected.mutable_commands()->add_robot_commands();

    command->set_id(0);

    command->set_wheelsspeed(false);

    command->set_veltangent(0.0);
    command->set_velnormal(0.0);
    command->set_velangular(0.0);

    command->set_kickspeedx(0.0);
    command->set_kickspeedz(0.0);
    command->set_spinner(false);

    // Protobuf packets do not have a standard equality operator, so we need to use
    // the protobuf MessageDifferencer
    bool messages_equal =
        google::protobuf::util::MessageDifferencer::Equals(result, expected);
    EXPECT_TRUE(messages_equal);
}

TEST(GrSimBackendTest, create_grsim_packet_positive_vel)
{
    GrSimBackend backend = GrSimBackend("127.0.0.1", 20011);

    grSim_Packet result = backend.createGrSimPacketWithRobotVelocity(
        6, BLUE, Point(89.6, 0.1589), Angle::ofRadians(1.23), 0.0, false, false);

    // Create the packet we expect.
    grSim_Packet expected;
    expected.mutable_commands()->set_isteamyellow(false);
    expected.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = expected.mutable_commands()->add_robot_commands();

    command->set_id(6);

    command->set_wheelsspeed(false);

    command->set_veltangent(89.6);
    command->set_velnormal(0.1589);
    command->set_velangular(1.23);

    command->set_kickspeedx(0.0);
    command->set_kickspeedz(0.0);
    command->set_spinner(false);

    // Protobuf packets do not have a standard equality operator, so we need to use
    // the protobuf MessageDifferencer
    bool messages_equal =
        google::protobuf::util::MessageDifferencer::Equals(result, expected);
    EXPECT_TRUE(messages_equal);
}

TEST(GrSimBackendTest, create_grsim_packet_negative_vel)
{
    GrSimBackend backend = GrSimBackend("127.0.0.1", 20011);

    grSim_Packet result = backend.createGrSimPacketWithRobotVelocity(
        1, YELLOW, Point(-0.001, 2.49), Angle::ofRadians(-0.04), 0.0, false, false);

    // Create the packet we expect.
    grSim_Packet expected;
    expected.mutable_commands()->set_isteamyellow(true);
    expected.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = expected.mutable_commands()->add_robot_commands();

    command->set_id(1);

    command->set_wheelsspeed(false);

    command->set_veltangent(-0.001);
    command->set_velnormal(2.49);
    command->set_velangular(-0.04);

    command->set_kickspeedx(0.0);
    command->set_kickspeedz(0.0);
    command->set_spinner(false);

    // Protobuf packets do not have a standard equality operator, so we need to use
    // the protobuf MessageDifferencer
    bool messages_equal =
        google::protobuf::util::MessageDifferencer::Equals(result, expected);
    EXPECT_TRUE(messages_equal);
}

TEST(GrSimBackendTest, create_grsim_packet_at_numeric_limits)
{
    GrSimBackend backend = GrSimBackend("127.0.0.1", 20011);

    grSim_Packet result = backend.createGrSimPacketWithRobotVelocity(
        2, YELLOW,
        Point(std::numeric_limits<float>::min(), std::numeric_limits<float>::min()),
        Angle::ofRadians(std::numeric_limits<float>::min()), 0.0, false, false);

    // Create the packet we expect.
    grSim_Packet expected;
    expected.mutable_commands()->set_isteamyellow(true);
    expected.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = expected.mutable_commands()->add_robot_commands();

    command->set_id(2);

    command->set_wheelsspeed(false);

    command->set_veltangent(std::numeric_limits<float>::min());
    command->set_velnormal(std::numeric_limits<float>::min());
    command->set_velangular(std::numeric_limits<float>::min());

    command->set_kickspeedx(0.0);
    command->set_kickspeedz(0.0);
    command->set_spinner(false);

    // Protobuf packets do not have a standard equality operator, so we need to use
    // the protobuf MessageDifferencer
    bool messages_equal =
        google::protobuf::util::MessageDifferencer::Equals(result, expected);
    EXPECT_TRUE(messages_equal);
}

TEST(GrSimBackendTest, create_grsim_packet_beyond_numeric_limits)
{
    GrSimBackend backend = GrSimBackend("127.0.0.1", 20011);

    grSim_Packet result = backend.createGrSimPacketWithRobotVelocity(
        2, YELLOW,
        Point(std::numeric_limits<double>::min(), std::numeric_limits<double>::min()),
        Angle::ofRadians(std::numeric_limits<double>::min()), 0.0, false, false);

    // Create the packet we expect.
    grSim_Packet expected;
    expected.mutable_commands()->set_isteamyellow(true);
    expected.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = expected.mutable_commands()->add_robot_commands();

    command->set_id(2);

    command->set_wheelsspeed(false);

    command->set_veltangent(std::numeric_limits<double>::min());
    command->set_velnormal(std::numeric_limits<double>::min());
    command->set_velangular(std::numeric_limits<double>::min());

    command->set_kickspeedx(0.0);
    command->set_kickspeedz(0.0);
    command->set_spinner(false);

    // Protobuf packets do not have a standard equality operator, so we need to use
    // the protobuf MessageDifferencer
    bool messages_equal =
        google::protobuf::util::MessageDifferencer::Equals(result, expected);
    EXPECT_TRUE(messages_equal);
}

TEST(GrSimBackendTest, create_grsim_packet_with_kick)
{
    GrSimBackend backend = GrSimBackend("127.0.0.1", 20011);

    grSim_Packet result = backend.createGrSimPacketWithRobotVelocity(
        0, YELLOW, Point(), Angle::zero(), 4.0, false, false);

    // Create the packet we expect.
    grSim_Packet expected;
    expected.mutable_commands()->set_isteamyellow(true);
    expected.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = expected.mutable_commands()->add_robot_commands();

    command->set_id(0);

    command->set_wheelsspeed(false);

    command->set_veltangent(0.0);
    command->set_velnormal(0.0);
    command->set_velangular(0.0);

    command->set_kickspeedx(4.0);
    command->set_kickspeedz(0.0);
    command->set_spinner(false);

    // Protobuf packets do not have a standard equality operator, so we need to use
    // the protobuf MessageDifferencer
    bool messages_equal =
        google::protobuf::util::MessageDifferencer::Equals(result, expected);
    EXPECT_TRUE(messages_equal);
}

TEST(GrSimBackendTest, create_grsim_packet_with_chip)
{
    GrSimBackend backend = GrSimBackend("127.0.0.1", 20011);

    grSim_Packet result = backend.createGrSimPacketWithRobotVelocity(
        2, YELLOW,
        Point(std::numeric_limits<double>::min(), std::numeric_limits<double>::min()),
        Angle::ofRadians(std::numeric_limits<double>::min()), 4.0, true, false);

    // Create the packet we expect.
    grSim_Packet expected;
    expected.mutable_commands()->set_isteamyellow(true);
    expected.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = expected.mutable_commands()->add_robot_commands();

    command->set_id(2);

    command->set_wheelsspeed(false);

    command->set_veltangent(std::numeric_limits<double>::min());
    command->set_velnormal(std::numeric_limits<double>::min());
    command->set_velangular(std::numeric_limits<double>::min());

    command->set_kickspeedx(4.0);
    command->set_kickspeedz(4.0);
    command->set_spinner(false);

    // Protobuf packets do not have a standard equality operator, so we need to use
    // the protobuf MessageDifferencer
    bool messages_equal =
        google::protobuf::util::MessageDifferencer::Equals(result, expected);
    EXPECT_TRUE(messages_equal);
}

TEST(GrSimBackendTest, create_grsim_packet_with_dribbler_on)
{
    GrSimBackend backend = GrSimBackend("127.0.0.1", 20011);

    grSim_Packet result = backend.createGrSimPacketWithRobotVelocity(
        2, YELLOW,
        Point(std::numeric_limits<double>::min(), std::numeric_limits<double>::min()),
        Angle::ofRadians(std::numeric_limits<double>::min()), 0.0, false, true);

    // Create the packet we expect.
    grSim_Packet expected;
    expected.mutable_commands()->set_isteamyellow(true);
    expected.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = expected.mutable_commands()->add_robot_commands();

    command->set_id(2);

    command->set_wheelsspeed(false);

    command->set_veltangent(std::numeric_limits<double>::min());
    command->set_velnormal(std::numeric_limits<double>::min());
    command->set_velangular(std::numeric_limits<double>::min());

    command->set_kickspeedx(0.0);
    command->set_kickspeedz(0.0);
    command->set_spinner(true);

    // Protobuf packets do not have a standard equality operator, so we need to use
    // the protobuf MessageDifferencer
    bool messages_equal =
        google::protobuf::util::MessageDifferencer::Equals(result, expected);
    EXPECT_TRUE(messages_equal);
}
