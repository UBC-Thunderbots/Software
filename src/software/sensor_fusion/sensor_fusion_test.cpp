#include "software/sensor_fusion/sensor_fusion.h"

#include <gtest/gtest.h>

#include "proto/message_translation/ssl_detection.h"
#include "proto/message_translation/ssl_geometry.h"
#include "proto/message_translation/ssl_wrapper.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "proto/parameters.pb.h"

class SensorFusionTest : public ::testing::Test
{
   public:
    SensorFusionTest()
        : config(TbotsProto::SensorFusionConfig()),
          sensor_fusion(config),
          yellow_robot_states(initYellowRobotStates()),
          blue_robot_states(initBlueRobotStates()),
          ball_state(Point(-1.2, 0), Vector(0.0, 0.0), 0.2),
          current_time(Timestamp::fromSeconds(8.03)),
          geom_data(initSSLDivBGeomData()),
          robot_status_msg_id_1(initRobotStatusId1()),
          robot_status_msg_id_2(initRobotStatusId2()),
          robot_status_msg_high_cap(initHighCapErrorCode()),
          robot_status_msg_dribble_motor_hot(initDribbleMotorHotErrorCode()),
          robot_status_msg_multiple_error_codes(initMultipleErrorCode()),
          robot_status_msg_no_error_code(initNoErrorCode()),
          referee_indirect_yellow(initRefereeIndirectYellow()),
          referee_indirect_blue(initRefereeIndirectBlue()),
          referee_normal_start(initRefereeNormalStart()),
          referee_ball_placement_yellow(initRefereeBallPlacementYellow()),
          referee_ball_placement_blue(initRefereeBallPlacementBlue()),
          referee_goalie_id(initRefereeGoalieId())
    {
        config.set_friendly_color_yellow(true);
    }

    TbotsProto::SensorFusionConfig config;
    SensorFusion sensor_fusion;
    std::vector<RobotStateWithId> yellow_robot_states;
    std::vector<RobotStateWithId> blue_robot_states;
    BallState ball_state;
    Timestamp current_time;
    std::unique_ptr<SSLProto::SSL_GeometryData> geom_data;
    // world associated with geom_data and detection_frame only
    std::unique_ptr<TbotsProto::RobotStatus> robot_status_msg_id_1;
    std::unique_ptr<TbotsProto::RobotStatus> robot_status_msg_id_2;
    std::unique_ptr<TbotsProto::RobotStatus> robot_status_msg_high_cap;
    std::unique_ptr<TbotsProto::RobotStatus> robot_status_msg_dribble_motor_hot;
    std::unique_ptr<TbotsProto::RobotStatus> robot_status_msg_multiple_error_codes;
    std::unique_ptr<TbotsProto::RobotStatus> robot_status_msg_no_error_code;
    std::unique_ptr<SSLProto::Referee> referee_indirect_yellow;
    std::unique_ptr<SSLProto::Referee> referee_indirect_blue;
    std::unique_ptr<SSLProto::Referee> referee_normal_start;
    std::unique_ptr<SSLProto::Referee> referee_ball_placement_yellow;
    std::unique_ptr<SSLProto::Referee> referee_ball_placement_blue;
    std::unique_ptr<SSLProto::Referee> referee_goalie_id;


    BallState initBallState()
    {
        return BallState(Point(-1.2, 0), Vector(0.0, 0.0), 0.2);
    }

    BallState initInvertedBallState()
    {
        return BallState(Point(1.2, 0), Vector(0.0, 0.0), 0.2);
    }

    std::vector<RobotStateWithId> initYellowRobotStates()
    {
        RobotState yellow_robot_state1(Point(1, 0), Vector(0, 0), Angle::fromRadians(2),
                                       AngularVelocity::zero());
        RobotState yellow_robot_state2(Point(0, 0), Vector(0, 0), Angle::fromRadians(1),
                                       AngularVelocity::zero());
        return {
            RobotStateWithId{.id = 1, .robot_state = yellow_robot_state1},
            RobotStateWithId{.id = 2, .robot_state = yellow_robot_state2},
        };
    }

    std::vector<RobotStateWithId> initInvertedYellowRobotStates()
    {
        RobotState yellow_robot_state1(Point(-1, 0), Vector(0, 0),
                                       Angle::fromRadians(2) + Angle::half(),
                                       AngularVelocity::zero());
        RobotState yellow_robot_state2(Point(0, 0), Vector(0, 0),
                                       Angle::fromRadians(1) + Angle::half(),
                                       AngularVelocity::zero());
        return {
            RobotStateWithId{.id = 1, .robot_state = yellow_robot_state1},
            RobotStateWithId{.id = 2, .robot_state = yellow_robot_state2},
        };
    }

    std::vector<RobotStateWithId> initBlueRobotStates()
    {
        RobotState blue_robot_state1(Point(1, 0), Vector(0, 0), Angle::fromRadians(0.5),
                                     AngularVelocity::zero());
        RobotState blue_robot_state2(Point(0, 0), Vector(0, 0), Angle::fromRadians(1.5),
                                     AngularVelocity::zero());
        RobotState blue_robot_state3(Point(-1, -1), Vector(0, 0), Angle::fromRadians(2.5),
                                     AngularVelocity::zero());
        return {
            RobotStateWithId{.id = 1, .robot_state = blue_robot_state1},
            RobotStateWithId{.id = 2, .robot_state = blue_robot_state2},
            RobotStateWithId{.id = 3, .robot_state = blue_robot_state3},
        };
    }

    std::vector<RobotStateWithId> initInvertedBlueRobotStates()
    {
        RobotState blue_robot_state1(Point(-1, 0), Vector(0, 0),
                                     Angle::fromRadians(0.5) + Angle::half(),
                                     AngularVelocity::zero());
        RobotState blue_robot_state2(Point(0, 0), Vector(0, 0),
                                     Angle::fromRadians(1.5) + Angle::half(),
                                     AngularVelocity::zero());
        RobotState blue_robot_state3(Point(1, 1), Vector(0, 0),
                                     Angle::fromRadians(2.5) + Angle::half(),
                                     AngularVelocity::zero());
        return {
            RobotStateWithId{.id = 1, .robot_state = blue_robot_state1},
            RobotStateWithId{.id = 2, .robot_state = blue_robot_state2},
            RobotStateWithId{.id = 3, .robot_state = blue_robot_state3},
        };
    }

    std::unique_ptr<SSLProto::SSL_DetectionFrame> initDetectionFrame()
    {
        const uint32_t camera_id    = 0;
        const uint32_t frame_number = 40391;

        return createSSLDetectionFrame(camera_id, current_time, frame_number,
                                       {ball_state}, yellow_robot_states,
                                       blue_robot_states);
    }

    std::unique_ptr<SSLProto::SSL_DetectionFrame> initDetectionFrameWithTime0()
    {
        const uint32_t camera_id    = 0;
        const uint32_t frame_number = 40391;

        return createSSLDetectionFrame(camera_id, Timestamp::fromSeconds(0), frame_number,
                                       {ball_state}, yellow_robot_states,
                                       blue_robot_states);
    }

    std::unique_ptr<SSLProto::SSL_DetectionFrame> initDetectionFrameWithFutureTime()
    {
        const uint32_t camera_id    = 0;
        const uint32_t frame_number = 40391;

        return createSSLDetectionFrame(camera_id, current_time + Duration::fromSeconds(1),
                                       frame_number, {ball_state}, yellow_robot_states,
                                       blue_robot_states);
    }

    std::unique_ptr<SSLProto::SSL_GeometryData> initSSLDivBGeomData()
    {
        Field field           = Field::createSSLDivisionBField();
        const float thickness = 0.005f;
        return createGeometryData(field, thickness);
    }

    World initWorld()
    {
        Field field(Field::createSSLDivisionBField());
        Ball ball(initBallState(), current_time);
        Team friendly_team;
        std::vector<Robot> friendly_robots;
        for (const auto &state : initYellowRobotStates())
        {
            friendly_robots.emplace_back(state.id, state.robot_state, current_time);
        }
        friendly_team.updateRobots(friendly_robots);
        friendly_team.assignGoalie(0);
        Team enemy_team;
        std::vector<Robot> enemy_robots;
        for (const auto &state : initBlueRobotStates())
        {
            enemy_robots.emplace_back(state.id, state.robot_state, current_time);
        }
        enemy_team.updateRobots(enemy_robots);
        enemy_team.assignGoalie(0);
        return World(field, ball, friendly_team, enemy_team);
    }

    World initInvertedWorld()
    {
        Field field(Field::createSSLDivisionBField());
        Ball ball(initInvertedBallState(), current_time);
        Team friendly_team;
        std::vector<Robot> friendly_robots;
        for (const auto &state : initInvertedYellowRobotStates())
        {
            friendly_robots.emplace_back(state.id, state.robot_state, current_time);
        }
        friendly_team.updateRobots(friendly_robots);
        friendly_team.assignGoalie(0);
        Team enemy_team;
        std::vector<Robot> enemy_robots;
        for (const auto &state : initInvertedBlueRobotStates())
        {
            enemy_robots.emplace_back(state.id, state.robot_state, current_time);
        }
        enemy_team.updateRobots(enemy_robots);
        enemy_team.assignGoalie(0);
        return World(field, ball, friendly_team, enemy_team);
    }

    std::unique_ptr<TbotsProto::RobotStatus> initRobotStatusId1()
    {
        auto robot_msg = std::make_unique<TbotsProto::RobotStatus>();

        robot_msg->set_robot_id(1);

        auto power_status_msg = std::make_unique<TbotsProto::PowerStatus>();
        power_status_msg->set_breakbeam_tripped(false);
        *(robot_msg->mutable_power_status()) = *power_status_msg;

        auto chipper_kicker_status = std::make_unique<TbotsProto::ChipperKickerStatus>();
        chipper_kicker_status->set_ms_since_chipper_fired(13);
        chipper_kicker_status->set_ms_since_kicker_fired(9);
        *(robot_msg->mutable_chipper_kicker_status()) = *chipper_kicker_status;

        return robot_msg;
    }

    std::unique_ptr<TbotsProto::RobotStatus> initRobotStatusId2()
    {
        auto robot_msg = std::make_unique<TbotsProto::RobotStatus>();

        robot_msg->set_robot_id(2);

        auto power_status_msg = std::make_unique<TbotsProto::PowerStatus>();
        power_status_msg->set_breakbeam_tripped(true);
        *(robot_msg->mutable_power_status()) = *power_status_msg;

        auto chipper_kicker_status = std::make_unique<TbotsProto::ChipperKickerStatus>();
        chipper_kicker_status->set_ms_since_chipper_fired(11);
        chipper_kicker_status->set_ms_since_kicker_fired(6);
        *(robot_msg->mutable_chipper_kicker_status()) = *chipper_kicker_status;

        return robot_msg;
    }

    std::unique_ptr<TbotsProto::RobotStatus> initHighCapErrorCode()
    {
        // Adding a HIGH_CAP error code to robotStatus of robot 2
        auto robot_msg = std::make_unique<TbotsProto::RobotStatus>();
        robot_msg->set_robot_id(2);
        robot_msg->add_error_code(TbotsProto::ErrorCode::HIGH_CAP);

        return robot_msg;
    }

    std::unique_ptr<TbotsProto::RobotStatus> initDribbleMotorHotErrorCode()
    {
        // Adding a DRIBBLER_MOTOR_HOT error code to robotStatus of robot 2
        auto robot_msg = std::make_unique<TbotsProto::RobotStatus>();
        robot_msg->set_robot_id(2);
        robot_msg->add_error_code(TbotsProto::ErrorCode::DRIBBLER_MOTOR_HOT);

        return robot_msg;
    }

    std::unique_ptr<TbotsProto::RobotStatus> initMultipleErrorCode()
    {
        auto robot_msg = std::make_unique<TbotsProto::RobotStatus>();
        robot_msg->set_robot_id(2);
        robot_msg->add_error_code(TbotsProto::ErrorCode::HIGH_CAP);
        robot_msg->add_error_code(TbotsProto::ErrorCode::DRIBBLER_MOTOR_HOT);
        return robot_msg;
    }

    std::unique_ptr<TbotsProto::RobotStatus> initNoErrorCode()
    {
        auto robot_msg = std::make_unique<TbotsProto::RobotStatus>();
        robot_msg->set_robot_id(2);

        return robot_msg;
    }

    std::unique_ptr<SSLProto::Referee> initRefereeIndirectYellow()
    {
        auto ref_msg = std::make_unique<SSLProto::Referee>();
        ref_msg->set_command(SSLProto::Referee_Command_INDIRECT_FREE_YELLOW);
        return ref_msg;
    }

    std::unique_ptr<SSLProto::Referee> initRefereeIndirectBlue()
    {
        auto ref_msg = std::make_unique<SSLProto::Referee>();
        ref_msg->set_command(SSLProto::Referee_Command_INDIRECT_FREE_BLUE);
        return ref_msg;
    }

    std::unique_ptr<SSLProto::Referee> initRefereeNormalStart()
    {
        auto ref_msg = std::make_unique<SSLProto::Referee>();
        ref_msg->set_command(SSLProto::Referee_Command_NORMAL_START);
        return ref_msg;
    }

    std::unique_ptr<SSLProto::Referee> initRefereeBallPlacementYellow()
    {
        auto ref_msg   = std::make_unique<SSLProto::Referee>();
        auto ref_point = std::make_unique<SSLProto::Referee_Point>();
        ref_point->set_x(50);
        ref_point->set_y(75);
        *(ref_msg->mutable_designated_position()) = *ref_point;
        ref_msg->set_command(SSLProto::Referee_Command_BALL_PLACEMENT_YELLOW);

        return ref_msg;
    }

    std::unique_ptr<SSLProto::Referee> initRefereeBallPlacementBlue()
    {
        auto ref_msg   = std::make_unique<SSLProto::Referee>();
        auto ref_point = std::make_unique<SSLProto::Referee_Point>();
        ref_point->set_x(20);
        ref_point->set_y(35);
        *(ref_msg->mutable_designated_position()) = *ref_point;
        ref_msg->set_command(SSLProto::Referee_Command_BALL_PLACEMENT_BLUE);

        return ref_msg;
    }

    std::unique_ptr<SSLProto::Referee> initRefereeGoalieId()
    {
        auto ref_msg           = std::make_unique<SSLProto::Referee>();
        auto ref_friendly_team = std::make_unique<SSLProto::Referee_TeamInfo>();
        auto ref_enemy_team    = std::make_unique<SSLProto::Referee_TeamInfo>();
        ref_friendly_team->set_goalkeeper(2);
        ref_enemy_team->set_goalkeeper(2);
        *(ref_msg->mutable_yellow()) = *ref_friendly_team;
        *(ref_msg->mutable_blue())   = *ref_enemy_team;

        return ref_msg;
    }
};

TEST_F(SensorFusionTest,
       test_making_chip_and_kick_robot_capabilities_unavailable_from_error_code)
{
    SensorProto sensor_msg;
    auto ssl_wrapper_packet =
        createSSLWrapperPacket(std::move(geom_data), initDetectionFrame());
    *(sensor_msg.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;
    *(sensor_msg.add_robot_status_msgs())  = *robot_status_msg_high_cap;
    sensor_fusion.processSensorProto(sensor_msg);

    std::optional<Robot> robot =
        sensor_fusion.getWorld().value().friendlyTeam().getRobotById(2);
    ASSERT_TRUE(robot);
    std::set<RobotCapability> robot_unavailable_capabilities =
        robot.value().getUnavailableCapabilities();
    EXPECT_EQ(2, robot_unavailable_capabilities.size());

    bool is_kick_disabled = robot_unavailable_capabilities.find(RobotCapability::Kick) !=
                            robot_unavailable_capabilities.end();
    ASSERT_TRUE(is_kick_disabled);

    bool is_chip_disabled = robot_unavailable_capabilities.find(RobotCapability::Chip) !=
                            robot_unavailable_capabilities.end();
    ASSERT_TRUE(is_chip_disabled);
}

TEST_F(SensorFusionTest,
       test_making_dribble_robot_capabilities_unavailable_from_error_code)
{
    SensorProto sensor_msg;
    auto ssl_wrapper_packet =
        createSSLWrapperPacket(std::move(geom_data), initDetectionFrame());
    *(sensor_msg.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;
    *(sensor_msg.add_robot_status_msgs())  = *robot_status_msg_dribble_motor_hot;
    sensor_fusion.processSensorProto(sensor_msg);

    std::optional<Robot> robot =
        sensor_fusion.getWorld().value().friendlyTeam().getRobotById(2);
    ASSERT_TRUE(robot);
    std::set<RobotCapability> robot_unavailable_capabilities =
        robot.value().getUnavailableCapabilities();
    EXPECT_EQ(1, robot_unavailable_capabilities.size());

    bool is_dribble_disabled =
        robot_unavailable_capabilities.find(RobotCapability::Dribble) !=
        robot_unavailable_capabilities.end();
    ASSERT_TRUE(is_dribble_disabled);
}

TEST_F(SensorFusionTest, test_making_all_robot_capabilities_unavailable_from_error_code)
{
    SensorProto sensor_msg;
    auto ssl_wrapper_packet =
        createSSLWrapperPacket(std::move(geom_data), initDetectionFrame());
    *(sensor_msg.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;
    *(sensor_msg.add_robot_status_msgs())  = *robot_status_msg_multiple_error_codes;
    sensor_fusion.processSensorProto(sensor_msg);

    std::optional<Robot> robot =
        sensor_fusion.getWorld().value().friendlyTeam().getRobotById(2);
    ASSERT_TRUE(robot);
    std::set<RobotCapability> robot_unavailable_capabilities =
        robot.value().getUnavailableCapabilities();
    EXPECT_EQ(3, robot_unavailable_capabilities.size());

    bool is_dribble_disabled =
        robot_unavailable_capabilities.find(RobotCapability::Dribble) !=
        robot_unavailable_capabilities.end();
    ASSERT_TRUE(is_dribble_disabled);

    bool is_kick_disabled = robot_unavailable_capabilities.find(RobotCapability::Kick) !=
                            robot_unavailable_capabilities.end();
    ASSERT_TRUE(is_kick_disabled);

    bool is_chip_disabled = robot_unavailable_capabilities.find(RobotCapability::Chip) !=
                            robot_unavailable_capabilities.end();
    ASSERT_TRUE(is_chip_disabled);
}

TEST_F(SensorFusionTest, test_emptying_robot_unavailable_capabilities_from_error_code)
{
    SensorProto sensor_msg;
    auto ssl_wrapper_packet =
        createSSLWrapperPacket(std::move(geom_data), initDetectionFrame());
    *(sensor_msg.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;
    *(sensor_msg.add_robot_status_msgs())  = *robot_status_msg_no_error_code;
    sensor_fusion.processSensorProto(sensor_msg);

    std::optional<Robot> robot =
        sensor_fusion.getWorld().value().friendlyTeam().getRobotById(2);
    ASSERT_TRUE(robot);
    std::set<RobotCapability> robot_unavailable_capabilities =
        robot.value().getUnavailableCapabilities();
    EXPECT_EQ(0, robot_unavailable_capabilities.size());
}

TEST_F(SensorFusionTest, test_geom_wrapper_packet)
{
    SensorProto sensor_msg;
    auto ssl_wrapper_packet = createSSLWrapperPacket(
        std::move(geom_data), std::unique_ptr<SSLProto::SSL_DetectionFrame>());
    *(sensor_msg.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;
    EXPECT_EQ(std::nullopt, sensor_fusion.getWorld());
    sensor_fusion.processSensorProto(sensor_msg);
    EXPECT_EQ(std::nullopt, sensor_fusion.getWorld());
}

TEST_F(SensorFusionTest, test_detection_frame_wrapper_packet)
{
    SensorProto sensor_msg;
    auto ssl_wrapper_packet = createSSLWrapperPacket(
        std::unique_ptr<SSLProto::SSL_GeometryData>(), initDetectionFrame());
    *(sensor_msg.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;
    EXPECT_EQ(std::nullopt, sensor_fusion.getWorld());
    sensor_fusion.processSensorProto(sensor_msg);
    EXPECT_EQ(std::nullopt, sensor_fusion.getWorld());
}

TEST_F(SensorFusionTest, test_inverted_detection_frame_wrapper_packet)
{
    EXPECT_EQ(std::nullopt, sensor_fusion.getWorld());

    SensorProto sensor_msg;
    SSLProto::Referee ssl_referee_packet;
    ssl_referee_packet.set_blue_team_on_positive_half(false);
    *(sensor_msg.mutable_ssl_referee_msg()) = ssl_referee_packet;

    sensor_fusion.processSensorProto(sensor_msg);

    auto ssl_wrapper_packet =
        createSSLWrapperPacket(std::move(geom_data), initDetectionFrame());

    SensorProto sensor_msg_2;
    *(sensor_msg_2.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;

    sensor_fusion.processSensorProto(sensor_msg_2);
    auto result = sensor_fusion.getWorld();

    ASSERT_TRUE(result);
    EXPECT_EQ(initInvertedWorld(), *result);
}

TEST_F(SensorFusionTest, test_complete_wrapper_packet)
{
    SensorProto sensor_msg;
    auto ssl_wrapper_packet =
        createSSLWrapperPacket(std::move(geom_data), initDetectionFrame());
    *(sensor_msg.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;
    EXPECT_EQ(std::nullopt, sensor_fusion.getWorld());
    sensor_fusion.processSensorProto(sensor_msg);
    ASSERT_TRUE(sensor_fusion.getWorld());
    World result = *sensor_fusion.getWorld();
    EXPECT_EQ(initWorld(), result);
}

TEST_F(SensorFusionTest, test_robot_status_msg_packet)
{
    SensorProto sensor_msg;
    *(sensor_msg.add_robot_status_msgs()) = *robot_status_msg_id_1;
    sensor_fusion.processSensorProto(sensor_msg);
    EXPECT_EQ(std::nullopt, sensor_fusion.getWorld());
}

TEST_F(SensorFusionTest, test_complete_wrapper_with_robot_status_msg_1_at_a_time)
{
    SensorProto sensor_msg_1;
    auto ssl_wrapper_packet =
        createSSLWrapperPacket(std::move(geom_data), initDetectionFrame());
    *(sensor_msg_1.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;
    *(sensor_msg_1.add_robot_status_msgs())  = *robot_status_msg_id_1;
    sensor_fusion.processSensorProto(sensor_msg_1);
    EXPECT_NE(std::nullopt, sensor_fusion.getWorld());
    ASSERT_TRUE(sensor_fusion.getWorld());
    World result_1 = *sensor_fusion.getWorld();
    // TODO (Issue #1276): Add checks on the state of World
    SensorProto sensor_msg_2;
    *(sensor_msg_2.add_robot_status_msgs()) = *robot_status_msg_id_2;
    sensor_fusion.processSensorProto(sensor_msg_2);
    World result_2 = *sensor_fusion.getWorld();
    // TODO (Issue #1276): Add checks on the state of World
}

TEST_F(SensorFusionTest, test_complete_wrapper_with_robot_status_msg_2_at_a_time)
{
    SensorProto sensor_msg_1;
    auto ssl_wrapper_packet =
        createSSLWrapperPacket(std::move(geom_data), initDetectionFrame());
    *(sensor_msg_1.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;
    sensor_fusion.processSensorProto(sensor_msg_1);
    EXPECT_NE(std::nullopt, sensor_fusion.getWorld());
    ASSERT_TRUE(sensor_fusion.getWorld());
    World result_1 = *sensor_fusion.getWorld();
    // TODO (Issue #1276): Add checks on the state of World
    SensorProto sensor_msg_2;
    *(sensor_msg_2.add_robot_status_msgs()) = *robot_status_msg_id_1;
    *(sensor_msg_2.add_robot_status_msgs()) = *robot_status_msg_id_2;
    sensor_fusion.processSensorProto(sensor_msg_2);
    World result_2 = *sensor_fusion.getWorld();
    // TODO (Issue #1276): Add checks on the state of World
}

TEST_F(SensorFusionTest, test_referee_yellow_then_normal)
{
    GameState expected_1;
    expected_1.updateRefereeCommand(RefereeCommand::INDIRECT_FREE_US);

    GameState expected_2 = expected_1;
    expected_2.updateRefereeCommand(RefereeCommand::NORMAL_START);

    SensorProto sensor_msg_1;
    auto ssl_wrapper_packet =
        createSSLWrapperPacket(std::move(geom_data), initDetectionFrame());
    // set vision msg so that world is valid
    *(sensor_msg_1.mutable_ssl_vision_msg())  = *ssl_wrapper_packet;
    *(sensor_msg_1.mutable_ssl_referee_msg()) = *referee_indirect_yellow;
    sensor_fusion.processSensorProto(sensor_msg_1);
    World result_1 = *sensor_fusion.getWorld();
    EXPECT_EQ(expected_1, result_1.gameState());

    SensorProto sensor_msg_2;
    *(sensor_msg_2.mutable_ssl_referee_msg()) = *referee_normal_start;
    sensor_fusion.processSensorProto(sensor_msg_2);
    World result_2 = *sensor_fusion.getWorld();
    EXPECT_EQ(expected_2, result_2.gameState());
}

TEST_F(SensorFusionTest, test_referee_blue_then_normal)
{
    GameState expected_1;
    expected_1.updateRefereeCommand(RefereeCommand::INDIRECT_FREE_THEM);

    GameState expected_2 = expected_1;
    expected_2.updateRefereeCommand(RefereeCommand::NORMAL_START);

    SensorProto sensor_msg_1;
    auto ssl_wrapper_packet =
        createSSLWrapperPacket(std::move(geom_data), initDetectionFrame());
    // set vision msg so that world is valid
    *(sensor_msg_1.mutable_ssl_vision_msg())  = *ssl_wrapper_packet;
    *(sensor_msg_1.mutable_ssl_referee_msg()) = *referee_indirect_blue;
    sensor_fusion.processSensorProto(sensor_msg_1);
    World result_1 = *sensor_fusion.getWorld();
    EXPECT_EQ(expected_1, result_1.gameState());

    SensorProto sensor_msg_2;
    *(sensor_msg_2.mutable_ssl_referee_msg()) = *referee_normal_start;
    sensor_fusion.processSensorProto(sensor_msg_2);
    World result_2 = *sensor_fusion.getWorld();
    EXPECT_EQ(expected_2, result_2.gameState());
}

TEST_F(SensorFusionTest, ball_placement_enemy_set_by_referee)
{
    SensorProto sensor_msg;

    // send Point(0.02, 0.035) to Referee message
    *(sensor_msg.mutable_ssl_referee_msg()) = *referee_ball_placement_blue;

    auto ssl_wrapper_packet =
        createSSLWrapperPacket(std::move(geom_data), initDetectionFrame());
    // set vision msg so that world is valid
    *(sensor_msg.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;

    sensor_fusion.processSensorProto(sensor_msg);
    World result = *sensor_fusion.getWorld();

    // result should be (0.02, 0.035)
    std::optional<Point> returned_point = result.gameState().getBallPlacementPoint();
    EXPECT_EQ(Point(0.02, 0.035), returned_point);
}

TEST_F(SensorFusionTest, ball_placement_friendly_invalid_point_set_by_referee)
{
    SensorProto sensor_msg;

    SSLProto::Referee ball_placement_ref_msg = *referee_ball_placement_yellow;

    // Remove ball placement point
    ball_placement_ref_msg.clear_designated_position();

    *(sensor_msg.mutable_ssl_referee_msg()) = ball_placement_ref_msg;
    auto ssl_wrapper_packet =
        createSSLWrapperPacket(std::move(geom_data), initDetectionFrame());
    // set vision msg so that world is valid
    *(sensor_msg.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;

    sensor_fusion.processSensorProto(sensor_msg);
    World result = *sensor_fusion.getWorld();

    // no valid ball placement point, so expect std::nullopt
    std::optional<Point> returned_point = result.gameState().getBallPlacementPoint();
    EXPECT_EQ(std::nullopt, returned_point);
}

TEST_F(SensorFusionTest,
       ball_placement_yellow_but_is_not_defending_positive_side_set_by_referee)
{
    SensorProto sensor_msg;

    // send (0.05, 0.075) to Referee message
    *(sensor_msg.mutable_ssl_referee_msg()) = *referee_ball_placement_yellow;

    auto ssl_wrapper_packet =
        createSSLWrapperPacket(std::move(geom_data), initDetectionFrame());
    // set vision msg so that world is valid
    *(sensor_msg.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;

    sensor_fusion.processSensorProto(sensor_msg);
    World result = *sensor_fusion.getWorld();

    Point returned_point = result.gameState().getBallPlacementPoint().value();
    EXPECT_EQ(Point(0.05, 0.075), returned_point);
}

TEST_F(SensorFusionTest,
       ball_placement_friendly_yellow_but_is_defending_positive_side_set_by_referee)
{
    SensorProto sensor_msg;

    // send Point(-0.05, -0.075) to Referee message
    *(sensor_msg.mutable_ssl_referee_msg()) = *referee_ball_placement_yellow;
    sensor_msg.mutable_ssl_referee_msg()->set_blue_team_on_positive_half(false);

    auto ssl_wrapper_packet =
        createSSLWrapperPacket(std::move(geom_data), initDetectionFrame());
    // set vision msg so that world is valid
    *(sensor_msg.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;

    sensor_fusion.processSensorProto(sensor_msg);
    World result = *sensor_fusion.getWorld();

    std::optional<Point> returned_point = result.gameState().getBallPlacementPoint();
    EXPECT_TRUE(returned_point);
    EXPECT_EQ(Point(-0.05, -0.075), returned_point.value());
}

TEST_F(SensorFusionTest,
       ball_placement_friendly_blue_but_is_defending_positive_side_set_by_referee)
{
    SensorProto sensor_msg;

    // Point (-0.02, -0.035) to Referee message
    *(sensor_msg.mutable_ssl_referee_msg()) = *referee_ball_placement_blue;
    sensor_msg.mutable_ssl_referee_msg()->set_blue_team_on_positive_half(true);

    TbotsProto::SensorFusionConfig sensor_fusion_blue_config =
        TbotsProto::SensorFusionConfig();
    sensor_fusion_blue_config.set_friendly_color_yellow(false);
    SensorFusion sensor_fusion_for_blue(sensor_fusion_blue_config);

    auto ssl_wrapper_packet =
        createSSLWrapperPacket(std::move(geom_data), initDetectionFrame());
    // set vision msg so that world is valid
    *(sensor_msg.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;

    sensor_fusion_for_blue.processSensorProto(sensor_msg);
    World result = *sensor_fusion_for_blue.getWorld();

    std::optional<Point> returned_point = result.gameState().getBallPlacementPoint();
    EXPECT_TRUE(returned_point);
    EXPECT_EQ(Point(-0.02, -0.035), returned_point.value());
}

TEST_F(SensorFusionTest,
       ball_placement_friendly_blue_but_is_not_defending_friendly_side_set_by_referee)
{
    SensorProto sensor_msg;

    // Point (0.02, 0.035) to Referee message
    *(sensor_msg.mutable_ssl_referee_msg()) = *referee_ball_placement_blue;
    sensor_msg.mutable_ssl_referee_msg()->set_blue_team_on_positive_half(false);

    TbotsProto::SensorFusionConfig sensor_fusion_blue_config =
        TbotsProto::SensorFusionConfig();
    sensor_fusion_blue_config.set_friendly_color_yellow(false);
    SensorFusion sensor_fusion_for_blue(sensor_fusion_blue_config);

    auto ssl_wrapper_packet =
        createSSLWrapperPacket(std::move(geom_data), initDetectionFrame());
    // set vision msg so that world is valid
    *(sensor_msg.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;

    sensor_fusion_for_blue.processSensorProto(sensor_msg);
    World result = *sensor_fusion_for_blue.getWorld();

    std::optional<Point> returned_point = result.gameState().getBallPlacementPoint();
    EXPECT_TRUE(returned_point);
    EXPECT_EQ(Point(0.02, 0.035), returned_point.value());
}

TEST_F(SensorFusionTest, goalie_id_set_by_referee)
{
    config.set_override_game_controller_friendly_goalie_id(false);
    config.set_override_game_controller_enemy_goalie_id(false);
    sensor_fusion = SensorFusion(config);

    SensorProto sensor_msg;

    *(sensor_msg.mutable_ssl_referee_msg()) = *referee_goalie_id;

    auto ssl_wrapper_packet =
        createSSLWrapperPacket(std::move(geom_data), initDetectionFrame());
    // set vision msg so that world is valid
    *(sensor_msg.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;

    sensor_fusion.processSensorProto(sensor_msg);
    World result = *sensor_fusion.getWorld();

    unsigned int friendly_goalie_id = result.friendlyTeam().getGoalieId().value();
    unsigned int enemy_goalie_id    = result.enemyTeam().getGoalieId().value();

    EXPECT_EQ(2, friendly_goalie_id);
    EXPECT_EQ(2, enemy_goalie_id);
}

TEST_F(SensorFusionTest, goalie_id_overridden)
{
    config.set_override_game_controller_friendly_goalie_id(true);
    config.set_override_game_controller_enemy_goalie_id(true);
    config.set_friendly_goalie_id(1);
    config.set_enemy_goalie_id(3);
    sensor_fusion = SensorFusion(config);

    SensorProto sensor_msg;

    *(sensor_msg.mutable_ssl_referee_msg()) = *referee_goalie_id;

    auto ssl_wrapper_packet =
        createSSLWrapperPacket(std::move(geom_data), initDetectionFrame());
    // set vision msg so that world is valid
    *(sensor_msg.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;

    sensor_fusion.processSensorProto(sensor_msg);
    World result = *sensor_fusion.getWorld();

    unsigned int friendly_goalie_id = result.friendlyTeam().getGoalieId().value();
    unsigned int enemy_goalie_id    = result.enemyTeam().getGoalieId().value();

    EXPECT_EQ(1, friendly_goalie_id);
    EXPECT_EQ(3, enemy_goalie_id);
}

TEST_F(SensorFusionTest, test_sensor_fusion_reset_behaviour_trigger_reset)
{
    config.set_override_game_controller_friendly_goalie_id(true);
    config.set_override_game_controller_enemy_goalie_id(true);
    config.set_friendly_goalie_id(0);
    config.set_enemy_goalie_id(0);
    sensor_fusion = SensorFusion(config);

    SensorProto sensor_msg;
    SensorProto sensor_msg_0;
    auto ssl_wrapper_packet =
        createSSLWrapperPacket(std::move(geom_data), initDetectionFrame());
    auto ssl_wrapper_packet_0 =
        createSSLWrapperPacket(std::move(geom_data), initDetectionFrameWithTime0());
    *(sensor_msg.mutable_ssl_vision_msg())   = *ssl_wrapper_packet;
    *(sensor_msg_0.mutable_ssl_vision_msg()) = *ssl_wrapper_packet_0;
    EXPECT_EQ(std::nullopt, sensor_fusion.getWorld());
    sensor_fusion.processSensorProto(sensor_msg);
    ASSERT_TRUE(sensor_fusion.getWorld());
    World result = *sensor_fusion.getWorld();
    EXPECT_EQ(initWorld(), result);
    for (unsigned int i = 0; i < SensorFusion::VISION_PACKET_RESET_COUNT_THRESHOLD; i++)
    {
        sensor_fusion.processSensorProto(sensor_msg_0);
        ASSERT_TRUE(sensor_fusion.getWorld());
        result = *sensor_fusion.getWorld();
        EXPECT_EQ(initWorld(), result);
    }
    sensor_fusion.processSensorProto(sensor_msg_0);
    EXPECT_FALSE(sensor_fusion.getWorld());
    sensor_fusion.processSensorProto(sensor_msg);
    ASSERT_TRUE(sensor_fusion.getWorld());
    result = *sensor_fusion.getWorld();
    EXPECT_EQ(initWorld(), result);
}

TEST_F(SensorFusionTest, test_sensor_fusion_reset_behaviour_ignore_bad_packets)
{
    config.set_override_game_controller_friendly_goalie_id(false);
    config.set_override_game_controller_enemy_goalie_id(false);
    config.set_friendly_goalie_id(0);
    config.set_enemy_goalie_id(0);
    sensor_fusion = SensorFusion(config);

    SensorProto sensor_msg;
    auto ssl_wrapper_packet =
        createSSLWrapperPacket(std::move(geom_data), initDetectionFrame());
    *(sensor_msg.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;

    SensorProto sensor_msg_0;
    auto ssl_wrapper_packet_0 =
        createSSLWrapperPacket(std::move(geom_data), initDetectionFrameWithTime0());
    *(sensor_msg_0.mutable_ssl_vision_msg()) = *ssl_wrapper_packet_0;

    SensorProto sensor_msg_future;
    auto ssl_wrapper_packet_future =
        createSSLWrapperPacket(std::move(geom_data), initDetectionFrameWithFutureTime());
    *(sensor_msg_future.mutable_ssl_vision_msg()) = *ssl_wrapper_packet_future;


    EXPECT_EQ(std::nullopt, sensor_fusion.getWorld());
    sensor_fusion.processSensorProto(sensor_msg);
    ASSERT_TRUE(sensor_fusion.getWorld());
    World result = *sensor_fusion.getWorld();
    EXPECT_EQ(initWorld(), result);
    for (unsigned int i = 0; i < SensorFusion::VISION_PACKET_RESET_COUNT_THRESHOLD - 1;
         i++)
    {
        sensor_fusion.processSensorProto(sensor_msg_0);
        ASSERT_TRUE(sensor_fusion.getWorld());
        result = *sensor_fusion.getWorld();
        EXPECT_EQ(initWorld(), result);
    }

    sensor_fusion.processSensorProto(sensor_msg_future);
    ASSERT_TRUE(sensor_fusion.getWorld());
    result = *sensor_fusion.getWorld();
    EXPECT_NE(initWorld(), result);
    for (unsigned int i = 0; i < SensorFusion::VISION_PACKET_RESET_COUNT_THRESHOLD; i++)
    {
        sensor_fusion.processSensorProto(sensor_msg_0);
        ASSERT_TRUE(sensor_fusion.getWorld());
    }

    sensor_fusion.processSensorProto(sensor_msg_0);
    EXPECT_FALSE(sensor_fusion.getWorld());
    sensor_fusion.processSensorProto(sensor_msg);
    ASSERT_TRUE(sensor_fusion.getWorld());
    result = *sensor_fusion.getWorld();
    EXPECT_EQ(initWorld(), result);
}
