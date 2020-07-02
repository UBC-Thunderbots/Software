#include "software/sensor_fusion/sensor_fusion.h"

#include <gtest/gtest.h>

#include "software/parameter/dynamic_parameters.h"
#include "software/proto/message_translation/ssl_detection.h"
#include "software/proto/message_translation/ssl_geometry.h"
#include "software/proto/message_translation/ssl_wrapper.h"

class SensorFusionTest : public ::testing::Test
{
   public:
    SensorFusionTest()
        : sensor_fusion(DynamicParameters->getSensorFusionConfig()),
          yellow_robot_states(initYellowRobotStates()),
          blue_robot_states(initBlueRobotStates()),
          ball_state(Point(-1.2, 0), Vector(0.0, 0.0), 0.2),
          current_time(Timestamp::fromSeconds(8.03)),
          geom_data(initSSLDivBGeomData()),
          detection_frame(initDetectionFrame()),
          test_world(initWorld()),
          tbots_robot_msg_id_1(initTbotsRobotMsgId1()),
          tbots_robot_msg_id_2(initTbotsRobotMsgId2()),
          referee_indirect_yellow(initRefereeIndirectYellow()),
          referee_indirect_blue(initRefereeIndirectBlue()),
          referee_normal_start(initRefereeNormalStart())
    {
    }

    SensorFusion sensor_fusion;
    std::vector<RobotStateWithId> yellow_robot_states;
    std::vector<RobotStateWithId> blue_robot_states;
    BallState ball_state;
    Timestamp current_time;
    std::unique_ptr<SSL_GeometryData> geom_data;
    std::unique_ptr<SSL_DetectionFrame> detection_frame;
    // world associated with geom_data and detection_frame only
    World test_world;
    std::unique_ptr<TbotsRobotMsg> tbots_robot_msg_id_1;
    std::unique_ptr<TbotsRobotMsg> tbots_robot_msg_id_2;
    std::unique_ptr<SSL_Referee> referee_indirect_yellow;
    std::unique_ptr<SSL_Referee> referee_indirect_blue;
    std::unique_ptr<SSL_Referee> referee_normal_start;

   private:
    /**
     * All private functions serve to initialize test variables
     */
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

    std::unique_ptr<SSL_DetectionFrame> initDetectionFrame()
    {
        const uint32_t camera_id    = 0;
        const uint32_t frame_number = 40391;

        return createSSLDetectionFrame(camera_id, current_time, frame_number,
                                       {ball_state}, yellow_robot_states,
                                       blue_robot_states);
    }

    std::unique_ptr<SSL_GeometryData> initSSLDivBGeomData()
    {
        Field field           = Field::createSSLDivisionBField();
        const float thickness = 0.005f;
        return createGeometryData(field, thickness);
    }

    World initWorld()
    {
        Field field(Field::createSSLDivisionBField());
        Ball ball(TimestampedBallState(ball_state, current_time));
        // TODO (Issue #960): Dependency inject config so that we don't have to implicitly
        // assume that friendly is yellow
        Team friendly_team;
        std::vector<Robot> friendly_robots;
        for (const auto &state : yellow_robot_states)
        {
            friendly_robots.emplace_back(
                state.id, TimestampedRobotState(state.robot_state, current_time));
        }
        friendly_team.updateRobots(friendly_robots);
        Team enemy_team;
        std::vector<Robot> enemy_robots;
        for (const auto &state : blue_robot_states)
        {
            enemy_robots.emplace_back(
                state.id, TimestampedRobotState(state.robot_state, current_time));
        }
        enemy_team.updateRobots(enemy_robots);
        return World(field, ball, friendly_team, enemy_team);
    }

    std::unique_ptr<TbotsRobotMsg> initTbotsRobotMsgId1()
    {
        auto robot_msg = std::make_unique<TbotsRobotMsg>();

        robot_msg->set_robot_id(1);

        auto break_beam_msg = std::make_unique<BreakBeamStatus>();
        break_beam_msg->set_ball_in_beam(false);
        robot_msg->set_allocated_break_beam_status(break_beam_msg.release());

        auto chipper_kicker_status = std::make_unique<ChipperKickerStatus>();
        chipper_kicker_status->set_ms_since_chipper_fired(13);
        chipper_kicker_status->set_ms_since_kicker_fired(9);
        robot_msg->set_allocated_chipper_kicker_status(chipper_kicker_status.release());

        return std::move(robot_msg);
    }

    std::unique_ptr<TbotsRobotMsg> initTbotsRobotMsgId2()
    {
        auto robot_msg = std::make_unique<TbotsRobotMsg>();

        robot_msg->set_robot_id(2);

        auto break_beam_msg = std::make_unique<BreakBeamStatus>();
        break_beam_msg->set_ball_in_beam(true);
        robot_msg->set_allocated_break_beam_status(break_beam_msg.release());

        auto chipper_kicker_status = std::make_unique<ChipperKickerStatus>();
        chipper_kicker_status->set_ms_since_chipper_fired(11);
        chipper_kicker_status->set_ms_since_kicker_fired(6);
        robot_msg->set_allocated_chipper_kicker_status(chipper_kicker_status.release());

        return std::move(robot_msg);
    }

    std::unique_ptr<SSL_Referee> initRefereeIndirectYellow()
    {
        auto ref_msg = std::make_unique<SSL_Referee>();
        ref_msg->set_command(SSL_Referee_Command_INDIRECT_FREE_YELLOW);
        return ref_msg;
    }

    std::unique_ptr<SSL_Referee> initRefereeIndirectBlue()
    {
        auto ref_msg = std::make_unique<SSL_Referee>();
        ref_msg->set_command(SSL_Referee_Command_INDIRECT_FREE_BLUE);
        return ref_msg;
    }

    std::unique_ptr<SSL_Referee> initRefereeNormalStart()
    {
        auto ref_msg = std::make_unique<SSL_Referee>();
        ref_msg->set_command(SSL_Referee_Command_NORMAL_START);
        return ref_msg;
    }
};

TEST_F(SensorFusionTest, test_geom_wrapper_packet)
{
    SensorMsg sensor_msg;
    auto ssl_wrapper_packet =
        createWrapperPacket(std::move(geom_data), std::unique_ptr<SSL_DetectionFrame>());
    *(sensor_msg.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;
    EXPECT_EQ(std::nullopt, sensor_fusion.getWorld());
    sensor_fusion.updateWorld(sensor_msg);
    EXPECT_EQ(std::nullopt, sensor_fusion.getWorld());
}

TEST_F(SensorFusionTest, test_detection_frame_wrapper_packet)
{
    SensorMsg sensor_msg;
    auto ssl_wrapper_packet = createWrapperPacket(std::unique_ptr<SSL_GeometryData>(),
                                                  std::move(detection_frame));
    *(sensor_msg.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;
    EXPECT_EQ(std::nullopt, sensor_fusion.getWorld());
    sensor_fusion.updateWorld(sensor_msg);
    EXPECT_EQ(std::nullopt, sensor_fusion.getWorld());
}

TEST_F(SensorFusionTest, test_complete_wrapper_packet)
{
    SensorMsg sensor_msg;
    auto ssl_wrapper_packet =
        createWrapperPacket(std::move(geom_data), std::move(detection_frame));
    *(sensor_msg.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;
    EXPECT_EQ(std::nullopt, sensor_fusion.getWorld());
    sensor_fusion.updateWorld(sensor_msg);
    ASSERT_TRUE(sensor_fusion.getWorld());
    World result = *sensor_fusion.getWorld();
    EXPECT_EQ(test_world, result);
}

TEST_F(SensorFusionTest, test_tbots_robot_msg_packet)
{
    SensorMsg sensor_msg;
    *(sensor_msg.add_tbots_robot_msgs()) = *tbots_robot_msg_id_1;
    sensor_fusion.updateWorld(sensor_msg);
    EXPECT_EQ(std::nullopt, sensor_fusion.getWorld());
}

TEST_F(SensorFusionTest, test_complete_wrapper_with_tbots_robot_msg_1_at_a_time)
{
    SensorMsg sensor_msg_1;
    auto ssl_wrapper_packet =
        createWrapperPacket(std::move(geom_data), std::move(detection_frame));
    *(sensor_msg_1.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;
    *(sensor_msg_1.add_tbots_robot_msgs())   = *tbots_robot_msg_id_1;
    sensor_fusion.updateWorld(sensor_msg_1);
    EXPECT_NE(std::nullopt, sensor_fusion.getWorld());
    ASSERT_TRUE(sensor_fusion.getWorld());
    World result_1 = *sensor_fusion.getWorld();
    // TODO (Issue #1276): Add checks on the state of World
    SensorMsg sensor_msg_2;
    *(sensor_msg_2.add_tbots_robot_msgs()) = *tbots_robot_msg_id_2;
    sensor_fusion.updateWorld(sensor_msg_2);
    World result_2 = *sensor_fusion.getWorld();
    // TODO (Issue #1276): Add checks on the state of World
}

TEST_F(SensorFusionTest, test_complete_wrapper_with_tbots_robot_msg_2_at_a_time)
{
    SensorMsg sensor_msg_1;
    auto ssl_wrapper_packet =
        createWrapperPacket(std::move(geom_data), std::move(detection_frame));
    *(sensor_msg_1.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;
    sensor_fusion.updateWorld(sensor_msg_1);
    EXPECT_NE(std::nullopt, sensor_fusion.getWorld());
    ASSERT_TRUE(sensor_fusion.getWorld());
    World result_1 = *sensor_fusion.getWorld();
    // TODO (Issue #1276): Add checks on the state of World
    SensorMsg sensor_msg_2;
    *(sensor_msg_2.add_tbots_robot_msgs()) = *tbots_robot_msg_id_1;
    *(sensor_msg_2.add_tbots_robot_msgs()) = *tbots_robot_msg_id_2;
    sensor_fusion.updateWorld(sensor_msg_2);
    World result_2 = *sensor_fusion.getWorld();
    // TODO (Issue #1276): Add checks on the state of World
}

TEST_F(SensorFusionTest, test_referee_yellow_then_normal)
{
    // TODO (Issue #960): Dependency inject config so that we don't have to implicitly
    // assume that friendly is yellow
    GameState expected_1;
    expected_1.updateRefboxGameState(RefboxGameState::INDIRECT_FREE_US);

    GameState expected_2 = expected_1;
    expected_2.updateRefboxGameState(RefboxGameState::NORMAL_START);

    SensorMsg sensor_msg_1;
    auto ssl_wrapper_packet =
        createWrapperPacket(std::move(geom_data), std::move(detection_frame));
    // set vision msg so that world is valid
    *(sensor_msg_1.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;
    *(sensor_msg_1.mutable_ssl_refbox_msg()) = *referee_indirect_yellow;
    sensor_fusion.updateWorld(sensor_msg_1);
    World result_1 = *sensor_fusion.getWorld();
    EXPECT_EQ(expected_1, result_1.gameState());

    SensorMsg sensor_msg_2;
    *(sensor_msg_2.mutable_ssl_refbox_msg()) = *referee_normal_start;
    sensor_fusion.updateWorld(sensor_msg_2);
    World result_2 = *sensor_fusion.getWorld();
    EXPECT_EQ(expected_2, result_2.gameState());
}

TEST_F(SensorFusionTest, test_referee_blue_then_normal)
{
    // TODO (Issue #960): Dependency inject config so that we don't have to implicitly
    // assume that friendly is yellow
    GameState expected_1;
    expected_1.updateRefboxGameState(RefboxGameState::INDIRECT_FREE_THEM);

    GameState expected_2 = expected_1;
    expected_2.updateRefboxGameState(RefboxGameState::NORMAL_START);

    SensorMsg sensor_msg_1;
    auto ssl_wrapper_packet =
        createWrapperPacket(std::move(geom_data), std::move(detection_frame));
    // set vision msg so that world is valid
    *(sensor_msg_1.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;
    *(sensor_msg_1.mutable_ssl_refbox_msg()) = *referee_indirect_blue;
    sensor_fusion.updateWorld(sensor_msg_1);
    World result_1 = *sensor_fusion.getWorld();
    EXPECT_EQ(expected_1, result_1.gameState());

    SensorMsg sensor_msg_2;
    *(sensor_msg_2.mutable_ssl_refbox_msg()) = *referee_normal_start;
    sensor_fusion.updateWorld(sensor_msg_2);
    World result_2 = *sensor_fusion.getWorld();
    EXPECT_EQ(expected_2, result_2.gameState());
}
