#include <gtest/gtest.h>

#include "shared/2021_robot_constants.h"
#include "software/ai/hl/stp/tactic/move_primitive.h"
#include "software/ai/hl/stp/tactic/stop_primitive.h"
#include "software/test_util/test_util.h"

class PrimitiveTest : public testing::Test
{
   public:
    PrimitiveTest()
    {
        TestUtil::setFriendlyRobotPositions(world, {Point(0, 0)},
                                            Timestamp::fromSeconds(0));
    }

   protected:
    RobotConstants_t robot_constants = create2021RobotConstants();
    Robot robot                      = TestUtil::createRobotAtPos(Point(0, 0));
    std::shared_ptr<World> world     = TestUtil::createBlankTestingWorld();
    RobotNavigationObstacleFactory obstacle_factory =
        RobotNavigationObstacleFactory(TbotsProto::RobotNavigationObstacleConfig());
};

TEST_F(PrimitiveTest, test_create_move_primitive)
{
    const Point destination(-5, 1);

    std::shared_ptr<MovePrimitive> move_primitive = std::make_shared<MovePrimitive>(
        robot, destination, Angle::threeQuarter(),
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode::SAFE, TbotsProto::DribblerMode::INDEFINITE,
        TbotsProto::BallCollisionType::AVOID, AutoChipOrKick(), std::optional<double>());

    EXPECT_GT(move_primitive->getEstimatedPrimitiveCost(), 0.0);

    auto move_primitive_msg =
        move_primitive->generatePrimitiveProtoMessage(world, {}, obstacle_factory);

    ASSERT_TRUE(move_primitive_msg->has_move());
    auto generated_destination =
        move_primitive_msg->move().xy_traj_params().destination();
    EXPECT_EQ(generated_destination.x_meters(), destination.x());
    EXPECT_EQ(generated_destination.y_meters(), destination.y());
    EXPECT_EQ(move_primitive_msg->move().w_traj_params().final_angle().radians(),
              Angle::threeQuarter().toRadians());
    EXPECT_EQ(move_primitive_msg->move().dribbler_mode(),
              TbotsProto::DribblerMode::INDEFINITE);
    EXPECT_EQ(move_primitive_msg->move().auto_chip_or_kick().autochip_distance_meters(),
              0.0);
    EXPECT_EQ(move_primitive_msg->move().auto_chip_or_kick().autokick_speed_m_per_s(),
              0.0);
    EXPECT_EQ(move_primitive_msg->move().xy_traj_params().max_speed_mode(),
              TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
}

TEST_F(PrimitiveTest, test_create_move_primitive_with_sub_destination)
{
    // Add friendly defense area as a motion constraint and path plan around it
    const Point start(-4, -1.5);
    const Point destination(-4, 1.5);

    robot.updateState(
        RobotState(start, Vector(0, 0), Angle::zero(), AngularVelocity::zero()),
        Timestamp::fromSeconds(0.0));

    std::shared_ptr<MovePrimitive> primitive = std::make_shared<MovePrimitive>(
        robot, destination, Angle::threeQuarter(),
        TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND,
        TbotsProto::ObstacleAvoidanceMode::SAFE, TbotsProto::DribblerMode::INDEFINITE,
        TbotsProto::BallCollisionType::AVOID, AutoChipOrKick(), std::optional<double>());

    EXPECT_GT(primitive->getEstimatedPrimitiveCost(), 0.0);

    auto move_primitive_msg = primitive->generatePrimitiveProtoMessage(
        world, {TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA}, obstacle_factory);

    ASSERT_TRUE(move_primitive_msg->has_move());
    TbotsProto::MovePrimitive move_primitive          = move_primitive_msg->move();
    TbotsProto::TrajectoryPathParams2D xy_traj_params = move_primitive.xy_traj_params();
    auto generated_destination                        = xy_traj_params.destination();
    EXPECT_EQ(generated_destination.x_meters(), destination.x());
    EXPECT_EQ(generated_destination.y_meters(), destination.y());

    ASSERT_EQ(xy_traj_params.sub_destinations().size(), 1);
    // Greater than 0 connection time indicating a sub destination
    EXPECT_GT(xy_traj_params.sub_destinations(0).connection_time_s(), 0.0);
    // Sub destination should not be the default (0,0)
    EXPECT_NE(xy_traj_params.sub_destinations(0).sub_destination().x_meters(), 0.0);
    EXPECT_NE(xy_traj_params.sub_destinations(0).sub_destination().y_meters(), 0.0);

    EXPECT_EQ(move_primitive.w_traj_params().final_angle().radians(),
              Angle::threeQuarter().toRadians());
    EXPECT_EQ(move_primitive.dribbler_mode(), TbotsProto::DribblerMode::INDEFINITE);
    EXPECT_EQ(move_primitive.auto_chip_or_kick().autochip_distance_meters(), 0.0);
    EXPECT_EQ(move_primitive.auto_chip_or_kick().autokick_speed_m_per_s(), 0.0);
    EXPECT_EQ(move_primitive.xy_traj_params().max_speed_mode(),
              TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND);
}

TEST_F(PrimitiveTest, test_create_move_primitive_with_autochip)
{
    const Point destination(-5, 1);

    std::shared_ptr<MovePrimitive> move_primitive = std::make_shared<MovePrimitive>(
        robot, destination, Angle::threeQuarter(),
        TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND,
        TbotsProto::ObstacleAvoidanceMode::SAFE, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::AVOID,
        AutoChipOrKick({AutoChipOrKickMode::AUTOCHIP, 2.5}), std::optional<double>());

    EXPECT_GT(move_primitive->getEstimatedPrimitiveCost(), 0.0);

    auto move_primitive_msg =
        move_primitive->generatePrimitiveProtoMessage(world, {}, obstacle_factory);

    ASSERT_TRUE(move_primitive_msg->has_move());
    auto generated_destination =
        move_primitive_msg->move().xy_traj_params().destination();
    EXPECT_EQ(generated_destination.x_meters(), destination.x());
    EXPECT_EQ(generated_destination.y_meters(), destination.y());
    EXPECT_EQ(move_primitive_msg->move().w_traj_params().final_angle().radians(),
              Angle::threeQuarter().toRadians());
    EXPECT_EQ(move_primitive_msg->move().dribbler_mode(), TbotsProto::DribblerMode::OFF);
    EXPECT_EQ(move_primitive_msg->move().xy_traj_params().max_speed_mode(),
              TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND);

    ASSERT_TRUE(move_primitive_msg->move().has_auto_chip_or_kick());
    EXPECT_EQ(move_primitive_msg->move().auto_chip_or_kick().autochip_distance_meters(),
              2.5);
}

TEST_F(PrimitiveTest, test_create_move_primitive_with_autokick)
{
    const Point destination(-5, 1);

    std::shared_ptr<MovePrimitive> move_primitive = std::make_shared<MovePrimitive>(
        robot, destination, Angle::threeQuarter(),
        TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND,
        TbotsProto::ObstacleAvoidanceMode::SAFE, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::AVOID,
        AutoChipOrKick({AutoChipOrKickMode::AUTOKICK, 3.5}), std::optional<double>());

    EXPECT_GT(move_primitive->getEstimatedPrimitiveCost(), 0.0);

    auto move_primitive_msg =
        move_primitive->generatePrimitiveProtoMessage(world, {}, obstacle_factory);

    ASSERT_TRUE(move_primitive_msg->has_move());
    auto generated_destination =
        move_primitive_msg->move().xy_traj_params().destination();
    EXPECT_EQ(generated_destination.x_meters(), destination.x());
    EXPECT_EQ(generated_destination.y_meters(), destination.y());
    EXPECT_EQ(move_primitive_msg->move().w_traj_params().final_angle().radians(),
              Angle::threeQuarter().toRadians());
    EXPECT_EQ(move_primitive_msg->move().dribbler_mode(), TbotsProto::DribblerMode::OFF);
    EXPECT_EQ(move_primitive_msg->move().xy_traj_params().max_speed_mode(),
              TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND);

    ASSERT_TRUE(move_primitive_msg->move().has_auto_chip_or_kick());
    EXPECT_EQ(move_primitive_msg->move().auto_chip_or_kick().autokick_speed_m_per_s(),
              3.5);
}

TEST_F(PrimitiveTest, test_create_stop_primitive)
{
    StopPrimitive stop_primitive;
    EXPECT_EQ(stop_primitive.getEstimatedPrimitiveCost(), 0.0);

    auto primitive_proto =
        stop_primitive.generatePrimitiveProtoMessage(world, {}, obstacle_factory);
    EXPECT_TRUE(primitive_proto->has_stop());
}
