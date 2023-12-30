#include "software/ai/hl/stp/tactic/move_primitive.h"
#include "software/ai/hl/stp/tactic/stop_primitive.h"

#include <gtest/gtest.h>

#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
#include "software/test_util/test_util.h"

class PrimitiveTest : public testing::Test
{
    // TODO (NIMA): Figure out why these tests are failing
protected:
    RobotConstants_t robot_constants = create2021RobotConstants();
    Robot robot = TestUtil::createRobotAtPos(Point(0, 0));
    World world = TestUtil::setFriendlyRobotPositions(
        TestUtil::createBlankTestingWorld(), {Point(0, 0)}, Timestamp::fromSeconds(0));
    RobotNavigationObstacleFactory obstacle_factory = RobotNavigationObstacleFactory(TbotsProto::RobotNavigationObstacleConfig());
};

TEST_F(PrimitiveTest, test_create_move_primitive)
{
    const Point destination(-5, 1);

    std::shared_ptr<MovePrimitive> move_primitive = std::make_shared<MovePrimitive>(
        robot, destination, Angle::threeQuarter(),
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, TbotsProto::DribblerMode::INDEFINITE,
        TbotsProto::BallCollisionType::AVOID, AutoChipOrKick(), std::optional<double>());

    auto move_primitive_msg = move_primitive->generatePrimitiveProtoMessage(
        world, {}, obstacle_factory);

    ASSERT_TRUE(move_primitive_msg->has_move());
    auto generated_destination = move_primitive_msg->move().xy_traj_params().destination();
    EXPECT_EQ(generated_destination.x_meters(), destination.x());
    EXPECT_EQ(generated_destination.y_meters(), destination.y());
    EXPECT_EQ(move_primitive_msg->move().w_traj_params().final_angle().radians(),
            Angle::threeQuarter().toRadians());
    EXPECT_EQ(move_primitive_msg->move().dribbler_mode(),
              TbotsProto::DribblerMode::INDEFINITE);
    EXPECT_FALSE(move_primitive_msg->move().has_auto_chip_or_kick());
    EXPECT_EQ(move_primitive_msg->move().xy_traj_params().max_speed_mode(),
              TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT); // TODO (NIMA): Check auto chip or kick is disabled
}

TEST_F(PrimitiveTest, test_create_move_primitive_with_autochip)
{
    const Point destination(-5, 1);

    std::shared_ptr<MovePrimitive> move_primitive = std::make_shared<MovePrimitive>(
            robot, destination, Angle::threeQuarter(),
            TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND, TbotsProto::DribblerMode::OFF,
            TbotsProto::BallCollisionType::AVOID, AutoChipOrKick({AutoChipOrKickMode::AUTOCHIP, 2.5}),
            std::optional<double>());

    auto move_primitive_msg = move_primitive->generatePrimitiveProtoMessage(
            world, {}, obstacle_factory);

    ASSERT_TRUE(move_primitive_msg->has_move());
    auto generated_destination = move_primitive_msg->move().xy_traj_params().destination();
    EXPECT_EQ(generated_destination.x_meters(), destination.x());
    EXPECT_EQ(generated_destination.y_meters(), destination.y());
    EXPECT_EQ(move_primitive_msg->move().w_traj_params().final_angle().radians(),
              Angle::threeQuarter().toRadians());
    EXPECT_EQ(move_primitive_msg->move().dribbler_mode(),
              TbotsProto::DribblerMode::OFF);
    EXPECT_FALSE(move_primitive_msg->move().has_auto_chip_or_kick());
    EXPECT_EQ(move_primitive_msg->move().xy_traj_params().max_speed_mode(),
              TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND);

    ASSERT_TRUE(move_primitive_msg->move().has_auto_chip_or_kick());
    EXPECT_EQ(move_primitive_msg->move().auto_chip_or_kick().autochip_distance_meters(), 2.5);
}

TEST_F(PrimitiveTest, test_create_move_primitive_with_autokick)
{
    const Point destination(-5, 1);

    std::shared_ptr<MovePrimitive> move_primitive = std::make_shared<MovePrimitive>(
            robot, destination, Angle::threeQuarter(),
            TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND, TbotsProto::DribblerMode::OFF,
            TbotsProto::BallCollisionType::AVOID, AutoChipOrKick({AutoChipOrKickMode::AUTOKICK, 3.5}),
            std::optional<double>());

    auto move_primitive_msg = move_primitive->generatePrimitiveProtoMessage(
            world, {}, obstacle_factory);

    ASSERT_TRUE(move_primitive_msg->has_move());
    auto generated_destination = move_primitive_msg->move().xy_traj_params().destination();
    EXPECT_EQ(generated_destination.x_meters(), destination.x());
    EXPECT_EQ(generated_destination.y_meters(), destination.y());
    EXPECT_EQ(move_primitive_msg->move().w_traj_params().final_angle().radians(),
              Angle::threeQuarter().toRadians());
    EXPECT_EQ(move_primitive_msg->move().dribbler_mode(),
              TbotsProto::DribblerMode::OFF);
    EXPECT_FALSE(move_primitive_msg->move().has_auto_chip_or_kick());
    EXPECT_EQ(move_primitive_msg->move().xy_traj_params().max_speed_mode(),
              TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND);

    ASSERT_TRUE(move_primitive_msg->move().has_auto_chip_or_kick());
    EXPECT_EQ(move_primitive_msg->move().auto_chip_or_kick().autokick_speed_m_per_s(), 3.5);
}
