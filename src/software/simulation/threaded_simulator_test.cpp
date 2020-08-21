#include "software/simulation/threaded_simulator.h"

#include <gtest/gtest.h>

#include "software/primitive/move_primitive.h"
#include "software/primitive/primitive.h"
#include "software/proto/message_translation/primitive_google_to_nanopb_converter.h"
#include "software/proto/message_translation/proto_creator_primitive_visitor.h"
#include "software/test_util/test_util.h"

class ThreadedSimulatorTest : public ::testing::Test
{
   protected:
    ThreadedSimulatorTest() : threaded_simulator(Field::createSSLDivisionBField()) {}

    void SetUp() override
    {
        most_recent_wrapper_packet = std::nullopt;
        callback_called            = false;
        auto callback              = [&](SSLProto::SSL_WrapperPacket packet) {
            callback_called            = true;
            most_recent_wrapper_packet = packet;
        };

        threaded_simulator.registerOnSSLWrapperPacketReadyCallback(callback);
    }

    void runSimulation(const Duration& duration)
    {
        threaded_simulator.startSimulation();
        // yield and sleep to give the simulation thread the best chance of running
        std::this_thread::yield();
        std::this_thread::sleep_for(std::chrono::milliseconds(
            static_cast<unsigned int>(duration.getMilliseconds())));
        threaded_simulator.stopSimulation();
    }

    std::optional<Point> getBallPosition()
    {
        if (!most_recent_wrapper_packet)
        {
            return std::nullopt;
        }

        if (!most_recent_wrapper_packet->has_detection())
        {
            return std::nullopt;
        }

        auto detection = most_recent_wrapper_packet->detection();
        if (detection.balls_size() < 1)
        {
            return std::nullopt;
        }

        auto ball = detection.balls(0);
        return Point(ball.x(), ball.y());
    }

    ThreadedSimulator threaded_simulator;
    bool callback_called;
    std::optional<SSLProto::SSL_WrapperPacket> most_recent_wrapper_packet;
};

TEST_F(ThreadedSimulatorTest, callbacks_triggered_during_simulation)
{
    runSimulation(Duration::fromSeconds(1));
    EXPECT_TRUE(callback_called);
}

TEST_F(ThreadedSimulatorTest, stop_simulation_when_not_running)
{
    threaded_simulator.stopSimulation();
    EXPECT_FALSE(callback_called);
}

TEST_F(ThreadedSimulatorTest, start_simulation_when_already_running)
{
    threaded_simulator.startSimulation();
    runSimulation(Duration::fromSeconds(1));
    EXPECT_TRUE(callback_called);
}

TEST_F(ThreadedSimulatorTest, start_and_stop_simulation_several_times)
{
    BallState ball_state(Point(0, 0), Vector(1, 0));
    threaded_simulator.setBallState(ball_state);

    runSimulation(Duration::fromSeconds(1.0));

    auto ball_position = getBallPosition();
    ASSERT_TRUE(ball_position);
    auto ball_x_1 = ball_position->x();
    EXPECT_GT(ball_x_1, 0);

    runSimulation(Duration::fromSeconds(1.0));

    ball_position = getBallPosition();
    ASSERT_TRUE(ball_position);
    auto ball_x_2 = ball_position->x();
    EXPECT_GT(ball_x_2, ball_x_1);

    runSimulation(Duration::fromSeconds(1.0));

    ball_position = getBallPosition();
    ASSERT_TRUE(ball_position);
    auto ball_x_3 = ball_position->x();
    EXPECT_GT(ball_x_3, ball_x_2);
}

TEST_F(ThreadedSimulatorTest, add_and_remove_ball_during_simulation)
{
    threaded_simulator.startSimulation();

    BallState ball_state(Point(0, 0), Vector(1, 0));
    threaded_simulator.setBallState(ball_state);

    // yield and sleep to give the simulation thread the best chance of running
    std::this_thread::yield();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    EXPECT_TRUE(callback_called);

    auto ball_position = getBallPosition();
    ASSERT_TRUE(ball_position);
    // Do a very coarse check for the ball's x coordinate because
    // we don't really know how much the simulation thread got to run.
    // This just roughly checks that physics is being updated as expected.
    EXPECT_NEAR(ball_position->x(), 1000, 800);
    EXPECT_NEAR(ball_position->y(), 0, 100);

    threaded_simulator.removeBall();

    // yield and sleep to give the simulation thread the best chance of running
    std::this_thread::yield();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    threaded_simulator.stopSimulation();
    ASSERT_FALSE(getBallPosition());
}

TEST_F(ThreadedSimulatorTest, add_robots_and_primitives_while_simulation_running)
{
    threaded_simulator.startSimulation();
    // yield and sleep to give the simulation thread the best chance of running
    std::this_thread::yield();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Simulate multiple robots with primitives to sanity check that everything is
    // connected properly and we can properly simulate multiple instances of the robot
    // firmware at once. We use the MovePrimitve because it is very commonly used and so
    // unlikely to be significantly changed or removed, and its behaviour is easy to
    // validate
    RobotState blue_robot_state1(Point(-1, 0), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero());
    RobotState blue_robot_state2(Point(-2, 1), Vector(0, 0), Angle::quarter(),
                                 AngularVelocity::zero());
    std::vector<RobotStateWithId> blue_robot_states = {
        RobotStateWithId{.id = 1, .robot_state = blue_robot_state1},
        RobotStateWithId{.id = 2, .robot_state = blue_robot_state2},
    };
    threaded_simulator.addBlueRobots(blue_robot_states);

    RobotState yellow_robot_state1(Point(1, 1.5), Vector(0, 0), Angle::half(),
                                   AngularVelocity::zero());
    RobotState yellow_robot_state2(Point(2.5, -1), Vector(0, 0), Angle::threeQuarter(),
                                   AngularVelocity::zero());
    std::vector<RobotStateWithId> yellow_robot_states = {
        RobotStateWithId{.id = 1, .robot_state = yellow_robot_state1},
        RobotStateWithId{.id = 2, .robot_state = yellow_robot_state2},
    };
    threaded_simulator.addYellowRobots(yellow_robot_states);

    std::unique_ptr<Primitive> blue_move_primitive1 = std::make_unique<MovePrimitive>(
        1, Point(-1, -1), Angle::zero(), 0.0, DribblerEnable::OFF, MoveType::NORMAL,
        AutochickType::NONE);
    std::unique_ptr<Primitive> blue_move_primitive2 = std::make_unique<MovePrimitive>(
        2, Point(-3, 0), Angle::half(), 0.0, DribblerEnable::OFF, MoveType::NORMAL,
        AutochickType::NONE);
    std::vector<std::unique_ptr<Primitive>> blue_robot_primitives;
    blue_robot_primitives.emplace_back(std::move(blue_move_primitive1));
    blue_robot_primitives.emplace_back(std::move(blue_move_primitive2));
    auto blue_primitives_ptr =
        std::make_shared<const std::vector<std::unique_ptr<Primitive>>>(
            std::move(blue_robot_primitives));
    for (const auto& primitive_ptr : *blue_primitives_ptr)
    {
        TbotsProto_Primitive primitive_msg = createNanoPbPrimitive(
            ProtoCreatorPrimitiveVisitor().createPrimitive(*primitive_ptr));

        threaded_simulator.setBlueRobotPrimitive(primitive_ptr->getRobotId(),
                                                 primitive_msg);
    }

    std::unique_ptr<Primitive> yellow_move_primitive1 = std::make_unique<MovePrimitive>(
        1, Point(1, 1), Angle::zero(), 0.0, DribblerEnable::OFF, MoveType::NORMAL,
        AutochickType::NONE);
    std::unique_ptr<Primitive> yellow_move_primitive2 = std::make_unique<MovePrimitive>(
        2, Point(3, -2), Angle::zero(), 0.0, DribblerEnable::OFF, MoveType::NORMAL,
        AutochickType::NONE);
    std::vector<std::unique_ptr<Primitive>> yellow_robot_primitives;
    yellow_robot_primitives.emplace_back(std::move(yellow_move_primitive1));
    yellow_robot_primitives.emplace_back(std::move(yellow_move_primitive2));
    auto yellow_primitives_ptr =
        std::make_shared<const std::vector<std::unique_ptr<Primitive>>>(
            std::move(yellow_robot_primitives));
    for (const auto& primitive_ptr : *yellow_primitives_ptr)
    {
        TbotsProto_Primitive primitive_msg = createNanoPbPrimitive(
            ProtoCreatorPrimitiveVisitor().createPrimitive(*primitive_ptr));

        threaded_simulator.setYellowRobotPrimitive(primitive_ptr->getRobotId(),
                                                   primitive_msg);
    }

    std::this_thread::yield();
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    threaded_simulator.stopSimulation();

    // TODO: These tests are currently very lenient, and don't test final velocities.
    //  This is because they currently rely on controller dynamics, and the existing
    //  bang-bang controller tends to overshoot with the current physics damping
    //  constants. In order to help decouple these tests from the controller / damping,
    //  the test tolerances are larger for now. They should be tightened again when the
    //  new controller is implemented.
    //  https://github.com/UBC-Thunderbots/Software/issues/1187

    auto ssl_wrapper_packet = most_recent_wrapper_packet;
    ASSERT_TRUE(ssl_wrapper_packet);
    ASSERT_TRUE(ssl_wrapper_packet->has_detection());
    auto detection_frame = ssl_wrapper_packet->detection();
    ASSERT_EQ(2, detection_frame.robots_yellow_size());
    ASSERT_EQ(2, detection_frame.robots_blue_size());

    auto yellow_robots  = detection_frame.robots_yellow();
    auto yellow_robot_1 = std::find_if(
        yellow_robots.begin(), yellow_robots.end(),
        [](SSLProto::SSL_DetectionRobot robot) { return robot.robot_id() == 1; });
    ASSERT_NE(yellow_robot_1, yellow_robots.end());
    EXPECT_NEAR(1000.0f, yellow_robot_1->x(), 200);
    EXPECT_NEAR(1000.0f, yellow_robot_1->y(), 200);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        Angle::zero(), Angle::fromRadians(yellow_robot_1->orientation()),
        Angle::fromDegrees(10)));

    auto yellow_robot_2 = std::find_if(
        yellow_robots.begin(), yellow_robots.end(),
        [](SSLProto::SSL_DetectionRobot robot) { return robot.robot_id() == 2; });
    ASSERT_NE(yellow_robot_2, yellow_robots.end());
    EXPECT_NEAR(3000.0f, yellow_robot_2->x(), 200);
    EXPECT_NEAR(-2000.0f, yellow_robot_2->y(), 200);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        Angle::zero(), Angle::fromRadians(yellow_robot_2->orientation()),
        Angle::fromDegrees(10)));

    auto blue_robots  = detection_frame.robots_blue();
    auto blue_robot_1 = std::find_if(
        blue_robots.begin(), blue_robots.end(),
        [](SSLProto::SSL_DetectionRobot robot) { return robot.robot_id() == 1; });
    ASSERT_NE(blue_robot_1, blue_robots.end());
    EXPECT_NEAR(-1000.0f, blue_robot_1->x(), 300);
    EXPECT_NEAR(-1000.0f, blue_robot_1->y(), 300);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        Angle::zero(), Angle::fromRadians(blue_robot_1->orientation()),
        Angle::fromDegrees(10)));

    auto blue_robot_2 = std::find_if(
        blue_robots.begin(), blue_robots.end(),
        [](SSLProto::SSL_DetectionRobot robot) { return robot.robot_id() == 2; });
    ASSERT_NE(blue_robot_2, blue_robots.end());
    EXPECT_NEAR(-3000.0f, blue_robot_2->x(), 300);
    EXPECT_NEAR(0.0f, blue_robot_2->y(), 300);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        Angle::half(), Angle::fromRadians(blue_robot_2->orientation()),
        Angle::fromDegrees(10)));
}

TEST_F(ThreadedSimulatorTest, add_individual_robots_at_position_while_simulation_running)
{
    threaded_simulator.startSimulation();
    // yield and sleep to give the simulation thread the best chance of running
    std::this_thread::yield();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    threaded_simulator.addYellowRobot(Point(0, 0));
    threaded_simulator.addBlueRobot(Point(2.1, 1));

    std::this_thread::yield();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    threaded_simulator.stopSimulation();

    auto ssl_wrapper_packet = most_recent_wrapper_packet;
    ASSERT_TRUE(ssl_wrapper_packet);
    ASSERT_TRUE(ssl_wrapper_packet->has_detection());
    auto detection_frame = ssl_wrapper_packet->detection();
    ASSERT_EQ(1, detection_frame.robots_yellow_size());
    ASSERT_EQ(1, detection_frame.robots_blue_size());
}
