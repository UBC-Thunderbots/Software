#include "software/backend/simulation/simulator_robot.h"
#include "software/backend/simulation/simulator_ball.h"
#include "software/backend/simulation/physics/physics_world.h"
#include "software/world/robot.h"
#include "software/world/world.h"
#include "software/test_util/test_util.h"
#include "shared/constants.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>

class SimulatorRobotTest : public testing::Test
{
protected:
    /**
     * TODO
     * @param robot
     * @param ball
     * @param enemy_robot_positions
     * @return
     */
    std::tuple<std::shared_ptr<PhysicsWorld>, std::shared_ptr<SimulatorRobot>, std::shared_ptr<SimulatorBall>> createWorldWithEnemyRobots(Robot robot, Ball ball, std::vector<Point> enemy_robot_positions) {
        World world = ::Test::TestUtil::createBlankTestingWorld();
        world = ::Test::TestUtil::setEnemyRobotPositions(world, enemy_robot_positions, Timestamp::fromSeconds(0));

        world.mutableFriendlyTeam().updateRobots({robot});
        world.mutableBall() = ball;
        auto physics_world = std::make_shared<PhysicsWorld>(world);

        std::shared_ptr<SimulatorRobot> simulator_robot;
        auto physics_robot = physics_world->getFriendlyPhysicsRobots().at(0);
        if(physics_robot.lock()) {
            simulator_robot = std::make_shared<SimulatorRobot>(physics_robot);
        }else {
            ADD_FAILURE() << "Failed to create a SimulatorRobot because a PhysicsRobot was invalid" << std::endl;
        }

        std::shared_ptr<SimulatorBall> simulator_ball;
        auto physics_ball = physics_world->getPhysicsBall();
        if(physics_ball.lock()) {
            simulator_ball = std::make_shared<SimulatorBall>(physics_ball);
        }else {
            ADD_FAILURE() << "Failed to create a SimulatorRobot because a PhysicsRobot was invalid" << std::endl;
        }

        return std::make_tuple(physics_world, simulator_robot, simulator_ball);
    }

    std::tuple<std::shared_ptr<PhysicsWorld>, std::shared_ptr<SimulatorRobot>, std::shared_ptr<SimulatorBall>> createWorld(Robot robot, Ball ball) {
        return createWorldWithEnemyRobots(robot, ball, {});
    }

    Point getDribblingPoint(std::shared_ptr<SimulatorRobot> simulator_robot) {
        Point robot_position = simulator_robot->position();
        Angle robot_orientation = Angle::fromRadians(simulator_robot->getOrientation());
        return getDribblingPoint(robot_position, robot_orientation);
    }

    Point getDribblingPoint(const Point& robot_position, const Angle& robot_orientation) {
        double dribbler_depth = PhysicsRobot::dribbler_depth;
        Point dribbling_point = robot_position + Vector::createFromAngle(robot_orientation).normalize(DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS - dribbler_depth);
        return dribbling_point;
    }

    const Robot robot_non_zero_state = Robot(7, Point(1.04, -0.8), Vector(-1.5, 0), Angle::fromRadians(2.12), AngularVelocity::fromRadians(-1.0), Timestamp::fromSeconds(0));
    const Ball ball_zero_state = Ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
};

TEST_F(SimulatorRobotTest, test_robot_id) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_EQ(simulator_robot->getRobotId(), 7);
}

TEST_F(SimulatorRobotTest, test_get_position) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_FLOAT_EQ(simulator_robot->getPositionX(), 1.04);
    EXPECT_FLOAT_EQ(simulator_robot->getPositionY(), -0.8);
    EXPECT_LT((simulator_robot->position() - Point(1.04, -0.8)).length(), 0.001);
}

TEST_F(SimulatorRobotTest, test_get_orientation) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_FLOAT_EQ(simulator_robot->getOrientation(), 2.12);
}

TEST_F(SimulatorRobotTest, test_get_linear_velocity) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_NEAR(simulator_robot->getVelocityX(), -1.5, 0.01);
    EXPECT_NEAR(simulator_robot->getVelocityY(), 0.0, 0.01);
    EXPECT_LT((simulator_robot->velocity() - Vector(-1.5, 0)).length(), 0.01);
}

TEST_F(SimulatorRobotTest, test_get_angular_velocity) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_FLOAT_EQ(simulator_robot->getVelocityAngular(), -1.0);
}

TEST_F(SimulatorRobotTest, test_get_battery_voltage) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_FLOAT_EQ(simulator_robot->getBatteryVoltage(), 16.0);
}

TEST_F(SimulatorRobotTest, test_get_dribbler_temperature) {
    auto [world, simulator_robot, simulator_ball] = createWorld(robot_non_zero_state, ball_zero_state);
    EXPECT_EQ(simulator_robot->getDribblerTemperatureDegC(), 25);
}

TEST_F(SimulatorRobotTest, test_kick) {
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Point dribbling_point = getDribblingPoint(robot.position(), robot.orientation());
    Ball ball(dribbling_point, Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, simulator_ball] = createWorld(robot, ball);

    // Simulate for 1/2 second without kicking
    for(unsigned int i = 0; i < 30; i++) {
        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    // Make sure we didn't kick
    EXPECT_LT(simulator_ball->velocity().length(), 0.001);
    EXPECT_LT((simulator_ball->position() - dribbling_point).length(), 0.01);

    simulator_robot->kick(5.0);

    // Simulate for 1/2 second after kicking
    for(unsigned int i = 0; i < 30; i++) {
        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    EXPECT_LT((simulator_ball->velocity() - Vector(5, 0)).length(), 0.005);
}

TEST_F(SimulatorRobotTest, test_chip) {
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Point dribbling_point = getDribblingPoint(robot.position(), robot.orientation());
    Ball ball(dribbling_point, Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, simulator_ball] = createWorld(robot, ball);

    // Simulate for 1/2 second without chipping
    for(unsigned int i = 0; i < 30; i++) {
        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    // Make sure we didn't chip
    EXPECT_LT(simulator_ball->velocity().length(), 0.001);
    EXPECT_LT((simulator_ball->position() - dribbling_point).length(), 0.01);

    simulator_robot->chip(5.0);

    // Simulate for 1/2 second after chipping
    for(unsigned int i = 0; i < 30; i++) {
        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    EXPECT_GT(simulator_ball->velocity().x(), 1);
    EXPECT_NEAR(simulator_ball->velocity().y(), 0, 0.001);
}

TEST_F(SimulatorRobotTest, test_robot_does_not_kick_ball_when_autokick_disabled) {
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(Point(0.15, 0), Vector(-0.25, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, simulator_ball] = createWorld(robot, ball);

    EXPECT_LT((simulator_ball->velocity() - Vector(-0.25, 0)).length(), 0.001);

    // Simulate for 1/2 second
    for(unsigned int i = 0; i < 30; i++) {
        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    EXPECT_LT(simulator_ball->velocity().length(), 0.25);
}

TEST_F(SimulatorRobotTest, test_robot_kicks_ball_with_autokick_enabled) {
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(Point(0.25, 0), Vector(-1, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, simulator_ball] = createWorld(robot, ball);

    EXPECT_LT((simulator_ball->velocity() - Vector(-1, 0)).length(), 0.001);

    simulator_robot->enableAutokick(5);

    // Simulate for 1/2 second
    for(unsigned int i = 0; i < 30; i++) {
        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    EXPECT_LT((simulator_ball->velocity() - Vector(4, 0)).length(), 0.001);
}

TEST_F(SimulatorRobotTest, test_robot_chips_ball_with_autochip_enabled_and_ball_does_not_land_in_obstacle) {
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(Point(0.15, 0), Vector(-0.25, 0), Timestamp::fromSeconds(0));
    std::vector<Point> enemy_robot_positions = {Point(0.4, 0), Point(1.9, 0)};
    auto [world, simulator_robot, simulator_ball] = createWorldWithEnemyRobots(robot, ball, enemy_robot_positions);

    EXPECT_LT((simulator_ball->velocity() - Vector(-0.25, 0)).length(), 0.001);

    simulator_robot->enableAutochip(2);

    // Simulate for 1.5 seconds
    for(unsigned int i = 0; i < 90; i++) {
        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    EXPECT_GT(simulator_ball->velocity().x(), 1);
    EXPECT_GT(simulator_ball->position().x(), 2.0);
}

TEST_F(SimulatorRobotTest, test_robot_chips_ball_with_autochip_enabled_and_ball_lands_on_obstacle) {
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(Point(0.15, 0), Vector(-0.25, 0), Timestamp::fromSeconds(0));
    // Add an enemy robot right where the ball will land
    std::vector<Point> enemy_robot_positions = {Point(2.0, 0)};
    auto [world, simulator_robot, simulator_ball] = createWorldWithEnemyRobots(robot, ball, enemy_robot_positions);

    EXPECT_LT((simulator_ball->velocity() - Vector(-0.25, 0)).length(), 0.001);

    simulator_robot->enableAutochip(2);

    // Simulate for 1.5 seconds
    for(unsigned int i = 0; i < 90; i++) {
        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    // Check the ball's velocity is very roughly within a reasonable range. We don't care
    // exactly what it is, just that it hasn't changed an impossible amount by
    // getting caught inside the robot it landed in and then squeezed out
    EXPECT_GT(simulator_ball->velocity().x(), 1);
    EXPECT_LT(simulator_ball->velocity().x(), 3);
    EXPECT_GT(simulator_ball->position().x(), 2.0);
}

TEST_F(SimulatorRobotTest, test_ball_does_not_bounce_off_front_of_robot_when_dribbler_on) {
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(Point(0.25, 0), Vector(-3, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, simulator_ball] = createWorld(robot, ball);

    EXPECT_LT((simulator_ball->velocity() - Vector(-3, 0)).length(), 0.001);

    // We use an arbitrarily large number here for speed
    simulator_robot->setDribblerSpeed(10000);

    // Simulate for 2 seconds
    for(unsigned int i = 0; i < 120; i++) {
        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    // Make sure we haven't caused the robot to move significantly by dribbling
    EXPECT_LT((simulator_robot->position() - Point(0, 0)).length(), 0.003);
    // Check the ball has stuck to the dribbler
    EXPECT_LT((simulator_ball->velocity() - Vector(0, 0)).length(), 0.001);
    Point dribbling_point = getDribblingPoint(simulator_robot);
    EXPECT_LT((simulator_ball->position() - dribbling_point).length(), 0.01);
}

TEST_F(SimulatorRobotTest, test_dribble_ball_while_moving_backwards) {
    Robot robot(0, Point(0, 0), Vector(-0.5, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(Point(0.15, 0), Vector(-3, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, simulator_ball] = createWorld(robot, ball);

    EXPECT_LT((simulator_ball->velocity() - Vector(-3, 0)).length(), 0.001);

    // We use an arbitrarily large number here for speed
    simulator_robot->setDribblerSpeed(10000);

    // Simulate for 2 seconds
    for(unsigned int i = 0; i < 120; i++) {
        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    // Check the ball has stuck to the dribbler
    EXPECT_LT((simulator_ball->velocity() - simulator_robot->velocity()).length(), 0.001);
    Point dribbling_point = getDribblingPoint(simulator_robot);
    EXPECT_LT((simulator_ball->position() - dribbling_point).length(), 0.01);
}

TEST_F(SimulatorRobotTest, test_dribble_ball_while_moving_forwards) {
    Robot robot(0, Point(0, 0), Vector(0.5, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(Point(0.15, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, simulator_ball] = createWorld(robot, ball);

    // We use an arbitrarily large number here for speed
    simulator_robot->setDribblerSpeed(10000);

    // Simulate for 1 second
    for(unsigned int i = 0; i < 60; i++) {
        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    // Check the ball has stuck to the dribbler
    EXPECT_LT((simulator_ball->velocity() - simulator_robot->velocity()).length(), 0.02);
    Point dribbling_point = getDribblingPoint(simulator_robot);
    EXPECT_LT((simulator_ball->position() - dribbling_point).length(), 0.01);
}

TEST_F(SimulatorRobotTest, test_dribble_ball_while_moving_spinning_in_place) {
    Robot robot(0, Point(0, 0), Vector(0.0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(Point(0.15, 0), Vector(-0.5, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, simulator_ball] = createWorld(robot, ball);

    // We use an arbitrarily large number here for speed
    simulator_robot->setDribblerSpeed(10000);

    // Simulate for 0.5 second so the ball makes contact with the dribbler
    for(unsigned int i = 0; i < 30; i++) {
        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    // Check the ball has stuck to the dribbler
    EXPECT_LT((simulator_ball->velocity() - Vector(0.0, 0)).length(), 0.005);
    Point dribbling_point = getDribblingPoint(simulator_robot);
    EXPECT_LT((simulator_ball->position() - dribbling_point).length(), 0.01);

    // Accelerate the robot up to an angular velocity of 4*pi rad/s (ie. 2 rpm)
    // The iteration limit is a safety so we don't loop forever if applyForce is broken
    for(unsigned int i = 0; i < 120 && simulator_robot->getVelocityAngular() < 4 * M_PI; i++) {
        simulator_robot->applyWheelForceFrontLeft(0.3);
        simulator_robot->applyWheelForceBackLeft(0.3);
        simulator_robot->applyWheelForceBackRight(0.3);
        simulator_robot->applyWheelForceFrontRight(0.3);

        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
        std::cout << simulator_robot->getVelocityAngular() << std::endl;
    }

    // Check the ball has stuck to the dribbler
    dribbling_point = getDribblingPoint(simulator_robot);
    EXPECT_LT((simulator_ball->position() - dribbling_point).length(), 0.01);

    for(unsigned int i = 0; i < 120; i++) {
        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    // Check the ball has stuck to the dribbler
    dribbling_point = getDribblingPoint(simulator_robot);
    EXPECT_LT((simulator_ball->position() - dribbling_point).length(), 0.015);
}

TEST_F(SimulatorRobotTest, test_dribbler_coast) {
    Point robot_position = Point(0, 0);
    Angle robot_orientation = Angle::zero();
    Point dribbling_point = getDribblingPoint(robot_position, robot_orientation);
    Robot robot(0, robot_position, Vector(-0.5, 0), robot_orientation, AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball(dribbling_point, Vector(-0.1, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, simulator_ball] = createWorld(robot, ball);

    simulator_robot->dribblerCoast();

    // Simulate for 1 second
    for(unsigned int i = 0; i < 60; i++) {
        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    // Check the ball has not stuck to the dribbler
    dribbling_point = getDribblingPoint(simulator_robot);
    EXPECT_GT((simulator_ball->position() - dribbling_point).length(), 0.1);
}

TEST_F(SimulatorRobotTest, test_robot_drive_forward) {
    Robot robot(0, Point(0, 0), Vector(0.0, 0), Angle::quarter(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, _] = createWorld(robot, ball);

    for (unsigned int i = 0; i < 60; i++)
    {
        simulator_robot->applyWheelForceFrontLeft(-0.5);
        simulator_robot->applyWheelForceBackLeft(-0.5);
        simulator_robot->applyWheelForceBackRight(0.5);
        simulator_robot->applyWheelForceFrontRight(0.5);

        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    EXPECT_NEAR(simulator_robot->getVelocityX(), 0, 1e-5);
    EXPECT_GT(simulator_robot->getVelocityY(), 0.5);
    EXPECT_NEAR(simulator_robot->getVelocityAngular(), 0, 1 * M_PI / 180.0);

    EXPECT_NEAR(simulator_robot->getPositionX(), 0, 1e-5);
    EXPECT_GT(simulator_robot->getPositionY(), 0.25);
    EXPECT_NEAR(simulator_robot->getOrientation(), M_PI_2, 1 * M_PI / 180.0);
}

TEST_F(SimulatorRobotTest, test_robot_drive_backwards) {
    Robot robot(0, Point(0, 0), Vector(0.0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, _] = createWorld(robot, ball);

    for (unsigned int i = 0; i < 60; i++)
    {
        simulator_robot->applyWheelForceFrontLeft(0.5);
        simulator_robot->applyWheelForceBackLeft(0.5);
        simulator_robot->applyWheelForceBackRight(-0.5);
        simulator_robot->applyWheelForceFrontRight(-0.5);

        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    EXPECT_LT(simulator_robot->getVelocityX(), -0.5);
    EXPECT_NEAR(simulator_robot->getVelocityY(), 0, 1e-5);
    EXPECT_NEAR(simulator_robot->getVelocityAngular(), 0, 1 * M_PI / 180.0);

    EXPECT_LT(simulator_robot->getPositionX(), -0.25);
    EXPECT_NEAR(simulator_robot->getPositionY(), 0, 1e-5);
    EXPECT_NEAR(simulator_robot->getOrientation(), 0, 1 * M_PI / 180.0);
}

TEST_F(SimulatorRobotTest, test_robot_spin_clockwise) {
    Robot robot(0, Point(0, 0), Vector(0.0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, _] = createWorld(robot, ball);

    for (unsigned int i = 0; i < 60; i++)
    {
        simulator_robot->applyWheelForceFrontLeft(-0.5);
        simulator_robot->applyWheelForceBackLeft(-0.5);
        simulator_robot->applyWheelForceBackRight(-0.5);
        simulator_robot->applyWheelForceFrontRight(-0.5);

        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    EXPECT_LT((simulator_robot->position() - Point(0, 0)).length(), 0.05);
    EXPECT_LT((simulator_robot->velocity() - Vector(0, 0)).length(), 0.05);
    EXPECT_LT(simulator_robot->getVelocityAngular(), -10);
}

TEST_F(SimulatorRobotTest, test_robot_spin_counterclockwise) {
    Robot robot(0, Point(0, 0), Vector(0.0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, _] = createWorld(robot, ball);

    for (unsigned int i = 0; i < 60; i++)
    {
        simulator_robot->applyWheelForceFrontLeft(0.5);
        simulator_robot->applyWheelForceBackLeft(0.5);
        simulator_robot->applyWheelForceBackRight(0.5);
        simulator_robot->applyWheelForceFrontRight(0.5);

        world->stepSimulation(Duration::fromSeconds(1.0/60.0));
    }

    EXPECT_LT((simulator_robot->position() - Point(0, 0)).length(), 0.05);
    EXPECT_LT((simulator_robot->velocity() - Vector(0, 0)).length(), 0.05);
    EXPECT_GT(simulator_robot->getVelocityAngular(), 10);
}

TEST_F(SimulatorRobotTest, test_get_motor_speeds_when_robot_not_moving) {
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, _] = createWorld(robot, ball);

    EXPECT_EQ(simulator_robot->getMotorSpeedFrontLeft(), 0.0);
    EXPECT_EQ(simulator_robot->getMotorSpeedBackLeft(), 0.0);
    EXPECT_EQ(simulator_robot->getMotorSpeedBackRight(), 0.0);
    EXPECT_EQ(simulator_robot->getMotorSpeedFrontRight(), 0.0);
}

TEST_F(SimulatorRobotTest, test_get_motor_speeds_when_robot_moving_forwards) {
    Robot robot(0, Point(0, 0), Vector(1, 0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, _] = createWorld(robot, ball);

    EXPECT_LT(simulator_robot->getMotorSpeedFrontLeft(), -1.0);
    EXPECT_LT(simulator_robot->getMotorSpeedBackLeft(), -1.0);
    EXPECT_GT(simulator_robot->getMotorSpeedBackRight(), 1.0);
    EXPECT_GT(simulator_robot->getMotorSpeedFrontRight(), 1.0);
    EXPECT_LT(simulator_robot->getMotorSpeedFrontLeft(), simulator_robot->getMotorSpeedBackLeft());
    EXPECT_GT(simulator_robot->getMotorSpeedFrontRight(), simulator_robot->getMotorSpeedBackRight());
    EXPECT_EQ(simulator_robot->getMotorSpeedFrontLeft(), -simulator_robot->getMotorSpeedFrontRight());
    EXPECT_EQ(simulator_robot->getMotorSpeedBackLeft(), -simulator_robot->getMotorSpeedBackRight());
}

TEST_F(SimulatorRobotTest, test_get_motor_speeds_when_robot_moving_along_wheel_axis) {
    // Move along the axis of the front-left wheel. This means the front-left wheel is perpendicular
    // to the direction of motion, and we don't expect it to be spinning
    Vector robot_velocity = Vector::createFromAngle(Angle::fromDegrees(ANGLE_TO_ROBOT_FRONT_WHEELS_DEG));
    Robot robot(0, Point(0, 0), robot_velocity, Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, _] = createWorld(robot, ball);

    EXPECT_NEAR(simulator_robot->getMotorSpeedFrontLeft(), 0.0, 0.1);
    EXPECT_LT(simulator_robot->getMotorSpeedBackLeft(), -1.0);
    EXPECT_LT(simulator_robot->getMotorSpeedBackRight(), -0.1);
    EXPECT_GT(simulator_robot->getMotorSpeedFrontRight(), 1.0);
}

TEST_F(SimulatorRobotTest, test_get_motor_speeds_when_robot_spinning) {
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::threeQuarter(), Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, _] = createWorld(robot, ball);

    EXPECT_GT(simulator_robot->getMotorSpeedFrontLeft(), 1.0);
    EXPECT_GT(simulator_robot->getMotorSpeedBackLeft(), 1.0);
    EXPECT_GT(simulator_robot->getMotorSpeedBackRight(), 1.0);
    EXPECT_GT(simulator_robot->getMotorSpeedFrontRight(), 1.0);
}

TEST_F(SimulatorRobotTest, test_brake_motors_when_robot_spinning_with_positive_angular_velocity) {
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::threeQuarter(), Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, _] = createWorld(robot, ball);

    for (unsigned int i = 0; i < 60; i++)
    {
        simulator_robot->brakeMotorFrontLeft();
        simulator_robot->brakeMotorBackLeft();
        simulator_robot->brakeMotorBackRight();
        simulator_robot->brakeMotorFrontRight();

        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    EXPECT_NEAR(simulator_robot->getVelocityAngular(), 0.0, 1.0 * M_PI / 180.0);
}

TEST_F(SimulatorRobotTest, test_brake_motors_when_robot_spinning_with_negative_angular_velocity) {
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::fromDegrees(-2 * 360), Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, _] = createWorld(robot, ball);

    for (unsigned int i = 0; i < 60; i++)
    {
        simulator_robot->brakeMotorFrontLeft();
        simulator_robot->brakeMotorBackLeft();
        simulator_robot->brakeMotorBackRight();
        simulator_robot->brakeMotorFrontRight();

        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    EXPECT_NEAR(simulator_robot->getVelocityAngular(), 0.0, 1.0 * M_PI / 180.0);
}

TEST_F(SimulatorRobotTest, test_brake_motors_when_robot_moving_linearly) {
    Robot robot(0, Point(0, 0), Vector(2.5, 1.0), Angle::threeQuarter(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, _] = createWorld(robot, ball);

    for (unsigned int i = 0; i < 240; i++)
    {
        simulator_robot->brakeMotorFrontLeft();
        simulator_robot->brakeMotorBackLeft();
        simulator_robot->brakeMotorBackRight();
        simulator_robot->brakeMotorFrontRight();

        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    EXPECT_LT((simulator_robot->velocity() - Vector(0, 0)).length(), 0.01);
    EXPECT_NEAR(simulator_robot->getVelocityAngular(), 0.0, 1.0 * M_PI / 180.0);
}

TEST_F(SimulatorRobotTest, test_brake_motors_when_robot_moving_and_spinning) {
    Robot robot(0, Point(0, 0), Vector(2.5, 1.0), Angle::threeQuarter(), AngularVelocity::full(), Timestamp::fromSeconds(0));
    // Put the ball very far away so it does not interfere
    Ball ball(Point(10000, 10000), Vector(0, 0), Timestamp::fromSeconds(0));
    auto [world, simulator_robot, _] = createWorld(robot, ball);

    for (unsigned int i = 0; i < 240; i++)
    {
        simulator_robot->brakeMotorFrontLeft();
        simulator_robot->brakeMotorBackLeft();
        simulator_robot->brakeMotorBackRight();
        simulator_robot->brakeMotorFrontRight();

        world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    EXPECT_LT((simulator_robot->velocity() - Vector(0, 0)).length(), 0.01);
    EXPECT_NEAR(simulator_robot->getVelocityAngular(), 0.0, 1.0 * M_PI / 180.0);
}
