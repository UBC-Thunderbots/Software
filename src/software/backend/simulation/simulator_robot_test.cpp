#include "software/backend/simulation/simulator_robot.h"
#include "software/backend/simulation/physics/physics_robot.h"
#include "software/world/robot.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>

class SimulatorRobotTest : public testing::Test
{
protected:
    virtual void SetUp()
    {
        b2Vec2 gravity(0, 0);
        world = std::make_shared<b2World>(gravity);

        Robot robot_parameter(7, Point(1.04, -0.8), Vector(-1.5, 0), Angle::fromRadians(2.12),
                              AngularVelocity::fromRadians(-1.0), Timestamp::fromSeconds(0));
        physics_robot = std::make_shared<PhysicsRobot>(world, robot_parameter, 1.0);
        physics_robot_weak_ptr = std::weak_ptr<PhysicsRobot>(physics_robot);
        simulator_robot_misc_position = SimulatorRobot(physics_robot_weak_ptr);
    }

    SimulatorRobot simulator_robot_misc_position;

private:
    // Note: we declare the b2World first so it is destroyed last. If it is destroyed before the
    // physics robots, segfaults will occur
    std::shared_ptr<b2World> world;
    std::shared_ptr<PhysicsRobot> physics_robot;
    std::weak_ptr<PhysicsRobot> physics_robot_weak_ptr;
};

TEST_F(SimulatorRobotTest, test_robot_id) {
    EXPECT_EQ(simulator_robot_misc_position.getRobotId(), 7);
}

TEST_F(SimulatorRobotTest, test_get_position) {
    EXPECT_FLOAT_EQ(simulator_robot_misc_position.getPositionX(), 1.04);
    EXPECT_FLOAT_EQ(simulator_robot_misc_position.getPositionY(), -0.8);
}

TEST_F(SimulatorRobotTest, test_get_orientation) {
    EXPECT_FLOAT_EQ(simulator_robot_misc_position.getOrientation(), 2.12);
}

TEST_F(SimulatorRobotTest, test_get_linear_velocity) {
    EXPECT_NEAR(simulator_robot_misc_position.getVelocityX(), -1.5, 0.01);
    EXPECT_NEAR(simulator_robot_misc_position.getVelocityY(), 0.0, 0.01);
}

TEST_F(SimulatorRobotTest, test_get_angular_velocity) {
    EXPECT_FLOAT_EQ(simulator_robot_misc_position.getVelocityAngular(), -1.0);
}

TEST_F(SimulatorRobotTest, test_get_battery_voltage) {
    EXPECT_FLOAT_EQ(simulator_robot_misc_position.getBatteryVoltage(), 16.0);
}

TEST_F(SimulatorRobotTest, test_enabling_and_disabling_autokick) {
    EXPECT_FALSE(simulator_robot_misc_position.getAutokickSpeed());
    simulator_robot_misc_position.enableAutokick(1.5);
    ASSERT_TRUE(simulator_robot_misc_position.getAutokickSpeed());
    EXPECT_NEAR(simulator_robot_misc_position.getAutokickSpeed().value(), 1.5, 1e-6);
    simulator_robot_misc_position.disableAutokick();
    EXPECT_FALSE(simulator_robot_misc_position.getAutokickSpeed());
}

TEST_F(SimulatorRobotTest, test_enabling_and_disabling_autochip) {
    EXPECT_FALSE(simulator_robot_misc_position.getAutochipDistance());
    simulator_robot_misc_position.enableAutochip(2.1);
    ASSERT_TRUE(simulator_robot_misc_position.getAutochipDistance());
    EXPECT_NEAR(simulator_robot_misc_position.getAutochipDistance().value(), 2.1, 1e-6);
    simulator_robot_misc_position.disableAutochip();
    EXPECT_FALSE(simulator_robot_misc_position.getAutochipDistance());
}

TEST_F(SimulatorRobotTest, test_changing_dribbler_speed) {
    EXPECT_EQ(simulator_robot_misc_position.getDribblerSpeed(), 0);
    simulator_robot_misc_position.setDribblerSpeed(50);
    EXPECT_EQ(simulator_robot_misc_position.getDribblerSpeed(), 50);
    simulator_robot_misc_position.setDribblerSpeed(10499);
    EXPECT_EQ(simulator_robot_misc_position.getDribblerSpeed(), 10499);
    simulator_robot_misc_position.setDribblerSpeed(0);
    EXPECT_EQ(simulator_robot_misc_position.getDribblerSpeed(), 0);
}

TEST_F(SimulatorRobotTest, test_dribbler_coast) {
    EXPECT_EQ(simulator_robot_misc_position.getDribblerSpeed(), 0);
    simulator_robot_misc_position.setDribblerSpeed(50);
    EXPECT_EQ(simulator_robot_misc_position.getDribblerSpeed(), 50);
    simulator_robot_misc_position.dribblerCoast();
    EXPECT_EQ(simulator_robot_misc_position.getDribblerSpeed(), 0);
}

TEST_F(SimulatorRobotTest, test_get_dribbler_temperature) {
    EXPECT_EQ(simulator_robot_misc_position.getDribblerTemperatureDegC(), 25);
}
