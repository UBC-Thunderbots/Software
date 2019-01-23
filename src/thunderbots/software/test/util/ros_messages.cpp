#include "util/ros_messages.h"

#include <gtest/gtest.h>

using namespace std::chrono;

class ROSMessageUtilTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        auto epoch = time_point<std::chrono::steady_clock>();
        // An arbitrary fixed point in time. 10000 seconds after the epoch
        auto since_epoch = std::chrono::seconds(10000);

        current_time = epoch + since_epoch;
    }

    steady_clock::time_point current_time;
};

TEST_F(ROSMessageUtilTest, create_ball_from_ros_message)
{
    thunderbots_msgs::Ball ball_msg;

    ball_msg.position.x = 1.2;
    ball_msg.position.y = -8.07;
    ball_msg.velocity.x = 0;
    ball_msg.velocity.y = 3;

    Ball ball = Util::ROSMessages::createBallFromROSMessage(ball_msg);

    EXPECT_EQ(Ball(Point(1.2, -8.07), Vector(0, 3)), ball);
}

TEST_F(ROSMessageUtilTest, create_robot_from_ros_message)
{
    unsigned int id                  = 2;
    Point position                   = Point(1.2, -8.07);
    Vector velocity                  = Vector(0, 3);
    Angle orientation                = Angle::ofRadians(1.16);
    AngularVelocity angular_velocity = AngularVelocity::ofRadians(-0.85);

    thunderbots_msgs::Robot robot_msg;

    robot_msg.id                                = id;
    robot_msg.position.x                        = position.x();
    robot_msg.position.y                        = position.y();
    robot_msg.velocity.x                        = velocity.x();
    robot_msg.velocity.y                        = velocity.y();
    robot_msg.orientation                       = orientation.toRadians();
    robot_msg.angular_velocity                  = angular_velocity.toRadians();
    robot_msg.timestamp_nanoseconds_since_epoch = static_cast<uint64_t>(
        std::chrono::nanoseconds(current_time.time_since_epoch()).count());

    Robot robot = Util::ROSMessages::createRobotFromROSMessage(robot_msg);

    EXPECT_EQ(Robot(id, position, velocity, orientation, angular_velocity, current_time),
              robot);
}

TEST_F(ROSMessageUtilTest, create_field_from_ros_message)
{
    double length               = 9.0;
    double width                = 6.0;
    double goal_width           = 1.0;
    double defense_width        = 2.0;
    double defense_length       = 1.0;
    double boundary_width       = 0.3;
    double center_circle_radius = 0.5;

    thunderbots_msgs::Field field_msg;

    field_msg.field_length         = length;
    field_msg.field_width          = width;
    field_msg.defense_length       = defense_length;
    field_msg.defense_width        = defense_width;
    field_msg.goal_width           = goal_width;
    field_msg.boundary_width       = boundary_width;
    field_msg.center_circle_radius = center_circle_radius;

    Field field = Util::ROSMessages::createFieldFromROSMessage(field_msg);

    Field field_other = Field(length, width, defense_length, defense_width, goal_width,
                              boundary_width, center_circle_radius);

    EXPECT_EQ(field_other, field);
}

TEST_F(ROSMessageUtilTest, create_team_from_ros_message)
{
    auto robot_expiry_buffer_milliseconds = milliseconds(1000);
    unsigned int goalie_id                = 5;

    unsigned int robot_id            = 5;
    Point position                   = Point(1.2, -8.07);
    Vector velocity                  = Vector(0, 3);
    Angle orientation                = Angle::ofRadians(1.16);
    AngularVelocity angular_velocity = AngularVelocity::ofRadians(-0.85);
    auto timestamp_nanoseconds_since_epoch =
        std::chrono::duration_cast<nanoseconds>(current_time.time_since_epoch());

    thunderbots_msgs::Robot robot_msg;

    robot_msg.id               = robot_id;
    robot_msg.position.x       = position.x();
    robot_msg.position.y       = position.y();
    robot_msg.velocity.x       = velocity.x();
    robot_msg.velocity.y       = velocity.y();
    robot_msg.orientation      = orientation.toRadians();
    robot_msg.angular_velocity = angular_velocity.toRadians();
    robot_msg.timestamp_nanoseconds_since_epoch =
        static_cast<uint64_t>(timestamp_nanoseconds_since_epoch.count());

    thunderbots_msgs::Team team_msg;

    team_msg.robots.emplace_back(robot_msg);
    team_msg.goalie_id = goalie_id;
    team_msg.robot_expiry_buffer_milliseconds =
        static_cast<unsigned int>(robot_expiry_buffer_milliseconds.count());

    Team team = Util::ROSMessages::createTeamFromROSMessage(team_msg);

    Team team_other = Team(robot_expiry_buffer_milliseconds);
    Robot robot =
        Robot(robot_id, position, velocity, orientation, angular_velocity, current_time);

    team_other.updateRobots({robot});
    team_other.assignGoalie(goalie_id);

    EXPECT_EQ(team_other, team);
}

TEST_F(ROSMessageUtilTest, create_team_from_ros_message_with_non_member)
{
    auto robot_expiry_buffer_milliseconds = milliseconds(1000);
    unsigned int goalie_id                = 5;

    unsigned int robot_id            = 9;
    Point position                   = Point(1.2, -8.07);
    Vector velocity                  = Vector(0, 3);
    Angle orientation                = Angle::ofRadians(1.16);
    AngularVelocity angular_velocity = AngularVelocity::ofRadians(-0.85);
    auto timestamp_nanoseconds_since_epoch =
        std::chrono::duration_cast<nanoseconds>(current_time.time_since_epoch());

    thunderbots_msgs::Robot robot_msg;

    robot_msg.id               = robot_id;
    robot_msg.position.x       = position.x();
    robot_msg.position.y       = position.y();
    robot_msg.velocity.x       = velocity.x();
    robot_msg.velocity.y       = velocity.y();
    robot_msg.orientation      = orientation.toRadians();
    robot_msg.angular_velocity = angular_velocity.toRadians();
    robot_msg.timestamp_nanoseconds_since_epoch =
        static_cast<uint64_t>(timestamp_nanoseconds_since_epoch.count());

    thunderbots_msgs::Team team_msg;

    team_msg.robots.emplace_back(robot_msg);
    team_msg.goalie_id = goalie_id;
    team_msg.robot_expiry_buffer_milliseconds =
        static_cast<unsigned int>(robot_expiry_buffer_milliseconds.count());

    ASSERT_THROW(Team team = Util::ROSMessages::createTeamFromROSMessage(team_msg);

                 Team team_other = Team(robot_expiry_buffer_milliseconds);
                 Robot robot     = Robot(robot_id, position, velocity, orientation,
                                     angular_velocity, current_time);

                 team_other.updateRobots({robot}); team_other.assignGoalie(goalie_id);
                 , std::invalid_argument);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
