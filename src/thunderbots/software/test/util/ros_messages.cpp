#include "util/ros_messages.h"

#include <gtest/gtest.h>

TEST(ROSMessageUtilTest, create_ball_from_ros_message)
{
    thunderbots_msgs::Ball ball_msg;

    ball_msg.position.x        = 1.2;
    ball_msg.position.y        = -8.07;
    ball_msg.velocity.x        = 0;
    ball_msg.velocity.y        = 3;
    ball_msg.timestamp_seconds = 1.4;

    Ball ball = Util::ROSMessages::createBallFromROSMessage(ball_msg);

    EXPECT_EQ(Ball(Point(1.2, -8.07), Vector(0, 3), Timestamp::fromSeconds(1.4)), ball);
}

TEST(ROSMessageUtilTest, convert_ball_to_ros_message)
{
    Ball ball = Ball(Point(-1.3, 0.2), Vector(0, 5.4), Timestamp::fromSeconds(123.45));
    thunderbots_msgs::Ball ball_msg = Util::ROSMessages::convertBallToROSMessage(ball);

    EXPECT_EQ(-1.3, ball_msg.position.x);
    EXPECT_EQ(0.2, ball_msg.position.y);
    EXPECT_EQ(0, ball_msg.velocity.x);
    EXPECT_EQ(5.4, ball_msg.velocity.y);
    EXPECT_EQ(123.45, ball_msg.timestamp_seconds);
}

TEST(ROSMessageUtilTest, create_robot_from_ros_message)
{
    unsigned int id                  = 2;
    Point position                   = Point(1.2, -8.07);
    Vector velocity                  = Vector(0, 3);
    Angle orientation                = Angle::ofRadians(1.16);
    AngularVelocity angular_velocity = AngularVelocity::ofRadians(-0.85);

    thunderbots_msgs::Robot robot_msg;

    robot_msg.id                = id;
    robot_msg.position.x        = position.x();
    robot_msg.position.y        = position.y();
    robot_msg.velocity.x        = velocity.x();
    robot_msg.velocity.y        = velocity.y();
    robot_msg.orientation       = orientation.toRadians();
    robot_msg.angular_velocity  = angular_velocity.toRadians();
    robot_msg.timestamp_seconds = 5.5;

    Robot robot = Util::ROSMessages::createRobotFromROSMessage(robot_msg);

    EXPECT_EQ(Robot(id, position, velocity, orientation, angular_velocity,
                    Timestamp::fromSeconds(5.5)),
              robot);
}

TEST(ROSMessageUtilTest, convert_robot_to_ros_message)
{
    Robot robot = Robot(1, Point(0, -5.01), Vector(0, 0), Angle::quarter(),
                        AngularVelocity::ofRadians(1), Timestamp::fromSeconds(4.4));
    thunderbots_msgs::Robot robot_msg =
        Util::ROSMessages::convertRobotToROSMessage(robot);

    EXPECT_EQ(1, robot_msg.id);
    EXPECT_EQ(0, robot_msg.position.x);
    EXPECT_EQ(-5.01, robot_msg.position.y);
    EXPECT_EQ(0, robot_msg.velocity.x);
    EXPECT_EQ(0, robot_msg.velocity.y);
    EXPECT_EQ(Angle::quarter().toRadians(), robot_msg.orientation);
    EXPECT_EQ(1, robot_msg.angular_velocity);
    EXPECT_EQ(4.4, robot_msg.timestamp_seconds);
}

TEST(ROSMessageUtilTest, create_field_from_ros_message)
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

TEST(ROSMessageUtilTest, convert_field_to_ros_message)
{
    double length               = 9.0;
    double width                = 6.0;
    double goal_width           = 1.0;
    double defense_width        = 2.0;
    double defense_length       = 1.0;
    double boundary_width       = 0.3;
    double center_circle_radius = 0.5;

    Field field = Field(length, width, defense_length, defense_width, goal_width,
                        boundary_width, center_circle_radius);

    thunderbots_msgs::Field field_msg =
        Util::ROSMessages::convertFieldToROSMessage(field);

    EXPECT_EQ(length, field_msg.field_length);
    EXPECT_EQ(width, field_msg.field_width);
    EXPECT_EQ(defense_length, field_msg.defense_length);
    EXPECT_EQ(defense_width, field_msg.defense_width);
    EXPECT_EQ(goal_width, field_msg.goal_width);
    EXPECT_EQ(boundary_width, field_msg.boundary_width);
    EXPECT_EQ(center_circle_radius, field_msg.center_circle_radius);
}

TEST(ROSMessageUtilTest, create_team_from_ros_message)
{
    auto robot_expiry_buffer_duration = Duration::fromMilliseconds(1000);
    unsigned int goalie_id            = 5;

    unsigned int robot_id            = 5;
    Point position                   = Point(1.2, -8.07);
    Vector velocity                  = Vector(0, 3);
    Angle orientation                = Angle::ofRadians(1.16);
    AngularVelocity angular_velocity = AngularVelocity::ofRadians(-0.85);

    thunderbots_msgs::Robot robot_msg;

    robot_msg.id                = robot_id;
    robot_msg.position.x        = position.x();
    robot_msg.position.y        = position.y();
    robot_msg.velocity.x        = velocity.x();
    robot_msg.velocity.y        = velocity.y();
    robot_msg.orientation       = orientation.toRadians();
    robot_msg.angular_velocity  = angular_velocity.toRadians();
    robot_msg.timestamp_seconds = 123.45;

    thunderbots_msgs::Team team_msg;

    team_msg.robots.emplace_back(robot_msg);
    team_msg.goalie_id = goalie_id;
    team_msg.robot_expiry_buffer_milliseconds =
        robot_expiry_buffer_duration.getMilliseconds();

    Team team = Util::ROSMessages::createTeamFromROSMessage(team_msg);

    Team team_other = Team(robot_expiry_buffer_duration);
    Robot robot     = Robot(robot_id, position, velocity, orientation, angular_velocity,
                        Timestamp::fromSeconds(123.45));

    team_other.updateRobots({robot});
    team_other.assignGoalie(goalie_id);

    EXPECT_EQ(team_other, team);
}

TEST(ROSMessageUtilTest, create_team_from_ros_message_with_non_member)
{
    auto robot_expiry_buffer_duration = Duration::fromMilliseconds(1000);
    unsigned int goalie_id            = 5;

    unsigned int robot_id            = 9;
    Point position                   = Point(1.2, -8.07);
    Vector velocity                  = Vector(0, 3);
    Angle orientation                = Angle::ofRadians(1.16);
    AngularVelocity angular_velocity = AngularVelocity::ofRadians(-0.85);

    thunderbots_msgs::Robot robot_msg;

    robot_msg.id                = robot_id;
    robot_msg.position.x        = position.x();
    robot_msg.position.y        = position.y();
    robot_msg.velocity.x        = velocity.x();
    robot_msg.velocity.y        = velocity.y();
    robot_msg.orientation       = orientation.toRadians();
    robot_msg.angular_velocity  = angular_velocity.toRadians();
    robot_msg.timestamp_seconds = 33;

    thunderbots_msgs::Team team_msg;

    team_msg.robots.emplace_back(robot_msg);
    team_msg.goalie_id = goalie_id;
    team_msg.robot_expiry_buffer_milliseconds =
        robot_expiry_buffer_duration.getMilliseconds();

    ASSERT_THROW(Team team = Util::ROSMessages::createTeamFromROSMessage(team_msg);

                 Team team_other = Team(robot_expiry_buffer_duration);
                 Robot robot     = Robot(robot_id, position, velocity, orientation,
                                     angular_velocity, Timestamp::fromSeconds(33));

                 team_other.updateRobots({robot}); team_other.assignGoalie(goalie_id);
                 , std::invalid_argument);
}

TEST(ROSMessageUtilTest, convert_team_with_goalie_to_ros_message)
{
    Robot robot = Robot(1, Point(0, -5.01), Vector(0, 0), Angle::quarter(),
                        AngularVelocity::ofRadians(1), Timestamp::fromSeconds(4.4));

    Team team = Team(Duration::fromMilliseconds(500));
    team.updateRobots({robot});
    team.assignGoalie(1);

    thunderbots_msgs::Team team_msg = Util::ROSMessages::convertTeamToROSMessage(team);

    EXPECT_EQ(500, team_msg.robot_expiry_buffer_milliseconds);
    EXPECT_EQ(1, team_msg.goalie_id);
    EXPECT_EQ(1, team_msg.robots.size());

    EXPECT_EQ(1, team_msg.robots.at(0).id);
    EXPECT_EQ(0, team_msg.robots.at(0).position.x);
    EXPECT_EQ(-5.01, team_msg.robots.at(0).position.y);
    EXPECT_EQ(0, team_msg.robots.at(0).velocity.x);
    EXPECT_EQ(0, team_msg.robots.at(0).velocity.y);
    EXPECT_EQ(Angle::quarter().toRadians(), team_msg.robots.at(0).orientation);
    EXPECT_EQ(1, team_msg.robots.at(0).angular_velocity);
    EXPECT_EQ(4.4, team_msg.robots.at(0).timestamp_seconds);
}

TEST(ROSMessageUtilTest, convert_team_with_no_goalie_to_ros_message)
{
    Robot robot = Robot(1, Point(0, -5.01), Vector(0, 0), Angle::quarter(),
                        AngularVelocity::ofRadians(1), Timestamp::fromSeconds(4.4));

    Team team = Team(Duration::fromMilliseconds(500));
    team.updateRobots({robot});
    team.clearGoalie();

    thunderbots_msgs::Team team_msg = Util::ROSMessages::convertTeamToROSMessage(team);

    EXPECT_EQ(500, team_msg.robot_expiry_buffer_milliseconds);
    EXPECT_EQ(-1, team_msg.goalie_id);
    EXPECT_EQ(1, team_msg.robots.size());

    EXPECT_EQ(1, team_msg.robots.at(0).id);
    EXPECT_EQ(0, team_msg.robots.at(0).position.x);
    EXPECT_EQ(-5.01, team_msg.robots.at(0).position.y);
    EXPECT_EQ(0, team_msg.robots.at(0).velocity.x);
    EXPECT_EQ(0, team_msg.robots.at(0).velocity.y);
    EXPECT_EQ(Angle::quarter().toRadians(), team_msg.robots.at(0).orientation);
    EXPECT_EQ(1, team_msg.robots.at(0).angular_velocity);
    EXPECT_EQ(4.4, team_msg.robots.at(0).timestamp_seconds);
}

TEST(ROSMessageUtilTest, transform_ball)
{
    Ball ball = Ball(Point(-1.3, 0.2), Vector(-1.3, 5.4), Timestamp::fromSeconds(123.45));
    thunderbots_msgs::Ball ball_msg = Util::ROSMessages::convertBallToROSMessage(ball);

    thunderbots_msgs::Ball transformed_ball_msg = Util::ROSMessages::transformBallMessage(ball_msg);

    EXPECT_EQ(1.3, transformed_ball_msg.position.x);
    EXPECT_EQ(-0.2, transformed_ball_msg.position.y);
    EXPECT_EQ(1.3, transformed_ball_msg.velocity.x);
    EXPECT_EQ(-5.4, transformed_ball_msg.velocity.y);
    EXPECT_EQ(Timestamp::fromSeconds(123.45).getSeconds(), transformed_ball_msg.timestamp_seconds);
}

TEST(ROSMessageUtilTest, transform_robot)
{
    Robot robot = Robot(1, Point(1, -5.01), Vector(-2, 3), Angle::quarter(),
                        AngularVelocity::ofRadians(1), Timestamp::fromSeconds(4.4));
    thunderbots_msgs::Robot robot_msg = Util::ROSMessages::convertRobotToROSMessage(robot);

    thunderbots_msgs::Robot transformed_robot_msg = Util::ROSMessages::transformRobotMessage(robot_msg);

    EXPECT_EQ(1, transformed_robot_msg.id);
    EXPECT_EQ(-1, transformed_robot_msg.position.x);
    EXPECT_EQ(5.01, transformed_robot_msg.position.y);
    EXPECT_EQ(2, transformed_robot_msg.velocity.x);
    EXPECT_EQ(-3, transformed_robot_msg.velocity.y);
    EXPECT_EQ(Angle::quarter().toRadians() + Angle::half().toRadians(), transformed_robot_msg.orientation);
    EXPECT_EQ(AngularVelocity::ofRadians(1).toRadians(), transformed_robot_msg.angular_velocity);
    EXPECT_EQ(Timestamp::fromSeconds(4.4).getSeconds(), transformed_robot_msg.timestamp_seconds);
}

TEST(ROSMessageUtilTest, transform_robots)
{
    Robot robot1 = Robot(1, Point(1, -5.01), Vector(-2, 3), Angle::quarter(),
                        AngularVelocity::ofRadians(1), Timestamp::fromSeconds(4.4));
    Robot robot2 = Robot(2, Point(10.4, -2.05), Vector(4, -6), Angle::half(),
                        AngularVelocity::ofRadians(1), Timestamp::fromSeconds(5.6));

    thunderbots_msgs::Robot robot1_msg = Util::ROSMessages::convertRobotToROSMessage(robot1);
    thunderbots_msgs::Robot robot2_msg = Util::ROSMessages::convertRobotToROSMessage(robot2);

    std::vector<thunderbots_msgs::Robot> robot_msgs = {robot1_msg, robot2_msg};

    std::vector<thunderbots_msgs::Robot> transformed_robot_msgs = Util::ROSMessages::transformRobotMessages(robot_msgs);

    thunderbots_msgs::Robot transformed_robot1_msg = transformed_robot_msgs.at(0);
    thunderbots_msgs::Robot transformed_robot2_msg = transformed_robot_msgs.at(1);

    EXPECT_EQ(1, transformed_robot1_msg.id);
    EXPECT_EQ(-1, transformed_robot1_msg.position.x);
    EXPECT_EQ(5.01, transformed_robot1_msg.position.y);
    EXPECT_EQ(2, transformed_robot1_msg.velocity.x);
    EXPECT_EQ(-3, transformed_robot1_msg.velocity.y);
    EXPECT_EQ(Angle::quarter().toRadians() + Angle::half().toRadians(), transformed_robot1_msg.orientation);
    EXPECT_EQ(AngularVelocity::ofRadians(1).toRadians(), transformed_robot1_msg.angular_velocity);
    EXPECT_EQ(Timestamp::fromSeconds(4.4).getSeconds(), transformed_robot1_msg.timestamp_seconds);

    EXPECT_EQ(2, transformed_robot2_msg.id);
    EXPECT_EQ(-10.4, transformed_robot2_msg.position.x);
    EXPECT_EQ(2.05, transformed_robot2_msg.position.y);
    EXPECT_EQ(-4, transformed_robot2_msg.velocity.x);
    EXPECT_EQ(6, transformed_robot2_msg.velocity.y);
    EXPECT_EQ(Angle::half().toRadians() + Angle::half().toRadians(), transformed_robot2_msg.orientation);
    EXPECT_EQ(AngularVelocity::ofRadians(1).toRadians(), transformed_robot1_msg.angular_velocity);
    EXPECT_EQ(Timestamp::fromSeconds(5.6).getSeconds(), transformed_robot2_msg.timestamp_seconds);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
