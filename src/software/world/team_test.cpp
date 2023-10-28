#include "software/world/team.h"

#include <gtest/gtest.h>
#include <include/gmock/gmock-matchers.h>

#include <stdexcept>
#include <unordered_set>

#include "proto/message_translation/tbots_protobuf.h"

class TeamTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        current_time = Timestamp::fromSeconds(123);

        one_second_future        = current_time + Duration::fromSeconds(1);
        two_seconds_future       = current_time + Duration::fromSeconds(2);
        two_seconds_100ms_future = current_time + Duration::fromMilliseconds(2100);
        three_seconds_future     = current_time + Duration::fromSeconds(3);

        one_second_past = current_time - Duration::fromSeconds(1);
    }

    Timestamp current_time;

    Timestamp one_second_future;
    Timestamp two_seconds_future;
    Timestamp two_seconds_100ms_future;
    Timestamp three_seconds_future;

    Timestamp one_second_past;
};

TEST_F(TeamTest, construction_with_expiry_duration)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    EXPECT_EQ(0, team.numRobots());
    EXPECT_EQ(std::nullopt, team.getRobotById(0));
    EXPECT_EQ(std::nullopt, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.goalie());
    EXPECT_EQ(std::vector<Robot>(), team.getAllRobots());
    EXPECT_EQ(Duration::fromMilliseconds(1000), team.getRobotExpiryBufferDuration());
}

TEST_F(TeamTest, construction_with_expiry_duration_and_team_robots)
{
    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(), Vector(-0.5, 4), Angle::quarter(),
                          AngularVelocity::half(), current_time);

    std::vector<Robot> robot_list = {robot_0, robot_1, robot_2};

    Team team = Team(robot_list, Duration::fromMilliseconds(1000));

    EXPECT_EQ(Duration::fromMilliseconds(1000), team.getRobotExpiryBufferDuration());

    EXPECT_EQ(3, team.numRobots());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
    EXPECT_EQ(robot_2, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.getRobotById(3));
    EXPECT_EQ(std::nullopt, team.goalie());
    EXPECT_EQ(robot_list, team.getAllRobots());
}

TEST_F(TeamTest, construct_with_protobuf)
{
    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(), Vector(-0.5, 4), Angle::quarter(),
                          AngularVelocity::half(), current_time);

    std::vector<Robot> robot_list = {robot_0, robot_1, robot_2};

    Team original_team = Team(robot_list);
    auto proto_team    = createTeamProto(original_team);
    Team proto_converted_team(*proto_team);

    // Proto representation of Team does not store the robot_expiry_buffer_duration, so we
    // will not test it
    EXPECT_EQ(original_team.getGoalieId(), proto_converted_team.getGoalieId());
    EXPECT_THAT(original_team.getAllRobots(),
                ::testing::ContainerEq(proto_converted_team.getAllRobots()));
}

TEST_F(TeamTest, update_with_3_robots)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(), Vector(-0.5, 4), Angle::quarter(),
                          AngularVelocity::half(), current_time);

    std::vector<Robot> robot_list = {robot_0, robot_1, robot_2};

    team.updateRobots(robot_list);

    EXPECT_EQ(3, team.numRobots());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
    EXPECT_EQ(robot_2, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.getRobotById(3));
    EXPECT_EQ(std::nullopt, team.goalie());
    EXPECT_EQ(robot_list, team.getAllRobots());
}

TEST_F(TeamTest, update_with_new_team)
{
    Team team_update = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(), Vector(-0.5, 4), Angle::quarter(),
                          AngularVelocity::half(), current_time);

    std::vector<Robot> robot_list = {robot_0, robot_1, robot_2};

    team_update.updateRobots(robot_list);

    Team team = Team(Duration::fromMilliseconds(1000));

    team.updateState(team_update);

    EXPECT_EQ(3, team.numRobots());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
    EXPECT_EQ(robot_2, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.getRobotById(3));
    EXPECT_EQ(std::nullopt, team.goalie());
    EXPECT_EQ(robot_list, team.getAllRobots());

    EXPECT_EQ(team_update, team);
}

TEST_F(TeamTest, remove_expired_robots_at_current_time_so_no_robots_expire)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    team.updateRobots({robot_0, robot_1});

    team.removeExpiredRobots(current_time);

    EXPECT_EQ(2, team.numRobots());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
}

TEST_F(TeamTest, remove_expired_robots_in_future_before_expiry_time_so_no_robots_expire)
{
    Team team = Team(Duration::fromMilliseconds(2100));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    team.updateRobots({robot_0, robot_1});

    team.removeExpiredRobots(two_seconds_future);

    EXPECT_EQ(2, team.numRobots());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
}

TEST_F(TeamTest, remove_expired_robots_in_past_so_no_robots_expire)
{
    Team team = Team(Duration::fromMilliseconds(2000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    team.updateRobots({robot_0, robot_1});

    EXPECT_EQ(2, team.numRobots());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
}

TEST_F(TeamTest, remove_expired_robots_in_future_so_all_robots_expire)
{
    Team team = Team(Duration::fromMilliseconds(2000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    team.updateRobots({robot_0, robot_1});

    team.removeExpiredRobots(two_seconds_100ms_future);

    EXPECT_EQ(0, team.numRobots());
    EXPECT_EQ(std::nullopt, team.getRobotById(0));
    EXPECT_EQ(std::nullopt, team.getRobotById(1));
}

TEST_F(TeamTest, remove_expired_robots_in_future_so_1_robot_expires_1_robot_does_not)
{
    Team team = Team(Duration::fromMilliseconds(2000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), one_second_future);

    team.updateRobots({robot_0, robot_1});

    team.removeExpiredRobots(two_seconds_100ms_future);

    EXPECT_EQ(1, team.numRobots());
    EXPECT_EQ(std::nullopt, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
}

TEST_F(TeamTest, removeRobotWithId_robot_with_id_on_team)
{
    Team team     = Team(Duration::fromMilliseconds(2000));
    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), one_second_future);
    team.updateRobots({robot_0, robot_1});

    team.removeRobotWithId(0);

    EXPECT_EQ(1, team.getAllRobots().size());
    EXPECT_EQ(std::vector<Robot>{robot_1}, team.getAllRobots());
}

TEST_F(TeamTest, removeRobotWithId_robot_with_id_not_on_team)
{
    Team team     = Team(Duration::fromMilliseconds(2000));
    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), one_second_future);
    team.updateRobots({robot_0, robot_1});

    team.removeRobotWithId(2);

    EXPECT_EQ(2, team.getAllRobots().size());
    EXPECT_EQ(std::vector<Robot>({robot_0, robot_1}), team.getAllRobots());
}


TEST_F(TeamTest, clear_all_robots)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    std::vector<Robot> robot_list = {robot_0, robot_1};

    team.updateRobots(robot_list);

    EXPECT_EQ(2, team.numRobots());
    EXPECT_EQ(robot_0, team.getRobotById(0));
    EXPECT_EQ(robot_1, team.getRobotById(1));
    EXPECT_EQ(std::nullopt, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.goalie());
    EXPECT_EQ(robot_list, team.getAllRobots());

    team.clearAllRobots();

    EXPECT_EQ(0, team.numRobots());
    EXPECT_EQ(std::nullopt, team.getRobotById(0));
    EXPECT_EQ(std::nullopt, team.getRobotById(1));
    EXPECT_EQ(std::nullopt, team.getRobotById(2));
    EXPECT_EQ(std::nullopt, team.goalie());
    EXPECT_EQ(std::vector<Robot>(), team.getAllRobots());
}

TEST_F(TeamTest, assign_goalie_starting_with_no_goalie)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    std::vector<Robot> robot_list = {robot_0, robot_1};

    team.updateRobots(robot_list);

    EXPECT_EQ(std::nullopt, team.goalie());

    team.assignGoalie(0);

    EXPECT_EQ(robot_0, team.goalie());
}

TEST_F(TeamTest, assign_goalie_starting_with_different_robot_as_goalie)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    std::vector<Robot> robot_list = {robot_0, robot_1};

    team.updateRobots(robot_list);
    team.assignGoalie(0);

    EXPECT_EQ(robot_0, team.goalie());

    team.assignGoalie(1);

    EXPECT_EQ(robot_1, team.goalie());
}

TEST_F(TeamTest, clear_goalie)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    std::vector<Robot> robot_list = {robot_0, robot_1};

    team.updateRobots(robot_list);
    team.assignGoalie(0);

    EXPECT_EQ(robot_0, team.goalie());

    team.clearGoalie();

    EXPECT_EQ(std::nullopt, team.goalie());
}

TEST_F(TeamTest, get_goalie_id_with_no_goalie)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    EXPECT_EQ(std::nullopt, team.getGoalieId());
}

TEST_F(TeamTest, get_goalie_id_with_goalie)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);
    team.updateRobots({robot_0});
    team.assignGoalie(0);

    EXPECT_EQ(0, team.getGoalieId());
}

TEST_F(TeamTest, get_robot_expiry_buffer)
{
    Team team = Team(Duration::fromMilliseconds(500));

    EXPECT_EQ(Duration::fromMilliseconds(500), team.getRobotExpiryBufferDuration());
}

TEST_F(TeamTest, set_robot_expiry_buffer)
{
    Team team = Team(Duration::fromMilliseconds(0));

    team.setRobotExpiryBuffer(Duration::fromMilliseconds(831));

    EXPECT_EQ(Duration::fromMilliseconds(831), team.getRobotExpiryBufferDuration());
}

TEST_F(TeamTest, set_unavailable_robot_capabilities_multiple_robots)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(3, -1), Vector(), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(1, 0), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    team.updateRobots({robot_0, robot_1});


    std::set<RobotCapability> unavailableCapabilities_0;
    std::set<RobotCapability> unavailableCapabilities_1;

    unavailableCapabilities_0.insert(RobotCapability::Move);
    unavailableCapabilities_0.insert(RobotCapability::Kick);
    unavailableCapabilities_1.insert(RobotCapability::Chip);
    unavailableCapabilities_1.insert(RobotCapability::Dribble);

    team.setUnavailableRobotCapabilities(0, unavailableCapabilities_0);
    team.setUnavailableRobotCapabilities(1, unavailableCapabilities_1);

    EXPECT_EQ(unavailableCapabilities_0,
              team.getRobotById(0).value().getUnavailableCapabilities());
    EXPECT_EQ(unavailableCapabilities_1,
              team.getRobotById(1).value().getUnavailableCapabilities());
}

TEST_F(TeamTest, set_unavailable_robot_capabilities_to_none)
{
    Team team     = Team(Duration::fromMilliseconds(1000));
    Robot robot_0 = Robot(0, Point(3, -1), Vector(), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);
    team.updateRobots({robot_0});

    // Make Move unavailable
    std::set<RobotCapability> unavailableCapabilities;
    unavailableCapabilities.insert(RobotCapability::Move);
    team.setUnavailableRobotCapabilities(0, unavailableCapabilities);
    EXPECT_EQ(unavailableCapabilities,
              team.getRobotById(0).value().getUnavailableCapabilities());

    // Reset unavailable capabilities
    std::set<RobotCapability> unavailableCapabilitiesEmpty;
    team.setUnavailableRobotCapabilities(0, unavailableCapabilitiesEmpty);
    EXPECT_EQ(unavailableCapabilitiesEmpty,
              team.getRobotById(0).value().getUnavailableCapabilities());
}

TEST_F(TeamTest, nearest_friendy_one_robot)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);


    team.updateRobots({robot_0});

    EXPECT_EQ(robot_0, team.getNearestRobot(Point(0, 0)));
}

TEST_F(TeamTest, nearest_friendy_multiple_robots)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(3, -1), Vector(), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(1, 0), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(4, 6), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);


    team.updateRobots({robot_0, robot_1, robot_2});

    EXPECT_EQ(robot_1, team.getNearestRobot(Point(0, 0)));
}

TEST_F(TeamTest, nearest_friendy_multiple_robots_closest_is_moving)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(3, -1), Vector(), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(1, 0), Vector(3, 3), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(4, 6), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);


    team.updateRobots({robot_0, robot_1, robot_2});

    EXPECT_EQ(robot_1, team.getNearestRobot(Point(0, 0)));
}

TEST_F(TeamTest, nearest_friendy_multiple_robots_all_moving)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(3, -1), Vector(3, 3), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(4, 6), Vector(3, 3), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(0, 1), Vector(3, 3), Angle::zero(),
                          AngularVelocity::zero(), current_time);


    team.updateRobots({robot_0, robot_1, robot_2});

    EXPECT_EQ(robot_2, team.getNearestRobot(Point(0, 0)));
}

TEST_F(TeamTest, nearest_friendy_one_robot_on_ball)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 0), Vector(), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(1, 0), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(4, 6), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);


    team.updateRobots({robot_0, robot_1, robot_2});

    EXPECT_EQ(robot_0, team.getNearestRobot(Point(0, 0)));
}

TEST_F(TeamTest, nearest_robot_zero_robots)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    EXPECT_EQ(std::nullopt, team.getNearestRobot(Point(0, 0)));
}

TEST_F(TeamTest, equality_operator_compare_team_with_itself)
{
    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Team team = Team(Duration::fromMilliseconds(1000));
    team.updateRobots({robot_0});
    team.assignGoalie(0);

    EXPECT_EQ(Team(Duration::fromMilliseconds(500)),
              Team(Duration::fromMilliseconds(500)));
    EXPECT_EQ(team, team);
}

TEST_F(TeamTest, equality_operator_teams_with_different_expiry_buffers)
{
    EXPECT_NE(Team(Duration::fromMilliseconds(50)),
              Team(Duration::fromMilliseconds(1000)));
}

TEST_F(TeamTest, equality_operator_teams_with_different_number_of_robots)
{
    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(), Vector(-0.5, 4), Angle::quarter(),
                          AngularVelocity::half(), current_time);

    Team team_0 = Team(Duration::fromMilliseconds(1000));
    team_0.updateRobots({robot_0, robot_1, robot_2});

    Team team_1 = Team(Duration::fromMilliseconds(1000));
    team_1.updateRobots({robot_0, robot_2});

    EXPECT_NE(team_0, team_1);
}

TEST_F(TeamTest, equality_operator_teams_with_same_number_of_robots_but_different_robots)
{
    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(), Vector(-0.5, 4), Angle::quarter(),
                          AngularVelocity::half(), current_time);

    Team team_0 = Team(Duration::fromMilliseconds(1000));
    team_0.updateRobots({robot_0, robot_1});

    Team team_1 = Team(Duration::fromMilliseconds(1000));
    team_1.updateRobots({robot_0, robot_2});

    EXPECT_NE(team_0, team_1);
}

TEST_F(TeamTest, equality_operator_teams_with_different_goalie)
{
    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(), Vector(-0.5, 4), Angle::quarter(),
                          AngularVelocity::half(), current_time);

    Team team_0 = Team(Duration::fromMilliseconds(1000));
    team_0.updateRobots({robot_0, robot_1});
    team_0.assignGoalie(0);

    Team team_1 = Team(Duration::fromMilliseconds(1000));
    team_1.updateRobots({robot_0, robot_1});
    team_1.assignGoalie(1);

    EXPECT_NE(team_0, team_1);
}

TEST_F(TeamTest, get_most_recent_timestamp)
{
    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), one_second_future);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), one_second_past);

    Robot robot_2 = Robot(2, Point(), Vector(-0.5, 4), Angle::quarter(),
                          AngularVelocity::half(), current_time);

    Team team = Team(Duration::fromMilliseconds(1000));
    team.updateRobots({robot_0, robot_1, robot_2});
    team.assignGoalie(0);

    EXPECT_EQ(one_second_future, team.getMostRecentTimestamp());
}

TEST_F(TeamTest, get_all_robots_except_goalie_in_team_of_3)
{
    Team team = Team(Duration::fromMilliseconds(2000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    team.updateRobots({robot_0, robot_1, robot_2});
    team.assignGoalie(0);

    std::unordered_set<int> returned_robot_ids;
    for (const auto& robot : team.getAllRobotsExceptGoalie())
    {
        returned_robot_ids.emplace(robot.id());
    }
    EXPECT_EQ(returned_robot_ids.find(0), returned_robot_ids.end());
    EXPECT_NE(returned_robot_ids.find(1), returned_robot_ids.end());
    EXPECT_NE(returned_robot_ids.find(2), returned_robot_ids.end());
}

TEST_F(TeamTest, get_all_robots_except_goalie_in_team_of_4)
{
    Team team = Team(Duration::fromMilliseconds(2000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(3, -1), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);
    Robot robot_3 = Robot(3, Point(2, 4), Vector(), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    team.updateRobots({robot_0, robot_1, robot_2, robot_3});
    team.assignGoalie(2);

    std::unordered_set<int> returned_robot_ids;
    for (const auto& robot : team.getAllRobotsExceptGoalie())
    {
        returned_robot_ids.emplace(robot.id());
    }
    EXPECT_EQ(returned_robot_ids.find(2), returned_robot_ids.end());
    EXPECT_NE(returned_robot_ids.find(0), returned_robot_ids.end());
    EXPECT_NE(returned_robot_ids.find(1), returned_robot_ids.end());
    EXPECT_NE(returned_robot_ids.find(3), returned_robot_ids.end());
}
