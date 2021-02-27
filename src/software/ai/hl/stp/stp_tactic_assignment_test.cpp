#include <gtest/gtest.h>

#include <algorithm>

#include "software/ai/hl/stp/play/halt_play.h"
#include "software/ai/hl/stp/stp.h"
#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/ai/hl/stp/tactic/test_tactics/goalie_test_tactic.h"
#include "software/ai/hl/stp/tactic/test_tactics/move_test_tactic.h"
#include "software/ai/hl/stp/tactic/test_tactics/stop_test_tactic.h"
#include "software/test_util/test_util.h"

/**
 * This file contains unit tests for the assignRobotsToTactics function
 * of the STP module. It is in a separate file because there are so many tests, and it
 * is cleaner to keep separate from the rest of the STP tests
 */

class STPTacticAssignmentTest : public ::testing::Test
{
   public:
    STPTacticAssignmentTest()
        : thunderbots_config(std::make_shared<const ThunderbotsConfig>()),
          stp([]() { return nullptr; }, thunderbots_config->getAiControlConfig(),
              thunderbots_config->getPlayConfig(), 0)
    {
    }

   protected:
    void SetUp() override
    {
        thunderbots_config            = std::make_shared<const ThunderbotsConfig>();
        auto default_play_constructor = [this]() -> std::unique_ptr<Play> {
            return std::make_unique<HaltPlay>(thunderbots_config->getPlayConfig());
        };
        // Give an explicit seed to STP so that our tests are deterministic
        stp   = STP(default_play_constructor, thunderbots_config->getAiControlConfig(),
                  thunderbots_config->getPlayConfig(), 0);
        world = ::TestUtil::createBlankTestingWorld();
    }

    /**
     * Check that all the given tactics have a robot assigned to them
     * @param tactics The tactics to check
     * @return True if all the given tactics have a robot assigned to them, false
     *         otherwise
     */
    bool allTacticsAssigned(
        std::vector<std::shared_ptr<const Tactic>> tactics,
        std::map<std::shared_ptr<const Tactic>, Robot> robot_tactic_assignment)
    {
        bool all_tactics_have_robot_assigned = true;

        for (auto tactic : tactics)
        {
            auto iter = robot_tactic_assignment.find(tactic);
            if (iter == robot_tactic_assignment.end())
            {
                all_tactics_have_robot_assigned = false;
                break;
            }
        }

        return all_tactics_have_robot_assigned;
    }

    std::shared_ptr<const ThunderbotsConfig> thunderbots_config;
    STP stp;
    World world = ::TestUtil::createBlankTestingWorld();
};

TEST_F(STPTacticAssignmentTest,
       test_correct_number_of_tactics_returned_when_equal_number_of_robots_and_tactics)
{
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    Robot robot_1(1, Point(1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0, robot_1});
    world.updateFriendlyTeamState(friendly_team);

    auto move_tactic_1 = std::make_shared<MoveTestTactic>();
    auto move_tactic_2 = std::make_shared<MoveTestTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0));
    move_tactic_2->updateControlParams(Point(1, 0));

    std::vector<std::shared_ptr<const Tactic>> tactics = {move_tactic_1, move_tactic_2};

    auto asst = stp.assignRobotsToTactics(tactics, world);

    EXPECT_TRUE(allTacticsAssigned(tactics, asst));
}

TEST_F(STPTacticAssignmentTest,
       test_correct_number_of_tactics_returned_when_more_tactics_than_robots)
{
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0});
    world.updateFriendlyTeamState(friendly_team);

    auto move_tactic_1 = std::make_shared<MoveTestTactic>();
    auto move_tactic_2 = std::make_shared<MoveTestTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0));
    move_tactic_2->updateControlParams(Point(1, 0));

    std::vector<std::shared_ptr<const Tactic>> tactics = {move_tactic_1, move_tactic_2};

    auto asst = stp.assignRobotsToTactics(tactics, world);

    EXPECT_TRUE(asst.find(move_tactic_1) != asst.end());
    EXPECT_FALSE(asst.find(move_tactic_2) != asst.end());
}

TEST_F(STPTacticAssignmentTest,
       test_correct_number_of_tactics_returned_when_more_robots_than_tactics)
{
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    Robot robot_1(1, Point(1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0, robot_1});
    world.updateFriendlyTeamState(friendly_team);

    auto move_tactic_1 = std::make_shared<MoveTestTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0));

    std::vector<std::shared_ptr<const Tactic>> tactics = {move_tactic_1};

    auto asst = stp.assignRobotsToTactics(tactics, world);

    EXPECT_TRUE(asst.find(move_tactic_1) != asst.end());
}

TEST_F(STPTacticAssignmentTest, test_0_tactics_returned_when_there_are_no_robots)
{
    Team friendly_team(Duration::fromSeconds(0));
    friendly_team.updateRobots({});
    world.updateFriendlyTeamState(friendly_team);

    auto move_tactic_1 = std::make_shared<MoveTestTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0));

    std::vector<std::shared_ptr<const Tactic>> tactics = {move_tactic_1};

    auto asst = stp.assignRobotsToTactics(tactics, world);

    EXPECT_FALSE(asst.find(move_tactic_1) != asst.end());
}

TEST_F(STPTacticAssignmentTest,
       test_correct_tactics_removed_when_more_tactics_than_robots)
{
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0});
    world.updateFriendlyTeamState(friendly_team);

    auto move_tactic_1 = std::make_shared<MoveTestTactic>();
    auto stop_tactic_1 = std::make_shared<StopTestTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0));

    std::vector<std::shared_ptr<const Tactic>> tactics = {move_tactic_1, stop_tactic_1};

    // Both robots are now closest to move_tactic_1's destination. We do NOT want
    // robot_0 to be assigned to move_tactic_1, because then robot_1 has to move all the
    // way around to move_tactic_2. What we expect is that robot_0 will be assigned to
    // move_tactic_2 and "slide over" to make room for robot_1

    auto asst = stp.assignRobotsToTactics(tactics, world);


    // move_tactic_1 should be the only Tactic assigned a robot, since stop_tactic_1 is a
    // lower priority than move_tactic_1 so it should be dropped since there's only 1
    // robot
    EXPECT_TRUE(asst.find(move_tactic_1) != asst.end());
    EXPECT_FALSE(asst.find(stop_tactic_1) != asst.end());
}


TEST_F(STPTacticAssignmentTest, test_assigning_1_tactic_to_1_robot)
{
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0});
    world.updateFriendlyTeamState(friendly_team);

    auto move_tactic_1 = std::make_shared<MoveTestTactic>();

    move_tactic_1->updateControlParams(Point(2, -3.2));

    std::vector<std::shared_ptr<const Tactic>> tactics = {move_tactic_1};

    auto asst = stp.assignRobotsToTactics(tactics, world);

    ASSERT_TRUE(asst.find(move_tactic_1) != asst.end());
    EXPECT_EQ(asst.find(move_tactic_1)->second, robot_0);
}

// Test the case where it is "obvious" which robots should be assigned to each tactic
// Each robot is already close to one of the tactic's destinations, so it is trivial to
// see the optimal assignment is for each robot to be assigned to the tactic whose
// destination it's closest to
TEST_F(STPTacticAssignmentTest, test_assigning_2_robots_to_2_tactics_no_overlap)
{
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    Robot robot_1(1, Point(1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0, robot_1});
    world.updateFriendlyTeamState(friendly_team);

    auto move_tactic_1 = std::make_shared<MoveTestTactic>();
    auto move_tactic_2 = std::make_shared<MoveTestTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0));
    move_tactic_2->updateControlParams(Point(1, 0));

    std::vector<std::shared_ptr<const Tactic>> tactics = {move_tactic_1, move_tactic_2};

    // Each robot is close to separate tactic destinations. They should each be trivially
    // assigned to the tactic with the destination closest to their position

    auto asst = stp.assignRobotsToTactics(tactics, world);

    ASSERT_TRUE(asst.find(move_tactic_1) != asst.end());
    ASSERT_TRUE(asst.find(move_tactic_2) != asst.end());

    EXPECT_EQ(asst.find(move_tactic_1)->second, robot_0);
    EXPECT_EQ(asst.find(move_tactic_2)->second, robot_1);
}

// Test a more complex case where each robot is closest to the same tactic destination.
// If robot0 were to be assigned to the tactic with dest1, robot1 would be forced to go
// all the way to dest2. This is what happened in our previous system that used greedy
// tactic assignment, and cause the robots to "overlap"
//
//     robot1
//                     robot0
//
//
//                     dest1             dest2
TEST_F(STPTacticAssignmentTest, test_assigning_2_robots_to_2_tactics_with_overlap)
{
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    Robot robot_1(1, Point(-3, 1.5), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0, robot_1});
    world.updateFriendlyTeamState(friendly_team);

    auto move_tactic_1 = std::make_shared<MoveTestTactic>();
    auto move_tactic_2 = std::make_shared<MoveTestTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0));
    move_tactic_2->updateControlParams(Point(1, 0));

    std::vector<std::shared_ptr<const Tactic>> tactics = {move_tactic_1, move_tactic_2};

    // Both robots are now closest to move_tactic_1's destination. We do NOT want
    // robot_0 to be assigned to move_tactic_1, because then robot_1 has to move all the
    // way around to move_tactic_2. What we expect is that robot_0 will be assigned to
    // move_tactic_2 and "slide over" to make room for robot_1

    auto asst = stp.assignRobotsToTactics(tactics, world);

    ASSERT_TRUE(asst.find(move_tactic_1) != asst.end());
    ASSERT_TRUE(asst.find(move_tactic_2) != asst.end());
    EXPECT_EQ(asst.find(move_tactic_1)->second, robot_1);
    EXPECT_EQ(asst.find(move_tactic_2)->second, robot_0);
}

TEST_F(STPTacticAssignmentTest, test_assigning_3_robots_to_2_tactics)
{
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-1.1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    Robot robot_1(1, Point(2, 0.81), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    Robot robot_2(2, Point(0, 5.0), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0, robot_1, robot_2});
    world.updateFriendlyTeamState(friendly_team);

    auto move_tactic_1 = std::make_shared<MoveTestTactic>();
    auto move_tactic_2 = std::make_shared<MoveTestTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0));
    move_tactic_2->updateControlParams(Point(1, 0));

    std::vector<std::shared_ptr<const Tactic>> tactics = {move_tactic_1, move_tactic_2};

    // robot_2 should not be assigned since both robot_0 and robot_1 are more optimal
    // to assign to the tactics. robot_2 is too far away

    auto asst = stp.assignRobotsToTactics(tactics, world);

    ASSERT_TRUE(asst.find(move_tactic_1) != asst.end());
    ASSERT_TRUE(asst.find(move_tactic_2) != asst.end());
    EXPECT_EQ(asst.find(move_tactic_1)->second, robot_0);
    EXPECT_EQ(asst.find(move_tactic_2)->second, robot_1);
}

TEST_F(STPTacticAssignmentTest,
       test_assigning_3_robots_to_3_tactics_all_with_the_same_cost)
{
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-1.1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    Robot robot_1(1, Point(2, 0.81), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    Robot robot_2(2, Point(0, 5.0), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0, robot_1, robot_2});
    world.updateFriendlyTeamState(friendly_team);

    auto stop_tactic_1 = std::make_shared<StopTestTactic>();
    auto stop_tactic_2 = std::make_shared<StopTestTactic>();
    auto stop_tactic_3 = std::make_shared<StopTestTactic>();

    std::vector<std::shared_ptr<const Tactic>> tactics = {stop_tactic_1, stop_tactic_2,
                                                          stop_tactic_3};

    // If all costs are equal, the robots and tactics are simply paired in order
    auto asst = stp.assignRobotsToTactics(tactics, world);

    ASSERT_TRUE(asst.find(stop_tactic_1) != asst.end());
    ASSERT_TRUE(asst.find(stop_tactic_2) != asst.end());
    ASSERT_TRUE(asst.find(stop_tactic_3) != asst.end());
    EXPECT_EQ(asst.find(stop_tactic_1)->second, robot_0);
    EXPECT_EQ(asst.find(stop_tactic_2)->second, robot_1);
    EXPECT_EQ(asst.find(stop_tactic_3)->second, robot_2);
}

TEST_F(STPTacticAssignmentTest,
       test_assigning_3_robots_to_3_tactics_with_2_of_the_same_cost)
{
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-1.1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    Robot robot_1(1, Point(2, 0.81), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    Robot robot_2(2, Point(0, 4.0), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0, robot_1, robot_2});
    world.updateFriendlyTeamState(friendly_team);

    auto stop_tactic_1 = std::make_shared<StopTestTactic>();
    auto move_tactic_1 = std::make_shared<MoveTestTactic>();
    auto stop_tactic_2 = std::make_shared<StopTestTactic>();

    // The destination of the move_tactic is relatively close to the robot positions, so
    // the cost of assigning any robot to the move_tactic should be less than the
    // stop_tactics
    move_tactic_1->updateControlParams(Point(0, 0));

    std::vector<std::shared_ptr<const Tactic>> tactics = {stop_tactic_1, move_tactic_1,
                                                          stop_tactic_2};

    auto asst = stp.assignRobotsToTactics(tactics, world);

    ASSERT_TRUE(asst.find(stop_tactic_1) != asst.end());
    ASSERT_TRUE(asst.find(move_tactic_1) != asst.end());
    ASSERT_TRUE(asst.find(stop_tactic_2) != asst.end());
    EXPECT_EQ(asst.find(stop_tactic_1)->second, robot_2);
    EXPECT_EQ(asst.find(move_tactic_1)->second, robot_0);
    EXPECT_EQ(asst.find(stop_tactic_2)->second, robot_1);
}

TEST_F(STPTacticAssignmentTest,
       test_assigning_2_robots_to_1_tactic_unsatisfied_robotcapabilityflags)
{
    // test that the robot that matches capability requirements is selected over the robot
    // that doesn't even though the former has lower cost
    Team friendly_team(Duration::fromSeconds(0));
    // this robot has no capabilities
    Robot robot_0(0, Point(0.1, 0.1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0), allRobotCapabilities());
    Robot robot_1(1, Point(-10, -10), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0, robot_1});
    world.updateFriendlyTeamState(friendly_team);

    auto move_tactic_1 = std::make_shared<MoveTestTactic>();

    move_tactic_1->updateControlParams(Point(0, 0));

    std::vector<std::shared_ptr<const Tactic>> tactics = {move_tactic_1};

    auto asst = stp.assignRobotsToTactics(tactics, world);

    ASSERT_TRUE(asst.find(move_tactic_1) != asst.end());
    EXPECT_EQ(asst.find(move_tactic_1)->second, robot_1);
}

TEST_F(STPTacticAssignmentTest,
       test_assigning_multiple_robots_to_multiple_goalie_tactic_goalie_not_set_on_team)
{
    // Test that if there is no team goalie, the "goalie" tactic
    // is not assigned a robot, even if there are enough robots

    // Put two robots right in front of the friendly goal
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-0.5, 0.2), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    Robot robot_1(1, Point(-0.5, -0.2), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0, robot_1});
    world.updateFriendlyTeamState(friendly_team);

    auto goalie_tactic_1 = std::make_shared<GoalieTestTactic>();
    auto goalie_tactic_2 = std::make_shared<GoalieTestTactic>();
    std::vector<std::shared_ptr<const Tactic>> tactics = {goalie_tactic_1,
                                                          goalie_tactic_2};

    auto asst = stp.assignRobotsToTactics(tactics, world);

    EXPECT_FALSE(asst.find(goalie_tactic_1) != asst.end());
    EXPECT_FALSE(asst.find(goalie_tactic_2) != asst.end());
}

TEST_F(STPTacticAssignmentTest,
       test_assigning_multiple_robots_to_single_goalie_tactic_goalie_set_on_team)
{
    // Test that only the robot set as the goalie on the team is assigned to the
    // goalie tactic

    // Put two robots right in front of the friendly goal
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-0.5, 0.2), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    // default is all capabilities, if not specified otherwise
    Robot robot_1(1, Point(-0.5, -0.2), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0, robot_1});

    friendly_team.assignGoalie(0);
    world.updateFriendlyTeamState(friendly_team);

    auto goalie_tactic_1 = std::make_shared<GoalieTestTactic>();
    std::vector<std::shared_ptr<const Tactic>> tactics = {goalie_tactic_1};

    auto asst = stp.assignRobotsToTactics(tactics, world);

    EXPECT_TRUE(allTacticsAssigned(tactics, asst));
    ASSERT_TRUE(asst.find(goalie_tactic_1) != asst.end());
    EXPECT_EQ(asst.find(goalie_tactic_1)->second, robot_0);

    // Change the goalie and perform the same check in case we have a fluke bug
    friendly_team.assignGoalie(1);
    world.updateFriendlyTeamState(friendly_team);

    asst = stp.assignRobotsToTactics(tactics, world);

    EXPECT_TRUE(allTacticsAssigned(tactics, asst));
    ASSERT_TRUE(asst.find(goalie_tactic_1) != asst.end());
    EXPECT_EQ(asst.find(goalie_tactic_1)->second, robot_1);
}

TEST_F(STPTacticAssignmentTest,
       test_assigning_multiple_robots_to_multiple_goalie_tactics_goalie_set_on_team)
{
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-0.5, 0.2), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    // default is all capabilities, if not specified otherwise
    Robot robot_1(1, Point(-0.5, -0.2), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0, robot_1});

    friendly_team.assignGoalie(0);
    world.updateFriendlyTeamState(friendly_team);

    auto goalie_tactic_1 = std::make_shared<GoalieTestTactic>();
    auto goalie_tactic_2 = std::make_shared<GoalieTestTactic>();
    std::vector<std::shared_ptr<const Tactic>> tactics = {goalie_tactic_1,
                                                          goalie_tactic_2};

    auto asst = stp.assignRobotsToTactics(tactics, world);

    ASSERT_TRUE(asst.find(goalie_tactic_1) != asst.end());
    EXPECT_FALSE(asst.find(goalie_tactic_2) != asst.end());
    EXPECT_EQ(asst.find(goalie_tactic_1)->second, robot_0);
}

TEST_F(STPTacticAssignmentTest,
       test_assigning_stop_tactics_to_unassigned_non_goalie_robots)
{
    // Test that StopTactic is assigned to remaining robots without tactics

    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    Robot robot_1(1, Point(-3, 5), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    Robot robot_2(2, Point(6, 7), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0, robot_1, robot_2});
    world.updateFriendlyTeamState(friendly_team);

    auto move_tactic_1 = std::make_shared<MoveTestTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0));

    std::vector<std::shared_ptr<const Tactic>> tactics = {move_tactic_1};
    std::vector<Robot> expected_robots_assigned        = {robot_0, robot_1, robot_2};

    auto asst = stp.assignRobotsToTactics(tactics, world);

    // Check each tactic is assigned to the intended robot
    for (unsigned int i = 0; i < tactics.size(); i++)
    {
        ASSERT_TRUE(asst.find(tactics[i]) != asst.end());
        EXPECT_EQ(asst.find(tactics[i])->second, expected_robots_assigned[i]);
    }
}
