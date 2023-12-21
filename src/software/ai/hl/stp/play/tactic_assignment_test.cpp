#include "software/ai/hl/stp/play/tactic_assignment.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <unordered_set>

#include "software/ai/hl/stp/play/offense/offense_play.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/test_util/test_util.h"

/**
 * This file contains unit tests for the assignTactics function
 * of the STP module. It is in a separate file because there are so many tests, and it
 * is cleaner to keep separate from the rest of the STP tests
 */

class TacticAssignmentTest : public ::testing::Test
{
   public:
    TacticAssignmentTest() : ai_config(TbotsProto::AiConfig()) {}

   protected:
    void SetUp() override
    {
        // Give an explicit seed to STP so that our tests are deterministic
        world = ::TestUtil::createBlankTestingWorld();
    }

    /**
     * Check that all the given tactics have a robot assigned to them
     * @param tactics The tactics to check
     * @return True if all the given tactics have a robot assigned to them, false
     *         otherwise
     */
    bool allTacticsAssigned(
        TacticVector tactics,
        std::map<std::shared_ptr<const Tactic>, RobotId> robot_tactic_assignment)
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

    TbotsProto::AiConfig ai_config;
    World world = ::TestUtil::createBlankTestingWorld();
};

TEST_F(TacticAssignmentTest,
       test_correct_number_of_tactics_returned_when_equal_number_of_robots_and_tactics)
{
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    Robot robot_1(1, Point(1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0, robot_1});
    world.updateFriendlyTeamState(friendly_team);

    auto move_tactic_1 = std::make_shared<MoveTactic>();
    auto move_tactic_2 = std::make_shared<MoveTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
    move_tactic_2->updateControlParams(Point(1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    TacticVector tactics = {move_tactic_1, move_tactic_2};

    auto robot_navigation_obstacle_config = ai_config.robot_navigation_obstacle_config();
    GlobalPathPlannerFactory path_planner_factory(robot_navigation_obstacle_config,
                                                  world.field());
    auto asst =
        assignTactics(path_planner_factory, world, tactics, friendly_team.getAllRobots());

    EXPECT_TRUE(allTacticsAssigned(tactics, std::get<2>(asst)));
}

TEST_F(TacticAssignmentTest,
       test_correct_number_of_tactics_returned_when_more_tactics_than_robots)
{
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0});
    world.updateFriendlyTeamState(friendly_team);

    auto move_tactic_1 = std::make_shared<MoveTactic>();
    auto move_tactic_2 = std::make_shared<MoveTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
    move_tactic_2->updateControlParams(Point(1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    TacticVector tactics = {move_tactic_1, move_tactic_2};

    auto robot_navigation_obstacle_config = ai_config.robot_navigation_obstacle_config();
    GlobalPathPlannerFactory path_planner_factory(robot_navigation_obstacle_config,
                                                  world.field());
    auto tup =
        assignTactics(path_planner_factory, world, tactics, friendly_team.getAllRobots());

    auto asst = std::get<2>(tup);

    EXPECT_TRUE(asst.find(move_tactic_1) != asst.end());
    EXPECT_FALSE(asst.find(move_tactic_2) != asst.end());
}

TEST_F(TacticAssignmentTest,
       test_correct_number_of_tactics_returned_when_more_robots_than_tactics)
{
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    Robot robot_1(1, Point(1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0, robot_1});
    world.updateFriendlyTeamState(friendly_team);

    auto move_tactic_1 = std::make_shared<MoveTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    TacticVector tactics = {move_tactic_1};

    auto robot_navigation_obstacle_config = ai_config.robot_navigation_obstacle_config();
    GlobalPathPlannerFactory path_planner_factory(robot_navigation_obstacle_config,
                                                  world.field());
    auto tup =
        assignTactics(path_planner_factory, world, tactics, friendly_team.getAllRobots());

    auto asst = std::get<2>(tup);

    EXPECT_TRUE(asst.find(move_tactic_1) != asst.end());
}

TEST_F(TacticAssignmentTest, test_0_tactics_returned_when_there_are_no_robots)
{
    Team friendly_team(Duration::fromSeconds(0));
    friendly_team.updateRobots({});
    world.updateFriendlyTeamState(friendly_team);

    auto move_tactic_1 = std::make_shared<MoveTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    TacticVector tactics = {move_tactic_1};

    auto robot_navigation_obstacle_config = ai_config.robot_navigation_obstacle_config();
    GlobalPathPlannerFactory path_planner_factory(robot_navigation_obstacle_config,
                                                  world.field());
    auto tup =
        assignTactics(path_planner_factory, world, tactics, friendly_team.getAllRobots());

    auto asst = std::get<2>(tup);

    EXPECT_FALSE(asst.find(move_tactic_1) != asst.end());
}

TEST_F(TacticAssignmentTest, test_correct_tactics_removed_when_more_tactics_than_robots)
{
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0});
    world.updateFriendlyTeamState(friendly_team);

    auto move_tactic_1 = std::make_shared<MoveTactic>();
    auto stop_tactic_1 = std::make_shared<StopTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    TacticVector tactics = {move_tactic_1, stop_tactic_1};

    // Both robots are now closest to move_tactic_1's destination. We do NOT want
    // robot_0 to be assigned to move_tactic_1, because then robot_1 has to move all the
    // way around to move_tactic_2. What we expect is that robot_0 will be assigned to
    // move_tactic_2 and "slide over" to make room for robot_1

    auto robot_navigation_obstacle_config = ai_config.robot_navigation_obstacle_config();
    GlobalPathPlannerFactory path_planner_factory(robot_navigation_obstacle_config,
                                                  world.field());
    auto tup =
        assignTactics(path_planner_factory, world, tactics, friendly_team.getAllRobots());

    auto asst = std::get<2>(tup);

    // move_tactic_1 should be the only Tactic assigned a robot, since stop_tactic_1 is a
    // lower priority than move_tactic_1 so it should be dropped since there's only 1
    // robot
    EXPECT_TRUE(asst.find(move_tactic_1) != asst.end());
    EXPECT_FALSE(asst.find(stop_tactic_1) != asst.end());
}

TEST_F(TacticAssignmentTest, test_assigning_1_tactic_to_1_robot)
{
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0});
    world.updateFriendlyTeamState(friendly_team);

    auto move_tactic_1 = std::make_shared<MoveTactic>();

    move_tactic_1->updateControlParams(Point(2, -3.2), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    TacticVector tactics = {move_tactic_1};

    auto robot_navigation_obstacle_config = ai_config.robot_navigation_obstacle_config();
    GlobalPathPlannerFactory path_planner_factory(robot_navigation_obstacle_config,
                                                  world.field());
    auto tup =
        assignTactics(path_planner_factory, world, tactics, friendly_team.getAllRobots());

    auto asst = std::get<2>(tup);


    ASSERT_TRUE(asst.find(move_tactic_1) != asst.end());
    EXPECT_EQ(asst.find(move_tactic_1)->second, robot_0.id());
}

// // Test the case where it is "obvious" which robots should be assigned to each tactic
// // Each robot is already close to one of the tactic's destinations, so it is trivial to
// // see the optimal assignment is for each robot to be assigned to the tactic whose
// // destination it's closest to
TEST_F(TacticAssignmentTest, test_assigning_2_robots_to_2_tactics_no_overlap)
{
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    Robot robot_1(1, Point(1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0, robot_1});
    world.updateFriendlyTeamState(friendly_team);

    auto move_tactic_1 = std::make_shared<MoveTactic>();
    auto move_tactic_2 = std::make_shared<MoveTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
    move_tactic_2->updateControlParams(Point(1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    TacticVector tactics = {move_tactic_1, move_tactic_2};

    // Each robot is close to separate tactic destinations. They should each be trivially
    // assigned to the tactic with the destination closest to their position

    auto robot_navigation_obstacle_config = ai_config.robot_navigation_obstacle_config();
    GlobalPathPlannerFactory path_planner_factory(robot_navigation_obstacle_config,
                                                  world.field());
    auto tup =
        assignTactics(path_planner_factory, world, tactics, friendly_team.getAllRobots());

    auto asst = std::get<2>(tup);


    ASSERT_TRUE(asst.find(move_tactic_1) != asst.end());
    ASSERT_TRUE(asst.find(move_tactic_2) != asst.end());

    EXPECT_EQ(asst.find(move_tactic_1)->second, robot_0.id());
    EXPECT_EQ(asst.find(move_tactic_2)->second, robot_1.id());
}

// // Test a more complex case where each robot is closest to the same tactic destination.
// // If robot0 were to be assigned to the tactic with dest1, robot1 would be forced to go
// // all the way to dest2. This is what happened in our previous system that used greedy
// // tactic assignment, and cause the robots to "overlap"
// //
// //     robot1
// //                     robot0
// //
// //
// //                     dest1             dest2
TEST_F(TacticAssignmentTest, test_assigning_2_robots_to_2_tactics_with_overlap)
{
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    Robot robot_1(1, Point(-3, 1.5), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0, robot_1});
    world.updateFriendlyTeamState(friendly_team);

    auto move_tactic_1 = std::make_shared<MoveTactic>();
    auto move_tactic_2 = std::make_shared<MoveTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
    move_tactic_2->updateControlParams(Point(1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    TacticVector tactics = {move_tactic_1, move_tactic_2};

    // Both robots are now closest to move_tactic_1's destination. We do NOT want
    // robot_0 to be assigned to move_tactic_1, because then robot_1 has to move all the
    // way around to move_tactic_2. What we expect is that robot_0 will be assigned to
    // move_tactic_2 and "slide over" to make room for robot_1

    auto robot_navigation_obstacle_config = ai_config.robot_navigation_obstacle_config();
    GlobalPathPlannerFactory path_planner_factory(robot_navigation_obstacle_config,
                                                  world.field());
    auto tup =
        assignTactics(path_planner_factory, world, tactics, friendly_team.getAllRobots());

    auto asst = std::get<2>(tup);


    ASSERT_TRUE(asst.find(move_tactic_1) != asst.end());
    ASSERT_TRUE(asst.find(move_tactic_2) != asst.end());
    EXPECT_EQ(asst.find(move_tactic_1)->second, robot_1.id());
    EXPECT_EQ(asst.find(move_tactic_2)->second, robot_0.id());
}

TEST_F(TacticAssignmentTest, test_assigning_3_robots_to_2_tactics)
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

    auto move_tactic_1 = std::make_shared<MoveTactic>();
    auto move_tactic_2 = std::make_shared<MoveTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
    move_tactic_2->updateControlParams(Point(1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    TacticVector tactics = {move_tactic_1, move_tactic_2};

    // robot_2 should not be assigned since both robot_0 and robot_1 are more optimal
    // to assign to the tactics. robot_2 is too far away

    auto robot_navigation_obstacle_config = ai_config.robot_navigation_obstacle_config();
    GlobalPathPlannerFactory path_planner_factory(robot_navigation_obstacle_config,
                                                  world.field());
    auto tup =
        assignTactics(path_planner_factory, world, tactics, friendly_team.getAllRobots());

    auto asst = std::get<2>(tup);

    ASSERT_TRUE(asst.find(move_tactic_1) != asst.end());
    ASSERT_TRUE(asst.find(move_tactic_2) != asst.end());
    EXPECT_EQ(asst.find(move_tactic_1)->second, robot_0.id());
    EXPECT_EQ(asst.find(move_tactic_2)->second, robot_1.id());
}

TEST_F(TacticAssignmentTest, test_assigning_3_robots_to_3_tactics_all_with_the_same_cost)
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

    auto stop_tactic_1 = std::make_shared<StopTactic>();
    auto stop_tactic_2 = std::make_shared<StopTactic>();
    auto stop_tactic_3 = std::make_shared<StopTactic>();

    TacticVector tactics = {stop_tactic_1, stop_tactic_2, stop_tactic_3};

    // If all costs are equal, the robots and tactics are simply paired in order
    auto robot_navigation_obstacle_config = ai_config.robot_navigation_obstacle_config();
    GlobalPathPlannerFactory path_planner_factory(robot_navigation_obstacle_config,
                                                  world.field());
    auto tup =
        assignTactics(path_planner_factory, world, tactics, friendly_team.getAllRobots());

    auto asst = std::get<2>(tup);


    ASSERT_TRUE(asst.find(stop_tactic_1) != asst.end());
    ASSERT_TRUE(asst.find(stop_tactic_2) != asst.end());
    ASSERT_TRUE(asst.find(stop_tactic_3) != asst.end());
    EXPECT_EQ(asst.find(stop_tactic_1)->second, robot_0.id());
    EXPECT_EQ(asst.find(stop_tactic_2)->second, robot_1.id());
    EXPECT_EQ(asst.find(stop_tactic_3)->second, robot_2.id());
}

TEST_F(TacticAssignmentTest, test_assigning_3_robots_to_3_tactics_with_2_of_the_same_cost)
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

    auto stop_tactic_1 = std::make_shared<StopTactic>();
    auto move_tactic_1 = std::make_shared<MoveTactic>();
    auto stop_tactic_2 = std::make_shared<StopTactic>();

    // The destination of the move_tactic is relatively close to the robot positions, so
    // the cost of assigning any robot to the move_tactic should be less than the
    // stop_tactics
    move_tactic_1->updateControlParams(Point(0, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    TacticVector tactics = {stop_tactic_1, move_tactic_1, stop_tactic_2};

    auto robot_navigation_obstacle_config = ai_config.robot_navigation_obstacle_config();
    GlobalPathPlannerFactory path_planner_factory(robot_navigation_obstacle_config,
                                                  world.field());
    auto tup =
        assignTactics(path_planner_factory, world, tactics, friendly_team.getAllRobots());

    auto asst = std::get<2>(tup);


    ASSERT_TRUE(asst.find(stop_tactic_1) != asst.end());
    ASSERT_TRUE(asst.find(move_tactic_1) != asst.end());
    ASSERT_TRUE(asst.find(stop_tactic_2) != asst.end());
    EXPECT_EQ(asst.find(stop_tactic_1)->second, robot_2.id());
    EXPECT_EQ(asst.find(move_tactic_1)->second, robot_0.id());
    EXPECT_EQ(asst.find(stop_tactic_2)->second, robot_1.id());
}

TEST_F(TacticAssignmentTest,
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

    auto move_tactic_1 = std::make_shared<MoveTactic>();

    move_tactic_1->updateControlParams(Point(0, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    TacticVector tactics = {move_tactic_1};

    auto robot_navigation_obstacle_config = ai_config.robot_navigation_obstacle_config();
    GlobalPathPlannerFactory path_planner_factory(robot_navigation_obstacle_config,
                                                  world.field());
    auto tup =
        assignTactics(path_planner_factory, world, tactics, friendly_team.getAllRobots());

    auto asst = std::get<2>(tup);


    ASSERT_TRUE(asst.find(move_tactic_1) != asst.end());
    EXPECT_EQ(asst.find(move_tactic_1)->second, robot_1.id());
}

TEST_F(TacticAssignmentTest,
       test_assigning_multiple_robots_to_goalie_tactic_goalie_set_on_team)
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

    TacticVector tactics = {};

    auto robot_navigation_obstacle_config = ai_config.robot_navigation_obstacle_config();
    GlobalPathPlannerFactory path_planner_factory(robot_navigation_obstacle_config,
                                                  world.field());
    auto tup =
        assignTactics(path_planner_factory, world, tactics, friendly_team.getAllRobots());

    auto asst = std::get<2>(tup);


    // EXPECT_EQ(1, asst.size());
    for (const auto& [tactic, robot] : asst)
    {
        UNUSED(tactic);
        EXPECT_EQ(robot, robot_0.id());
    }

    // Change the goalie and perform the same check in case we have a fluke bug
    friendly_team.assignGoalie(1);
    world.updateFriendlyTeamState(friendly_team);

    tup =
        assignTactics(path_planner_factory, world, tactics, friendly_team.getAllRobots());
    asst = std::get<2>(tup);


    // EXPECT_EQ(1, asst.size());
    for (const auto& [tactic, robot] : asst)
    {
        UNUSED(tactic);
        EXPECT_EQ(robot, robot_1.id());
    }
}

TEST_F(TacticAssignmentTest, test_assigning_stop_tactics_to_unassigned_non_goalie_robots)
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

    auto move_tactic_1 = std::make_shared<MoveTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    TacticVector tactics                        = {move_tactic_1};
    std::vector<Robot> expected_robots_assigned = {robot_0, robot_1, robot_2};

    auto robot_navigation_obstacle_config = ai_config.robot_navigation_obstacle_config();
    GlobalPathPlannerFactory path_planner_factory(robot_navigation_obstacle_config,
                                                  world.field());
    auto tup =
        assignTactics(path_planner_factory, world, tactics, friendly_team.getAllRobots());

    auto asst = std::get<2>(tup);

    // Check each tactic is assigned to the intended robot
    for (unsigned int i = 0; i < tactics.size(); i++)
    {
        ASSERT_TRUE(asst.find(tactics[i]) != asst.end());
        EXPECT_EQ(asst.find(tactics[i])->second, expected_robots_assigned[i].id());
    }
}

TEST_F(TacticAssignmentTest, test_greediness_of_tiered_assignment)
{
    // Assigning {{tactic_1, tactic_2}} should result in the lowest cost
    // assignment for all 2 tactics among the 2 robots.
    //
    // Assigning {{tactic_1}, {tactic_2}} should result in a more greedy
    // assignment, where the cost of assigning tactic_1 will be minimized first,
    // followed by tactic_2.
    //
    // Lets keep this on one axis to make the test easier to visualize
    //
    //              (r0) --- [m0] - (r1) ---- [m1]
    //
    //              -1.5      0      0.5       2
    //              robot0 tactic0  robot1  tactic1
    //
    // {{move_tactic_1, move_tactic_2}} we expect (r2 to m1) and (r1 to m2)
    // {{move_tactic_1}, {move_tactic_2}} we expect (r1 to m1) and (r2 to m2)
    Team friendly_team(Duration::fromSeconds(0));

    Robot robot_0(0, Point(-1.5, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    Robot robot_1(1, Point(0.5, 5), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));

    friendly_team.updateRobots({robot_0, robot_1});
    world.updateFriendlyTeamState(friendly_team);

    auto move_tactic_0 = std::make_shared<MoveTactic>();
    auto move_tactic_1 = std::make_shared<MoveTactic>();

    move_tactic_0->updateControlParams(Point(0, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
    move_tactic_1->updateControlParams(Point(2, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    TacticVector normal_tactics = {move_tactic_0, move_tactic_1};

    auto robot_navigation_obstacle_config = ai_config.robot_navigation_obstacle_config();
    GlobalPathPlannerFactory path_planner_factory(robot_navigation_obstacle_config,
                                                  world.field());
    auto tup = assignTactics(path_planner_factory, world, normal_tactics,
                             friendly_team.getAllRobots());

    auto asst = std::get<2>(tup);

    ASSERT_TRUE(asst.find(move_tactic_0) != asst.end());
    ASSERT_TRUE(asst.find(move_tactic_1) != asst.end());
    EXPECT_EQ(asst.find(move_tactic_0)->second, robot_0.id());
    EXPECT_EQ(asst.find(move_tactic_1)->second, robot_1.id());
}

TEST_F(TacticAssignmentTest, test_assignment_with_tiered_assignment)
{
    // Regardless of how the play yields the tactics to be assigned,
    // the goalie should always be assigned to the goalie assigned to the team
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-0.5, 0.2), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    // default is all capabilities, if not specified otherwise
    Robot robot_1(1, Point(-0.5, -0.2), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0, robot_1});

    friendly_team.assignGoalie(1);
    world.updateFriendlyTeamState(friendly_team);

    auto move_tactic_0 = std::make_shared<MoveTactic>();
    auto move_tactic_1 = std::make_shared<MoveTactic>();

    TacticVector request = {move_tactic_0, move_tactic_1};

    auto robot_navigation_obstacle_config = ai_config.robot_navigation_obstacle_config();
    GlobalPathPlannerFactory path_planner_factory(robot_navigation_obstacle_config,
                                                  world.field());
    auto tup =
        assignTactics(path_planner_factory, world, request, friendly_team.getAllRobots());

    auto asst = std::get<2>(tup);

    EXPECT_EQ(2, asst.size());

    {
        // bool has_goalie = false;
        bool has_move_0 = false;
        for (const auto& [tactic, robot] : asst)
        {
            if (objectTypeName(*tactic) == TYPENAME(GoalieTactic))
            {
                // has_goalie = true;
                EXPECT_EQ(robot_1.id(), robot);
            }
            if (tactic == move_tactic_0)
            {
                has_move_0 = true;
                EXPECT_EQ(robot_0.id(), robot);
            }
        }
        // EXPECT_TRUE(has_goalie);
        EXPECT_TRUE(has_move_0);
    }

    request = {move_tactic_0, move_tactic_1};
    tup =
        assignTactics(path_planner_factory, world, request, friendly_team.getAllRobots());

    asst = std::get<2>(tup);

    EXPECT_EQ(2, asst.size());

    {
        // bool has_goalie = false;
        bool has_move_0 = false;
        for (const auto& [tactic, robot] : asst)
        {
            if (objectTypeName(*tactic) == TYPENAME(GoalieTactic))
            {
                // has_goalie = true;
                EXPECT_EQ(robot_1.id(), robot);
            }
            if (tactic == move_tactic_0)
            {
                has_move_0 = true;
                EXPECT_EQ(robot_0.id(), robot);
            }
        }
        // EXPECT_TRUE(has_goalie);
        EXPECT_TRUE(has_move_0);
    }

    request = {move_tactic_1, move_tactic_0};
    tup =
        assignTactics(path_planner_factory, world, request, friendly_team.getAllRobots());

    asst = std::get<2>(tup);
    EXPECT_EQ(2, asst.size());

    {
        // bool has_goalie = false;
        bool has_move_1 = false;
        for (const auto& [tactic, robot] : asst)
        {
            if (objectTypeName(*tactic) == TYPENAME(GoalieTactic))
            {
                // has_goalie = true;
                EXPECT_EQ(robot_1.id(), robot);
            }
            if (tactic == move_tactic_1)
            {
                has_move_1 = true;
                EXPECT_EQ(robot_0.id(), robot);
            }
        }
        // EXPECT_TRUE(has_goalie);
        EXPECT_TRUE(has_move_1);
    }
}

TEST_F(TacticAssignmentTest, test_offense_play_with_substitution)
{
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-1.1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    Robot robot_1(1, Point(2, 0.81), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    Robot robot_2(2, Point(0, 5.0), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));

    friendly_team.updateRobots({robot_0, robot_1, robot_2});
    std::vector<Robot> injured_robots;
    injured_robots.push_back(robot_1);
    friendly_team.setInjuredRobots(injured_robots);
    world.updateFriendlyTeamState(friendly_team);

    OffensePlay play(ai_config);

    auto robot_navigation_obstacle_config = ai_config.robot_navigation_obstacle_config();
    GlobalPathPlannerFactory path_planner_factory(robot_navigation_obstacle_config,
                                                  world.field());
    InterPlayCommunication comm;
    auto p_set = play.get(path_planner_factory, world, comm,
                          [this](InterPlayCommunication comm) {});

    auto injured_primitive = p_set->robot_primitives().at(1);
    Point expected_pos(0, world.field().totalYLength() / 2);
    Point injured_robot_pos =
        createPoint(injured_primitive.move().motion_control().requested_destination());

    EXPECT_EQ(expected_pos, injured_robot_pos);
}
