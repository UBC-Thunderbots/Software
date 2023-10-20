#include <gtest/gtest.h>

#include <algorithm>
#include <unordered_set>

#include "software/ai/hl/stp/play/halt_play.h"
#include "software/ai/hl/stp/stp.h"
#include "software/ai/hl/stp/tactic/all_tactics.h"
#include "software/ai/hl/stp/tactic/halt/halt_tactic.h"
#include "software/test_util/test_util.h"

/**
 * This file contains unit tests for the assignRobotsToTactics function
 * of the STP module. It is in a separate file because there are so many tests, and it
 * is cleaner to keep separate from the rest of the STP tests
 */

class STPTacticAssignmentTest : public ::testing::Test
{
   public:
    STPTacticAssignmentTest() : ai_config(TbotsProto::AiConfig()), stp(ai_config) {}

   protected:
    void SetUp() override
    {
        // Give an explicit seed to STP so that our tests are deterministic
        stp   = STP(ai_config);
        world = ::TestUtil::createBlankTestingWorld();
    }

    /**
     * Check that all the given tactics have a robot assigned to them
     * @param tactics The tactics to check
     * @return True if all the given tactics have a robot assigned to them, false
     *         otherwise
     */
    bool allTacticsAssigned(
        ConstTacticVector tactics,
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

    TbotsProto::AiConfig ai_config;
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

    auto move_tactic_1 = std::make_shared<MoveTactic>();
    auto move_tactic_2 = std::make_shared<MoveTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
    move_tactic_2->updateControlParams(Point(1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    ConstTacticVector tactics = {move_tactic_1, move_tactic_2};

    auto asst = stp.assignRobotsToTactics({tactics}, world, true);

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

    auto move_tactic_1 = std::make_shared<MoveTactic>();
    auto move_tactic_2 = std::make_shared<MoveTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
    move_tactic_2->updateControlParams(Point(1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    ConstTacticVector tactics = {move_tactic_1, move_tactic_2};

    auto asst = stp.assignRobotsToTactics({tactics}, world, false);

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

    auto move_tactic_1 = std::make_shared<MoveTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    ConstTacticVector tactics = {move_tactic_1};

    auto asst = stp.assignRobotsToTactics({tactics}, world, false);

    EXPECT_TRUE(asst.find(move_tactic_1) != asst.end());
}

TEST_F(STPTacticAssignmentTest, test_0_tactics_returned_when_there_are_no_robots)
{
    Team friendly_team(Duration::fromSeconds(0));
    friendly_team.updateRobots({});
    world.updateFriendlyTeamState(friendly_team);

    auto move_tactic_1 = std::make_shared<MoveTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    ConstTacticVector tactics = {move_tactic_1};

    auto asst = stp.assignRobotsToTactics({tactics}, world, false);

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

    auto move_tactic_1 = std::make_shared<MoveTactic>();
    auto halt_tactic_1 = std::make_shared<HaltTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    ConstTacticVector tactics = {move_tactic_1, halt_tactic_1};

    // Both robots are now closest to move_tactic_1's destination. We do NOT want
    // robot_0 to be assigned to move_tactic_1, because then robot_1 has to move all the
    // way around to move_tactic_2. What we expect is that robot_0 will be assigned to
    // move_tactic_2 and "slide over" to make room for robot_1

    auto asst = stp.assignRobotsToTactics({tactics}, world, false);


    // move_tactic_1 should be the only Tactic assigned a robot, since halt_tactic_1 is a
    // lower priority than move_tactic_1 so it should be dropped since there's only 1
    // robot
    EXPECT_TRUE(asst.find(move_tactic_1) != asst.end());
    EXPECT_FALSE(asst.find(halt_tactic_1) != asst.end());
}


TEST_F(STPTacticAssignmentTest, test_assigning_1_tactic_to_1_robot)
{
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0});
    world.updateFriendlyTeamState(friendly_team);

    auto move_tactic_1 = std::make_shared<MoveTactic>();

    move_tactic_1->updateControlParams(Point(2, -3.2), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    ConstTacticVector tactics = {move_tactic_1};

    auto asst = stp.assignRobotsToTactics({tactics}, world, false);

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

    auto move_tactic_1 = std::make_shared<MoveTactic>();
    auto move_tactic_2 = std::make_shared<MoveTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
    move_tactic_2->updateControlParams(Point(1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    ConstTacticVector tactics = {move_tactic_1, move_tactic_2};

    // Each robot is close to separate tactic destinations. They should each be trivially
    // assigned to the tactic with the destination closest to their position

    auto asst = stp.assignRobotsToTactics({tactics}, world, false);

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

    auto move_tactic_1 = std::make_shared<MoveTactic>();
    auto move_tactic_2 = std::make_shared<MoveTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
    move_tactic_2->updateControlParams(Point(1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    ConstTacticVector tactics = {move_tactic_1, move_tactic_2};

    // Both robots are now closest to move_tactic_1's destination. We do NOT want
    // robot_0 to be assigned to move_tactic_1, because then robot_1 has to move all the
    // way around to move_tactic_2. What we expect is that robot_0 will be assigned to
    // move_tactic_2 and "slide over" to make room for robot_1

    auto asst = stp.assignRobotsToTactics({tactics}, world, false);

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

    auto move_tactic_1 = std::make_shared<MoveTactic>();
    auto move_tactic_2 = std::make_shared<MoveTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
    move_tactic_2->updateControlParams(Point(1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    ConstTacticVector tactics = {move_tactic_1, move_tactic_2};

    // robot_2 should not be assigned since both robot_0 and robot_1 are more optimal
    // to assign to the tactics. robot_2 is too far away

    auto asst = stp.assignRobotsToTactics({tactics}, world, false);

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

    auto halt_tactic_1 = std::make_shared<HaltTactic>();
    auto halt_tactic_2 = std::make_shared<HaltTactic>();
    auto halt_tactic_4 = std::make_shared<HaltTactic>();

    ConstTacticVector tactics = {halt_tactic_1, halt_tactic_2, halt_tactic_4};

    // If all costs are equal, the robots and tactics are simply paired in order
    auto asst = stp.assignRobotsToTactics({tactics}, world, false);

    ASSERT_TRUE(asst.find(halt_tactic_1) != asst.end());
    ASSERT_TRUE(asst.find(halt_tactic_2) != asst.end());
    ASSERT_TRUE(asst.find(halt_tactic_4) != asst.end());
    EXPECT_EQ(asst.find(halt_tactic_1)->second, robot_0);
    EXPECT_EQ(asst.find(halt_tactic_2)->second, robot_1);
    EXPECT_EQ(asst.find(halt_tactic_4)->second, robot_2);
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

    auto halt_tactic_1 = std::make_shared<HaltTactic>();
    auto move_tactic_1 = std::make_shared<MoveTactic>();
    auto halt_tactic_2 = std::make_shared<HaltTactic>();

    // The destination of the move_tactic is relatively close to the robot positions, so
    // the cost of assigning any robot to the move_tactic should be less than the
    // halt_tactics
    move_tactic_1->updateControlParams(Point(0, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    ConstTacticVector tactics = {halt_tactic_1, move_tactic_1, halt_tactic_2};

    auto asst = stp.assignRobotsToTactics({tactics}, world, false);

    ASSERT_TRUE(asst.find(halt_tactic_1) != asst.end());
    ASSERT_TRUE(asst.find(move_tactic_1) != asst.end());
    ASSERT_TRUE(asst.find(halt_tactic_2) != asst.end());
    EXPECT_EQ(asst.find(halt_tactic_1)->second, robot_2);
    EXPECT_EQ(asst.find(move_tactic_1)->second, robot_0);
    EXPECT_EQ(asst.find(halt_tactic_2)->second, robot_1);
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

    auto move_tactic_1 = std::make_shared<MoveTactic>();

    move_tactic_1->updateControlParams(Point(0, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    ConstTacticVector tactics = {move_tactic_1};

    auto asst = stp.assignRobotsToTactics({tactics}, world, false);

    ASSERT_TRUE(asst.find(move_tactic_1) != asst.end());
    EXPECT_EQ(asst.find(move_tactic_1)->second, robot_1);
}

TEST_F(STPTacticAssignmentTest,
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

    auto asst = stp.assignRobotsToTactics({}, world, true);

    EXPECT_EQ(1, asst.size());
    for (const auto& [tactic, robot] : asst)
    {
        UNUSED(tactic);
        EXPECT_EQ(robot, robot_0);
    }

    // Change the goalie and perform the same check in case we have a fluke bug
    friendly_team.assignGoalie(1);
    world.updateFriendlyTeamState(friendly_team);

    asst = stp.assignRobotsToTactics({}, world, true);

    EXPECT_EQ(1, asst.size());
    for (const auto& [tactic, robot] : asst)
    {
        UNUSED(tactic);
        EXPECT_EQ(robot, robot_1);
    }
}

TEST_F(STPTacticAssignmentTest,
       test_assigning_halt_tactics_to_unassigned_non_goalie_robots)
{
    // Test that HaltTactic is assigned to remaining robots without tactics

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

    ConstTacticVector tactics                   = {move_tactic_1};
    std::vector<Robot> expected_robots_assigned = {robot_0, robot_1, robot_2};

    auto asst = stp.assignRobotsToTactics({tactics}, world, false);

    // Check each tactic is assigned to the intended robot
    for (unsigned int i = 0; i < tactics.size(); i++)
    {
        ASSERT_TRUE(asst.find(tactics[i]) != asst.end());
        EXPECT_EQ(asst.find(tactics[i])->second, expected_robots_assigned[i]);
    }
}

TEST_F(STPTacticAssignmentTest, test_greediness_of_tiered_assignment)
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

    ConstPriorityTacticVector normal_tactics = {{move_tactic_0, move_tactic_1}};

    auto asst = stp.assignRobotsToTactics(normal_tactics, world, false);

    ASSERT_TRUE(asst.find(move_tactic_0) != asst.end());
    ASSERT_TRUE(asst.find(move_tactic_1) != asst.end());
    EXPECT_EQ(asst.find(move_tactic_0)->second, robot_0);
    EXPECT_EQ(asst.find(move_tactic_1)->second, robot_1);

    ConstPriorityTacticVector greedy_tactics = {{move_tactic_0}, {move_tactic_1}};

    asst = stp.assignRobotsToTactics(greedy_tactics, world, false);

    ASSERT_TRUE(asst.find(move_tactic_0) != asst.end());
    ASSERT_TRUE(asst.find(move_tactic_1) != asst.end());
    EXPECT_EQ(asst.find(move_tactic_0)->second, robot_0);
    EXPECT_EQ(asst.find(move_tactic_1)->second, robot_1);
}

TEST_F(STPTacticAssignmentTest, test_assignment_with_tiered_assignment)
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

    ConstPriorityTacticVector request = {{move_tactic_0, move_tactic_1}};

    auto asst = stp.assignRobotsToTactics(request, world, true);

    EXPECT_EQ(2, asst.size());

    {
        bool has_goalie = false;
        bool has_move_0 = false;
        for (const auto& [tactic, robot] : asst)
        {
            if (objectTypeName(*tactic) == TYPENAME(GoalieTactic))
            {
                has_goalie = true;
                EXPECT_EQ(robot_1, robot);
            }
            if (tactic == move_tactic_0)
            {
                has_move_0 = true;
                EXPECT_EQ(robot_0, robot);
            }
        }
        EXPECT_TRUE(has_goalie);
        EXPECT_TRUE(has_move_0);
    }

    request = {{move_tactic_0}, {move_tactic_1}};
    asst    = stp.assignRobotsToTactics(request, world, true);

    EXPECT_EQ(2, asst.size());

    {
        bool has_goalie = false;
        bool has_move_0 = false;
        for (const auto& [tactic, robot] : asst)
        {
            if (objectTypeName(*tactic) == TYPENAME(GoalieTactic))
            {
                has_goalie = true;
                EXPECT_EQ(robot_1, robot);
            }
            if (tactic == move_tactic_0)
            {
                has_move_0 = true;
                EXPECT_EQ(robot_0, robot);
            }
        }
        EXPECT_TRUE(has_goalie);
        EXPECT_TRUE(has_move_0);
    }

    request = {{move_tactic_1, move_tactic_0}};
    asst    = stp.assignRobotsToTactics(request, world, true);

    EXPECT_EQ(2, asst.size());

    {
        bool has_goalie = false;
        bool has_move_1 = false;
        for (const auto& [tactic, robot] : asst)
        {
            if (objectTypeName(*tactic) == TYPENAME(GoalieTactic))
            {
                has_goalie = true;
                EXPECT_EQ(robot_1, robot);
            }
            if (tactic == move_tactic_1)
            {
                has_move_1 = true;
                EXPECT_EQ(robot_0, robot);
            }
        }
        EXPECT_TRUE(has_goalie);
        EXPECT_TRUE(has_move_1);
    }
}

TEST_F(STPTacticAssignmentTest, test_multi_tier_assignment_with_tiered_assignment)
{
    Team friendly_team;
    friendly_team =
        TestUtil::setRobotPositionsHelper(friendly_team,
                                          {Point(-4, -2), Point(-3, -3), Point(-3.5, 2),
                                           Point(2.2, 3), Point(0.6, 0.3), Point(4.5, 3)},
                                          Timestamp::fromSeconds(323));
    friendly_team.assignGoalie(0);
    world.updateFriendlyTeamState(friendly_team);

    // TODO-AKHIL: This test is failing because the robot assigned to the move tactic is
    // not
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics = {
        std::make_shared<CreaseDefenderTactic>(
            ai_config.robot_navigation_obstacle_config()),
        std::make_shared<CreaseDefenderTactic>(
            ai_config.robot_navigation_obstacle_config())};

    Pass passer_pass(Point(2, 3), Point(0.5, 0.3), 2);
    auto attacker = std::make_shared<AttackerTactic>(ai_config);
    attacker->updateControlParams(passer_pass, true);
    auto receiver = std::make_shared<ReceiverTactic>();

    auto move_tactic                  = std::make_shared<MoveTactic>();
    ConstPriorityTacticVector request = {
        {attacker, receiver},
        {move_tactic, std::get<0>(crease_defender_tactics),
         std::get<1>(crease_defender_tactics)}};
    auto asst = stp.assignRobotsToTactics(request, world, true);
    EXPECT_EQ(6, asst.size());
    std::unordered_set<RobotId> assigned_robot_ids;
    for (const auto& [tactic, robot] : asst)
    {
        UNUSED(tactic);
        assigned_robot_ids.insert(robot.id());
    }
    EXPECT_EQ(6, assigned_robot_ids.size());
    EXPECT_TRUE(allTacticsAssigned(
        {attacker, receiver, move_tactic, std::get<0>(crease_defender_tactics),
         std::get<1>(crease_defender_tactics)},
        asst));
}
