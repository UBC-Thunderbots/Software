
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
    world->updateFriendlyTeamState(friendly_team);

    auto move_tactic_1 = std::make_shared<MoveTactic>();

    move_tactic_1->updateControlParams(Point(-1, 0), Angle::zero(), 0,
                                       TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    TacticVector tactics                        = {move_tactic_1};
    std::vector<Robot> expected_robots_assigned = {robot_0, robot_1, robot_2};
    auto robot_navigation_obstacle_config = ai_config.robot_navigation_obstacle_config();

    RobotNavigationObstacleFactory robot_navigation_obstacle_factory(
        robot_navigation_obstacle_config);
    TbotsProto::ObstacleList obstacle_list;
    TbotsProto::PathVisualization path_visualization;
    auto tup = assignTactics(world, tactics, friendly_team.getAllRobots(),
                             robot_navigation_obstacle_factory, obstacle_list,
                             path_visualization);

    auto asst = std::get<2>(tup);

    // Check each tactic is assigned to the intended robot
    for (unsigned int i = 0; i < tactics.size(); i++)
    {
        ASSERT_TRUE(asst.find(tactics[i]) != asst.end());
        EXPECT_EQ(asst.find(tactics[i])->second, expected_robots_assigned[i].id());
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
    world->updateFriendlyTeamState(friendly_team);

    OffensePlay play(ai_config);

    auto robot_navigation_obstacle_config = ai_config.robot_navigation_obstacle_config();
    world->field();
    InterPlayCommunication comm;
    auto p_set = play.get(world, comm, [this](InterPlayCommunication comm) {});

    auto injured_primitive = p_set->robot_primitives().at(1);
    Point expected_pos(0, world->field().totalYLength() / 2);
    Point injured_robot_pos =
        createPoint(injured_primitive.move().xy_traj_params().destination());
    EXPECT_EQ(expected_pos, injured_robot_pos);
}
