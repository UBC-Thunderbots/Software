#include "software/test_util/test_util.h"

#include "software/new_geom/util/distance.h"

namespace TestUtil
{
    World createBlankTestingWorld()
    {
        Field field        = Field::createSSLDivisionBField();
        Team friendly_team = Team(Duration::fromMilliseconds(1000));
        Team enemy_team    = Team(Duration::fromMilliseconds(1000));
        Ball ball          = Ball(Point(), Vector(), Timestamp::fromSeconds(0));

        World world = World(field, ball, friendly_team, enemy_team);

        return world;
    }

    Team setRobotPositionsHelper(Team team, const std::vector<Point> &robot_positions,
                                 const Timestamp &timestamp)
    {
        std::vector<Robot> robots;
        unsigned int robot_id_index = 0;
        for (const Point &robot_position : robot_positions)
        {
            Robot robot = Robot(robot_id_index, robot_position, Vector(), Angle::zero(),
                                AngularVelocity::zero(), timestamp);
            robots.emplace_back(robot);

            robot_id_index++;
        }

        team.clearAllRobots();
        team.updateRobots(robots);

        return team;
    }

    World setFriendlyRobotPositions(World world, std::vector<Point> robot_positions,
                                    const Timestamp &timestamp)
    {
        Team new_friendly_team =
            setRobotPositionsHelper(world.friendlyTeam(), robot_positions, timestamp);
        world.updateFriendlyTeamState(new_friendly_team);

        return world;
    }

    World setEnemyRobotPositions(World world, std::vector<Point> robot_positions,
                                 const Timestamp &timestamp)
    {
        Team new_enemy_team =
            setRobotPositionsHelper(world.enemyTeam(), robot_positions, timestamp);
        world.updateEnemyTeamState(new_enemy_team);

        return world;
    }

    World setBallPosition(World world, Point ball_position, Timestamp timestamp)
    {
        TimestampedBallState ballState =
            TimestampedBallState(ball_position, world.ball().velocity(), timestamp);
        world.updateBallStateWithTimestamp(ballState);

        return world;
    }

    World setBallVelocity(World world, Vector ball_velocity, Timestamp timestamp)
    {
        TimestampedBallState ballState =
            TimestampedBallState(world.ball().position(), ball_velocity, timestamp);
        world.updateBallStateWithTimestamp(ballState);

        return world;
    }

    std::vector<RefboxGameState> getAllRefboxGameStates()
    {
        std::vector<RefboxGameState> game_states;
        for (int i = 0; i < static_cast<int>(RefboxGameState::REFBOX_GAME_STATE_COUNT);
             i++)
        {
            game_states.push_back(static_cast<RefboxGameState>(i));
        }
        return game_states;
    }

    Robot createRobotAtPos(const Point &pt)
    {
        static RobotId robot_id_counter = 0;
        return Robot(robot_id_counter++, pt, Vector(), Angle(), AngularVelocity(),
                     Timestamp());
    }

    ::testing::AssertionResult equalWithinTolerance(const Polygon &poly1,
                                                    const Polygon &poly2,
                                                    double tolerance)
    {
        auto ppts1 = poly1.getPoints();
        auto ppts2 = poly2.getPoints();
        if (std::equal(ppts1.begin(), ppts1.end(), ppts2.begin(),
                       [tolerance](const Point &p1, const Point &p2) {
                           return equalWithinTolerance(p1, p2, tolerance);
                       }))
        {
            return ::testing::AssertionSuccess();
        }
        else
        {
            return ::testing::AssertionFailure()
                   << "Polygon 1 was " << poly1 << ", polygon 2 was " << poly2;
        }
    }

    ::testing::AssertionResult equalWithinTolerance(const Circle &c1, const Circle &c2,
                                                    double tolerance)
    {
        if (equalWithinTolerance(c1.getOrigin(), c2.getOrigin(), tolerance) &&
            equalWithinTolerance(c1.getRadius(), c2.getRadius(), tolerance))
        {
            return ::testing::AssertionSuccess();
        }
        else
        {
            return ::testing::AssertionFailure()
                   << "Circle 1 was " << c1 << ", circle 2 was " << c2;
        }
    }

    ::testing::AssertionResult equalWithinTolerance(const Angle &a1, const Angle &a2,
                                                    const Angle &tolerance)
    {
        // subtract a fixed epsilon for error in:
        // - angle subtraction (internal to minDiff)
        // - angle clamping (internal to minDiff)
        // - angle absolute value (internal to minDiff)
        // - the tolerance abs()
        auto difference = a1.minDiff(a2) - Angle::fromRadians(FIXED_EPSILON * 4);
        if (difference < tolerance.abs())
        {
            return ::testing::AssertionSuccess();
        }
        else
        {
            return ::testing::AssertionFailure()
                   << "Angle 1 was " << a1 << ", angle 2 was " << a2;
        }
    }


    ::testing::AssertionResult equalWithinTolerance(const Vector &v1, const Vector &v2,
                                                    double tolerance)
    {
        double distance = (v1 - v2).length();
        if (equalWithinTolerance(distance, 0, tolerance))
        {
            return ::testing::AssertionSuccess();
        }
        else
        {
            return ::testing::AssertionFailure()
                   << "Vector 1 was " << v1 << ", vector 2 was " << v2;
        }
    }

    ::testing::AssertionResult equalWithinTolerance(const Point &pt1, const Point &pt2,
                                                    double tolerance)
    {
        double dist = distance(pt1, pt2);
        if (equalWithinTolerance(dist, 0, tolerance))
        {
            return ::testing::AssertionSuccess();
        }
        else
        {
            return ::testing::AssertionFailure()
                   << "Point 1 was " << pt1 << ", point 2 was " << pt2;
        }
    }

    ::testing::AssertionResult equalWithinTolerance(double val1, double val2,
                                                    double tolerance)
    {
        // subtracting one fixed epsilon to account for the error in fabs and one fixed
        // epsilon to account for the error in subtracting the two vals
        double difference = fabs(val1 - val2) - FIXED_EPSILON * 2;
        if (difference < tolerance)
        {
            return ::testing::AssertionSuccess();
        }
        else
        {
            return ::testing::AssertionFailure()
                   << "Value 1 was " << val1 << ", value 2 was " << val2;
        }
    }

    ::testing::AssertionResult equalWithinTolerance(const RobotState &state1,
                                                    const RobotState &state2,
                                                    const double linear_tolerance,
                                                    const Angle &angular_tolerance)
    {
        auto position_equality_result =
            equalWithinTolerance(state1.position(), state2.position(), linear_tolerance);
        auto velocity_equality_result =
            equalWithinTolerance(state1.velocity(), state2.velocity(), linear_tolerance);
        auto orientation_equality_result = equalWithinTolerance(
            state1.orientation(), state2.orientation(), angular_tolerance);
        auto angular_velocity_equality_result = equalWithinTolerance(
            state1.angularVelocity(), state2.angularVelocity(), angular_tolerance);

        auto assertion_result = ::testing::AssertionSuccess();

        if (!position_equality_result || !velocity_equality_result ||
            !orientation_equality_result || !angular_velocity_equality_result)
        {
            assertion_result = ::testing::AssertionFailure();

            if (!position_equality_result)
            {
                assertion_result << "The first state's position was " << state1.position()
                                 << ", the second state's position was "
                                 << state2.position();
            }
            if (!velocity_equality_result)
            {
                assertion_result << std::endl
                                 << "The first state's velocity was " << state1.velocity()
                                 << ", the second state's velocity was "
                                 << state2.velocity();
            }
            if (!orientation_equality_result)
            {
                assertion_result
                    << std::endl
                    << "The first state's orientation was " << state1.orientation()
                    << ", the second state's orientation was " << state2.orientation();
            }
            if (!angular_velocity_equality_result)
            {
                assertion_result << std::endl
                                 << "The first state's angular velocity was "
                                 << state1.angularVelocity()
                                 << ", the second state's angular velocity was "
                                 << state2.angularVelocity();
            }
        }

        return assertion_result;
    }

    ::testing::AssertionResult equalWithinTolerance(const RobotStateWithId &state1,
                                                    const RobotStateWithId &state2,
                                                    const double linear_tolerance,
                                                    const Angle &angular_tolerance)
    {
        if (state1.id != state2.id)
        {
            return ::testing::AssertionFailure()
                   << "The first state's id was " << state1.id
                   << ", the second state's id was " << state2.id;
        }
        auto state_equality_result = equalWithinTolerance(
            state1.robot_state, state2.robot_state, linear_tolerance, angular_tolerance);
        if (!state_equality_result)
        {
            return state_equality_result;
        }

        return ::testing::AssertionSuccess();
    }

    ::testing::AssertionResult equalWithinTolerance(const BallState &state1,
                                                    const BallState &state2,
                                                    double tolerance)
    {
        auto position_equality_result =
            equalWithinTolerance(state1.position(), state2.position(), tolerance);
        auto velocity_equality_result =
            equalWithinTolerance(state1.velocity(), state2.velocity(), tolerance);

        auto assertion_result = ::testing::AssertionSuccess();

        if (!position_equality_result || !velocity_equality_result)
        {
            assertion_result = ::testing::AssertionFailure();

            if (!position_equality_result)
            {
                assertion_result << "The first state's position was " << state1.position()
                                 << ", the second state's position was "
                                 << state2.position();
            }
            if (!velocity_equality_result)
            {
                assertion_result << std::endl
                                 << "The first state's velocity was " << state1.velocity()
                                 << ", the second state's velocity was "
                                 << state2.velocity();
            }
        }

        return assertion_result;
    }

    double secondsSince(std::chrono::time_point<std::chrono::system_clock> start_time)
    {
        return millisecondsSince(start_time) / MILLISECONDS_PER_SECOND;
    }

    double millisecondsSince(
        std::chrono::time_point<std::chrono::system_clock> start_time)
    {
        const auto end_time = std::chrono::system_clock::now();
        return static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                       end_time - start_time)
                                       .count()) /
               NANOSECONDS_PER_MILLISECOND;
    }
};  // namespace TestUtil
