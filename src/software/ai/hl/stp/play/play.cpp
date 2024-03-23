#include "software/ai/hl/stp/play/play.h"

#include <munkres/munkres.h>

#include <Tracy.hpp>

#include "proto/message_translation/tbots_protobuf.h"
#include "software/ai/hl/stp/play/tactic_assignment.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/ai/motion_constraint/motion_constraint_set_builder.h"
#include "software/geom/angle.h"
#include "software/logger/logger.h"


Play::Play(TbotsProto::AiConfig ai_config, bool requires_goalie)
    : ai_config(ai_config),
      goalie_tactic(std::make_shared<GoalieTactic>(ai_config)),
      stop_tactics(),
      requires_goalie(requires_goalie),
      tactic_sequence(boost::bind(&Play::getNextTacticsWrapper, this, _1)),
      world_ptr_(std::nullopt),
      obstacle_factory(ai_config.robot_navigation_obstacle_config())
{
    for (unsigned int i = 0; i < MAX_ROBOT_IDS; i++)
    {
        stop_tactics.push_back(std::make_shared<StopTactic>());
    }
}

PriorityTacticVector Play::getTactics(const WorldPtr &world_ptr)
{
    // Update the member variable that stores the world. This will be used by the
    // getNextTacticsWrapper function (inside the coroutine) to pass the World data to
    // the getNextTactics function. This is easier than directly passing the World data
    // into the coroutine
    world_ptr_ = world_ptr;
    // Check the coroutine status to see if it has any more work to do.
    if (tactic_sequence)
    {
        // Run the coroutine. This will call the bound getNextTactics function
        tactic_sequence();
    }
    else
    {
        // Make a new tactic_sequence
        tactic_sequence = TacticCoroutine::pull_type(
            boost::bind(&Play::getNextTacticsWrapper, this, _1));
        // Run the coroutine. This will call the bound getNextTactics function
        tactic_sequence();
    }

    // Check if the coroutine is still valid before getting the result. This makes
    // sure we don't try get the result after "running out the bottom" of the
    // coroutine function
    if (tactic_sequence)
    {
        // Extract the result from the coroutine. This will be whatever value was
        // yielded by the getNextTactics function
        auto next_tactics = tactic_sequence.get();
        return next_tactics;
    }
    else
    {
        // Make a new tactic_sequence
        tactic_sequence = TacticCoroutine::pull_type(
            boost::bind(&Play::getNextTacticsWrapper, this, _1));
        // Run the coroutine. This will call the bound getNextTactics function
        tactic_sequence();
        if (tactic_sequence)
        {
            // Extract the result from the coroutine. This will be whatever value was
            // yielded by the getNextTactics function
            auto next_tactics = tactic_sequence.get();
            return next_tactics;
        }
        else
        {
            LOG(WARNING) << "Failed to restart play" << std::endl;
        }
    }
    // If the coroutine "iterator" is done, the getNextTactics function has completed
    // and has no more work to do. Therefore, the Play is done so we return an empty
    // vector
    return PriorityTacticVector();
}

std::unique_ptr<TbotsProto::PrimitiveSet> Play::get(
    const WorldPtr &world_ptr, const InterPlayCommunication &inter_play_communication,
    const SetInterPlayCommunicationCallback &set_inter_play_communication_fun)
{
    PriorityTacticVector priority_tactics;
    unsigned int num_tactics =
        static_cast<unsigned int>(world_ptr->friendlyTeam().numRobots());
    if (requires_goalie && world_ptr->friendlyTeam().goalie())
    {
        num_tactics--;
    }

    {
        ZoneNamedN(_tracy_tactics, "Play: Get Tactics from Play", true);

        updateTactics(PlayUpdate(
            world_ptr, num_tactics,
            [&priority_tactics](PriorityTacticVector new_tactics) {
                priority_tactics = std::move(new_tactics);
            },
            inter_play_communication, set_inter_play_communication_fun));
    }

    auto primitives_to_run = std::make_unique<TbotsProto::PrimitiveSet>();

    // Reset the visualization protobufs
    obstacle_list.Clear();
    path_visualization.Clear();

    tactic_robot_id_assignment.clear();

    std::optional<Robot> goalie_robot = world_ptr->friendlyTeam().goalie();
    std::vector<Robot> robots         = world_ptr->friendlyTeam().getAllRobots();
    std::vector<Robot> injured_robots = world_ptr->friendlyTeam().getInjuredRobots();

    if (requires_goalie)
    {
        if (goalie_robot.has_value())
        {
            RobotId goalie_robot_id = goalie_robot.value().id();
            tactic_robot_id_assignment.emplace(goalie_tactic, goalie_robot_id);

            robots.erase(std::remove(robots.begin(), robots.end(), goalie_robot.value()),
                         robots.end());

            auto motion_constraints =
                buildMotionConstraintSet(world_ptr->gameState(), *goalie_tactic);
            auto primitives = goalie_tactic->get(world_ptr);
            CHECK(primitives.contains(goalie_robot_id))
                << "Couldn't find a primitive for robot id " << goalie_robot_id;
            auto primitive_proto =
                primitives[goalie_robot_id]->generatePrimitiveProtoMessage(
                    world_ptr, motion_constraints, obstacle_factory);

            primitives_to_run->mutable_robot_primitives()->insert(
                {goalie_robot_id, *primitive_proto});
            goalie_tactic->setLastExecutionRobot(goalie_robot_id);

            primitives[goalie_robot_id]->getVisualizationProtos(obstacle_list,
                                                                path_visualization);
        }
        else if (world_ptr->friendlyTeam().getGoalieId().has_value())
        {
            LOG(WARNING) << "Robot not found for goalie ID: "
                         << std::to_string(
                                world_ptr->friendlyTeam().getGoalieId().value())
                         << std::endl;
        }
        else
        {
            LOG(WARNING) << "No goalie ID set!" << std::endl;
        }
    }

    /* substitute injured robots when the game is in play */
    if (injured_robots.size() > 0)
    {
        unsigned int num_injured = (unsigned int)injured_robots.size();
        num_tactics -= num_injured;

        // substitution tactic is just a move tactic to a decided location
        std::shared_ptr<MoveTactic> auto_sub_tactic = std::make_shared<MoveTactic>();

        // move to middle of court and to positive y boundary and stop
        auto_sub_tactic->updateControlParams(
            Point(0, world_ptr->field().totalYLength() / 2), Angle::zero(), 0);

        for (auto robot : injured_robots)
        {
            // assign robot to auto_sub tactic
            tactic_robot_id_assignment.emplace(auto_sub_tactic, robot.id());

            // remove substitution robot from robot list
            robots.erase(std::remove(robots.begin(), robots.end(), robot), robots.end());

            auto motion_constraints =
                buildMotionConstraintSet(world_ptr->gameState(), *auto_sub_tactic);
            auto primitives = auto_sub_tactic->get(world_ptr);
            CHECK(primitives.contains(robot.id()))
                << "Couldn't find a primitive for robot id " << robot.id();
            auto primitive_proto = primitives[robot.id()]->generatePrimitiveProtoMessage(
                world_ptr, motion_constraints, obstacle_factory);
            primitives_to_run->mutable_robot_primitives()->insert(
                google::protobuf::MapPair(robot.id(), *primitive_proto));
            auto_sub_tactic->setLastExecutionRobot(robot.id());
        }
    }

    // This functions optimizes the assignment of robots to tactics by minimizing
    // the total cost of assignment using the Hungarian algorithm
    // (also known as the Munkres algorithm)
    // https://en.wikipedia.org/wiki/Hungarian_algorithm
    //
    // https://github.com/saebyn/munkres-cpp is the implementation of the Hungarian
    // algorithm that we use here
    {
        ZoneNamedN(_tracy_tactic_assignment, "Play: Assign tactics to robots", true);

        for (unsigned int i = 0; i < priority_tactics.size(); i++)
        {
            auto tactic_vector = priority_tactics[i];
            size_t num_tactics = tactic_vector.size();

            if (robots.size() < tactic_vector.size())
            {
                // We do not have enough robots to assign all the tactics to. We "drop"
                // (aka don't assign) the tactics at the end of the vector since they are
                // considered lower priority
                tactic_vector.resize(robots.size());
            }
            else if (i == (priority_tactics.size() - 1))
            {
                // If assigning the last tactic vector, then assign rest of robots with
                // StopTactics
                for (unsigned int ii = 0; ii < (robots.size() - num_tactics); ii++)
                {
                    tactic_vector.push_back(stop_tactics[ii]);
                }
            }

            auto [remaining_robots, new_primitives_to_assign,
                  current_tactic_robot_id_assignment] =
                assignTactics(world_ptr, tactic_vector, robots, obstacle_factory,
                              obstacle_list, path_visualization);

            tactic_robot_id_assignment.merge(current_tactic_robot_id_assignment);

            for (auto &[robot_id, primitive] :
                 new_primitives_to_assign->robot_primitives())
            {
                primitives_to_run->mutable_robot_primitives()->insert(
                    google::protobuf::MapPair(robot_id, primitive));
            }

            robots = remaining_robots;
        }
    }

    // TODO (#3104): Remove duplicated obstacles from obstacle_list
    // Visualize all obstacles and paths
    LOG(VISUALIZE) << obstacle_list;
    LOG(VISUALIZE) << path_visualization;

    primitives_to_run->mutable_time_sent()->set_epoch_timestamp_seconds(
        world_ptr->getMostRecentTimestamp().toSeconds());
    primitives_to_run->set_sequence_number(sequence_number++);

    return primitives_to_run;
}

const std::map<std::shared_ptr<const Tactic>, RobotId> &Play::getTacticRobotIdAssignment()
    const
{
    return tactic_robot_id_assignment;
}

void Play::getNextTacticsWrapper(TacticCoroutine::push_type &yield)
{
    // Yield an empty vector the very first time the function is called. This value will
    // never be seen/used by the rest of the system.
    yield({});

    // The getNextTactics function is given the World as a parameter rather than using
    // the member variable since it's more explicit and obvious where the World
    // comes from when implementing Plays. The World is passed as a reference, so when
    // the world member variable is updated the implemented Plays will have access
    // to the updated world as well.
    if (world_ptr_)
    {
        getNextTactics(yield, world_ptr_.value());
    }
}

// TODO (#2359): delete once all plays are not coroutines
void Play::updateTactics(const PlayUpdate &play_update)
{
    play_update.set_tactics(getTactics(play_update.world_ptr));
}

std::vector<std::string> Play::getState()
{
    // by default just return the name of the play
    return {objectTypeName(*this)};
}

std::vector<Robot> Play::getInjuredRobots(const World &world)
{
    std::vector<Robot> injured_robots;
    return injured_robots;
}
