#include "software/ai/hl/stp/stp.h"

#include <google/protobuf/map.h>
#include <munkres/munkres.h>

#include <algorithm>
#include <boost/bind.hpp>
#include <chrono>
#include <exception>
#include <random>

#include "proto/play_info_msg.pb.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/halt_play.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/all_tactics.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/stop_intent.h"
#include "software/ai/motion_constraint/motion_constraint_set_builder.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"
#include "software/util/typename/typename.h"

STP::STP(std::shared_ptr<const AiConfig> ai_config)
    : robot_tactic_assignment(),
      ai_config(ai_config),
      goalie_tactic(std::make_shared<GoalieTactic>(ai_config)),
      stop_tactics(),
      current_play(std::make_unique<HaltPlay>(ai_config)),
      fsm(std::make_unique<FSM<PlaySelectionFSM>>(PlaySelectionFSM{ai_config})),
      override_play(nullptr)
{
    ai_config->getAiControlConfig()->getCurrentAiPlay()->registerCallbackFunction(
        [this, ai_config](std::string new_override_play_name) {
            if (ai_config->getAiControlConfig()->getOverrideAiPlay()->value())
            {
                overridePlayFromName(new_override_play_name);
            }
        });

    ai_config->getAiControlConfig()->getOverrideAiPlay()->registerCallbackFunction(
        [this, ai_config](bool new_override_ai_play) {
            if (new_override_ai_play)
            {
                overridePlayFromName(
                    ai_config->getAiControlConfig()->getCurrentAiPlay()->value());
            }
        });

    for (unsigned int i = 0; i < MAX_ROBOT_IDS; i++)
    {
        stop_tactics.push_back(std::make_shared<StopTactic>(false));
    }
}

std::vector<std::unique_ptr<Intent>> STP::getIntentsFromCurrentPlay(const World& world)
{
    fsm->process_event(PlaySelectionFSM::Update(
        [this](std::unique_ptr<Play> play) { current_play = std::move(play); },
        world.gameState()));

    auto assignment_function = [this](const ConstPriorityTacticVector& tactics,
                                      const World& world,
                                      bool automatically_assign_goalie) {
        return assignRobotsToTactics(tactics, world, automatically_assign_goalie);
    };
    auto motion_constraint_function = [world](const Tactic& tactic) {
        return buildMotionConstraintSet(world.gameState(), tactic);
    };

    if (static_cast<bool>(override_play))
    {
        return override_play->get(assignment_function, motion_constraint_function, world);
    }
    else
    {
        return current_play->get(assignment_function, motion_constraint_function, world);
    }
}

std::vector<std::unique_ptr<Intent>> STP::getIntents(const World& world)
{
    auto intents = getIntentsFromCurrentPlay(world);

    auto all_tactics = stop_tactics;
    all_tactics.push_back(goalie_tactic);
    for (auto tactic : all_tactics)
    {
        auto iter = robot_tactic_assignment.find(tactic);
        if (iter != robot_tactic_assignment.end())
        {
            auto intent = tactic->get(iter->second, world);
            intent->setMotionConstraints(
                buildMotionConstraintSet(world.gameState(), *tactic));
            intents.push_back(std::move(intent));
        }
    }

    return intents;
}

TbotsProto::PlayInfo STP::getPlayInfo()
{
    std::string info_play_name = objectTypeName(*current_play);
    TbotsProto::PlayInfo info;
    info.mutable_play()->set_play_name(info_play_name);

    for (const auto& [tactic, robot] : robot_tactic_assignment)
    {
        TbotsProto::PlayInfo_Tactic tactic_msg;
        tactic_msg.set_tactic_name(objectTypeName(*tactic));
        tactic_msg.set_tactic_fsm_state(tactic->getFSMState());
        (*info.mutable_robot_tactic_assignment())[robot.id()] = tactic_msg;
    }

    return info;
}

std::map<std::shared_ptr<const Tactic>, Robot> STP::assignRobotsToTactics(
    ConstPriorityTacticVector tactics, const World& world,
    bool automatically_assign_goalie)
{
    robot_tactic_assignment.clear();

    std::optional<Robot> goalie_robot = world.friendlyTeam().goalie();
    std::vector<Robot> robots         = world.friendlyTeam().getAllRobots();

    if (goalie_robot && automatically_assign_goalie)
    {
        robot_tactic_assignment.emplace(goalie_tactic, goalie_robot.value());

        robots.erase(std::remove(robots.begin(), robots.end(), goalie_robot.value()),
                     robots.end());
    }

    // This functions optimizes the assignment of robots to tactics by minimizing
    // the total cost of assignment using the Hungarian algorithm
    // (also known as the Munkres algorithm)
    // https://en.wikipedia.org/wiki/Hungarian_algorithm
    //
    // https://github.com/saebyn/munkres-cpp is the implementation of the Hungarian
    // algorithm that we use here
    for (auto tactic_vector : tactics)
    {
        size_t num_tactics = tactic_vector.size();

        if (robots.size() < tactic_vector.size())
        {
            // We do not have enough robots to assign all the tactics to. We "drop"
            // (aka don't assign) the tactics at the end of the vector since they are
            // considered lower priority
            tactic_vector.resize(robots.size());
            num_tactics = tactic_vector.size();
        }
        else
        {
            // Assign rest of robots with StopTactic
            for (unsigned int i = 0; i < (robots.size() - tactic_vector.size()); i++)
            {
                tactic_vector.push_back(stop_tactics[i]);
            }
        }

        size_t num_rows = robots.size();
        size_t num_cols = tactic_vector.size();

        // The Matrix constructor will assert if the rows and columns of the matrix are
        // not >= 1, so we perform that check first and skip over this tactic_vector if
        // it is empty. This represents the cases where there are either no tactics or no
        // robots
        if (num_rows == 0 || num_cols == 0)
        {
            continue;
        }

        // The rows of the matrix are the "workers" (the robots) and the columns are the
        // "jobs" (the Tactics).
        Matrix<double> matrix(num_rows, num_cols);

        // Initialize the matrix with the cost of assigning each Robot to each Tactic
        for (size_t row = 0; row < num_rows; row++)
        {
            for (size_t col = 0; col < num_cols; col++)
            {
                Robot robot                           = robots.at(row);
                std::shared_ptr<const Tactic>& tactic = tactic_vector.at(col);
                double robot_cost_for_tactic = tactic->calculateRobotCost(robot, world);

                std::set<RobotCapability> required_capabilities =
                    tactic->robotCapabilityRequirements();
                std::set<RobotCapability> robot_capabilities =
                    robot.getAvailableCapabilities();
                std::set<RobotCapability> missing_capabilities;
                std::set_difference(
                    required_capabilities.begin(), required_capabilities.end(),
                    robot_capabilities.begin(), robot_capabilities.end(),
                    std::inserter(missing_capabilities, missing_capabilities.begin()));

                if (missing_capabilities.size() > 0)
                {
                    matrix(row, col) = robot_cost_for_tactic + 10.0f;
                }
                else
                {
                    // capability requirements are satisfied, use real cost
                    matrix(row, col) = robot_cost_for_tactic;
                }
            }
        }

        // Apply the Munkres/Hungarian algorithm to the matrix.
        Munkres<double> m;
        m.solve(matrix);

        // The Munkres matrix gets solved such that there will be exactly one 0 in every
        // row and exactly one 0 in every column. All other values will be -1. The 0's
        // indicate the "workers" and "jobs" (robots and tactics for us) that are most
        // optimally paired together
        //
        // Example matrices:
        //        -1, 0,-1,         and            0,-1,
        //         0,-1,-1,                       -1, 0,
        //        -1,-1, 0,
        auto remaining_robots = robots;

        for (size_t row = 0; row < num_rows; row++)
        {
            for (size_t col = 0; col < num_tactics; col++)
            {
                auto val = matrix(row, col);
                if (val == 0)
                {
                    robot_tactic_assignment.emplace(tactic_vector.at(col),
                                                    robots.at(row));
                    remaining_robots.erase(
                        std::remove_if(remaining_robots.begin(), remaining_robots.end(),
                                       [robots, row](const Robot& robot) {
                                           return robot.id() == robots.at(row).id();
                                       }),
                        remaining_robots.end());
                    break;
                }
            }
        }

        robots = remaining_robots;
    }

    return robot_tactic_assignment;
}

void STP::overridePlay(std::unique_ptr<Play> play)
{
    override_play = std::move(play);
}

void STP::overridePlayFromName(std::string name)
{
    overridePlay(GenericFactory<std::string, Play, AiConfig>::create(name, ai_config));
}
