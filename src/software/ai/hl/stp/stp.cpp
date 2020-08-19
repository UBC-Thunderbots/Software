#include "software/ai/hl/stp/stp.h"

#include <munkres/munkres.h>

#include <algorithm>
#include <chrono>
#include <exception>
#include <random>

#include "software/ai/hl/stp/action/action_world_params_update_visitor.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play_info.h"
#include "software/ai/hl/stp/stp_impl.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/hl/stp/tactic/tactic_world_params_update_visitor.h"
#include "software/ai/intent/stop_intent.h"
#include "software/logger/logger.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/util/design_patterns/generic_factory.h"
#include "software/util/typename/typename.h"

STP::STP(std::function<std::unique_ptr<Play>()> default_play_constructor,
         std::shared_ptr<const AIControlConfig> control_config, long random_seed)
    : default_play_constructor(default_play_constructor),
      random_number_generator(random_seed),
      control_config(control_config)
{
}

void STP::updateSTPState(const World& world)
{
    updateGameState(world);
    updateAIPlay(world);
}

void STP::updateGameState(const World& world)
{
    current_game_state = world.gameState();
    if (control_config->OverrideRefereeCommand()->value())
    {
        std::string previous_state_string =
            control_config->PreviousRefereeCommand()->value();
        std::string current_state_string =
            control_config->CurrentRefereeCommand()->value();
        try
        {
            RefereeCommand previous_state =
                fromStringToRefereeCommand(previous_state_string);
            current_game_state.updateRefereeCommand(previous_state);
            RefereeCommand current_state =
                fromStringToRefereeCommand(current_state_string);
            current_game_state.updateRefereeCommand(current_state);
        }
        catch (std::invalid_argument e)
        {
            LOG(WARNING) << e.what();
        }
    }
}

void STP::updateAIPlay(const World& world)
{
    bool play_overridden = overrideAIPlayIfApplicable();
    if (!play_overridden)
    {
        bool no_current_play = !current_play || current_play->done();
        if (no_current_play || !current_play->invariantHolds(world))
        {
            try
            {
                current_play = calculateNewPlay(world);
            }
            catch (const std::runtime_error& e)
            {
                auto default_play = default_play_constructor();
                LOG(WARNING) << "Unable to assign a new Play. No Plays are valid"
                             << std::endl;
                LOG(WARNING) << "Falling back to the default Play - "
                             << TYPENAME(*default_play) << std::endl;
                current_play = std::move(default_play);
            }
        }
    }
}

std::vector<std::unique_ptr<Intent>> STP::getIntentsFromCurrentPlay(const World& world)
{
    current_tactics = current_play->getTactics(world);

    std::vector<std::unique_ptr<Intent>> intents;
    assignRobotsToTactics(world, current_tactics);

    ActionWorldParamsUpdateVisitor action_world_params_update_visitor(world);
    TacticWorldParamsUpdateVisitor tactic_world_params_update_visitor(world);

    for (const std::shared_ptr<Tactic>& tactic : current_tactics)
    {
        tactic->accept(tactic_world_params_update_visitor);

        // Try to get an intent from the tactic
        std::shared_ptr<Action> action = tactic->getNextAction();
        std::unique_ptr<Intent> intent;
        if (action)
        {
            action->accept(action_world_params_update_visitor);
            intent = action->getNextIntent();
        }

        if (intent)
        {
            auto motion_constraints = motion_constraint_manager.getMotionConstraints(
                current_game_state, *tactic);
            intent->setMotionConstraints(motion_constraints);

            intents.emplace_back(std::move(intent));
        }
        else if (tactic->getAssignedRobot())
        {
            // If we couldn't get an intent, we send the robot a StopIntent so
            // it doesn't do anything crazy until it starts running a new Tactic
            intents.emplace_back(
                std::make_unique<StopIntent>(tactic->getAssignedRobot()->id(), false, 0));
        }
        else
        {
            LOG(WARNING) << "Tried to run a tactic that didn't yield an Intent "
                         << "and did not have a robot assigned!";
        }
    }

    return intents;
}

std::vector<std::unique_ptr<Intent>> STP::getIntents(const World& world)
{
    updateSTPState(world);
    return getIntentsFromCurrentPlay(world);
}

std::unique_ptr<Play> STP::calculateNewPlay(const World& world)
{
    std::vector<std::unique_ptr<Play>> applicable_plays;
    for (const auto& play_constructor :
         GenericFactory<std::string, Play>::getRegisteredConstructors())
    {
        auto play = play_constructor();
        if (play->isApplicable(world))
        {
            applicable_plays.emplace_back(std::move(play));
        }
    }

    if (applicable_plays.empty())
    {
        throw std::runtime_error(
            "No new Play could be calculated because no Plays are applicable");
    }

    // Create a uniform distribution over the indices of the applicable_plays
    // https://en.cppreference.com/w/cpp/numeric/random/uniform_int_distribution
    auto uniform_distribution = std::uniform_int_distribution<std::mt19937::result_type>(
        0, applicable_plays.size() - 1);
    auto play_index = uniform_distribution(random_number_generator);

    return std::move(applicable_plays[play_index]);
}

std::optional<std::string> STP::getCurrentPlayName() const
{
    if (current_play)
    {
        return std::make_optional(TYPENAME(*current_play));
    }

    return std::nullopt;
}

PlayInfo STP::getPlayInfo()
{
    PlayInfo info;
    std::string info_referee_command = toString(current_game_state.getRefereeCommand());
    std::string info_play_name = getCurrentPlayName() ? *getCurrentPlayName() : "No Play";
    std::unordered_set<std::string> info_robot_tactic_assignment = {};
    info = PlayInfo(info_referee_command, info_play_name, info_robot_tactic_assignment);

    // Sort the tactics by the id of the robot they are assigned to, so we can report
    // the tactics in order or robot id. This makes it much easier to read if tactics
    // or robots change, since the order of the robots won't change
    if (current_play)
    {
        auto compare_tactic_by_robot_id = [](auto t1, auto t2) {
            if (t1->getAssignedRobot() && t2->getAssignedRobot())
            {
                return t1->getAssignedRobot()->id() < t2->getAssignedRobot()->id();
            }
            else if (!t1->getAssignedRobot() && t2->getAssignedRobot())
            {
                return false;
            }
            else if (t1->getAssignedRobot() && !t2->getAssignedRobot())
            {
                return true;
            }
            else
            {
                return true;
            }
        };
        auto tactics = current_tactics;
        std::sort(tactics.begin(), tactics.end(), compare_tactic_by_robot_id);

        for (const auto& tactic : tactics)
        {
            auto robot = tactic->getAssignedRobot();
            if (!robot)
            {
                continue;
            }
            std::string s = "Robot " + std::to_string(tactic->getAssignedRobot()->id()) +
                            "  -  " + TYPENAME(*tactic);
            info.addRobotTacticAssignment(s);
        }
    }

    return info;
}

bool STP::overrideAIPlayIfApplicable()
{
    previous_override_play           = override_play;
    override_play                    = control_config->OverrideAIPlay()->value();
    bool override_play_value_changed = previous_override_play != override_play;

    previous_override_play_name = override_play_name;
    override_play_name          = control_config->CurrentAIPlay()->value();
    bool override_play_name_value_changed =
        previous_override_play_name != override_play_name;

    bool no_current_play = !current_play || current_play->done();

    if (override_play)
    {
        if (no_current_play || override_play_name_value_changed ||
            override_play_value_changed)
        {
            try
            {
                current_play =
                    GenericFactory<std::string, Play>::create(override_play_name);
            }
            catch (std::invalid_argument)
            {
                auto default_play = default_play_constructor();
                LOG(WARNING) << "Error: The Play \"" << override_play_name
                             << "\" specified in the override is not valid." << std::endl;
                LOG(WARNING) << "Falling back to the default Play - "
                             << TYPENAME(*default_play) << std::endl;
                current_play = std::move(default_play);
            }
            return true;
        }
    }
    return false;
}
