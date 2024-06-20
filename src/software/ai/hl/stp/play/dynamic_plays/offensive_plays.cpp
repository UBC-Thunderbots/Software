#include "software/ai/hl/stp/play/dynamic_plays/offensive_plays.h"

#include "software/util/generic_factory/generic_factory.h"

OffensivePlay::OffensivePlay(std::shared_ptr<Strategy> strategy,
                             std::unique_ptr<FeasibilityScorer> feasibility_scorer,
                             std::shared_ptr<AttackerTactic> attacker_tactic)
    : DynamicPlay(strategy, std::move(feasibility_scorer)),
      attacker_tactic_(attacker_tactic),
      defense_play_(std::make_unique<DefensePlay>(strategy))
{
}

void OffensivePlay::terminate(const WorldPtr& world_ptr)
{
    DynamicPlay::terminate(world_ptr);
    attacker_tactic_->terminate(world_ptr);
}

void OffensivePlay::updateTactics(const PlayUpdate& play_update)
{
    PriorityTacticVector tactics;

    const bool is_attacker_suspended =
        attacker_tactic_->tryResumingIfSuspended(play_update.world_ptr);

    // The AttackerTactic should only be returned if it has not suspended
    // execution of its current skill (it will not yield primitives if suspended)
    if (!is_attacker_suspended)
    {
        tactics.push_back({attacker_tactic_});
    }

    // Log visualize the state of the attacker's current skill
    attacker_tactic_->visualizeSkillState(*play_update.world_ptr);

    int num_attackers  = static_cast<unsigned int>(tactics.size());
    int num_defenders  = std::min(play_update.num_tactics - num_attackers, 2u);
    int num_supporters = play_update.num_tactics - num_attackers - num_defenders;

    // Get defense tactics from DefensePlay
    std::vector<std::shared_ptr<Tactic>> defense_tactics;
    defense_play_->updateTactics(PlayUpdate(
        play_update.world_ptr, num_defenders,
        [&](PriorityTacticVector new_tactics)
        {
            for (auto& tactic_vec : new_tactics)
            {
                defense_tactics.insert(defense_tactics.end(), tactic_vec.begin(),
                                       tactic_vec.end());
            }
        },
        play_update.inter_play_communication,
        play_update.set_inter_play_communication_fun));

    // Create more support tactics if necessary
    // (so that `support_tactics_` has `num_supporters` in it)
    updateSupportTactics(num_supporters);

    if (!support_tactics_.empty())
    {
        SkillState skill_state = attacker_tactic_->getSkillState();

        std::vector<Point> existing_receiving_positions;
        auto support_tactics_it = support_tactics_.begin();

        // If the attacker has committed to a pass, set the receiving position
        // of the first support tactic to the receiver point of the pass
        if (skill_state.pass_committed && skill_state.pass)
        {
            Point receiving_position = skill_state.pass->receiverPoint();
            support_tactics_.front()->updateReceivingPosition(receiving_position);
            existing_receiving_positions.push_back(receiving_position);
            ++support_tactics_it;
        }

        unsigned int num_positions_to_generate = static_cast<unsigned int>(
            support_tactics_.size() - existing_receiving_positions.size());

        // Generate receiving positions for the remaining support tactics
        std::vector<Point> receiving_positions = strategy->getBestReceivingPositions(
            num_positions_to_generate, existing_receiving_positions);

        for (const Point& receiving_position : receiving_positions)
        {
            (*support_tactics_it)->updateReceivingPosition(receiving_position);
            ++support_tactics_it;
        }
    }

    // Add all defense and support tactics to the main PriorityTacticVector
    TacticVector secondary_tactics;
    secondary_tactics.insert(secondary_tactics.end(), defense_tactics.begin(),
                             defense_tactics.end());
    secondary_tactics.insert(secondary_tactics.end(), support_tactics_.begin(),
                             support_tactics_.end());
    tactics.push_back(secondary_tactics);

    play_update.set_tactics(tactics);
}

static TGenericFactory<std::string, Play, OffensiveFriendlyThirdPlay,
                       std::shared_ptr<Strategy>, std::shared_ptr<AttackerTactic>>
    offensive_friendly_third_play_factory;

static TGenericFactory<std::string, Play, OffensiveMiddleThirdPlay,
                       std::shared_ptr<Strategy>, std::shared_ptr<AttackerTactic>>
    offensive_middle_third_play_factory;

static TGenericFactory<std::string, Play, OffensiveEnemyThirdPlay,
                       std::shared_ptr<Strategy>, std::shared_ptr<AttackerTactic>>
    offensive_enemy_third_play_factory;
