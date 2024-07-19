#include "software/ai/hl/stp/play/dynamic_plays/offense_play.h"

#include "software/util/generic_factory/generic_factory.h"

OffensePlay::OffensePlay(std::shared_ptr<Strategy> strategy)
    : DynamicPlay(strategy,
                  {std::make_shared<TypedSupportTacticCandidate<ReceiverTactic>>()}),
      attacker_tactic_(std::make_shared<AttackerTactic>(strategy)),
      defense_play_(std::make_unique<DefensePlay>(strategy))
{
}

void OffensePlay::terminate(const WorldPtr& world_ptr)
{
    DynamicPlay::terminate(world_ptr);
    attacker_tactic_->terminate(world_ptr);
}

void OffensePlay::updateTactics(const PlayUpdate& play_update)
{
    PriorityTacticVector tactics;

    int num_defenders_and_supporters = static_cast<int>(play_update.num_tactics);

    // AttackerTactic should always be assigned
    if (play_update.num_tactics > 0)
    {
        const bool attacker_not_suspended =
            attacker_tactic_->tryResumingIfSuspended(play_update.world_ptr);

        // The AttackerTactic should only be returned if it has not suspended
        // execution of its current skill (it will not yield primitives if suspended)
        if (attacker_not_suspended)
        {
            tactics.push_back({attacker_tactic_});

            // We have one less defender/supporter to assign since we will assign one
            // robot to be the attacker
            --num_defenders_and_supporters;
        }

        // Log visualize the state of the attacker's current skill
        attacker_tactic_->visualizeSkillState(*play_update.world_ptr);
    }

    // Determine number of defense and support tactics to assign
    auto [num_defenders, num_supporters] =
        assignNumOfDefendersAndSupporters(std::max(num_defenders_and_supporters, 0));

    // Get defense tactics from DefensePlay
    std::vector<std::shared_ptr<Tactic>> defense_tactics;
    defense_play_->updateTactics(PlayUpdate(
        play_update.world_ptr, num_defenders,
        [&](PriorityTacticVector new_tactics) {
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
        std::optional<Point> pass_origin_override = std::nullopt;
        auto support_tactics_it                   = support_tactics_.begin();

        // Check if the attacker has committed to a pass
        if (skill_state.pass_committed && skill_state.pass)
        {
            // Get the receiver point of the committed pass
            Point receiving_position = skill_state.pass->receiverPoint();

            // Set the receiving position of the first support tactic to the
            // committed pass receiver point
            support_tactics_.front()->updateReceivingPosition(receiving_position);
            existing_receiving_positions.push_back(receiving_position);
            ++support_tactics_it;

            // Assume the ball will be received by the receiver.
            // Set the pass_origin_override passed to the ReceiverPositionGenerator
            // to the committed pass receiver point so that receiving positions generated
            // will be relative to where the ball is expected to be received.
            pass_origin_override = receiving_position;
        }

        unsigned int num_positions_to_generate = static_cast<unsigned int>(
            support_tactics_.size() - existing_receiving_positions.size());

        // Generate receiving positions for the remaining support tactics
        std::vector<Point> receiving_positions = strategy->getBestReceivingPositions(
            num_positions_to_generate, existing_receiving_positions,
            pass_origin_override);

        for (const Point& receiving_position : receiving_positions)
        {
            (*support_tactics_it)->updateReceivingPosition(receiving_position);
            ++support_tactics_it;
        }
    }

    // Add all defense and support tactics to the main PriorityTacticVector
    TacticVector secondary_tactics;
    secondary_tactics.insert(secondary_tactics.end(), support_tactics_.begin(),
                             support_tactics_.end());
    secondary_tactics.insert(secondary_tactics.end(), defense_tactics.begin(),
                             defense_tactics.end());
    tactics.push_back(secondary_tactics);

    play_update.set_tactics(tactics);
}

std::tuple<unsigned int, unsigned int> OffensePlay::assignNumOfDefendersAndSupporters(
    unsigned int num_tactics)
{
    unsigned int num_defenders, num_supporters;
    switch (num_tactics)
    {
        case 0:
            num_defenders  = 0;
            num_supporters = 0;
            break;
        case 1:
            num_defenders  = 0;
            num_supporters = 1;
            break;
        case 2:
            num_defenders  = 1;
            num_supporters = 1;
            break;
        case 3:
            num_defenders  = 1;
            num_supporters = 2;
            break;
        default:
            num_defenders  = 2;
            num_supporters = num_tactics - 2;
    }
    return {num_defenders, num_supporters};
}

static TGenericFactory<std::string, Play, OffensePlay, std::shared_ptr<Strategy>>
    offense_play_factory;
