#include "software/ai/hl/stp/play/dynamic_plays/dynamic_play.h"

#include "software/util/generic_factory/generic_factory.h"

DynamicPlay::DynamicPlay(std::shared_ptr<Strategy> strategy)
    : Play(true, strategy),
      support_tactic_candidates_(allSupportTacticCandidates()),
      support_tactic_feasibility_scorer_(std::make_unique<FeasibilityScorer>()),
      support_tactic_duplication_scorer_(std::make_unique<DuplicationScorer>()),
      support_tactic_success_scorer_(std::make_unique<SuccessScorer>()),
      attacker_tactic_(std::make_shared<AttackerTactic>(strategy)),
      support_tactics_(),
      defense_play(std::make_unique<DefensePlay>(strategy))
{
}

void DynamicPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                 const WorldPtr &world_ptr)
{
    // This function doesn't get called, it should be removed once coroutines
    // are phased out
    while (true)
    {
        yield({{}});
    }
}

void DynamicPlay::updateTactics(const PlayUpdate &play_update)
{
    PriorityTacticVector tactics_to_return;

    PossessionStrategy possession_strategy =
        (*strategy)->getPossessionStrategy(play_update.num_tactics);

    // Defense
    defense_play->updateControlParams(TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
    defense_play->updateTactics(PlayUpdate(
        play_update.world_ptr, possession_strategy.num_defenders,
        [&tactics_to_return](PriorityTacticVector new_tactics) {
            for (const auto &tactic_vector : new_tactics)
            {
                tactics_to_return.push_back(tactic_vector);
            }
        },
        play_update.inter_play_communication,
        play_update.set_inter_play_communication_fun));

    if (attacker_tactic_->done())
    {
        support_tactics_.clear();

        unsigned int num_support_tactics = possession_strategy.num_support;
        while (num_support_tactics > support_tactics_.size())
        {
            for (auto &candidate : support_tactic_candidates_)
            {
                candidate->clearScores();
                candidate->score(*support_tactic_feasibility_scorer_);
                candidate->score(*support_tactic_duplication_scorer_);
                candidate->score(*support_tactic_success_scorer_);
            }

            auto best_candidate = std::max_element(support_tactic_candidates_.begin(),
                                                   support_tactic_candidates_.end());

            std::shared_ptr<OffenseSupportTactic> support_tactic =
                (*best_candidate)->createSupportTactic();
            support_tactics_.push_back(support_tactic);

            (*best_candidate)->updateScorer(*support_tactic_duplication_scorer_);
        }

        std::vector<OffenseSupportType> committed_support;
        std::transform(support_tactics_.begin(), support_tactics_.end(),
                       std::back_inserter(committed_support),
                       [](const std::shared_ptr<OffenseSupportTactic> &tactic) {
                           return tactic->getOffenseSupportType();
                       });
    }

    tactics_to_return.push_back({attacker_tactic_});

    std::vector support_tactic_vec = std::vector<std::shared_ptr<Tactic>>();
    std::transform(support_tactics_.begin(), support_tactics_.end(),
                   std::back_inserter(support_tactic_vec),
                   [](const std::shared_ptr<OffenseSupportTactic> &tactic) {
                       std::shared_ptr<Tactic> t = tactic;
                       return t;
                   });
    tactics_to_return.push_back(support_tactic_vec);

    play_update.set_tactics(tactics_to_return);
}

std::vector<std::shared_ptr<SupportTacticCandidate>> allSupportTacticCandidates()
{
    return {std::make_shared<TypedSupportTacticCandidate<ReceiverTactic>>()};
}

// Temp: Migrate to OffensePlay
static TGenericFactory<std::string, Play, DynamicPlay, std::shared_ptr<Strategy>> factory;
