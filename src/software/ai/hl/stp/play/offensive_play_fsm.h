#pragma once

#include <include/boost/sml.hpp>

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/play/offensive_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/receiver_tactic.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/pass_generator.h"
#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"

using Zones = std::unordered_set<EighteenZoneId>;

struct OffensivePlayFSM
{
    class AttemptToShootWhileLookingForAPassState;
    class TakePassState;
    class StartState;

    struct ControlParams
    {
        // number of additional offensive attacks on top of the minimum of 2 (passer and
        // receiver)
        unsigned int num_additional_offensive_tactics;
    };

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS


    explicit OffensivePlayFSM(std::shared_ptr<const PlayConfig> play_config)
        : play_config(play_config),
          attacker_tactic(
              std::make_shared<AttackerTactic>(play_config->getAttackerTacticConfig())),
          receiver_tactic(std::make_shared<ReceiverTactic>(
              Field::createSSLDivisionBField(), Team(), Team(), Pass(Point(), Point(), 0),
              Ball(Point(), Vector(), Timestamp::fromSeconds(0)), false)),
          aggressive_positioning_tactics(std::vector<std::shared_ptr<MoveTactic>>()),
          pass_generator(PassGenerator<EighteenZoneId>(
              std::make_shared<const EighteenZonePitchDivision>(
                  Field::createSSLDivisionBField()),
              play_config->getPassingConfig())),
          pass_optimization_start_time(Timestamp::fromSeconds(0)),
          best_pass_and_score_so_far(
              PassWithRating{.pass = Pass(Point(), Point(), 0), .rating = 0}),
          time_since_commit_stage_start(Duration::fromSeconds(0)),
          min_pass_score_threshold(0)
    {
    }


    static std::vector<std::shared_ptr<MoveTactic>> getOffensivePositioningTactics(
        std::vector<EighteenZoneId> ranked_zones,
        PassEvaluation<EighteenZoneId> pass_eval, unsigned int num_tactics)
    {
        // These two tactics will set robots to roam around the field, trying to put
        // themselves into a good position to receive a pass
        std::vector<std::shared_ptr<MoveTactic>> offensive_positioning_tactics(
            num_tactics);
        std::generate(offensive_positioning_tactics.begin(),
                      offensive_positioning_tactics.end(),
                      []() { return std::make_shared<MoveTactic>(false); });

        for (unsigned int i = 0; i < offensive_positioning_tactics.size(); i++)
        {
            auto pass1 = pass_eval.getBestPassInZones({ranked_zones[i]}).pass;

            offensive_positioning_tactics[i]->updateControlParams(
                pass1.receiverPoint(), pass1.receiverOrientation(), 0.0,
                MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        }
        return offensive_positioning_tactics;
    }

    auto operator()()
    {
        using namespace boost::sml;

        const auto look_for_pass_s = state<AttemptToShootWhileLookingForAPassState>;
        const auto take_pass_s     = state<TakePassState>;
        const auto start_state_s   = state<StartState>;
        const auto update_e        = event<Update>;

        const auto look_for_pass = [this](auto event) {
            auto pitch_division = std::make_shared<const EighteenZonePitchDivision>(
                event.common.world.field());

            auto pass_eval    = pass_generator.generatePassEvaluation(event.common.world);
            auto ranked_zones = pass_eval.rankZonesForReceiving(
                event.common.world, event.common.world.ball().position());

            best_pass_and_score_so_far = pass_eval.getBestPassOnField();


            // Wait for a good pass by starting out only looking for "perfect" passes
            // (with a score of 1) and decreasing this threshold over time
            // This boolean indicates if we're ready to perform a pass
            double abs_min_pass_score =
                play_config->getShootOrPassPlayConfig()->getAbsMinPassScore()->value();
            double pass_score_ramp_down_duration = play_config->getShootOrPassPlayConfig()
                                                       ->getPassScoreRampDownDuration()
                                                       ->value();
            pass_eval = pass_generator.generatePassEvaluation(event.common.world);
            best_pass_and_score_so_far = pass_eval.getBestPassOnField();

            auto offensive_positioning_tactics = getOffensivePositioningTactics(
                ranked_zones, pass_eval,
                event.control_params.num_additional_offensive_tactics + 1);

            // update the best pass in the attacker tactic
            attacker_tactic->updateControlParams(best_pass_and_score_so_far.pass, false);

            // If we've assigned a robot as the passer in the PassGenerator, we
            // lower our threshold based on how long the PassGenerator as been
            // running since we set it
            time_since_commit_stage_start = event.common.world.getMostRecentTimestamp() -
                                            pass_optimization_start_time;
            min_pass_score_threshold =
                1 - std::min(time_since_commit_stage_start.toSeconds() /
                                 pass_score_ramp_down_duration,
                             1.0 - abs_min_pass_score);
            PriorityTacticVector ret_tactics = {{attacker_tactic}, {}};
            ret_tactics[1].insert(ret_tactics[1].end(),
                                  offensive_positioning_tactics.begin(),
                                  offensive_positioning_tactics.end());
            event.common.set_tactics(ret_tactics);
        };

        const auto start_looking_for_pass = [this, look_for_pass](auto event) {
            Timestamp pass_optimization_start_time =
                event.common.world.getMostRecentTimestamp();
            look_for_pass(event);
        };

        const auto take_pass = [this](auto event) {
            // Commit to a pass
            LOG(DEBUG) << "Committing to pass: " << best_pass_and_score_so_far.pass;
            LOG(DEBUG) << "Score of pass we committed to: "
                       << best_pass_and_score_so_far.rating;

            auto pass_eval = pass_generator.generatePassEvaluation(event.common.world);

            auto ranked_zones = pass_eval.rankZonesForReceiving(
                event.common.world, best_pass_and_score_so_far.pass.receiverPoint());

            // if we make it here then we have committed to the pass
            attacker_tactic->updateControlParams(best_pass_and_score_so_far.pass, true);
            receiver_tactic->updateControlParams(best_pass_and_score_so_far.pass);

            if (!attacker_tactic->done())
            {
                auto offensive_positioning_tactics = getOffensivePositioningTactics(
                    ranked_zones, pass_eval,
                    event.control_params.num_additional_offensive_tactics);
                PriorityTacticVector ret_tactics = {{attacker_tactic, receiver_tactic},
                                                    {}};
                ret_tactics[1].insert(ret_tactics[1].end(),
                                      offensive_positioning_tactics.begin(),
                                      offensive_positioning_tactics.end());

                event.common.set_tactics(ret_tactics);
            }
            else
            {
                auto offensive_positioning_tactics = getOffensivePositioningTactics(
                    ranked_zones, pass_eval,
                    event.control_params.num_additional_offensive_tactics + 1);
                PriorityTacticVector ret_tactics = {{receiver_tactic}, {}};
                ret_tactics[1].insert(ret_tactics[1].end(),
                                      offensive_positioning_tactics.begin(),
                                      offensive_positioning_tactics.end());

                event.common.set_tactics(ret_tactics);
            }
        };

        const auto pass_found = [this](auto event) {
            return best_pass_and_score_so_far.rating > min_pass_score_threshold;
        };
        const auto should_abort   = [this](auto event) { return false; };
        const auto pass_completed = [this](auto event) {
            return attacker_tactic->done() && receiver_tactic->done();
        };
        const auto took_shot = [this](auto event) { return false; };


        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *start_state_s + update_e / start_looking_for_pass      = look_for_pass_s,
            look_for_pass_s + update_e[pass_found] / take_pass      = take_pass_s,
            look_for_pass_s + update_e[!pass_found] / look_for_pass = look_for_pass_s,
            look_for_pass_s + update_e[took_shot]                   = X,
            take_pass_s + update_e[!pass_completed] / take_pass     = take_pass_s,
            take_pass_s + update_e[should_abort] / start_looking_for_pass =
                look_for_pass_s,
            take_pass_s + update_e[pass_completed] / take_pass = X);
    }

   private:
    std::shared_ptr<const PlayConfig> play_config;
    std::shared_ptr<AttackerTactic> attacker_tactic;
    std::shared_ptr<ReceiverTactic> receiver_tactic;
    std::vector<std::shared_ptr<MoveTactic>> aggressive_positioning_tactics;
    PassGenerator<EighteenZoneId> pass_generator;
    Timestamp pass_optimization_start_time;
    PassWithRating best_pass_and_score_so_far;
    Duration time_since_commit_stage_start;
    double min_pass_score_threshold;
};
