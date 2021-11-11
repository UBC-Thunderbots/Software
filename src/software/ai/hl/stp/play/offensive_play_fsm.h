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

// This callback is used to return an intent from the fsm
using SetTacticsCallback = std::function<void(TacticVector)>;

// The tactic update struct is used to update tactics and set the new intent
struct PlayUpdate
{
    PlayUpdate(const World &world, const SetTacticsCallback &set_tactics_fun)
        : world(world), set_tactics(set_tactics_fun)
    {
    }
    // updated world
    World world;
    // callback to return the next tactics
    SetTacticsCallback set_tactics;
};

/**
 * The Update struct is the only event that a tactic fsm should respond to and it is
 * composed of the following structs:
 *
 * ControlParams - uniquely defined by each tactic to control the FSM
 * TacticUpdate - common struct that contains World and SetTacticsCallback
 */
#define DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS                         \
    struct Update                                                                        \
    {                                                                                    \
        Update(const ControlParams &control_params, const PlayUpdate &common)            \
            : control_params(control_params), common(common)                             \
        {                                                                                \
        }                                                                                \
        ControlParams control_params;                                                    \
        PlayUpdate common;                                                               \
    };



struct OffensivePlayFSM
{
    class AttemptToShootWhileLookingForAPassState;
    class TakePassState;

    struct ControlParams
    {
        // number of additional offensive attacks on top of the minimum of 2 (passer and
        // receiver)
        unsigned int num_additional_offensive_tactics;
    };

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS


    explicit OffensivePlayFSM(
        std::shared_ptr<const PlayConfig> play_config,
        PassWithRating best_pass_and_score_so_far, Duration time_since_commit_stage_start,
        double min_pass_score_threshold, std::shared_ptr<AttackerTactic> attacker_tactic,
        std::shared_ptr<ReceiverTactic> receiver_tactic,
        std::vector<std::shared_ptr<MoveTactic>> aggressive_positioning_tactics,
        PassGenerator<EighteenZoneId> pass_generator)
        : play_config(play_config),
          best_pass_and_score_so_far(best_pass_and_score_so_far),
          time_since_commit_stage_start(time_since_commit_stage_start),
          min_pass_score_threshold(min_pass_score_threshold),
          attacker_tactic(attacker_tactic),
          receiver_tactic(receiver_tactic),
          aggressive_positioning_tactics(aggressive_positioning_tactics),
          pass_generator(pass_generator)
    {
    }

    auto operator()()
    {
        using namespace boost::sml;

        const auto look_for_pass_s = state<AttemptToShootWhileLookingForAPassState>;
        const auto take_pass_s     = state<TakePassState>;
        const auto update_e        = event<Update>;

        const auto look_for_pass = [this](auto event) {
            auto pitch_division = std::make_shared<const EighteenZonePitchDivision>(
                event.common.world.field());

            auto pass_eval    = pass_generator.generatePassEvaluation(event.common.world);
            auto ranked_zones = pass_eval.rankZonesForReceiving(
                event.common.world, event.common.world.ball().position());

            this->best_pass_and_score_so_far = pass_eval.getBestPassOnField();

            // These two tactics will set robots to roam around the field, trying tdouble
            // o put themselves into a good position to receive a pass
            std::vector<std::shared_ptr<MoveTactic>> offensive_positioning_tactics(
                event.control_params.num_additional_offensive_tactics);
            std::generate(offensive_positioning_tactics.begin(),
                          offensive_positioning_tactics.end(),
                          []() { return std::make_shared<MoveTactic>(false); });

            // Wait for a good pass by starting out only looking for "perfect" passes
            // (with a score of 1) and decreasing this threshold over time
            min_pass_score_threshold = 1.0;
            Timestamp pass_optimization_start_time =
                event.common.world.getMostRecentTimestamp();
            // This boolean indicates if we're ready to perform a pass
            double abs_min_pass_score =
                play_config->getShootOrPassPlayConfig()->getAbsMinPassScore()->value();
            double pass_score_ramp_down_duration = play_config->getShootOrPassPlayConfig()
                                                       ->getPassScoreRampDownDuration()
                                                       ->value();
            LOG(DEBUG) << "Best pass so far is: " << best_pass_and_score_so_far.pass;
            LOG(DEBUG) << "      with score of: " << best_pass_and_score_so_far.rating;

            pass_eval = pass_generator.generatePassEvaluation(event.common.world);
            best_pass_and_score_so_far = pass_eval.getBestPassOnField();

            for (unsigned int i = 0; i < offensive_positioning_tactics.size(); i++)
            {
                auto pass1 = pass_eval.getBestPassInZones({ranked_zones[i]}).pass;

                offensive_positioning_tactics[i]->updateControlParams(
                    pass1.receiverPoint(), pass1.receiverOrientation(), 0.0,
                    MaxAllowedSpeedMode::PHYSICAL_LIMIT);
            }

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
            TacticVector retval = {attacker_tactic};
            retval.insert(retval.end(), offensive_positioning_tactics.begin(),
                          offensive_positioning_tactics.end());
            event.common.set_tactics(retval);
        };

        const auto take_pass = [this](auto event) {
            // Commit to a pass
            Pass pass = best_pass_and_score_so_far.pass;

            LOG(DEBUG) << "Committing to pass: " << best_pass_and_score_so_far.pass;
            LOG(DEBUG) << "Score of pass we committed to: "
                       << best_pass_and_score_so_far.rating;

            auto pass_eval = pass_generator.generatePassEvaluation(event.common.world);

            auto ranked_zones = pass_eval.rankZonesForReceiving(
                event.common.world, best_pass_and_score_so_far.pass.receiverPoint());
            Zones cherry_pick_region_1 = {ranked_zones[0]};
            Zones cherry_pick_region_2 = {ranked_zones[1]};

            auto pass1 = pass_eval.getBestPassInZones(cherry_pick_region_1).pass;
            auto pass2 = pass_eval.getBestPassInZones(cherry_pick_region_2).pass;

            auto cherry_pick_tactic_1 = std::make_shared<MoveTactic>(false);
            auto cherry_pick_tactic_2 = std::make_shared<MoveTactic>(false);
            cherry_pick_tactic_1->updateControlParams(
                pass1.receiverPoint(), pass1.receiverOrientation(), 0.0,
                MaxAllowedSpeedMode::PHYSICAL_LIMIT);
            cherry_pick_tactic_2->updateControlParams(
                pass2.receiverPoint(), pass2.receiverOrientation(), 0.0,
                MaxAllowedSpeedMode::PHYSICAL_LIMIT);

            // if we make it here then we have committed to the pass
            attacker_tactic->updateControlParams(pass, true);
            receiver_tactic->updateControlParams(pass);

            if (!attacker_tactic->done())
            {
                event.common.set_tactics(
                    {attacker_tactic, receiver_tactic, cherry_pick_tactic_1});
            }
            else
            {
                event.common.set_tactics(
                    {receiver_tactic, cherry_pick_tactic_1, cherry_pick_tactic_2});
            }
        };

        const auto pass_found = [this](auto event) {
            return best_pass_and_score_so_far.rating > min_pass_score_threshold;
        };
        const auto should_abort   = [this](auto event) { return false; };
        const auto pass_completed = [this](auto event) {
            return receiver_tactic->done();
        };
        const auto took_shot = [this](auto event) { return false; };


        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *look_for_pass_s + update_e[pass_found] / take_pass     = take_pass_s,
            look_for_pass_s + update_e[!pass_found] / look_for_pass = look_for_pass_s,
            look_for_pass_s + update_e[took_shot]                   = X,
            take_pass_s + update_e[!pass_completed] / take_pass     = take_pass_s,
            take_pass_s + update_e[should_abort] / look_for_pass    = look_for_pass_s,
            take_pass_s + update_e[pass_completed] / take_pass      = X,
            X + update_e / look_for_pass                            = look_for_pass_s);
    }

   private:
    std::shared_ptr<const PlayConfig> play_config;
    PassWithRating best_pass_and_score_so_far;
    Duration time_since_commit_stage_start;
    double min_pass_score_threshold;
    std::shared_ptr<AttackerTactic> attacker_tactic;
    std::shared_ptr<ReceiverTactic> receiver_tactic;
    std::vector<std::shared_ptr<MoveTactic>> aggressive_positioning_tactics;
    PassGenerator<EighteenZoneId> pass_generator;
};
