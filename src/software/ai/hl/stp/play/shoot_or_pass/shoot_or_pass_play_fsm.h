#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/pass_generator.hpp"
#include "software/logger/logger.h"

using Zones = std::unordered_set<EighteenZoneId>;

struct ShootOrPassPlayFSM
{
    class AttemptShotState;
    class TakePassState;
    class StartState;

    struct ControlParams
    {
    };

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS


    /**
     * Creates a shoot or pass play FSM
     *
     * @param ai_config the play config for this play FSM
     */
    explicit ShootOrPassPlayFSM(TbotsProto::AiConfig ai_config);

    /**
     * Updates the offensive positioning tactics
     *
     *
     * @param the ranked zones to look for offensive positions in
     * @param pass_eval The pass evaluation to help find best passes
     * @param num_tactics the number of tactics to return
     * @param current_offensive_positioning_tactics The current offensive positioning tactics
     *
     * @return the updated offensive positioning tactics
     */
    static std::vector<std::shared_ptr<MoveTactic>>  updateOffensivePositioningTactics(std::vector<EighteenZoneId> ranked_zones,
                                           PassEvaluation<EighteenZoneId> pass_eval,
                                           unsigned int num_tactics, std::vector<std::shared_ptr<MoveTactic>> current_offensive_positioning_tactics);

    /**
     * Action that looks for a pass
     *
     * @param event the ShootOrPassPlayFSM Update event
     */
    void lookForPass(const Update& event);

    /**
     * Action to restart looking for a pass
     *
     * @param event the ShootOrPassPlayFSM Update event
     */
    void startLookingForPass(const Update& event);

    /**
     * Action to take a pass
     *
     * @param event the ShootOrPassPlayFSM Update event
     */
    void takePass(const Update& event);

    /**
     * Guard to check if a pass has been found
     *
     * @param event the ShootOrPassPlayFSM Update event
     *
     * @return whether a pass has been found
     */
    bool passFound(const Update& event);

    /**
     * Guard on whether to abort the pass
     *
     * @param event the ShootOrPassPlayFSM Update event
     *
     * @return whether the pass should be aborted
     */
    bool shouldAbortPass(const Update& event);

    /**
     * Guard on whether the pass has completed
     *
     * @param event the ShootOrPassPlayFSM Update event
     *
     * @return whether the pass has completed
     */
    bool passCompleted(const Update& event);

    /**
     * Guard on whether a shot has been taken
     *
     * @param event the ShootOrPassPlayFSM Update event
     *
     * @return whether the shot has been taken
     */
    bool tookShot(const Update& event);

    bool hasPassInProgress(const Update& event);

    void maintainPassInProgress(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(AttemptShotState)
        DEFINE_SML_STATE(TakePassState)
        DEFINE_SML_STATE(StartState)
        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(lookForPass)
        DEFINE_SML_ACTION(startLookingForPass)
        DEFINE_SML_ACTION(takePass)
        DEFINE_SML_ACTION(maintainPassInProgress)

        DEFINE_SML_GUARD(passFound)
        DEFINE_SML_GUARD(shouldAbortPass)
        DEFINE_SML_GUARD(passCompleted)
        DEFINE_SML_GUARD(tookShot)
        DEFINE_SML_GUARD(hasPassInProgress)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *StartState_S + Update_E[hasPassInProgress_G] / maintainPassInProgress_A =
                TakePassState_S,
            StartState_S + Update_E / startLookingForPass_A         = AttemptShotState_S,
            AttemptShotState_S + Update_E[passFound_G] / takePass_A = TakePassState_S,
            AttemptShotState_S + Update_E[!passFound_G] / lookForPass_A =
                AttemptShotState_S,
            AttemptShotState_S + Update_E[tookShot_G]                 = X,
            TakePassState_S + Update_E[!passCompleted_G] / takePass_A = TakePassState_S,
            TakePassState_S + Update_E[shouldAbortPass_G] / startLookingForPass_A =
                AttemptShotState_S,
            TakePassState_S + Update_E[passCompleted_G] / takePass_A = X,
            X + Update_E / startLookingForPass_A = AttemptShotState_S);
    }

   private:
    TbotsProto::AiConfig ai_config;
    std::shared_ptr<AttackerTactic> attacker_tactic;
    std::shared_ptr<ReceiverTactic> receiver_tactic;
    std::vector<std::shared_ptr<MoveTactic>> offensive_positioning_tactics;
    PassGenerator<EighteenZoneId> pass_generator;
    Timestamp pass_optimization_start_time;
    PassWithRating best_pass_and_score_so_far;
    Duration time_since_commit_stage_start;
    double min_pass_score_threshold;
    Pass pass_in_progress;
};
