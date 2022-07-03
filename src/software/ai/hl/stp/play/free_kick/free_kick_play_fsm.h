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

struct FreeKickPlayFSM
{
    class AlignToBallState;
    class ShootOrFindPassState;
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
    explicit FreeKickPlayFSM(TbotsProto::AiConfig ai_config);

    /**
     * Updates the offensive positioning tactics
     *
     *
     * @param the ranked zones to look for offensive positions in
     * @param pass_eval The pass evaluation to help find best passes
     * @param num_tactics the number of tactics to return
     */
    void updateOffensivePositioningTactics(std::vector<EighteenZoneId> ranked_zones,
                                           PassEvaluation<EighteenZoneId> pass_eval,
                                           unsigned int num_tactics);

    /**
     * TODO: javadocs
     */
    void shootOrFindPass(const Update& event);
    void alignToBall(const Update& event);
    void takePass(const Update& event);
    bool freeKickerReady(const Update& event);
    bool passFound(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(ShootOrFindPassState)
        DEFINE_SML_STATE(TakePassState)
        DEFINE_SML_STATE(StartState)
        DEFINE_SML_STATE(AlignToBallState)
        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(shootOrFindPass)
        DEFINE_SML_ACTION(alignToBall)
        DEFINE_SML_ACTION(takePass)

        DEFINE_SML_GUARD(freeKickerReady)
        DEFINE_SML_GUARD(passFound)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *StartState_S + Update_E / alignToBall_A = AlignToBallState_S,
            AlignToBallState_S + Update_E[freeKickerReady_G] / shootOrFindPass_A =
                ShootOrFindPassState_S,
            AlignToBallState_S + Update_E[!freeKickerReady_G] / alignToBall_A =
                AlignToBallState_S,
            ShootOrFindPassState_S + Update_E[!passFound_G] / shootOrFindPass_A =
                ShootOrFindPassState_S,
            ShootOrFindPassState_S + Update_E[passFound_G] / takePass_A = TakePassState_S,
            X + Update_E = X);
    }

   private:
    /**
     * Update the tactic that aligns the robot to the ball in preparation to pass
     *
     * @param align_to_ball_tactic
     * @param world The current state of the world
     */
    void updateAlignToBallTactic(std::shared_ptr<MoveTactic> align_to_ball_tactic,
                                 const World& world);

    TbotsProto::AiConfig ai_config;
    std::shared_ptr<AttackerTactic> attacker_tactic;
    std::shared_ptr<ReceiverTactic> receiver_tactic;
    std::shared_ptr<MoveTactic> align_to_ball_tactic;
    std::vector<std::shared_ptr<MoveTactic>> offensive_positioning_tactics;
    PassGenerator<EighteenZoneId> pass_generator;
    Timestamp pass_optimization_start_time;
    PassWithRating best_pass_and_score_so_far;
    Duration time_since_commit_stage_start;
    double min_pass_score_threshold;
    Pass pass_in_progress;

    // The maximum time that we will wait before committing to a pass
    const Duration MAX_TIME_TO_COMMIT_TO_PASS;
};
