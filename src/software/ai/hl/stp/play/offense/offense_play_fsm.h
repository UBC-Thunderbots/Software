#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/pass_generator.h"
#include "software/ai/passing/receiver_position_generator.hpp"
#include "software/logger/logger.h"

struct OffensePlayFSM
{
    class AttackState;
    class PassState;
    class ReceiveState;

    struct ControlParams
    {
    };

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    struct Terminate
    {
        WorldPtr world_ptr;
    };

    /**
     * Creates an OffensePlayFSM
     *
     * @param ai_config the AI configuration
     */
    explicit OffensePlayFSM(TbotsProto::AiConfig ai_config);

    /**
     * Guard to check whether the AttackerTactic has completed executing
     * its current skill.
     *
     * @param event the OffensePlayFSM Update event
     *
     * @return whether the AttackerTactic is done executing its current skill
     */
    bool attackerDone(const Update& event) const;

    /**
     * Guard to check whether the AttackerTactic is currently executing
     * a passing skill.
     *
     * @param event the OffensePlayFSM Update event
     *
     * @return whether the AttackerTactic is currently executing a passing skill
     */
    bool attackerPassing(const Update& event) const;

    /**
     * Guard to check whether to abort the current pass (if one is in progress).
     *
     * @param event the OffensePlayFSM Update event
     *
     * @return whether to abort the current pass
     */
    bool shouldAbortPass(const Update& event);

    /**
     * Guard to check whether the ReceiverTactic has completed receiving the
     * current pass (if one is in progress).
     *
     * @param event the OffensePlayFSM Update event
     *
     * @return whether the ReceiverTactic completed receiving the current pass
     */
    bool passReceived(const Update& event) const;

    /**
     * Action to update and assign an AttackerTactic, ReceiverTactic, and MoveTactics
     * (for offensive positioning).
     *
     * @param event the OffensePlayFSM Update event
     */
    void attack(const Update& event);

    /**
     * Action to update and assign a ReceiverTactic and MoveTactics (for offensive
     * positioning).
     *
     * @param event the OffensePlayFSM Update event
     */
    void receive(const Update& event);

    /**
     * Resets the AttackerTactic and ReceiverTactic.
     * The AttackerTactic will select a new skill to execute.
     *
     * @param event the OffensePlayFSM Update event
     */
    void resetTactics(const Update& event);

    /**
     * Terminates the play, ending the current episode.
     *
     * @param event the OffensePlayFSM Terminate event
     */
    void terminate(const Terminate& event);

    /**
     * Updates the MoveTactics that will move to offensive receiver positions.
     *
     * @param world the current world
     * @param num_tactics the number of tactics to assign
     * @param existing_receiver_positions a set of positions of existing receiver
     * positions that should be taken into account when assigning additional
     * offensive tactics
     * @param pass_origin_override an optional point that the pass origin should be
     * overridden to
     */
    void updateOffensivePositioningTactics(
        const WorldPtr world, unsigned int num_tactics,
        const std::vector<Point>& existing_receiver_positions = {},
        const std::optional<Point>& pass_origin_override      = std::nullopt);


    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(AttackState)
        DEFINE_SML_STATE(PassState)
        DEFINE_SML_STATE(ReceiveState)

        DEFINE_SML_EVENT(Update)
        DEFINE_SML_EVENT(Terminate)

        DEFINE_SML_GUARD(attackerDone)
        DEFINE_SML_GUARD(attackerPassing)
        DEFINE_SML_GUARD(shouldAbortPass)
        DEFINE_SML_GUARD(passReceived)

        DEFINE_SML_ACTION(attack)
        DEFINE_SML_ACTION(receive)
        DEFINE_SML_ACTION(resetTactics)
        DEFINE_SML_ACTION(terminate)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *AttackState_S + Update_E[attackerPassing_G] / attack_A = PassState_S,
            AttackState_S + Update_E[attackerDone_G] / (resetTactics_A, attack_A),
            AttackState_S + Update_E / attack_A,
            AttackState_S + Terminate_E / terminate_A,

            PassState_S + Update_E[attackerDone_G] / receive_A = ReceiveState_S,
            PassState_S + Update_E[shouldAbortPass_G] / (resetTactics_A, attack_A) =
                AttackState_S,
            PassState_S + Update_E / attack_A,
            PassState_S + Terminate_E / terminate_A = AttackState_S,

            ReceiveState_S + Update_E[shouldAbortPass_G || passReceived_G] /
                                 (resetTactics_A, attack_A) = AttackState_S,
            ReceiveState_S + Update_E / receive_A,
            ReceiveState_S + Terminate_E / terminate_A = AttackState_S,

            X + Update_E = X);
    }

   private:
    TbotsProto::AiConfig ai_config_;

    std::shared_ptr<AttackerTactic> attacker_tactic_;
    std::shared_ptr<ReceiverTactic> receiver_tactic_;
    std::vector<std::shared_ptr<MoveTactic>> offensive_positioning_tactics_;

    ReceiverPositionGenerator<EighteenZoneId> receiver_position_generator_;
    PassGenerator pass_generator_;
    PassWithRating best_pass_with_rating_;
};
