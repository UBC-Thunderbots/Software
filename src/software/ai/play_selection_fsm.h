#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/dynamic_plays/offense_play.h"

struct PlaySelectionFSM
{
    class HaltState;
    class StopState;
    class SetPlayState;
    class OffensePlayState;
    class DefensePlayState;

    struct Update
    {
        Update(const std::function<void(std::shared_ptr<Play>)>& set_current_play,
               const WorldPtr& world_ptr)
            : set_current_play(set_current_play), world_ptr(world_ptr)
        {
        }
        std::function<void(std::shared_ptr<Play>)> set_current_play;
        WorldPtr world_ptr;
    };

    /**
     * Creates a play selection FSM
     *
     * @param ai_config the default play config for this play fsm
     */
    explicit PlaySelectionFSM(std::shared_ptr<Strategy> strategy);

    /**
     * Guards for whether the game state is stopped, halted, playing, or in set up
     *
     * @param event The PlaySelection::Update event
     *
     * @return whether the gamestate is stopped, halted, playing, or in set up
     */
    bool gameStateStopped(const Update& event);
    bool gameStateHalted(const Update& event);
    bool gameStatePlaying(const Update& event);
    bool gameStateSetupRestart(const Update& event);

    /**
     * Guard to check whether the enemy team has possession of the ball
     *
     * @param event The PlaySelection::Update event
     *
     * @return whether the enemy team has possession of the ball
     */
    bool enemyHasPossession(const Update& event);

    /**
     * Action to set up the OverridePlay, SetPlay, StopPlay, HaltPlay,
     * OffensePlay, or DefensePlay
     *
     * @param event The PlaySelection::Update event
     */
    void setupSetPlay(const Update& event);
    void setupStopPlay(const Update& event);
    void setupHaltPlay(const Update& event);
    void setupOffensePlay(const Update& event);
    void setupDefensePlay(const Update& event);

    /**
     * Action to reset the current SetPlay to none
     *
     * @param event The PlaySelection::Update event
     */
    void resetSetPlay(const Update& event);

    /**
     * Action to terminate the OffensePlay
     *
     * @param event The PlaySelection::Update event
     */
    void terminateOffensePlay(const Update& event);

    /**
     * Guard to determine if the ball is in a stagnant state
     *
     * @param event The PlaySelection::Update event
     */
    bool ballIsStagnant(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(HaltState)
        DEFINE_SML_STATE(StopState)
        DEFINE_SML_STATE(SetPlayState)
        DEFINE_SML_STATE(OffensePlayState)
        DEFINE_SML_STATE(DefensePlayState)

        DEFINE_SML_GUARD(gameStateStopped)
        DEFINE_SML_GUARD(gameStateHalted)
        DEFINE_SML_GUARD(gameStatePlaying)
        DEFINE_SML_GUARD(gameStateSetupRestart)
        DEFINE_SML_GUARD(enemyHasPossession)
        DEFINE_SML_GUARD(ballIsStagnant)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(setupSetPlay)
        DEFINE_SML_ACTION(setupStopPlay)
        DEFINE_SML_ACTION(setupHaltPlay)
        DEFINE_SML_ACTION(setupOffensePlay)
        DEFINE_SML_ACTION(setupDefensePlay)
        DEFINE_SML_ACTION(resetSetPlay)
        DEFINE_SML_ACTION(terminateOffensePlay)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state

            *HaltState_S + Update_E[gameStateStopped_G] / setupStopPlay_A = StopState_S,
            HaltState_S + Update_E[gameStatePlaying_G && !enemyHasPossession_G] /
                              setupOffensePlay_A = OffensePlayState_S,
            HaltState_S + Update_E[gameStatePlaying_G && enemyHasPossession_G] /
                              setupDefensePlay_A = DefensePlayState_S,
            HaltState_S + Update_E[gameStateSetupRestart_G] / setupSetPlay_A =
                SetPlayState_S,

            StopState_S + Update_E[gameStateHalted_G] / setupHaltPlay_A = HaltState_S,
            StopState_S + Update_E[gameStatePlaying_G && !enemyHasPossession_G] /
                              setupOffensePlay_A = OffensePlayState_S,
            StopState_S + Update_E[gameStatePlaying_G && enemyHasPossession_G] /
                              setupDefensePlay_A = DefensePlayState_S,
            StopState_S + Update_E[gameStateSetupRestart_G] / setupSetPlay_A =
                SetPlayState_S,

            OffensePlayState_S +
                Update_E[gameStateHalted_G] / (terminateOffensePlay_A, setupHaltPlay_A) =
                HaltState_S,
            OffensePlayState_S +
                Update_E[gameStateStopped_G] / (terminateOffensePlay_A, setupStopPlay_A) =
                StopState_S,
            OffensePlayState_S + Update_E[gameStateSetupRestart_G] /
                                     (terminateOffensePlay_A, setupSetPlay_A) =
                SetPlayState_S,
            OffensePlayState_S + Update_E[enemyHasPossession_G && !ballIsStagnant_G] /
                                     (terminateOffensePlay_A, setupDefensePlay_A) =
                DefensePlayState_S,

            DefensePlayState_S + Update_E[gameStateHalted_G] / setupHaltPlay_A =
                HaltState_S,
            DefensePlayState_S + Update_E[gameStateStopped_G] / setupStopPlay_A =
                StopState_S,
            DefensePlayState_S + Update_E[gameStateSetupRestart_G] / setupSetPlay_A =
                SetPlayState_S,
            DefensePlayState_S + Update_E[!enemyHasPossession_G] / setupOffensePlay_A =
                OffensePlayState_S,
            DefensePlayState_S + Update_E[ballIsStagnant_G] / setupOffensePlay_A =
                OffensePlayState_S,

            SetPlayState_S + Update_E[gameStateHalted_G] /
                                 (resetSetPlay_A, setupHaltPlay_A) = HaltState_S,
            SetPlayState_S + Update_E[gameStateStopped_G] /
                                 (resetSetPlay_A, setupStopPlay_A) = StopState_S,
            SetPlayState_S + Update_E[gameStatePlaying_G && !enemyHasPossession_G] /
                                 (resetSetPlay_A, setupOffensePlay_A) =
                OffensePlayState_S,
            SetPlayState_S + Update_E[gameStatePlaying_G && enemyHasPossession_G] /
                                 (resetSetPlay_A, setupDefensePlay_A) =
                DefensePlayState_S,
            SetPlayState_S + Update_E[gameStateSetupRestart_G] / setupSetPlay_A,

            X + Update_E = X);
    }

   private:
    std::shared_ptr<Strategy> strategy_;
    std::shared_ptr<OffensePlay> offense_play_;
    std::optional<TbotsProto::PlayName> current_set_play_;
    Point enemy_possession_ball_position;
    double enemy_possession_epoch_time_s;
};
