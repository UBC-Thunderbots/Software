#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/dynamic_plays/offensive_plays.h"

struct PlaySelectionFSM
{
    class Halt;
    class Stop;
    class SetPlay;
    class OffensivePlay;
    class DefensivePlay;

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
     * Guard to check whether the current game state is a free kick for the
     * friendly team
     *
     * @param event The PlaySelection::Update event
     *
     * @return whether the current game state is a free kick for the friendly team
     */
    bool isFriendlyFreeKick(const Update& event);

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
     * OffensivePlay, or DefensivePlay
     *
     * @param event The PlaySelection::Update event
     */
    void setupOverridePlay(Update event);
    void setupSetPlay(const Update& event);
    void setupStopPlay(const Update& event);
    void setupHaltPlay(const Update& event);
    void setupOffensivePlay(const Update& event);
    void setupDefensivePlay(const Update& event);

    /**
     * Action to evaluate the currently running DynamicPlay
     *
     * @param event The PlaySelection::Update event
     */
    void evaluateDynamicPlay(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(Halt)
        DEFINE_SML_STATE(Stop)
        DEFINE_SML_STATE(SetPlay)
        DEFINE_SML_STATE(OffensivePlay)
        DEFINE_SML_STATE(DefensivePlay)

        DEFINE_SML_GUARD(gameStateStopped)
        DEFINE_SML_GUARD(gameStateHalted)
        DEFINE_SML_GUARD(gameStatePlaying)
        DEFINE_SML_GUARD(gameStateSetupRestart)
        DEFINE_SML_GUARD(isFriendlyFreeKick)
        DEFINE_SML_GUARD(enemyHasPossession)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(setupSetPlay)
        DEFINE_SML_ACTION(setupStopPlay)
        DEFINE_SML_ACTION(setupHaltPlay)
        DEFINE_SML_ACTION(setupOffensivePlay)
        DEFINE_SML_ACTION(setupDefensivePlay)
        DEFINE_SML_ACTION(evaluateDynamicPlay)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state

            *Halt_S + Update_E[gameStateStopped_G] / setupStopPlay_A = Stop_S,
            Halt_S + Update_E[gameStatePlaying_G && !enemyHasPossession_G] /
                         setupOffensivePlay_A = OffensivePlay_S,
            Halt_S + Update_E[gameStatePlaying_G && enemyHasPossession_G] /
                         setupDefensivePlay_A = DefensivePlay_S,
            Halt_S + Update_E[gameStateSetupRestart_G && isFriendlyFreeKick_G] /
                         setupOffensivePlay_A                           = OffensivePlay_S,
            Halt_S + Update_E[gameStateSetupRestart_G] / setupSetPlay_A = SetPlay_S,

            Stop_S + Update_E[gameStateHalted_G] / setupHaltPlay_A = Halt_S,
            Stop_S + Update_E[gameStatePlaying_G && !enemyHasPossession_G] /
                         setupOffensivePlay_A = OffensivePlay_S,
            Stop_S + Update_E[gameStatePlaying_G && enemyHasPossession_G] /
                         setupDefensivePlay_A = DefensivePlay_S,
            Stop_S + Update_E[gameStateSetupRestart_G && isFriendlyFreeKick_G] /
                         setupOffensivePlay_A                           = OffensivePlay_S,
            Stop_S + Update_E[gameStateSetupRestart_G] / setupSetPlay_A = SetPlay_S,

            OffensivePlay_S + Update_E[gameStateHalted_G] /
                                  (evaluateDynamicPlay_A, setupHaltPlay_A) = Halt_S,
            OffensivePlay_S + Update_E[gameStateStopped_G] /
                                  (evaluateDynamicPlay_A, setupStopPlay_A) = Stop_S,
            OffensivePlay_S + Update_E[gameStateSetupRestart_G && isFriendlyFreeKick_G] /
                                  (evaluateDynamicPlay_A, setupOffensivePlay_A) =
                OffensivePlay_S,
            OffensivePlay_S + Update_E[gameStateSetupRestart_G] /
                                  (evaluateDynamicPlay_A, setupSetPlay_A) = SetPlay_S,
            OffensivePlay_S + Update_E[enemyHasPossession_G] /
                                  (evaluateDynamicPlay_A, setupDefensivePlay_A) =
                DefensivePlay_S,

            DefensivePlay_S + Update_E[gameStateHalted_G] / setupHaltPlay_A  = Halt_S,
            DefensivePlay_S + Update_E[gameStateStopped_G] / setupStopPlay_A = Stop_S,
            DefensivePlay_S + Update_E[gameStateSetupRestart_G && isFriendlyFreeKick_G] /
                                  (evaluateDynamicPlay_A, setupOffensivePlay_A) =
                OffensivePlay_S,
            DefensivePlay_S + Update_E[gameStateSetupRestart_G] / setupSetPlay_A =
                SetPlay_S,
            DefensivePlay_S + Update_E[!enemyHasPossession_G] / setupOffensivePlay_A =
                OffensivePlay_S,

            SetPlay_S + Update_E[gameStateHalted_G] / setupHaltPlay_A  = Halt_S,
            SetPlay_S + Update_E[gameStateStopped_G] / setupStopPlay_A = Stop_S,
            SetPlay_S + Update_E[gameStatePlaying_G && !enemyHasPossession_G] /
                            setupOffensivePlay_A = OffensivePlay_S,
            SetPlay_S + Update_E[gameStatePlaying_G && enemyHasPossession_G] /
                            setupDefensivePlay_A = DefensivePlay_S,

            X + Update_E = X);
    }

   private:
    std::shared_ptr<Strategy> strategy_;
    std::shared_ptr<DynamicPlay> current_dynamic_play_;
    std::shared_ptr<OffensiveFriendlyThirdPlay> offensive_friendly_third_play_;
    std::shared_ptr<OffensiveMiddleThirdPlay> offensive_middle_third_play_;
    std::shared_ptr<OffensiveEnemyThirdPlay> offensive_enemy_third_play_;
};
