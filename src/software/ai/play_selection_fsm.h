#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play.h"

struct PlaySelectionFSM
{
    class Halt;
    class Playing;
    class Stop;
    class SetPlay;
    class OverridePlay;

    struct Update
    {
        Update(const std::function<void(std::unique_ptr<Play>)>& set_current_play,
               const GameState& game_state, const TbotsProto::AiConfig& ai_config)
            : set_current_play(set_current_play),
              game_state(game_state),
              ai_config(ai_config)
        {
        }
        std::function<void(std::unique_ptr<Play>)> set_current_play;
        GameState game_state;
        TbotsProto::AiConfig ai_config;
    };

    /**
     * Creates a play selection FSM
     *
     * @param ai_config the default play config for this play fsm
     */
    explicit PlaySelectionFSM(TbotsProto::AiConfig ai_config);

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
     * Guards for whether the play is being overridden
     *
     * @param event The PlaySelection::Update event
     *
     * @return whether the play is being overridden
     */
    bool playOverridden(const Update& event);

    /**
     * Action to set up the OverridePlay, SetPlay, StopPlay, HaltPlay, or OffensePlay
     *
     * @param event The PlaySelection::Update event
     *
     */
    void setupOverridePlay(Update event);
    void setupSetPlay(const Update& event);
    void setupStopPlay(const Update& event);
    void setupHaltPlay(const Update& event);
    void setupOffensePlay(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(SetPlay)
        DEFINE_SML_STATE(Halt)
        DEFINE_SML_STATE(Playing)
        DEFINE_SML_STATE(Stop)

        DEFINE_SML_GUARD(gameStateStopped)
        DEFINE_SML_GUARD(gameStateHalted)
        DEFINE_SML_GUARD(gameStatePlaying)
        DEFINE_SML_GUARD(gameStateSetupRestart)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(setupSetPlay)
        DEFINE_SML_ACTION(setupStopPlay)
        DEFINE_SML_ACTION(setupHaltPlay)
        DEFINE_SML_ACTION(setupOffensePlay)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state

            // Check for transitions to other states, if not then default to running the
            // current play
            *Halt_S + Update_E[gameStateStopped_G] / setupStopPlay_A    = Stop_S,
            Halt_S + Update_E[gameStatePlaying_G] / setupOffensePlay_A  = Playing_S,
            Halt_S + Update_E[gameStateSetupRestart_G] / setupSetPlay_A = SetPlay_S,

            // Check for transitions to other states, if not then default to running the
            // current play
            Stop_S + Update_E[gameStateHalted_G] / setupHaltPlay_A      = Halt_S,
            Stop_S + Update_E[gameStatePlaying_G] / setupOffensePlay_A  = Playing_S,
            Stop_S + Update_E[gameStateSetupRestart_G] / setupSetPlay_A = SetPlay_S,

            // Check for transitions to other states, if not then default to running the
            // current play
            Playing_S + Update_E[gameStateHalted_G] / setupHaltPlay_A      = Halt_S,
            Playing_S + Update_E[gameStateStopped_G] / setupStopPlay_A     = Stop_S,
            Playing_S + Update_E[gameStateSetupRestart_G] / setupSetPlay_A = SetPlay_S,

            // Check for transitions to other states, if not then default to running the
            // current play
            SetPlay_S + Update_E[gameStateHalted_G] / setupHaltPlay_A     = Halt_S,
            SetPlay_S + Update_E[gameStateStopped_G] / setupStopPlay_A    = Stop_S,
            SetPlay_S + Update_E[gameStatePlaying_G] / setupOffensePlay_A = Playing_S,

            X + Update_E = X);
    }

   private:
    TbotsProto::AiConfig ai_config;
    std::shared_ptr<Play> current_play;
};
