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

    struct Update
    {
        Update(const GameState& game_state, const TbotsProto::AiConfig& ai_config)
            : game_state(game_state), ai_config(ai_config)
        {
        }
        GameState game_state;
        TbotsProto::AiConfig ai_config;
    };

    struct Override
    {
        explicit Override(std::unique_ptr<Play> play) : play(std::move(play)) {}
        std::shared_ptr<Play> play;
    };

    struct Reset : Override
    {
        using Override::Override;
    };

    /**
     * Creates a play selection FSM
     *
     * @param ai_config_ptr pointer to the default play config for this play fsm
     */
    explicit PlaySelectionFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

    /**
     * Gets the currently selected play
     *
     * @return the override play if one exists, otherwise the current play
     */
    Play& getSelectedPlay() const;

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
     * Action to set up the override play
     *
     * @param event The PlaySelection::Override event
     */
    void setupOverridePlay(const Override& event);

    /**
     * Action to reset play selection and set up the override play
     *
     * @param event The PlaySelection::Reset event
     */
    void resetPlaySelection(const Reset& event);

    /**
     * Action to set up the SetPlay, StopPlay, HaltPlay, or OffensePlay
     *
     * @param event The PlaySelection::Update event
     *
     */
    void setupSetPlay(const Update& event);
    void setupStopPlay(const Update& event);
    void setupHaltPlay(const Update& event);
    void setupOffensePlay(const Update& event);

    /**
     * Action to reset the current SetPlay to none
     *
     * @param event The PlaySelection::Update event
     */
    void resetSetPlay(const Update& event);

    /**
     * Sets the current play
     *
     * @param play the new current play
     */
    void setCurrentPlay(std::unique_ptr<Play> play);

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
        DEFINE_SML_EVENT(Override)
        DEFINE_SML_EVENT(Reset)

        DEFINE_SML_ACTION(setupOverridePlay)
        DEFINE_SML_ACTION(resetPlaySelection)
        DEFINE_SML_ACTION(setupSetPlay)
        DEFINE_SML_ACTION(setupStopPlay)
        DEFINE_SML_ACTION(setupHaltPlay)
        DEFINE_SML_ACTION(setupOffensePlay)
        DEFINE_SML_ACTION(resetSetPlay)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state

            // Check for transitions to other states, if not then default to running the
            // current play
            *Halt_S + Update_E[gameStateStopped_G] / setupStopPlay_A    = Stop_S,
            Halt_S + Update_E[gameStatePlaying_G] / setupOffensePlay_A  = Playing_S,
            Halt_S + Update_E[gameStateSetupRestart_G] / setupSetPlay_A = SetPlay_S,
            Halt_S + Override_E / setupOverridePlay_A,
            Halt_S + Reset_E / resetPlaySelection_A = Halt_S,

            // Check for transitions to other states, if not then default to running the
            // current play
            Stop_S + Update_E[gameStateHalted_G] / setupHaltPlay_A      = Halt_S,
            Stop_S + Update_E[gameStatePlaying_G] / setupOffensePlay_A  = Playing_S,
            Stop_S + Update_E[gameStateSetupRestart_G] / setupSetPlay_A = SetPlay_S,
            Stop_S + Override_E / setupOverridePlay_A,
            Stop_S + Reset_E / resetPlaySelection_A = Halt_S,

            // Check for transitions to other states, if not then default to running the
            // current play
            Playing_S + Update_E[gameStateHalted_G] / setupHaltPlay_A      = Halt_S,
            Playing_S + Update_E[gameStateStopped_G] / setupStopPlay_A     = Stop_S,
            Playing_S + Update_E[gameStateSetupRestart_G] / setupSetPlay_A = SetPlay_S,
            Playing_S + Override_E / setupOverridePlay_A,
            Playing_S + Reset_E / resetPlaySelection_A = Halt_S,

            // Check for transitions to other states, if not then default to running the
            // current play
            SetPlay_S + Update_E[gameStateHalted_G] / (resetSetPlay_A, setupHaltPlay_A) =
                Halt_S,
            SetPlay_S + Update_E[gameStateStopped_G] / (resetSetPlay_A, setupStopPlay_A) =
                Stop_S,
            SetPlay_S + Update_E[gameStatePlaying_G] /
                            (resetSetPlay_A, setupOffensePlay_A) = Playing_S,
            SetPlay_S + Update_E[gameStateSetupRestart_G] / setupSetPlay_A,
            SetPlay_S + Override_E / setupOverridePlay_A,
            SetPlay_S + Reset_E / resetPlaySelection_A = Halt_S,

            X + Update_E = X);
    }

   private:
    std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr;
    std::optional<TbotsProto::PlayName> current_set_play;
    std::shared_ptr<Play> current_play;
    std::shared_ptr<Play> override_play;
};
