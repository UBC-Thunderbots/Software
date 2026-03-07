#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/evaluation/find_open_areas.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play/play_fsm.hpp"
#include "software/ai/hl/stp/tactic/chip/chip_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/logger/logger.h"

struct KickoffFriendlyPlayFSM : PlayFSM<KickoffFriendlyPlayFSM>
{
    class SetupState;
    class ChipState;

    struct ControlParams
    {
    };


    /**
     * Creates a kickoff friendly play FSM
     *
     * @param ai_config_ptr the play config for this play FSM
     */
    explicit KickoffFriendlyPlayFSM(
        const std::shared_ptr<const TbotsProto::AiConfig>& ai_config_ptr);


    /**
     * create a vector of setup positions if not already existing.
     *
     * @param world_ptr the world pointer
     */
    void createKickoffSetupPositions(const WorldPtr& world_ptr);

    /**
     * Action to move robots to starting positions
     *
     * @param event the FreeKickPlayFSM Update event
     */
    void setupKickoff(const Update& event);

    /**
     * Action to chip the ball forward over the defenders.
     *
     * @param event the FreeKickPlayFSM Update event
     */
    void chipBall(const Update& event);

    /**
     * Guard that checks if positions are set up.
     *
     * @param event the FreeKickPlayFSM Update event
     */
    static bool isSetupDone(const Update& event);

    /**
     * Guard that checks if game has started (ball kicked).
     *
     * @param event the FreeKickPlayFSM Update event
     */
    static bool isPlaying(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(SetupState)
        DEFINE_SML_STATE(ChipState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(setupKickoff)
        DEFINE_SML_ACTION(chipBall)

        DEFINE_SML_GUARD(isSetupDone)
        DEFINE_SML_GUARD(isPlaying)
        return make_transition_table(
            *SetupState_S + Update_E[!isSetupDone_G] / setupKickoff_A = SetupState_S,
            SetupState_S + Update_E[isSetupDone_G]                    = ChipState_S,
            ChipState_S + Update_E[!isPlaying_G] / chipBall_A         = ChipState_S,
            ChipState_S + Update_E[isPlaying_G]                       = X,

            X + Update_E = X);
    }

   private:
    TbotsProto::AiConfig ai_config;
    std::shared_ptr<KickoffChipTactic> kickoff_chip_tactic;
    std::vector<std::shared_ptr<MoveTactic>> move_tactics;
    std::vector<Point> kickoff_setup_positions;
};
