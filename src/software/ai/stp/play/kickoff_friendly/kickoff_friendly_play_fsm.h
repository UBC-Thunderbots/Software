#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/evaluation/find_open_areas.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play/play_fsm.hpp"
#include "software/ai/hl/stp/tactic/kick_or_chip/kick_or_chip_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/logger/logger.h"


/**
 * This FSM implements the Kickoff Friendly Play. It manages kickoff when the friendly
 * side is kicking.
 * - It positions robots to starting points.
 * - It stays ready to start the game.
 * - It chips the ball into the largest open circle that is sufficiently close to the
 * enemy net, but also reasonably far from the edges of the field.
 * - Terminates after the ball is touched, passing control to OffensePlay.
 */
struct KickoffFriendlyPlayFSM : PlayFSM<KickoffFriendlyPlayFSM>
{
    static constexpr double ENEMY_X_PADDING_M          = 2.0;
    static constexpr double SIDELINE_PADDING_M         = 0.3;
    static constexpr double FALLBACK_TARGET_X_FRACTION = 1.0 / 6.0;

    class SetupState;
    class ChipState;

    /**
     * Control Parameters for a Kickoff Friendly Play
     */
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
     * - Creates a rectangle within the enemy half of the field with padding.
     * - Finds the largest open circles between enemy bots.
     * - Chooses the largest viable open circle that is closest to the enemy net.
     * - Defaults to a short chip if no open circle returned.
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
    std::shared_ptr<KickoffChipTactic> kickoff_chip_tactic;
    std::vector<std::shared_ptr<MoveTactic>> move_tactics;
    std::vector<Point> kickoff_setup_positions;
};
