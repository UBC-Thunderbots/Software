#pragma once

#include <functional>

#include "proto/play_info_msg.pb.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/strategy/strategy.h"
#include "software/ai/play_selection_fsm.h"
#include "software/time/timestamp.h"
#include "software/world/world.h"

/**
 * This class wraps all our AI logic and decision making.
 */
class Ai final
{
   public:
    Ai() = delete;

    /**
     * Create an AI
     * 
     * @param strategy the Strategy
     */
    explicit Ai(std::shared_ptr<Strategy> strategy);

    /**
     * Overrides the play
     *
     * @param play play to override with
     */
    void overridePlay(std::unique_ptr<Play> play);

    /**
     * Overrides the play from the play proto
     *
     * @param play_proto the play proto
     */
    void overridePlayFromProto(const TbotsProto::Play& play_proto);

    /**
     * Overrides the play with AssignedTacticsPlay and overrides the tactics
     *
     * @param assigned_tactic_play_control_params the control params for AssignedTacticsPlay 
     */
    void overrideTactics(
        const TbotsProto::AssignedTacticPlayControlParams& assigned_tactic_play_control_params);

    /**
     * Calculates the Primitives that should be run by our Robots given the current
     * state of the world.
     *
     * @param world The state of the World with which to make the decisions
     *
     * @return the Primitives that should be run by our Robots given the current
     * state of the world.
     */
    std::unique_ptr<TbotsProto::PrimitiveSet> getPrimitives(const World& world);

    /**
     * Returns information about the currently running plays and tactics, including the
     * name of the play, and which robots are running which tactics
     *
     * @return information about the currently running plays and tactics
     */
    TbotsProto::PlayInfo getPlayInfo() const;

   private:
    std::shared_ptr<Strategy> strategy;
    std::unique_ptr<FSM<PlaySelectionFSM>> fsm;
    std::unique_ptr<Play> override_play;
    std::shared_ptr<Play> current_play;

    // inter play communication
    InterPlayCommunication inter_play_communication;
};
