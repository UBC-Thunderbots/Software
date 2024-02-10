#pragma once

#include <functional>

#include "proto/play_info_msg.pb.h"
#include "software/ai/hl/stp/play/play.h"
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
     * Create an AI with given configurations
     * @param ai_config_ The AI configuration
     */
    explicit Ai(const TbotsProto::AiConfig& ai_config);

    /**
     * Overrides the play
     *
     * @param play play to override with
     */
    void overridePlay(std::unique_ptr<Play> play);

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

    /**
     * Overrides the play from the play proto
     *
     * @param play_proto the play proto
     */
    void overridePlayFromProto(TbotsProto::Play play_proto);

    /**
     * Update the AiConfig proto
     *
     * @param ai_config The new AiConfig proto
     */
    void updateAiConfig(TbotsProto::AiConfig& ai_config);

   private:
    void checkAiConfig();

    TbotsProto::AiConfig ai_config_;
    std::unique_ptr<FSM<PlaySelectionFSM>> fsm;
    std::unique_ptr<Play> override_play;
    std::unique_ptr<Play> current_play;
    TbotsProto::Play current_override_play_proto;
    bool ai_config_changed;

    // inter play communication
    InterPlayCommunication inter_play_communication;

    struct TracyLiterals
    {
    };
};
