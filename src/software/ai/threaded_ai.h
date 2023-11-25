#pragma once

#include <mutex>

#include "proto/parameters.pb.h"
#include "proto/play.pb.h"
#include "proto/play_info_msg.pb.h"
#include "proto/tactic.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "software/ai/ai.h"
#include "software/multithreading/first_in_first_out_threaded_observer.h"
#include "software/multithreading/subject.hpp"
#include "software/world/world.h"

/**
 * This class wraps an `AI` object, performing all the work of receiving World
 * objects, passing them to the `AI`, getting the primitives to send to the
 * robots based on the World state, and sending them out.
 */
class ThreadedAi : public FirstInFirstOutThreadedObserver<World>,
                   public FirstInFirstOutThreadedObserver<TbotsProto::ThunderbotsConfig>,
                   public Subject<TbotsProto::PrimitiveSet>,
                   public Subject<TbotsProto::PlayInfo>
{
   public:
    ThreadedAi() = delete;

    /**
     * Constructs a new ThreadedAi object.
     *
     * @param tbots_proto The AI configuration
     */
    explicit ThreadedAi(const TbotsProto::AiConfig& ai_config);

    /**
     * Override the AI play
     *
     * @param play_proto The play proto to use to override
     */
    void overridePlay(TbotsProto::Play play_proto);

    /**
     * Override Tactics
     *
     * @param assigned_tactic_play_control_params
     */
    void overrideTactics(
        TbotsProto::AssignedTacticPlayControlParams assigned_tactic_play_control_params);

   private:
    void onValueReceived(World world) override;
    void onValueReceived(TbotsProto::ThunderbotsConfig config) override;

    /**
     * Get primitives for the new world from the AI and pass them to observers
     *
     * @param world the new world
     */
    void runAiAndSendPrimitives(const World& world);

    Ai ai;
    TbotsProto::AiConfig ai_config;
    TbotsProto::AiControlConfig ai_control_config;
    std::mutex ai_mutex;
};
