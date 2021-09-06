#pragma once

#include "shared/proto/tbots_software_msgs.pb.h"
#include "software/ai/ai.h"
#include "software/gui/drawing/draw_functions.h"
#include "software/multithreading/first_in_first_out_threaded_observer.h"
#include "software/multithreading/subject.h"
#include "software/proto/play_info_msg.pb.h"
#include "software/world/world.h"

/**
 * This class wraps an `AI` object, performing all the work of receiving World
 * objects, passing them to the `AI`, getting the primitives to send to the
 * robots based on the World state, and sending them out.
 */
class ThreadedAI : public FirstInFirstOutThreadedObserver<World>,
                   public Subject<TbotsProto::PrimitiveSet>,
                   public Subject<AIDrawFunction>,
                   public Subject<PlayInfo>
{
   public:
    ThreadedAI() = delete;

    /**
     * Create an AI with the given config
     *
     * @param ai_config The AI configuration
     * @param control_config The AI control configuration
     * @param play_config The play configuration
     */
    explicit ThreadedAI(std::shared_ptr<const AiConfig> ai_config,
                        std::shared_ptr<const AiControlConfig> control_config,
                        std::shared_ptr<const PlayConfig> play_config);

   private:
    void onValueReceived(World world) override;

    /**
     * Get primitives for the new world from the AI and pass them to observers
     *
     * @param world the new world
     */
    void runAIAndSendPrimitives(const World &world);

    /**
     * Publishes draw functions
     */
    void drawAI();

    AI ai;
    std::shared_ptr<const AiControlConfig> control_config;
};
