#pragma once

#include "software/ai/ai.h"
#include "software/ai/hl/stp/play_info.h"
#include "software/multithreading/subject.h"
#include "software/multithreading/threaded_observer.h"
#include "software/visualizer/drawing/draw_functions.h"
#include "software/world/world.h"

/**
 * This class wraps an `AI` object, performing all the work of receiving World
 * objects, passing them to the `AI`, getting the primitives to send to the
 * robots based on the World state, and sending them out.
 */
class AIWrapper : public ThreadedObserver<World>,
                  public Subject<ConstPrimitiveVectorPtr>,
                  public Subject<AIDrawFunction>,
                  public Subject<PlayInfo>
{
   public:
    AIWrapper() = delete;

    /**
     * Create an AI with the given config
     *
     * @param config The AI configuration
     */
    explicit AIWrapper(std::shared_ptr<const AIConfig> ai_config,
                       std::shared_ptr<const AIControlConfig> control_config);

   private:
    void onValueReceived(World world) override;

    /**
     * Get primitives for the currently known world from the AI and pass them to
     * observers
     */
    void runAIAndSendPrimitives();
    void drawAI();

    AI ai;
    std::shared_ptr<const AIControlConfig> control_config;
    World most_recent_world;
};
