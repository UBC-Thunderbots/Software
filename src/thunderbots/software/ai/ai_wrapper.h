#pragma once

#include "software/ai/ai.h"
#include "software/ai/world/world.h"
#include "software/multithreading/subject.h"
#include "software/multithreading/threaded_observer.h"
#include "software/typedefs.h"
#include "software/gui/drawing/draw_functions.h"
#include "software/ai/hl/stp/play_info.h"

/**
 * This class wraps an `AI` object, performing all the work of receiving World
 * objects, passing them to the `AI`, getting the primitives to send to the
 * robots based on the World state, and sending them out.
 */
class AIWrapper : public ThreadedObserver<World>, public Subject<ConstPrimitiveVectorPtr>, public Subject<AIDrawFunction>,
        public Subject<PlayInfo>
{
   public:
    AIWrapper() = default;

   private:
    void onValueReceived(World world) override;

    /**
     * Get primitives for the currently known world from the AI and pass them to
     * observers
     */
    void runAIAndSendPrimitives();

    /**
     * Publish the requisite ROS messages to draw the current state of the world
     */
    void drawWorld();
    void drawAI();

    AI ai;
    World most_recent_world;
};
