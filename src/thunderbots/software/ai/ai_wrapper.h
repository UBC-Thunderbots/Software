#pragma once

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include "software/ai/ai.h"
#include "software/ai/world/world.h"
#include "software/multithreading/subject.h"
#include "software/multithreading/threaded_observer.h"
#include "software/primitive/primitive.h"
#include "software/thunderbots_msgs/PlayInfo.h"
#include "software/typedefs.h"

/**
 * This class wraps an `AI` object, performing all the work of receiving World
 * objects, passing them to the `AI`, getting the primitives to send to the
 * robots based on the World state, and sending them out.
 */
class AIWrapper : public ThreadedObserver<World>, public Subject<ConstPrimitiveVectorPtr>
{
   public:
    AIWrapper() = delete;

    explicit AIWrapper(ros::NodeHandle node_handle);

   private:
    static const int PLAY_INFO_QUEUE_SIZE = 1;

    void onValueReceived(World world) override;

    /**
     * Publish the play info on a ROS topic
     */
    void publishPlayInfo();

    /**
     * Get primitives for the currently known world from the AI and pass them to
     * observers
     */
    void runAIAndSendPrimitives();

    /**
     * Publish the requisite ROS messages to draw the current state of the world
     */
    void drawWorld();

    /**
     * Convert the given PlayInfo into it's equivalent ROS type
     *
     * @param play_info The PlayInfo to convert
     *
     * @return The equivalent ROS message for the given PlayInfo
     */
    static thunderbots_msgs::PlayInfo convertPlayInfoToROSMessage(
        const PlayInfo& play_info);

    AI ai;
    World most_recent_world;
    ros::Publisher play_info_publisher;
};
