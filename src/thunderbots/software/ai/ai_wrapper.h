#pragma once

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <thread>

#include "ai/ai.h"
#include "ai/world/world.h"
#include "multithreading/subject.h"
#include "multithreading/threaded_observer.h"
#include "primitive/primitive.h"
#include "thunderbots_msgs/PlayInfo.h"

// TODO: Better name for this class
// TODO: javadoc comment here
// TODO: explain shared_ptr->const_vec->unique ptr for primitives here
//       (WHY NOT JUST CONST REFERENCE??)
class AIWrapper
    : public ThreadedObserver<World>,
      public Subject<std::shared_ptr<const std::vector<std::unique_ptr<Primitive>>>>
{
    // TODO: javadoc comments for all these functions and members
   public:
    AIWrapper() = delete;

    explicit AIWrapper(ros::NodeHandle node_handle);


   private:
    void onValueReceived(World world) override;

    void publishPlayInfo();

    void runAIAndSendPrimitives();
    void drawWorld();

    static thunderbots_msgs::PlayInfo convertPlayInfoToROSMessage(
        const PlayInfo& play_info);

    AI ai;

    World currently_known_world;

    ros::Publisher play_info_publisher;
};
