#pragma once

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <thread>

#include "ai/ai.h"
#include "ai/world/world.h"
#include "multithreading/observable.h"
#include "multithreading/observer.h"
#include "primitive/primitive.h"
#include "thunderbots_msgs/World.h"

// TODO: Better name for this class
// TODO: javadoc comment here
// TODO: explain shared_ptr->const vec->unique ptr
class AIWrapper
    : public Observer<thunderbots_msgs::World>,
      public Observable<std::shared_ptr<const std::vector<std::unique_ptr<Primitive>>>>
{
    // TODO: javadoc comments for all these functions and members
   public:
    AIWrapper() = delete;

    explicit AIWrapper(ros::NodeHandle node_handle);

    ~AIWrapper() override;

   private:
    void updateKnownWorld(thunderbots_msgs::World world_msg);

    void updateWorldFromBuffer();

    void publishPlayInfo();

    void runAIContinuously();

    void runAIAndSendPrimitives();
    void drawWorld();

    std::thread ai_thread;
    AI ai;

    World currently_known_world;

    ros::Publisher play_info_publisher;

    std::mutex in_destructor_mutex;
    bool in_destructor;
};
