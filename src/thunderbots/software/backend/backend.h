#pragma once

#include "ai/primitive/primitive.h"
#include "ai/world/world.h"
#include "multithreading/observable.h"
#include "multithreading/observer.h"

// TODO: This should be removed eventually
#include "thunderbots_msgs/World.h"

/**
 * A Backend is an abstraction around all I/O operations that our system may need
 * to perform. It produces Worlds that may be used, and consumes primitives that
 * need to be sent out (generally to the robots).
 *
 * This produce/consume pattern is performed by extending both "Observer" and
 * "Observable". Please see the the implementation of those classes for details.
 */
class Backend
    : public Observable<thunderbots_msgs::World>,
      public Observer<std::shared_ptr<const std::vector<std::unique_ptr<Primitive>>>>
{
   public:
    // TODO: Eventually this should be a non-ROS type
    using World = thunderbots_msgs::World;
    // TODO: Eventually this should be a non-ROS type
    using WorldBuffer  = ThreadSafeBuffer<World>;
    using PrimitiveVec = std::vector<std::unique_ptr<Primitive>>;
    using PrimitiveVecPtr =
        std::shared_ptr<const std::vector<std::unique_ptr<Primitive>>>;

    Backend() = default;

    virtual ~Backend() = default;

    // Delete the copy and assignment operators because this class really shouldn't need
    // them and we don't want to risk doing anything nasty with the internal
    // multithreading this class potentially uses
    Backend& operator=(const Backend&) = delete;
    Backend(const Backend&)            = delete;
};