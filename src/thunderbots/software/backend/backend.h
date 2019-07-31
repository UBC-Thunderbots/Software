#pragma once

#include "ai/primitive/primitive.h"
#include "multithreading/thread_safe_buffer.h"
#include "ai/world/world.h"

// TODO: This should be removed eventually
#include "thunderbots_msgs/World.h"

#include <thread>

/**
 * A Backend is an abstraction around all I/O operations that our system may need
 * to perform. It produces Worlds that may be used, and consumes primitives that
 * need to be sent out (generally to the robots).
 *
 * This produce/consume is done via shared buffers that are given to the backend. Things
 * can be pulled from the output buffer given to this class, and pushed to it's input
 * buffer.
 */
class Backend {
public:

    // Eventually this should be a WorldState, or some non-ROS type
    using World = thunderbots_msgs::World;
    using WorldBuffer = ThreadSafeBuffer<World>;
    using PrimitiveVec = std::vector<std::unique_ptr<Primitive>>;
    using PrimitiveVecBuffer = ThreadSafeBuffer<PrimitiveVec>;
    using WorldBufferPtr = std::shared_ptr<WorldBuffer>;
    using PrimitiveVecBufferPtr = std::shared_ptr<PrimitiveVecBuffer>;

    Backend() = delete;

    /**
     *
     * @param world_buffer The world buffer for this backend. New world objects will
     *                     automatically be pushed to this buffer as they are received
     *                     by the backend
     * @param primitive_vector_buffer The primitive vector buffer for this backend. Any
     *                                primitive vectors pushed into this buffer will
     *                                automatically be sent out (usually to the robots)
     *                                as fast as possible
     */
    Backend(WorldBufferPtr world_buffer, PrimitiveVecBufferPtr primitive_vector_buffer);

    virtual ~Backend();

    // Delete the copy and assignment operators because this class really shouldn't need
    // them and we don't want to risk doing anything nasty with the internal
    // multithreading this class potentially uses
    Backend& operator=(const Backend&) = delete;
    Backend(const Backend&)            = delete;

protected:

    /**
     * Add the given world to the world_buffer
     *
     * @param world
     */
    void addWorldToBuffer(World world);

    /**
     * Get the most recent primitive vector from the buffer.
     *
     * This will block until a primitive vector is available
     *
     * @return The most recent primitive vector from the buffer
     */
    PrimitiveVec getPrimitiveVecFromBuffer();

private:
    // TODO: doc comments for these
    WorldBufferPtr world_buffer;
    PrimitiveVecBufferPtr primitive_vector_buffer;
};