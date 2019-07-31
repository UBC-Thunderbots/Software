#pragma once

#include "backend/backend.h"
#include "backend/input/network/networking/network_client.h"
#include "backend/output/grsim/grsim_output.h"

class GrSimBackend : public Backend
{
   public:
    GrSimBackend() = delete;

    GrSimBackend(WorldBufferPtr world_buffer,
                 PrimitiveVecBufferPtr primitive_vector_buffer);

   private:
    // TODO: javadoc comments for all functions here

    void continuouslyUpdatePrimitivesFromBuffer();

    void setMostRecentlyReceivedWorld(Backend::World world);

    void setMostRecentlyReceivedPrimitives(Backend::PrimitiveVec primitives);

    void receivePrimitives(Backend::PrimitiveVec world);
    void receiveWorld(Backend::World world);

    // TODO: Doc comment
    void updateGrSim();

    // The interface with the network that lets us get new information about the world
    NetworkClient network_input;

    // The interface that lets us send primitives to grsim
    GrSimOutput grsim_output;

    // A thread that constantly pulls new primitive vectors from the buffer
    // and sends them out
    std::thread grsim_output_thread;

    // A boolean indicating if we're in the destructor for this class
    bool in_destructor;
    std::mutex in_destructor_mutex;

    // TODO: doc comments for these?
    std::optional<Backend::World> most_recently_received_world;
    std::mutex most_recently_received_world_mutex;

    // TODO: doc comments for these?
    std::optional<PrimitiveVec> most_recently_received_primitives;
    std::mutex most_recently_received_primitives_mutex;
};