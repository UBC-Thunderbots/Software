#pragma once

#include "backend/backend.h"
#include "backend/input/network/networking/network_client.h"
#include "backend/output/radio/grsim_output.h"

class RadioBackend : public Backend
{
   public:
    RadioBackend() = delete;

    RadioBackend(ros::NodeHandle node_handle);

   private:

    static const int DEFAULT_RADIO_CONFIG = 0;
    // TODO: javadoc comments for all functions here

    void continuouslyUpdatePrimitivesFromBuffer();

    void setMostRecentlyReceivedWorld(Backend::World world);

    void setMostRecentlyReceivedPrimitives(Backend::PrimitiveVecPtr primitives);

    void receivePrimitives(Backend::PrimitiveVecPtr world);

    void receiveWorld(Backend::World world);

    void updateRadio();

    // The interface with the network that lets us get new information about the world
    NetworkClient network_input;

    // The interface that lets us send primitives to the robots over radio
    RadioOutput radio_output;

    // A thread that constantly pulls new primitive vectors from the buffer
    // and sends them out
    std::thread radio_output_thread;

    // A boolean indicating if we're in the destructor for this class
    bool in_destructor;
    std::mutex in_destructor_mutex;

    // TODO: doc comments for these?
    std::optional<Backend::World> most_recently_received_world;
    std::mutex most_recently_received_world_mutex;

    // TODO: doc comments for these?
    PrimitiveVecPtr most_recently_received_primitives;
    std::mutex most_recently_received_primitives_mutex;
};