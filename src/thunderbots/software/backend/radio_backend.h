#pragma once

#include "backend/backend.h"
#include "backend/input/network/networking/network_client.h"
#include "backend/output/radio/radio_output.h"

class RadioBackend : public Backend
{
   public:
    RadioBackend() = delete;

    explicit RadioBackend(ros::NodeHandle node_handle);

   private:

    // TODO: make this configurable?
    static const int DEFAULT_RADIO_CONFIG = 0;

    // TODO: javadoc comments for all functions here

    void onValueReceived(Backend::PrimitiveVecPtr primitives) override;

    void receiveWorld(World world);

    // The interface with the network that lets us get new information about the world
    NetworkClient network_input;

    // The interface that lets us send primitives to the robots over radio
    RadioOutput radio_output;

    // TODO: doc comments for these?
    std::optional<World> most_recently_received_world;
    std::mutex most_recently_received_world_mutex;

    // TODO: doc comments for these?
    PrimitiveVecPtr most_recently_received_primitives;
    std::mutex most_recently_received_primitives_mutex;
};