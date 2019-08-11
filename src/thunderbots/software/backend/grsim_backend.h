#pragma once

#include "backend/backend.h"
#include "backend/input/network/networking/network_client.h"
#include "backend/output/grsim/grsim_output.h"
#include "ai/world/world.h"

class GrSimBackend : public Backend
{
   public:
    GrSimBackend();

   private:
    // TODO: jdoc comments!

    void onValueReceived(Backend::PrimitiveVecPtr primitives) override;

    void setMostRecentlyReceivedWorld(World world);

    void setMostRecentlyReceivedPrimitives(Backend::PrimitiveVecPtr primitives);

    void receiveWorld(World world);

    void updateGrSim();

    // The interface with the network that lets us get new information about the world
    NetworkClient network_input;

    // The interface that lets us send primitives to grsim
    GrSimOutput grsim_output;

    // TODO: doc comments for these?
    std::optional<World> most_recently_received_world;
    std::mutex most_recently_received_world_mutex;

    // TODO: doc comments for these?
    PrimitiveVecPtr most_recently_received_primitives;
    std::mutex most_recently_received_primitives_mutex;
};