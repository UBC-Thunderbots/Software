#pragma once

#include "backend/backend.h"
#include "backend/input/network/networking/network_client.h"
#include "backend/output/grsim/grsim_output.h"

class GrSimBackend : public Backend
{
   public:
    GrSimBackend();

   private:
    void newValueCallback(Backend::PrimitiveVecPtr primitives) override;

    // TODO: javadoc comments for all functions here

    void continuouslyUpdatePrimitivesFromBuffer();

    void setMostRecentlyReceivedWorld(Backend::World world);

    void setMostRecentlyReceivedPrimitives(Backend::PrimitiveVecPtr primitives);


    void receiveWorld(Backend::World world);

    // TODO: Doc comment
    void updateGrSim();

    // The interface with the network that lets us get new information about the world
    NetworkClient network_input;

    // The interface that lets us send primitives to grsim
    GrSimOutput grsim_output;

    // TODO: doc comments for these?
    std::optional<Backend::World> most_recently_received_world;
    std::mutex most_recently_received_world_mutex;

    // TODO: doc comments for these?
    PrimitiveVecPtr most_recently_received_primitives;
    std::mutex most_recently_received_primitives_mutex;
};