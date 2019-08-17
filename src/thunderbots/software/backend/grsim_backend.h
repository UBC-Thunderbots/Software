#pragma once

#include "ai/world/world.h"
#include "backend/backend.h"
#include "backend/input/network/networking/network_client.h"
#include "backend/output/grsim/grsim_output.h"

class GrSimBackend : public Backend
{
   public:
    static const std::string name;
    GrSimBackend();

   private:
    void onValueReceived(Backend::PrimitiveVecPtr primitives) override;

    /**
     * Set the most recently received world
     *
     * @param world The new most recently received world
     */
    void setMostRecentlyReceivedWorld(World world);

    /**
     * Set the most recently received primitives
     *
     * @param world The new most recently received primitives
     */
    void setMostRecentlyReceivedPrimitives(Backend::PrimitiveVecPtr primitives);

    /**
     * This is registered as an async callback function so that it is called
     * with a new world every time one is available
     *
     * @param world The new world
     */
    void receiveWorld(World world);

    /**
     * Send the current state of the world and the primitives for each robot to GrSim
     */
    void updateGrSim();

    // The interface with the network that lets us get new information about the world
    NetworkClient network_input;

    // The interface that lets us send primitives to grsim
    GrSimOutput grsim_output;

    std::optional<World> most_recently_received_world;
    std::mutex most_recently_received_world_mutex;

    PrimitiveVecPtr most_recently_received_primitives;
    std::mutex most_recently_received_primitives_mutex;
};
