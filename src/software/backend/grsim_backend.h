#pragma once

#include "software/backend/backend.h"
#include "software/backend/input/network/networking/network_client.h"
#include "software/backend/output/grsim/grsim_output.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/world/world.h"

class GrSimBackend : public Backend
{
   public:
    static const std::string name;
    GrSimBackend(std::shared_ptr<const NetworkConfig> network_config =
                     DynamicParameters->getNetworkConfig());

   private:
    void onValueReceived(ConstPrimitiveVectorPtr primitives) override;

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
    void setMostRecentlyReceivedPrimitives(ConstPrimitiveVectorPtr primitives);

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

    const std::shared_ptr<const NetworkConfig> network_config;

    // The interface with the network that lets us get new information about the world
    NetworkClient network_input;

    // The interface that lets us send primitives to grsim
    GrSimOutput grsim_output;

    std::optional<World> most_recently_received_world;
    std::mutex most_recently_received_world_mutex;

    ConstPrimitiveVectorPtr most_recently_received_primitives;
    std::mutex most_recently_received_primitives_mutex;
};
