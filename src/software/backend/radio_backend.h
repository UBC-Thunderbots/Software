#pragma once

#include "software/backend/backend.h"
#include "software/backend/input/network/networking/network_client.h"
#include "software/backend/output/radio/radio_output.h"

class RadioBackend : public Backend
{
   public:
    static const std::string name;

    RadioBackend();

   private:
    static const int DEFAULT_RADIO_CONFIG = 0;

    void onValueReceived(ConstPrimitiveVectorPtr primitives) override;

    /**
     * This is registered as an async callback function so that it is called
     * with a new world every time one is available
     *
     * @param world The new world
     */
    void receiveWorld(World world);

    /**
     * Convert robot_status to TbotsRobotMsg and send as a SensorMsg to observers
     *
     * @param robot_status The RobotStatus
     */
    void receiveRobotStatus(RobotStatus robot_status);

    // The interface with the network that lets us get new information about the world
    NetworkClient network_input;

    // The interface that lets us send primitives to the robots over radio
    RadioOutput radio_output;

    std::optional<World> most_recently_received_world;
    std::mutex most_recently_received_world_mutex;

    ConstPrimitiveVectorPtr most_recently_received_primitives;
    std::mutex most_recently_received_primitives_mutex;
};
