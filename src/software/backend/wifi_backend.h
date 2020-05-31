#pragma once

#include "shared/proto/tbots_robot_msg.pb.h"
#include "shared/proto/tbots_software_msgs.pb.h"
#include "software/backend/backend.h"
#include "software/networking/proto_multicast_listener.h"
#include "software/networking/proto_multicast_sender.h"

class WifiBackend : public Backend
{
   public:
    WifiBackend();

    static const std::string name;

   private:
    /**
     * This is registered as an async callback function so that it is called
     * with a new world every time one is available
     *
     * @param world The new world
     */
    void receiveWorld(World world);

    /**
     * This is called when we receive a primitive vector because
     * the backend is a ThreadedObserver of the ConstantPrimitiveVectorPtr
     *
     * @param primitives The vector of primitives to pack and send to robots
     */
    void onValueReceived(ConstPrimitiveVectorPtr primitives) override;

    // The interface with the network that lets us get new information about the world
    NetworkClient network_input;

    // ProtoMulticast** to communicate with robots
    std::unique_ptr<ProtoMulticastSender<VisionMsg>> vision_output;
    std::unique_ptr<ProtoMulticastSender<PrimitiveMsg>> primitive_output;
    std::unique_ptr<ProtoMulticastListener<TbotsRobotMsg>> robot_status_input;
};
