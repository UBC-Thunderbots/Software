#pragma once

#include "shared/proto/tbots_robot_msg.pb.h"
#include "shared/proto/tbots_software_msgs.pb.h"
#include "software/backend/backend.h"
#include "software/backend/input/network/networking/network_client.h"
#include "software/networking/threaded_proto_multicast_listener.h"
#include "software/networking/threaded_proto_multicast_sender.h"

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
     * This is registered as an async callback function on the robot_msg_input
     * to receive msgs from the robot.
     *
     * @param robot_msg The robot_msg from a robot
     */
    void receiveTbotsRobotMsg(TbotsRobotMsg robot_msg);

    /**
     * This is called when we receive a primitive vector because
     * the backend is a ThreadedObserver of the ConstantPrimitiveVectorPtr
     *
     * @param primitives The vector of primitives to pack and send to robots
     */
    void onValueReceived(ConstPrimitiveVectorPtr primitives) override;

    /**
     * Joins the specified multicast group on the vision_output, primitive_output
     * and robot_msg_input. Multicast Channel and Multicast Group are used
     * interchangeably.
     *
     * NOTE: This will terminate the existing connection on the previous channel
     * if it exists.
     *
     * @param channel The channel to join, index of MULTICAST_CHANNELS in
     * shared/constants.h
     * @param interface The interface to join the multicast group on (lo, eth0, enp3s0f1,
     * etc.)
     */
    void joinMulticastChannel(int channel, const std::string& interface);


    // The interface with the network that lets us get new information about the world
    NetworkClient network_input;

    // ProtoMulticast** to communicate with robots
    std::unique_ptr<ThreadedProtoMulticastSender<VisionMsg>> vision_output;
    std::unique_ptr<ThreadedProtoMulticastSender<PrimitiveSetMsg>> primitive_output;
    std::unique_ptr<ThreadedProtoMulticastListener<TbotsRobotMsg>> robot_msg_input;
};
