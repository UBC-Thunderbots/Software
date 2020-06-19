#include <chrono>
#include <thread>

#include "boost/array.hpp"
#include "boost/asio.hpp"
#include "boost/bind.hpp"
#include "shared/constants.h"
#include "shared/proto/tbots_software_msgs.pb.h"
#include "google/protobuf/message.h"
#include "software/logger/logger.h"
#include "software/networking/proto_multicast_listener.h"


using boost::asio::ip::udp;
using google::protobuf::Message;


/*
 * This file serves as a testing file to easily send proto at a fixed rate.
 * Plug the computers ethernet port into an ethernet switch/router.
 *
 * Run `ifconfig` in the terminal and find your ethernet interface.
 * On linux it will usually be eth0 or enp3s0f1
 *
 * Plug the STM32H7 into the same switch/router and then run this with
 * bazel run //firmware_new/tools:send_proto_over_udp -- your_interface_here
 *
 */
void callback(VisionMsg test){
}

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        throw std::invalid_argument(
            "Please provide the interface you wish to multicast over");
    }

    GOOGLE_PROTOBUF_VERIFY_VERSION;

    VisionMsg test;

    // create an io service and run it in a thread to handle async calls
    boost::asio::io_service io_service;
    auto io_service_thread = std::thread([&]() { io_service.run(); });

    // create ProtoMulticastSender to send proto
    auto sender = std::make_unique<ProtoMulticastListener<VisionMsg>>(
        io_service, std::string(MULTICAST_CHANNELS[0]) + "%" + std::string(argv[1]),
        ROBOT_STATUS_PORT, &callback);

    while (1)
    {
        //sender->sendProto(control_req);

        // 4000 hz test
        std::this_thread::sleep_for(std::chrono::nanoseconds(250000));
    }

    google::protobuf::ShutdownProtobufLibrary();

    return 0;
}
