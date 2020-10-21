#include <chrono>
#include <thread>

#include "boost/array.hpp"
#include "boost/asio.hpp"
#include "boost/bind.hpp"
#include "google/protobuf/message.h"
#include "shared/constants.h"
#include "shared/proto/robot_log_msg.pb.h"
#include "shared/proto/robot_status_msg.pb.h"
#include "shared/proto/tbots_software_msgs.pb.h"
#include "software/logger/logger.h"
#include "software/networking/threaded_proto_multicast_listener.h"
#include "software/networking/threaded_proto_multicast_sender.h"
#include "software/proto/message_translation/tbots_protobuf.h"


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
void status_callback(TbotsProto::RobotStatus test)
{
    if (test.has_time_sent())
    {
        std::cout << "robot status received: "
                  << test.time_sent().epoch_timestamp_seconds() << std::endl;
    }
    else
    {
        std::cout << "robot status received: but no time set" << std::endl;
    }
}

void log_callback(TbotsProto::RobotLog log)
{
    std::string log_level = "";

    switch (log.log_level())
    {
        case TbotsProto::LogLevel::DEBUG:
            log_level = "DEBUG";
            break;
        case TbotsProto::LogLevel::INFO:
            log_level = "DEBUG";
            break;
        case TbotsProto::LogLevel::WARN:
            log_level = "WARN";
            break;
        case TbotsProto::LogLevel::FATAL:
            log_level = "FATAL";
            break;
        default:
            log_level = "UNKNOWN";
            break;
    }

    std::cout << "[" << log_level << "]"
              << "[Robot " << log.robot_id() << "]"
              << "[" << log.file_name() << ":" << log.line_number()
              << "]: " << log.log_msg() << std::endl;
}

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        throw std::invalid_argument(
            "Please provide the interface you wish to multicast over");
    }

    GOOGLE_PROTOBUF_VERIFY_VERSION;

    TbotsProto::Vision test_vision_msg;
    TbotsProto::Primitive test_primitive_msg;

    // create an io service and run it in a thread to handle async calls
    boost::asio::io_service io_service;
    auto io_service_thread = std::thread([&]() { io_service.run(); });

    auto vision_sender =
        std::make_unique<ThreadedProtoMulticastSender<TbotsProto::Vision>>(
            std::string(MULTICAST_CHANNELS[0]) + "%" + std::string(argv[1]), VISION_PORT);

    auto primitive_sender =
        std::make_unique<ThreadedProtoMulticastSender<TbotsProto::Primitive>>(
            std::string(MULTICAST_CHANNELS[0]) + "%" + std::string(argv[1]),
            PRIMITIVE_PORT);

    auto status_listener =
        std::make_unique<ThreadedProtoMulticastListener<TbotsProto::RobotStatus>>(
            std::string(MULTICAST_CHANNELS[0]) + "%" + std::string(argv[1]),
            ROBOT_STATUS_PORT, std::function(status_callback));

    auto log_listener =
        std::make_unique<ThreadedProtoMulticastListener<TbotsProto::RobotLog>>(
            std::string(MULTICAST_CHANNELS[0]) + "%" + std::string(argv[1]),
            ROBOT_LOGS_PORT, std::function(log_callback));

    while (1)
    {
        // primitive and vision sender
        // primitive_sender->sendProto(test_primitive_msg);
        //*(test_vision_msg.mutable_time_sent()) = *createCurrentTimestamp();
        // vision_sender->sendProto(test_vision_msg);

        // 100 hz test
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    google::protobuf::ShutdownProtobufLibrary();

    return 0;
}
