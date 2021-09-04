#include <boost/program_options.hpp>
#include <iostream>
#include <numeric>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/logger/network_logger.h"
#include "shared/proto/robot_log_msg.pb.h"
#include "shared/proto/tbots_software_msgs.pb.h"
#include "software/constants.h"
#include "software/networking/threaded_proto_udp_listener.h"
#include "software/networking/threaded_proto_udp_sender.h"



void helper(TbotsProto::RobotLog log){
    std::cout<<"received"<<std::endl;

//    LOG(INFO) << "[ROBOT " << log.robot_id() << " " << LogLevel_Name(log.log_level())
//              << "]"
//              << "]: " << log.log_msg() << std::endl;
}
//std::unique_ptr<ThreadedProtoUdpListener<TbotsProto::RobotLog>> log_input;


int main(int argc, char **argv)
{
    // load command line arguments
    auto args           = std::make_shared<NetworkLoggerMainCommandLineArgs>();
    bool help_requested = args->loadFromCommandLineArguments(argc, argv);

    NetworkLoggerSingleton::initializeLogger(0,"wlp2s0",0);


    if (!help_requested)
    {
        int channel = args->getChannel()->value();
        std::string interface = args->getInterface()->value();
        TbotsProto::RobotLog test_log;
        //TbotsProto::Primitive test_primitive_msg;


//        // create an io service and run it in a thread to handle async calls
//        boost::asio::io_service io_service;
//        auto io_service_thread = std::thread([&]() { io_service.run(); });

//        log_input.reset(new ThreadedProtoUdpListener<TbotsProto::RobotLog>(
//                std::string(NETWORK_LOGGING_MULTICAST_CHANNELS[channel]) + "%" + interface, NETWORK_LOGS_PORT,
//                boost::bind(helper, _1), true));

        auto log_input =
                std::make_unique<ThreadedProtoUdpListener<TbotsProto::RobotLog>>(
                        std::string(NETWORK_LOGGING_MULTICAST_CHANNELS[channel]) + "%" + "wlp2s0", NETWORK_LOGS_PORT,
                        std::function(helper), true);

//        auto primitive_input =
//                std::make_unique<ThreadedProtoUdpListener<TbotsProto::Primitive>>(
//                        std::string(NETWORK_LOGGING_MULTICAST_CHANNELS[channel]) + "%" + "wlp2s0", NETWORK_LOGS_PORT,
//                        std::function(helper), true);
//
//        auto log_output = std::make_unique<ThreadedProtoUdpSender<TbotsProto::Primitive>>(
//                std::string(NETWORK_LOGGING_MULTICAST_CHANNELS[channel]) + "%" + interface, NETWORK_LOGS_PORT,
//                true);

        std::cout<<std::string(NETWORK_LOGGING_MULTICAST_CHANNELS[channel]) + "%" + interface<<std::endl;

//        LOG(INFO) << "logger listenting on channel and interface "<<channel<<" , "<<interface;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

//        for(int i = 0; i < 3; i++){
//            //log_output->sendProto(test_primitive_msg);
//
//        }
        while(true){
            //do nothing
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        // This blocks forever without using the CPU
        //std::promise<void>().get_future().wait();

    }

    return 0;
}
