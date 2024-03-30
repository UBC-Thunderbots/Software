#include "software/networking/benchmarking_utils/latency_tester_node.h"
#include "shared/constants.h"

LatencyTesterNode::LatencyTesterNode(const std::string& interface, const int listen_channel,
                                     const unsigned short listen_port,
                                     const int send_channel,
                                     const unsigned short send_port,
                                     ReceiveCallback receive_callback)
    : io_listener_service_(),
      io_sender_service_()
{
    std::string listen_ip = ROBOT_MULTICAST_CHANNELS.at(listen_channel);
    listen_ip.append(1, '%').append(interface);
    listener_ = std::make_unique<UdpListener>(io_listener_service_, listen_ip, listen_port, true, receive_callback);

    std::string send_ip = ROBOT_MULTICAST_CHANNELS.at(send_channel);
    send_ip.append(1, '%').append(interface);
    sender_ = std::make_unique<UdpSender>(io_sender_service_, send_ip, send_port, true);

    listener_thread_ = std::thread([this]() { io_listener_service_.run(); });
    sender_thread_   = std::thread([this]() { io_sender_service_.run(); }); 
}

LatencyTesterNode::~LatencyTesterNode()
{
    // Stop the io_service. This is safe to call from another thread.
    // https://stackoverflow.com/questions/4808848/boost-asio-stopping-io-service
    // This MUST be done before attempting to join the thread because otherwise the
    // io_service will not stop and the thread will not join
    io_listener_service_.stop();
    io_sender_service_.stop();

    // Join the io_service_thread so that we wait for it to exit before destructing the
    // thread object. If we do not wait for the thread to
    // finish executing, it will call
    // `std::terminate` when we deallocate the thread object and kill our whole program
    listener_thread_.join();
    sender_thread_.join();
}

void LatencyTesterNode::sendString(const std::string& message)
{
    sender_->sendString(message);
}
