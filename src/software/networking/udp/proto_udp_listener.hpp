#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

#include "software/logger/logger.h"
#include "software/networking/udp/network_utils.h"
#include "software/networking/udp/proto_udp_listener.hpp"
#include "software/util/typename/typename.h"

template <class ReceiveProtoT>
class ProtoUdpListener
{
   public:
    /**
     * Creates an ProtoUdpListener that will listen for ReceiveProtoT packets from
     * the network on the multicast group of given address and port. For every
     * ReceiveProtoT packet received, the receive_callback will be called to perform any
     * operations desired by the caller
     *
     * The caller must check that the error is not set before using the listener
     *
     * @param io_service The io_service to use to service incoming ReceiveProtoT data
     * @param ip_address The ip address of on which to listen for the given ReceiveProtoT
     * packets (IPv4 in dotted decimal or IPv6 in hex string) example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8
     * @param port The port on which to listen for ReceiveProtoT packets
     * @param listen_interface The interface to listen on
     * @param receive_callback The function to run for every ReceiveProtoT packet received
     * from the network
     * @param multicast If true, joins the multicast group of given ip_address
     * @param error A user-provided optional string to store any error messages
     */
    ProtoUdpListener(boost::asio::io_service& io_service, const std::string& ip_address,
                     unsigned short port, const std::string& listen_interface,
                     std::function<void(ReceiveProtoT&)> receive_callback, bool multicast,
                     std::optional<std::string>& error);

    /**
     * Creates an ProtoUdpListener that will listen for ReceiveProtoT packets from
     * the network on any local address with given port. For every ReceiveProtoT packet
     * received, the receive_callback will be called to perform any operations desired by
     * the caller
     *
     * The caller must check that the error is not set before using the listener
     *
     * @param io_service The io_service to use to service incoming ReceiveProtoT data
     * @param port The port on which to listen for ReceiveProtoT packets
     * @param receive_callback The function to run for every ReceiveProtoT packet received
     * from the network
     * @param error A user-provided optional string to store any error messages
     */
    ProtoUdpListener(boost::asio::io_service& io_service, unsigned short port,
                     std::function<void(ReceiveProtoT&)> receive_callback,
                     std::optional<std::string>& error);
    /**
     * Closes the socket associated to the UDP listener
     */
    virtual void close();

    virtual ~ProtoUdpListener();


   private:
    /**
     * This function is setup as the callback to handle packets received over the network.
     *
     * @param error The error code obtained when receiving the incoming data
     * @param num_bytes_received How many bytes of data were received
     */
    void handleDataReception(const boost::system::error_code& error,
                             size_t num_bytes_received);

    /**
     * Sets up multicast for the given ip_address and listen_interface
     *
     * Any errors during setup will be stored in the error string
     *
     * @param ip_address The ip address of the multicast group to join
     * @param listen_interface The interface to listen on
     * @param error A user-provided optional string to store any error messages
     */
    void setupMulticast(const boost::asio::ip::address& ip_address,
                        const std::string& listen_interface,
                        std::optional<std::string>& error);

    /**
     * Start listening for data
     */
    void startListen();

    // A UDP socket that we listen on for ReceiveProtoT messages from the network
    boost::asio::ip::udp::socket socket_;
    // The endpoint for the sender
    boost::asio::ip::udp::endpoint sender_endpoint_;

    static constexpr unsigned int MAX_BUFFER_LENGTH = 9000;
    std::array<char, MAX_BUFFER_LENGTH> raw_received_data_;

    // The function to call on every received packet of ReceiveProtoT data
    std::function<void(ReceiveProtoT&)> receive_callback;

    // Whether or not the listener is running
    bool running_ = true;
};

template <class ReceiveProtoT>
ProtoUdpListener<ReceiveProtoT>::ProtoUdpListener(
    boost::asio::io_service& io_service, const std::string& ip_address,
    const unsigned short port, const std::string& listen_interface,
    std::function<void(ReceiveProtoT&)> receive_callback, bool multicast,
    std::optional<std::string>& error)
    : socket_(io_service), receive_callback(receive_callback)
{
    boost::asio::ip::address boost_ip = boost::asio::ip::make_address(ip_address);
    if (isIpv6(ip_address))
    {
        boost_ip = boost::asio::ip::make_address(ip_address + "%" + listen_interface);
    }
    boost::asio::ip::udp::endpoint listen_endpoint(boost_ip, port);
    socket_.open(listen_endpoint.protocol());
    socket_.set_option(boost::asio::socket_base::reuse_address(true));
    try
    {
        socket_.bind(listen_endpoint);
    }
    catch (const boost::exception& ex)
    {
        std::stringstream ss;
        ss << "UdpListener: There was an issue binding the socket to "
              "the listen_endpoint when trying to connect to the "
              "address. This may be due to another instance of the "
              "UdpListener running and using the port already. "
              "(ip = "
           << ip_address << ", port = " << port << ")";
        error = ss.str();
        return;
    }

    if (multicast)
    {
        setupMulticast(boost_ip, listen_interface, error);
        if (error)
        {
            return;
        }
    }

    startListen();
}

template <class ReceiveProtoT>
ProtoUdpListener<ReceiveProtoT>::ProtoUdpListener(
    boost::asio::io_service& io_service, const unsigned short port,
    std::function<void(ReceiveProtoT&)> receive_callback,
    std::optional<std::string>& error)
    : socket_(io_service), receive_callback(receive_callback)
{
    boost::asio::ip::udp::endpoint listen_endpoint(boost::asio::ip::udp::v6(), port);
    socket_.open(listen_endpoint.protocol());
    // Explicitly set the v6_only option to be false to accept both ipv4 and ipv6 packets
    socket_.set_option(boost::asio::ip::v6_only(false));
    try
    {
        socket_.bind(listen_endpoint);
    }
    catch (const boost::exception& ex)
    {
        std::stringstream ss;
        ss << "UdpListener: There was an issue binding the socket to "
              "the listen_endpoint when trying to connect to the "
              "address. This may be due to another instance of the "
              "UdpListener running and using the port already. "
              "(port = "
           << port << ")";
        error = ss.str();
        return;
    }

    startListen();
}

template <class ReceiveProtoT>
void ProtoUdpListener<ReceiveProtoT>::startListen()
{
    // Start listening for data asynchronously
    // See here for a great explanation about asynchronous operations:
    // https://stackoverflow.com/questions/34680985/what-is-the-difference-between-asynchronous-programming-and-multithreading
    socket_.async_receive_from(boost::asio::buffer(raw_received_data_, MAX_BUFFER_LENGTH),
                               sender_endpoint_,
                               boost::bind(&ProtoUdpListener::handleDataReception, this,
                                           boost::asio::placeholders::error,
                                           boost::asio::placeholders::bytes_transferred));
}

template <class ReceiveProtoT>
void ProtoUdpListener<ReceiveProtoT>::handleDataReception(
    const boost::system::error_code& error, size_t num_bytes_received)
{
    if (!running_)
    {
        return;
    }

    if (!error)
    {
        auto packet_data = ReceiveProtoT();
        packet_data.ParseFromArray(raw_received_data_.data(),
                                   static_cast<int>(num_bytes_received));
        receive_callback(packet_data);
        // Once we've handled the data, start listening again
        startListen();
    }
    else
    {
        LOG(WARNING) << "An unknown network error occurred when attempting to receive "
                     << TYPENAME(ReceiveProtoT)
                     << " Data. The boost system error is: " << error.message()
                     << std::endl;

        if (!running_)
        {
            return;
        }

        // Start listening again to receive the next data
        startListen();
    }

    if (num_bytes_received > MAX_BUFFER_LENGTH)
    {
        LOG(WARNING)
            << "num_bytes_received > MAX_BUFFER_LENGTH, "
            << "which means that the receive buffer is full and data loss has potentially occurred. "
            << "Consider increasing MAX_BUFFER_LENGTH";
    }
}

template <class ReceiveProtoT>
void ProtoUdpListener<ReceiveProtoT>::setupMulticast(
    const boost::asio::ip::address& ip_address, const std::string& listen_interface,
    std::optional<std::string>& error)
{
    if (ip_address.is_v4())
    {
        std::string interface_ip;
        if (!getLocalIp(listen_interface, interface_ip))
        {
            std::stringstream ss;
            ss << "Could not find the local ip address for the given interface: "
               << listen_interface << std::endl;
            error = ss.str();
            return;
        }
        socket_.set_option(boost::asio::ip::multicast::join_group(
            ip_address.to_v4(),
            boost::asio::ip::address::from_string(interface_ip).to_v4()));
        return;
    }
    socket_.set_option(boost::asio::ip::multicast::join_group(ip_address));
}

template <class ReceiveProtoT>
ProtoUdpListener<ReceiveProtoT>::~ProtoUdpListener()
{
}

template <class ReceiveProtoT>
void ProtoUdpListener<ReceiveProtoT>::close()
{
    running_ = false;

    // Shutdown both send and receive on the socket
    boost::system::error_code error_code;
    socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, error_code);
    if (error_code)
    {
        LOG(WARNING)
            << "An unknown network error occurred when attempting to shutdown UDP socket for "
            << TYPENAME(ReceiveProtoT) << ". The boost system error is: " << error_code
            << ": " << error_code.message() << std::endl;
    }

    socket_.close(error_code);
    if (error_code)
    {
        LOG(WARNING)
            << "An unknown network error occurred when attempting to close UDP socket for "
            << TYPENAME(ReceiveProtoT)
            << ". The boost system error is: " << error_code.message() << std::endl;
    }
}
