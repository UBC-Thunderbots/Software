#include "software/networking/udp/udp_listener.h"

#include "software/networking/udp/network_utils.h"
#include "software/logger/logger.h"

UdpListener::UdpListener(boost::asio::io_service& io_service,
                         const std::string& ip_address, unsigned short port,
                         const std::string& listen_interface,
                         bool multicast, ReceiveCallback receive_callback,
                         std::optional<std::string>& error)
    : running_(true), socket_(io_service), receive_callback_(receive_callback)
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

UdpListener::UdpListener(boost::asio::io_service& io_service, const unsigned short port,
                         ReceiveCallback receive_callback, std::optional<std::string>& error)
    : running_(true), socket_(io_service), receive_callback_(receive_callback)
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

void UdpListener::setupMulticast(
    const boost::asio::ip::address& ip_address, const std::string& listen_interface,
    std::optional<std::string>& error)
{
    if (ip_address.is_v4())
    {
        std::optional<std::string> interface_ip = getLocalIp(listen_interface, true);
        if (!interface_ip)
        {
            std::stringstream ss;
            ss << "Could not find the local ip address for the given interface: "
               << listen_interface << std::endl;
            error = ss.str();
            return;
        }
        socket_.set_option(boost::asio::ip::multicast::join_group(
            ip_address.to_v4(),
            boost::asio::ip::address::from_string(interface_ip.value()).to_v4()));
        return;
    }
    socket_.set_option(boost::asio::ip::multicast::join_group(ip_address));
}

UdpListener::~UdpListener()
{
}

void UdpListener::close()
{
    running_ = false;

    // Shutdown both the read and write on the socket
    boost::system::error_code error;
    socket_.shutdown(boost::asio::ip::udp::socket::shutdown_both, error);

    if (error)
    {
        LOG(WARNING)
            << "An unknown network error occurred when attempting to close UDP socket. The boost system error is: "
            << error.message() << std::endl;
    }
}

void UdpListener::startListen()
{
    // Start listening for data asynchronously
    // See here for a great explanation about asynchronous operations:
    // https://stackoverflow.com/questions/34680985/what-is-the-difference-between-asynchronous-programming-and-multithreading
    socket_.async_receive_from(boost::asio::buffer(raw_received_data_, MAX_BUFFER_LENGTH),
                               sender_endpoint_,
                               boost::bind(&UdpListener::handleDataReception, this,
                                           boost::asio::placeholders::error,
                                           boost::asio::placeholders::bytes_transferred));
}

void UdpListener::handleDataReception(const boost::system::error_code& error,
                                      std::size_t num_bytes_received)
{
    if (!running_)
    {
        return;
    }

    if (!error)
    {
        // Call the receive callback with the received data
        receive_callback_(raw_received_data_.data(), num_bytes_received);

        // Start listening for more data
        startListen();
    }
    else
    {
        // Start listening again to receive the next data
        startListen();

        LOG(WARNING) << "UdpListener: Error receiving data: " << error.message()
                     << std::endl;
    }

    if (num_bytes_received > MAX_BUFFER_LENGTH)
    {
        LOG(WARNING)
            << "num_bytes_received > MAX_BUFFER_LENGTH, "
            << "which means that the receive buffer is full and data loss has potentially occurred. "
            << "Consider increasing MAX_BUFFER_LENGTH";
    }
}
