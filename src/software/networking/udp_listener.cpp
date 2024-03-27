#include "software/networking/udp_listener.h"

#include "software/logger/logger.h"

UdpListener::UdpListener(boost::asio::io_service& io_service, const std::string& ip_address,
                         unsigned short port, bool multicast, ReceiveCallback receive_callback)
    : running_(true),
      socket_(io_service),
      receive_callback(receive_callback)
{
    boost::asio::ip::address listen_address = boost::asio::ip::make_address(ip_address);

    boost::asio::ip::udp::endpoint listen_endpoint(listen_address, port);
    socket_.open(listen_endpoint.protocol());
    socket_.set_option(boost::asio::socket_base::reuse_address(true));

    try
    {
        socket_.bind(listen_endpoint);
    }
    catch (const boost::exception& ex)
    {
        LOG(FATAL) << "UdpListener: There was an issue binding the socket to the listen_endpoint when trying to connect to the address. This may be due to another instance of the UdpListener running and using the port already. (ip = " << ip_address << ", port = " << port << ")" << std::endl;
    }

    // If we are using multicast, join the multicast group
    if (multicast)
    {
        socket_.set_option(listen_address);
    }

    startListen();
}

UdpListener::UdpListener(boost::asio::io_service& io_service, const unsigned short port, ReceiveCallback receive_callback)
    : running_(true),
      socket_(io_service),
      receive_callback(receive_callback)
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
        LOG(FATAL) << "UdpListener: There was an issue binding the socket to the listen_endpoint when trying to connect to the address. This may be due to another instance of the UdpListener running and using the port already. (port = " << port << ")" << std::endl;
    }

    startListen();
}

UdpListener::~UdpListener()
{
    close();
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
            << "An unknown network error occurred when attempting to close UDP socket. The boost system error is: " <<
            error_code.message() << std::endl;
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

void UdpListener::handleDataReception(const boost::system::error_code& error, std::size_t num_bytes_received)
{
    if (!running_)
    {
        return;
    }

    if (!error)
    {
        // Call the receive callback with the received data
        receive_callback(std::string(raw_received_data_, num_bytes_received));

        // Start listening for more data
        startListen();
    }
    else
    {
        // Start listening again to receive the next data
        startListen();

        LOG(ERROR) << "UdpListener: Error receiving data: " << error.message() << std::endl;
    }

    if (num_bytes_received > MAX_BUFFER_LENGTH)
    {
        LOG(WARNING)
            << "num_bytes_received > MAX_BUFFER_LENGTH, "
            << "which means that the receive buffer is full and data loss has potentially occurred. "
            << "Consider increasing MAX_BUFFER_LENGTH";
    }
}
