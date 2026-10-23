#include "software/networking/udp/udp_listener.h"

#include <boost/asio/post.hpp>

#include "software/logger/logger.h"
#include "software/networking/tbots_network_exception.h"
#include "software/networking/udp/network_utils.h"

UdpListener::UdpListener(boost::asio::io_context& io_context,
                         const std::string& ip_address, unsigned short port,
                         const std::string& listen_interface, bool multicast,
                         ReceiveCallback receive_callback)
    : state_(std::make_shared<ReceiveState>(io_context, std::move(receive_callback)))
{
    boost::asio::ip::address boost_ip = boost::asio::ip::make_address(ip_address);
    if (isIpv6(ip_address))
    {
        boost_ip = boost::asio::ip::make_address(ip_address + "%" + listen_interface);
    }
    boost::asio::ip::udp::endpoint listen_endpoint(boost_ip, port);
    state_->socket.open(listen_endpoint.protocol());
    state_->socket.set_option(boost::asio::socket_base::reuse_address(true));
    try
    {
        state_->socket.bind(listen_endpoint);
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
        throw TbotsNetworkException(ss.str());
    }

    if (multicast)
    {
        setupMulticast(state_->socket, boost_ip, listen_interface);
    }

    startListen(state_);
}

UdpListener::UdpListener(boost::asio::io_context& io_context, const unsigned short port,
                         ReceiveCallback receive_callback)
    : state_(std::make_shared<ReceiveState>(io_context, std::move(receive_callback)))
{
    boost::asio::ip::udp::endpoint listen_endpoint(boost::asio::ip::udp::v6(), port);
    state_->socket.open(listen_endpoint.protocol());
    // Explicitly set the v6_only option to be false to accept both ipv4 and ipv6 packets
    state_->socket.set_option(boost::asio::ip::v6_only(false));
    try
    {
        state_->socket.bind(listen_endpoint);
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
        throw TbotsNetworkException(ss.str());
    }

    startListen(state_);
}

void UdpListener::setupMulticast(boost::asio::ip::udp::socket& socket,
                                 const boost::asio::ip::address& ip_address,
                                 const std::string& listen_interface)
{
    if (ip_address.is_v4())
    {
        std::optional<std::string> interface_ip = getLocalIp(listen_interface, true);
        if (!interface_ip)
        {
            std::stringstream ss;
            ss << "Could not find the local ip address for the given interface: "
               << listen_interface << std::endl;
            throw TbotsNetworkException(ss.str());
        }
        socket.set_option(boost::asio::ip::multicast::join_group(
            ip_address.to_v4(),
            boost::asio::ip::address::from_string(interface_ip.value()).to_v4()));
        return;
    }
    socket.set_option(boost::asio::ip::multicast::join_group(ip_address));
}

UdpListener::~UdpListener()
{
    requestStop();
}

void UdpListener::close()
{
    requestStop();
    state_->shutdown_future.wait();
}

void UdpListener::requestStop()
{
    if (state_->stop_requested.exchange(true))
    {
        return;
    }

    auto state = state_;
    boost::asio::post(state->socket.get_executor(),
                      [state]()
                      {
                          state->stopping = true;

                          boost::system::error_code error;
                          state->socket.close(error);
                          if (error)
                          {
                              LOG(WARNING)
                                  << "UdpListener: Error closing UDP socket. The boost "
                                     "system error is: "
                                  << error.message() << std::endl;
                          }

                          maybeSignalShutdown(state);
                      });
}

void UdpListener::startListen(const std::shared_ptr<ReceiveState>& state)
{
    state->receive_in_flight.store(true);
    state->socket.async_receive_from(
        boost::asio::buffer(state->raw_received_data, MAX_BUFFER_LENGTH),
        state->sender_endpoint,
        [state](const boost::system::error_code& error,
                const std::size_t bytes_transferred)
        {
            state->receive_in_flight.store(false);
            handleDataReception(state, error, bytes_transferred);
        });
}

void UdpListener::handleDataReception(const std::shared_ptr<ReceiveState>& state,
                                      const boost::system::error_code& error,
                                      std::size_t num_bytes_received)
{
    if (state->stopping)
    {
        maybeSignalShutdown(state);
        return;
    }

    if (!error)
    {
        // Call the receive callback with the received data
        state->receive_callback(state->raw_received_data.data(), num_bytes_received);
    }
    else
    {
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

    // Start listening for more data
    startListen(state);
}

void UdpListener::maybeSignalShutdown(const std::shared_ptr<ReceiveState>& state)
{
    if (state->stopping && !state->receive_in_flight.load() && !state->shutdown_signalled)
    {
        state->shutdown_signalled = true;
        state->shutdown_promise.set_value();
    }
}
