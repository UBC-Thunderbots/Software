#include "udp_sender.h"

#include "software/networking/tbots_network_exception.h"
#include "software/networking/udp/network_utils.h"

UdpSender::UdpSender(boost::asio::io_service& io_service, const std::string& ip_address,
                     const unsigned short port, const std::string& interface,
                     bool multicast)
    : socket_(io_service), interface_(interface), ip_address_(ip_address)
{
    boost::asio::ip::address boost_ip = boost::asio::ip::make_address(ip_address);
    if (isIpv6(ip_address))
    {
        boost_ip = boost::asio::ip::make_address(ip_address + "%" + interface);
    }

    // The receiver endpoint identifies where this UdpSender will send data to
    receiver_endpoint = boost::asio::ip::udp::endpoint(boost_ip, port);

    socket_.open(receiver_endpoint.protocol());

    if (multicast)
    {
        setupMulticast(boost_ip, interface);
    }
}

std::string UdpSender::getInterface() const
{
    return interface_;
}

std::string UdpSender::getIpAddress() const
{
    return ip_address_;
}

void UdpSender::sendString(const std::string& message)
{
    socket_.send_to(boost::asio::buffer(message, message.length()), receiver_endpoint);
}

void UdpSender::sendStringAsync(const std::string& message)
{
    // The caller is responsible for keeping the buffer until the completion handler is
    // called. The trick here is to use a smart pointer to manage the buffer's lifetime
    // and associate it with the completion handler.
    std::shared_ptr<std::string> message_copy = std::make_shared<std::string>(message);
    socket_.async_send_to(
        boost::asio::buffer(*message_copy, message_copy->length()), receiver_endpoint,
        [message_copy](const boost::system::error_code&, std::size_t) {});
}

void UdpSender::setupMulticast(const boost::asio::ip::address& ip_address,
                               const std::string& interface)
{
    if (ip_address.is_v4())
    {
        std::optional<std::string> interface_ip = getLocalIp(interface);
        if (!interface_ip.has_value())
        {
            std::stringstream ss;
            ss << "UdpSender: Could not get the local IP address for the interface "
                  "specified. (interface = "
               << interface << ")";
            throw TbotsNetworkException(ss.str());
        }

        socket_.set_option(boost::asio::ip::multicast::join_group(
            ip_address.to_v4(),
            boost::asio::ip::address::from_string(interface_ip.value()).to_v4()));
        return;
    }

    socket_.set_option(boost::asio::ip::multicast::join_group(ip_address));
}


UdpSender::~UdpSender()
{
    socket_.close();
}
