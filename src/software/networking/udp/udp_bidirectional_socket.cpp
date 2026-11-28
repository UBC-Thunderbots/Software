#include "software/networking/udp/udp_bidirectional_socket.h"

#include "software/logger/logger.h"

void UdpBidirectionalSocket::sendTo(const std::string& message,
                                    const boost::asio::ip::udp::endpoint& destination)
{
    auto buf = std::make_shared<std::string>(message);
    // sockets aren't thread-safe, so the actual send needs to happen on the io_context
    // thread
    boost::asio::post(socket_.get_executor(),
                      [this, buf, destination]() { doSend(buf, destination); });
}

void UdpBidirectionalSocket::doSend(std::shared_ptr<std::string> buf,
                                    const boost::asio::ip::udp::endpoint& destination)
{
    socket_.async_send_to(boost::asio::buffer(*buf), destination,
                          [buf](const boost::system::error_code& ec, std::size_t)
                          {
                              // this callback runs on io_context post-send. `buf` is
                              // captured by this, which means that the shared pointer
                              // lives right up until this lambda finishes running.
                              if (ec)
                              {
                                  LOG(WARNING)
                                      << "send failed from UdpBidirectionalSocket: "
                                      << ec.message();
                              }
                          });
}
