#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "google/protobuf/wrappers.pb.h"
#include "software/networking/udp/proto_udp_bidrectional.hpp"

TEST(ProtoUdpBidirectionalTest, receives_a_request_and_replies)
{
    using Udp     = boost::asio::ip::udp;
    using Message = google::protobuf::StringValue;
    for (bool all_interfaces : {false, true})
    {
        boost::asio::io_service io;
        const auto loopback = boost::asio::ip::make_address("127.0.0.1");
        Udp::socket reservation(io, Udp::endpoint(loopback, 0));
        const auto port = reservation.local_endpoint().port();
        reservation.close();

        std::unique_ptr<ProtoUdpBidirectional<Message, Message>> server;
        int callbacks = 0;
        auto echo     = [&](Message& message, const Udp::endpoint& sender)
        {
            ++callbacks;
            server->sendTo(message, sender);
        };
        if (all_interfaces)
            server =
                std::make_unique<ProtoUdpBidirectional<Message, Message>>(io, port, echo);
        else
            server = std::make_unique<ProtoUdpBidirectional<Message, Message>>(
                io, "127.0.0.1", port, "", echo, false);

        Udp::socket client(io, Udp::endpoint(loopback, 0));
        client.send_to(boost::asio::buffer(std::string(1, '\xff')),
                       Udp::endpoint(loopback, port));
        Message request;
        request.set_value(std::string("hello\0udp", 9));
        client.send_to(boost::asio::buffer(request.SerializeAsString()),
                       Udp::endpoint(loopback, port));

        std::array<char, 128> reply;
        Udp::endpoint sender;
        bool received = false;
        client.async_receive_from(boost::asio::buffer(reply), sender,
                                  [&](const boost::system::error_code& error, size_t size)
                                  {
                                      EXPECT_FALSE(error) << error.message();
                                      Message response;
                                      EXPECT_TRUE(response.ParseFromArray(
                                          reply.data(), static_cast<int>(size)));
                                      EXPECT_EQ(response.value(), request.value());
                                      EXPECT_EQ(sender, Udp::endpoint(loopback, port));
                                      received = true;
                                      io.stop();
                                  });
        io.run_for(std::chrono::seconds(1));
        EXPECT_TRUE(received);
        EXPECT_EQ(callbacks, 1);
    }
}
