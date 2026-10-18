#include "software/networking/udp/threaded_proto_udp_bidirectional.hpp"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "google/protobuf/wrappers.pb.h"

TEST(ThreadedProtoUdpBidirectionalTest, replies_from_callback_and_calling_thread)
{
    using Udp      = boost::asio::ip::udp;
    using Request  = google::protobuf::StringValue;
    using Response = google::protobuf::BytesValue;
    using Server   = ThreadedProtoUdpBidirectional<Request, Response>;
    for (bool all_interfaces : {false, true})
    {
        boost::asio::io_service io;
        const auto loopback = boost::asio::ip::make_address("127.0.0.1");
        Udp::socket reservation(io, Udp::endpoint(loopback, 0));
        const auto port = reservation.local_endpoint().port();
        reservation.close();

        std::promise<Udp::endpoint> received;
        auto result = received.get_future();
        std::unique_ptr<Server> server;
        auto echo = [&](Request& request, const Udp::endpoint& sender)
        {
            Response response;
            response.set_value(request.value());
            server->sendTo(response, sender);
            received.set_value(sender);
        };
        if (all_interfaces)
        {
            server = std::make_unique<Server>(port, echo);
        }
        else
        {
            server = std::make_unique<Server>("127.0.0.1", port, "", echo, false);
        }

        Udp::socket client(io, Udp::endpoint(loopback, 0));
        Request request;
        request.set_value(std::string("hello\0udp", 9));
        client.send_to(boost::asio::buffer(request.SerializeAsString()),
                       Udp::endpoint(loopback, port));
        ASSERT_EQ(result.wait_for(std::chrono::seconds(2)), std::future_status::ready);

        Response extra_reply;
        extra_reply.set_value("second reply");
        server->sendTo(extra_reply, result.get());
        client.non_blocking(true);
        for (const auto& expected : {request.value(), extra_reply.value()})
        {
            std::array<char, 128> buffer;
            Udp::endpoint sender;
            boost::system::error_code error;
            const auto size =
                client.receive_from(boost::asio::buffer(buffer), sender, 0, error);
            ASSERT_FALSE(error) << error.message();
            Response reply;
            ASSERT_TRUE(reply.ParseFromArray(buffer.data(), static_cast<int>(size)));
            EXPECT_EQ(reply.value(), expected);
            EXPECT_EQ(sender, Udp::endpoint(loopback, port));
        }
        server->close();
        server->close();
    }
}
