#include <iostream>
#include <string>

#include "cpp_redis/core/client.hpp"
#include "software/test_util/test_util.h"

TEST(ServerTest, DISABLED_Get)
{
    cpp_redis::client client;
    client.connect(
        "127.0.0.1", 6379,
        [](const std::string &host, std::size_t port, cpp_redis::connect_state status) {
            if (status == cpp_redis::connect_state::dropped)
            {
                std::cout << "client disconnected from " << host << ":" << port
                          << std::endl;
            }
        });

    client.set("A", "1", [](cpp_redis::reply &reply) {
        std::cout << "Setting A to 1: " << reply << std::endl;
    });

    client.get("A", [](cpp_redis::reply &reply) {
        std::cout << "Getting A: " << reply << std::endl;
        ASSERT_EQ(atoi(reply.as_string().c_str()), 1);
    });

    client.set("A", "2", [](cpp_redis::reply &reply) {
        std::cout << "Setting A to 2: " << reply << std::endl;
    });

    client.get("A", [](cpp_redis::reply &reply) {
        std::cout << "Getting A: " << reply << std::endl;
        ASSERT_EQ(atoi(reply.as_string().c_str()), 2);
    });
    client.sync_commit();
}
