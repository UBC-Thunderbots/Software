#include <gtest/gtest.h>

#include <iostream>
#include <string>

#include "cpp_redis/core/client.hpp"


TEST(ServerTest, Get)
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

    client.set("hello", "42", [](cpp_redis::reply &reply) {
        std::cout << "set hello 42: " << reply << std::endl;
    });

    client.decrby("hello", 12, [](cpp_redis::reply &reply) {
        std::cout << "decrby hello 12: " << reply << std::endl;
    });

    client.get("hello", [](cpp_redis::reply &reply) {
        std::cout << "get hello: " << reply << std::endl;
    });

    client.sync_commit();
}
