#include <chrono>
#include <ctime>
#include <iostream>
#include <string>

#include "cpp_redis/core/client.hpp"
#include "software/test_util/test_util.h"

TEST(RedisKeyValueStoreTests, DISABLED_cpp_redis_get_test)
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


TEST(RedisKeyValueStoreTests, DISABLED_cpp_redis_get_and_set_speed_test)
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
    auto start             = std::chrono::system_clock::now();
    std::time_t start_time = std::chrono::system_clock::to_time_t(start);
    std::cout << "Started computation at " << std::ctime(&start_time);
    for (int i = 0; i <= 200; i++)
    {
        client.set("A", "1", [](cpp_redis::reply &reply) {});

        client.get("A", [](cpp_redis::reply &reply) {
            ASSERT_EQ(atoi(reply.as_string().c_str()), 1);
        });

        client.set("A", "2", [](cpp_redis::reply &reply) {});

        client.get("A", [](cpp_redis::reply &reply) {
            ASSERT_EQ(atoi(reply.as_string().c_str()), 2);
        });
        client.sync_commit();
    }
    auto end                                      = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);

    std::cout << "Finished computation at " << std::ctime(&end_time)
              << "Elapsed time: " << elapsed_seconds.count() << "s\n";
}
