#include "software/jetson_nano/redis/redis_client.h"

#include <chrono>
#include <ctime>
#include <iostream>
#include <string>

#include "cpp_redis/core/client.hpp"
#include "cpp_redis/core/subscriber.hpp"
#include "software/test_util/test_util.h"

// TODO #2805: Convert this into a test fixture so that we can start and run a REDIS
// server during the lifetime of the test fixture. Then re-enable all tests

TEST(RedisKeyValueStoreTests, DISABLED_cpp_redis_get_test)
{
    cpp_redis::client client;
    client.connect(
        REDIS_DEFAULT_HOST, REDIS_DEFAULT_PORT,
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
        REDIS_DEFAULT_HOST, REDIS_DEFAULT_PORT,
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
    }
    auto end                                      = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);

    std::cout << "Finished computation at " << std::ctime(&end_time)
              << "Elapsed time: " << elapsed_seconds.count() << "s\n";
}


TEST(RedisImplTests, DISABLED_redis_client_get_and_set_impl_test)
{
    auto redis = RedisClient("127.0.0.1", 6379);

    cpp_redis::client client;
    client.connect(
        REDIS_DEFAULT_HOST, REDIS_DEFAULT_PORT,
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

    for (int i = 0; i <= 20; i++)
    {
        redis.setSync(std::to_string(i), "test_value_new");
        auto reply = redis.getSync(std::to_string(i));
        ASSERT_EQ(reply, "test_value_new");
    }

    auto end                                      = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);

    ASSERT_LE(elapsed_seconds.count() * 1000, 25);
    ASSERT_EQ(redis.getAllKeyValuePairs().size(), 20);

    std::cout << "Finished computation at " << std::ctime(&end_time)
              << "Elapsed time: " << elapsed_seconds.count() << "s\n";
}
