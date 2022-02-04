#include "redis.h"


RedisService::RedisService(std::string host, size_t port)
        : subscriber(), client(), host_(host), port_(port) {
    subscriber.connect(
            host_, port_,
            [](const std::string &host, std::size_t port, cpp_redis::connect_state status) {
                if (status == cpp_redis::connect_state::dropped) {
                    LOG(WARNING) << "Subscriber connection dropped";
                } else if (status == cpp_redis::connect_state::failed) {
                    LOG(WARNING) << "Subscriber connection failed";
                } else if (status == cpp_redis::connect_state::ok) {
                    LOG(INFO) << "Subscriber connection successful";
                }
            });
    client.connect(
            host_, port_,
            [](const std::string &host, std::size_t port, cpp_redis::connect_state status) {
                if (status == cpp_redis::connect_state::dropped) {
                    LOG(WARNING) << "Client connection dropped";
                } else if (status == cpp_redis::connect_state::failed) {
                    LOG(WARNING) << "Client connection failed";
                } else if (status == cpp_redis::connect_state::ok) {
                    LOG(INFO) << "Client connection successful";
                }
            });
}

void RedisService::subscribe(const std::string &channel,
                             void (*subscribe_callback)(std::string, std::string)) {
    subscriber.subscribe(channel, subscribe_callback);
}
// blocking
cpp_redis::reply RedisService::get(const std::string &key) {
    auto future = client.get(key);
    client.commit();
    return future.get();
}

// non blocking
std::optional<cpp_redis::reply> RedisService::poll(const std::string &key) {
    auto future = client.get(key);
    std::chrono::system_clock::time_point zero_seconds = std::chrono::system_clock::now() + std::chrono::seconds(0);
    std::future_status status = future.wait_until(zero_seconds);
    if (status == std::future_status::ready) {
        return future.get();
    }
    return {};
}

void RedisService::set(const std::string &key, const std::string &value) {
    auto future = client.set(key, value);
    client.commit();
}


