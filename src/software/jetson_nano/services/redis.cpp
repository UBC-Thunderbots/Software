#include "redis.h"

#include "software/logger/logger.h"

RedisService::RedisService(std::string host, size_t port)
        : subscriber(), host_(host), port_(port) {
}

void RedisService::start() {
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

void RedisService::stop() {
    subscriber.disconnect(false);
}

void RedisService::subscribe(const std::string &channel,
                             void (*subscribe_callback)(std::string, std::string)) {
    subscriber.subscribe(channel, subscribe_callback);
}

cpp_redis::reply RedisService::get(const std::string &key) {
    auto future = client.get(key);
    client.commit();
    return future.get();
}

cpp_redis::reply RedisService::set(const std::string &key, const std::string &value) {
    auto future = client.set(key, value);
    client.commit();
    return future.get();
}


