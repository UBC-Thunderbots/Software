#include "redis.h"


RedisService::RedisService(std::string host, size_t port)
        : subscriber(), client(), host_(host), port_(port) {
    subscriber.connect(
            host_, port_,
            [](const std::string &host, std::size_t port, cpp_redis::connect_state status) {
                if (status == cpp_redis::connect_state::dropped) {
                    LOG(WARNING) << "Redis subscriber connection dropped";
                } else if (status == cpp_redis::connect_state::failed) {
                    LOG(WARNING) << "Redis subscriber connection failed";
                } else if (status == cpp_redis::connect_state::ok) {
                    LOG(INFO) << "Redis subscriber connection successful";
                }
            });
    client.connect(
            host_, port_,
            [](const std::string &host, std::size_t port, cpp_redis::connect_state status) {
                if (status == cpp_redis::connect_state::dropped) {
                    LOG(WARNING) << "Redis client connection dropped";
                } else if (status == cpp_redis::connect_state::failed) {
                    LOG(WARNING) << "Redis client connection failed";
                } else if (status == cpp_redis::connect_state::ok) {
                    LOG(INFO) << "Redis client connection successful";
                }
            });

    // ensure that keyspace events are configured properly
    client.config_get("notify-keyspace-events", [this](cpp_redis::reply &reply) {
        if (reply.as_array()[1].as_string().empty()) {
            LOG(INFO) << "Redis keyspace event notifications are disabled. Enabling.";
            std::cout << "Redis keyspace event notifications are disabled. Enabling." << std::endl;
            client.config_set("notify-keyspace-events", "KEA");
            client.commit();
        }
    }).commit();

    // ensure that redis server is accepting all connection's
//    client.config_get("bind", [this](cpp_redis::reply &reply) {
//        if (reply.as_array()[1].as_string().empty()) {
//            LOG(INFO) << "Redis keyspace event notifications are disabled. Enabling.";
//            client.config_set("bind", "0.0.0.0");
//            client.commit();
//        }
//    }).commit();

    // subscribe to key 'set' event within the keyspace
    // adds key and its value to the key value set
    subscriber.subscribe("__keyevent@0__:set", [this](const std::string &channel, const std::string &key) {
//        std::cout << "BRUH" << std::endl;
        client.get(key, [this, key](cpp_redis::reply &value) {
            key_value_set_[key] = value.as_string();
            std::cout << key << " : " << value.as_string().c_str() << std::endl;
        });
        client.commit();
    });
    subscriber.commit();
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

void RedisService::set(const std::string &key, const std::string &value) {
    client.set(key, value);
    client.sync_commit();
}


void RedisService::start() {};

void RedisService::stop() {};

RedisService::~RedisService() {}




