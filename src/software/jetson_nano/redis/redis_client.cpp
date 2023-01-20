#include "software/jetson_nano/redis/redis_client.h"


RedisClient::RedisClient(std::string host, size_t port)
    : subscriber_(), client_(), host_(host), port_(port)
{
    auto connection_callback = [](const std::string &host, std::size_t port,
                                  cpp_redis::connect_state status) {
        if (status == cpp_redis::connect_state::dropped)
        {
            LOG(WARNING) << "Redis subscriber_ connection dropped";
        }
        else if (status == cpp_redis::connect_state::failed)
        {
            LOG(WARNING) << "Redis subscriber_ connection failed";
        }
        else if (status == cpp_redis::connect_state::ok)
        {
            LOG(INFO) << "Redis subscriber_ connection successful";
        }
    };

    subscriber_.connect(host_, port_, connection_callback);
    client_.connect(host_, port_, connection_callback);

    // ensure that keyspace events are configured properly
    client_.config_set("notify-keyspace-events", "KEA");

    // ensure that redis server is accepting connections from any host (potentially
    // unsafe)
    client_.config_set("bind", "0.0.0.0");

    // ensure that redis server has AOF (append-only file) persistence
    client_.config_set("appendonly", "yes");

    // subscribe to key 'set' event within the keyspace
    // adds key and its value to the key value set
    subscriber_.subscribe("__keyevent@0__:set",
                          [this](const std::string &channel, const std::string &key) {
                              client_.get(key, [this, key](cpp_redis::reply &value) {
                                  key_value_set_[key] = value.as_string();
                              });
                              client_.commit();
                          });
    subscriber_.commit();
}

std::string RedisClient::get(const std::string &key)
{
    auto future = client_.get(key);
    client_.commit();
    return future.get().as_string();
}

void RedisClient::set(const std::string &key, const std::string &value)
{
    client_.set(key, value);
    client_.sync_commit();
}

std::unordered_map<std::string, std::string> RedisClient::getAllKeyValuePairs()
{
    return key_value_set_;
}

RedisClient::~RedisClient() {}
