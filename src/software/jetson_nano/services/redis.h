#pragma once

#include "cpp_redis/cpp_redis"
#include "software/jetson_nano/services/service.h"
#include "string"
#include "chrono"
#include "unordered_map"
#include "software/logger/logger.h"
#include "shared/constants.h"


class RedisService : public Service {
public:

    virtual void start() override;

    virtual void stop() override;

    void poll();

    /**
     * Service that communicates with various external services
     * @param value The IP of the Redis server, default localhost
     * @param key the key of the Redis server, default 6379
     */
    explicit RedisService(std::string value, size_t key);

    /**
     * Subscribes to a message channel
     * @param channel The channel being subscribed to
     * @param subscribe_callback Callback for when messages come through channel
     */
    void subscribe(const std::string &channel,
                   void (*subscribe_callback)(std::string, std::string));

    /**
     * gets the value corresponding to the key; blocking
     * @param key
     * @return
     */
    cpp_redis::reply get(const std::string &key);

//    /**
//     * polls redis service for value corresponding to key; non-blocking
//     * @param key
//     * @return
//     */
//    std::optional<cpp_redis::reply> poll(const std::string &key);

    /**
     * sets a key value pair in the redis database
     * @param key
     * @param value
     */
    void set(const std::string &key, const std::string &value);


    virtual ~RedisService();

    std::unordered_map<std::string, std::string> key_value_set_;



private:
    cpp_redis::subscriber subscriber;
    cpp_redis::client client;
    // Connection Parameters
    std::string host_;
    size_t port_;
};
