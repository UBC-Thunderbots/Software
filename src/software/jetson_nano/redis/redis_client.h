#pragma once

#include "chrono"
#include "cpp_redis/cpp_redis"
#include "shared/constants.h"
#include "software/logger/logger.h"
#include "string"
#include "unordered_map"


class RedisClient
{
   public:
    /**
     * Client that communicates with the a redis server
     * @param value The IP of the Redis server, default localhost
     * @param key the key of the Redis server, default 6379
     */
    explicit RedisClient(std::string value, size_t key);

    virtual ~RedisClient();


    /**
     * Subscribes to a message channel
     * @param channel The channel being subscribed to
     * @param subscribe_callback callback for when messages are published to channel
     */
    void subscribe(const std::string &channel,
                   void (*subscribe_callback)(std::string, std::string));

    /**
     * gets the value corresponding to the key; blocking
     * @param key
     * @return a redis reply object
     */
    cpp_redis::reply get(const std::string &key);

    /**
     * sets a key value pair in the redis database
     * @param key
     * @param value
     */
    void set(const std::string &key, const std::string &value);

    std::unordered_map<std::string, std::string> getAllKeyValuePairs();

   private:
    cpp_redis::subscriber subscriber_;
    cpp_redis::client client_;
    std::unordered_map<std::string, std::string> key_value_set_;
    // Connection Parameters
    std::string host_;
    size_t port_;
};
