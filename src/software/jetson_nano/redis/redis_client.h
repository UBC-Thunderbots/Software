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
     * @param address The IP of the Redis server, default localhost
     * @param port the key of the Redis server, default 6379
     */
    explicit RedisClient(std::string address, size_t port);

    virtual ~RedisClient();


    /**
     * Gets the value corresponding to the key; blocking
     *
     * @param key
     * @return the value
     */
    std::string get(const std::string &key);

    /**
     * Sets a key value pair in the redis database
     *
     * @param key
     * @param value
     */
    void set(const std::string &key, const std::string &value);

    /**
     * @return a map of all the key value pairs in redis server
     */
    std::unordered_map<std::string, std::string> getAllKeyValuePairs();

   private:
    cpp_redis::subscriber subscriber_;
    cpp_redis::client client_;
    std::unordered_map<std::string, std::string> key_value_set_;
    // Connection Parameters
    std::string host_;
    size_t port_;
};
