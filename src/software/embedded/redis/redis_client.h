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
    std::string getSync(const std::string &key);

    /**
     * Gets the value corresponding to the key, non blocking - executes the callback once
     * a reply is received.
     *
     * @param key
     * @param reply_callback callback function once the value is obtained
     */
    void getAsync(const std::string &key,
                  const cpp_redis::reply_callback_t &reply_callback);

    /**
     * Gets the value corresponding to the key, asynchronously but doesn't commit it. Must
     * manually call asyncCommit() to commit it to the REDIS server.
     *
     * Allows us to batch call a number of GET/SET requests to the REDIS server with one
     * network request.
     *
     * @param key
     * @param reply_callback callback function once the value is obtained
     */
    void getAsyncNoCommit(const std::string &key,
                          const cpp_redis::reply_callback_t &reply_callback);

    /**
     * Sets a key value pair in the redis database asynchronously
     *
     * @param key
     * @param value
     */
    void setAsync(const std::string &key, const std::string &value);

    /**
     * Gets the value corresponding to the key; blocking synchronously
     *
     * @param key
     * @param the value
     */
    void setSync(const std::string &key, const std::string &value);

    /**
     * Sets a key value pair in the redis database but does not commit it. Waits for the
     * next commit() to send it to the REDIS server.
     *
     * Allows us to batch call a number of GET/SET requests to the REDIS server with one
     * network request.
     *
     * @param key   key for key-value pair
     * @param value value to set to key-value pair
     */
    void setNoCommit(const std::string &key, const std::string &value);

    /**
     * Asynchronously commit all pending requests since last commit.
     */
    void asyncCommit();

    /**
     * Synchronously commit all pending requests since last commit.
     */
    void syncCommit();

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
