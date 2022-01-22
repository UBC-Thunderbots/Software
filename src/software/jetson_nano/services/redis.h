#pragma once

#include "cpp_redis/cpp_redis"
#include "software/jetson_nano/services/service.h"
#include "string"


class RedisService : public Service {
public:
    explicit RedisService(std::string host, size_t port);

    virtual void start() override;

    virtual void stop() override;

    void subscribe(const std::string &channel,
                   void (*subscribe_callback)(std::string, std::string));

    cpp_redis::reply set(const std::string &key, const std::string &value);

    cpp_redis::reply get(const std::string &key);

private:
    cpp_redis::subscriber subscriber;
    cpp_redis::client client;
    std::string host_;
    size_t port_;
};
