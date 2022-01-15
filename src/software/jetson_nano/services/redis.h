#pragma once

#include "cpp_redis/cpp_redis"
#include "software/jetson_nano/services/service.h"
#include "string"



class RedisService : public Service
{
   public:
    explicit RedisService(std::string host, size_t port);

    virtual void start() override;

    virtual void stop() override;

    void subscribe(const std::string &channel,
                   void (*subscribe_callback)(std::string, std::string));


   private:
    cpp_redis::subscriber subscriber;
    std::string host;
    size_t port;
};
