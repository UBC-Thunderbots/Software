#pragma once

#include "cpp_redis/cpp_redis"
#include "cpp_redis/core/client.hpp"


class Redis : public Service {
public:

    explicit Redis();

    void start() override;

    void stop() override;

    void subscribe(const std::string &channel,
                   void (*subscribe_callback)(std::string, std::string));


private:
    cpp_redis::subscriber subscriber;
};