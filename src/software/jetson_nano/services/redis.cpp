#include "service.h"
#include "redis.h"

#include "string"

#include "software/logger/logger.h"

// Redis config
static size_t PORT = 6379;

Redis::Redis() : subscriber() {
}

void Redis::start() {
    subscriber.connect("127.0.0.1", PORT,
                       [](const std::string &host, std::size_t port, cpp_redis::connect_state status) {
                           // Should be logged instead
                           if (status == cpp_redis::connect_state::dropped) {
                               // Should be logged
                               LOG(WARNING) << "connection dropped";
                               std::cout << "client disconnected from " << host << ":" << port << std::endl;
                           } else if (status == cpp_redis::connect_state::failed) {
                               LOG(WARNING) << "connection failed";
                           } else if (status == cpp_redis::connect_state::ok) {
                               LOG(INFO) << "connection succesful";
                           }
                       });
}

void Redis::stop() {
    subscriber.disconnect(false);
}

void Redis::subscribe(const std::string &channel,
                      void (*subscribe_callback)(std::string, std::string)) {
    subscriber.subscribe(channel, subscribe_callback);
}

