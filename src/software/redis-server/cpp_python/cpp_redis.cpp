#include <iostream>
#include <string>

#include "cpp-redis/cpp_redis/core/client.hpp"

using namespace sw::redis;
cpp_redis::client client;

int main(int argc, char *argv[]){
    client.connect();
    try {
        // Create an Redis object, which is movable but NOT copyable.
        auto redis = Redis("tcp://127.0.0.1:6379");

        std::string operation = argv[1];

        if(operation.compare("set") == 0){
            redis.set(argv[2],argv[3]);
        } else if (operation.compare("get") == 0){
            auto val = redis.get(argv[2]);
            if (val) {
            // Dereference val to get the returned value of std::string type.
            std::cout << *val << std::endl;
            }else{
                std::cout<<"Key doesn't exist"<<std::endl;
            }
        }
    } catch (const Error &e) {
        std::cout<<"Error"<<std::endl;
    }
}
