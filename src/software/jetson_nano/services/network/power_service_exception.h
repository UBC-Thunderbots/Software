#ifndef THUNDERBOTS_SOFTWARE_POWER_SERVICE_EXCEPTION_H
#define THUNDERBOTS_SOFTWARE_POWER_SERVICE_EXCEPTION_H

#include <exception>
#include <string>

class PowerServiceException {
public:
    PowerServiceException(const std::string &message);
    const char* what() const noexcept;
private:
    std::string message_;
};


#endif //THUNDERBOTS_SOFTWARE_POWER_SERVICE_EXCEPTION_H
