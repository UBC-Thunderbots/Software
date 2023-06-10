#include "power_service_exception.h"

PowerServiceException::PowerServiceException(const std::string& message)
    : message_(message)
{
}

const char* PowerServiceException::what() const noexcept
{
    return message_.c_str();
}
