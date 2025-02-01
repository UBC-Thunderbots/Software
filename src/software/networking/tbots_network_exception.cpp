#include "software/networking/tbots_network_exception.h"

const char* TbotsNetworkException::what() const noexcept
{
    return message.c_str();
}
