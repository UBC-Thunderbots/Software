#include "errors.h"

#include <iostream>
#include <sstream>
#include <string>

namespace
{
    std::string make_transfer_error_message(unsigned int endpoint, const char *msg)
    {
        std::stringstream ss;
        ss << msg << " on " << ((endpoint & 0x80) ? "IN" : "OUT") << " endpoint "
           << (endpoint & 0x7F) << std::endl;
        return ss.str();
    }

}  // namespace

USB::Error::Error(const std::string &msg) : std::runtime_error(msg) {}

USB::TransferError::TransferError(unsigned int endpoint, const char *msg)
    : Error(make_transfer_error_message(endpoint, msg))
{
}

USB::TransferTimeoutError::TransferTimeoutError(unsigned int endpoint)
    : TransferError(endpoint, "Transfer timed out")
{
}

USB::TransferStallError::TransferStallError(unsigned int endpoint)
    : TransferError(endpoint, "Transfer stalled")
{
}

USB::TransferCancelledError::TransferCancelledError(unsigned int endpoint)
    : TransferError(endpoint, "Transfer cancelled")
{
}
