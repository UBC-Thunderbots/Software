#include "misc.h"

#include "errors.h"
#include "transfer.h"

long USB::check_fn(const char *call, long err, unsigned int endpoint)
{
    if (err >= 0)
    {
        return err;
    }

    const char *msg;
    const char *detail = nullptr;
    switch (err)
    {
        case LIBUSB_ERROR_IO:
            msg    = "Input/output error";
            detail = std::strerror(errno);
            break;

        case LIBUSB_ERROR_INVALID_PARAM:
            msg = "Invalid parameter";
            break;

        case LIBUSB_ERROR_ACCESS:
            msg = "Access denied (insufficient permissions)";
            break;

        case LIBUSB_ERROR_NO_DEVICE:
            msg = "No such device (it may have been disconnected)";
            break;

        case LIBUSB_ERROR_NOT_FOUND:
            msg = "Entity not found";
            break;

        case LIBUSB_ERROR_BUSY:
            msg = "Resource busy";
            break;

        case LIBUSB_ERROR_TIMEOUT:
            throw USB::TransferTimeoutError(endpoint);

        case LIBUSB_ERROR_OVERFLOW:
            msg = "Overflow";
            break;

        case LIBUSB_ERROR_PIPE:
            throw USB::TransferStallError(endpoint);

        case LIBUSB_ERROR_INTERRUPTED:
            msg = "System call interrupted (perhaps due to signal)";
            break;

        case LIBUSB_ERROR_NO_MEM:
            throw std::bad_alloc();

        case LIBUSB_ERROR_NOT_SUPPORTED:
            msg = "Operation not supported or unimplemented on this platform";
            break;

        case LIBUSB_ERROR_OTHER:
            msg = "Other error";
            break;

        default:
            throw std::runtime_error("Error fetching error message");
    }
    std::string s;
    s.reserve(std::strlen(call) + 2 + std::strlen(msg) +
              (detail ? (3 + std::strlen(detail)) : 0));
    s.append(call);
    s.append(": ");
    s.append(msg);
    if (detail)
    {
        s.append(" (");
        s.append(detail);
        s.append(")");
    }
    throw USB::Error(s);
}
