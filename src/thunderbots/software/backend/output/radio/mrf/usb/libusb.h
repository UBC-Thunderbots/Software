#ifndef UTIL_LIBUSB_H
#define UTIL_LIBUSB_H

#include <libusb.h>

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <list>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "software/bulktransfer.h"
#include "software/controltransfer.h"
#include "software/device.h"
#include "software/devicehandle.h"
#include "software/errors.h"
#include "software/interrupttransfer.h"
#include "software/misc.h"
#include "software/transfer.h"
#include "software/backend/output/radio/mrf/util/async_operation.h"
#include "software/backend/output/radio/mrf/util/noncopyable.h"

namespace USB
{
    /**
     * A libusb context. This is the entry point to the libusb wrapper.
     */
    class Context final : public NonCopyable
    {
       public:
        /**
         * Initializes the library and creates a context.
         */
        explicit Context();

        /**
         * Deinitializes the library and destroys the context.
         *
         * This must be invoked after all associated DeviceList and DeviceHandle
         * objects have been destroyed.
         */
        ~Context();

       protected:
        std::vector<USB::DeviceHandle *> open_devices;

       private:
        friend class DeviceList;
        friend class DeviceHandle;

        libusb_context *context;
    };

    /**
     * An RAII-style way to set a configuration on a device.
     */
    class ConfigurationSetter final : public NonCopyable
    {
       public:
        /**
         * Sets a device’s configuration.
         *
         * @param[in] device the device to set
         *
         * @param[in] configuration the configuration number to set
         */
        explicit ConfigurationSetter(DeviceHandle &device, int configuration);

        /**
         * Resets the device to its original configuration.
         */
        ~ConfigurationSetter();

       private:
        DeviceHandle &device;
        int original_configuration;
    };

    /**
     * An RAII-style way to claim an interface on a device.
     */
    class InterfaceClaimer final : public NonCopyable
    {
       public:
        /**
         * Claims an interface on a device.
         *
         * @param[in] device the device whose interface should be claimed
         *
         * @param[in] interface the interface to claim
         */
        explicit InterfaceClaimer(DeviceHandle &device, int interface);

        /**
         * Releases the interface.
         */
        ~InterfaceClaimer();

       private:
        DeviceHandle &device;
        int interface;
    };

}  // namespace USB

#endif
