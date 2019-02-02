#pragma once
#include <libusb.h>

#include <cstddef>
#include <cstdint>
#include <string>

#include "util/noncopyable.h"

namespace USB
{
    /* Forward declaration of the libusb context */
    class Context;

    /**
     * A collection of information about a USB device.
     */
    class Device final
    {
       public:
        /**
         * Makes a copy of a device record.
         *
         * @param[in] copyref the object to copy
         */
        Device(const Device &copyref);

        /**
         * Destroys the device information record.
         */
        ~Device();

        /**
         * Assigns a device information record.
         *
         * @param[in] assgref the object to copy from
         *
         * @return this object
         */
        Device &operator=(const Device &assgref);

        /**
         * Returns the 16-bit vendor ID from the device’s device descriptor.
         *
         * @return the vendor ID
         */
        unsigned int vendor_id() const
        {
            return device_descriptor.idVendor;
        }

        /**
         * Returns the 16-bit product ID from the device’s device descriptor.
         *
         * @return the product ID
         */
        unsigned int product_id() const
        {
            return device_descriptor.idProduct;
        }

        /**
         * Returns the serial number from the device’s device and string
         * descriptors.
         *
         * @return the serial number, or an empty string if the device does not
         * expose a serial number
         */
        std::string serial_number() const;

       private:
        friend class DeviceList;
        friend class DeviceHandle;

        libusb_context *context;
        libusb_device *device;
        libusb_device_descriptor device_descriptor;

        explicit Device(libusb_device *device);
    };

    /**
     * A list of USB devices.
     */
    class DeviceList final : public NonCopyable
    {
       public:
        /**
         * Constructs a list of all USB devices attached to the system.
         *
         * @param[in] context the library context in which to operate
         */
        explicit DeviceList(Context &context);

        /**
         * Frees the list of devices.
         */
        ~DeviceList();

        /**
         * Returns the size of the list.
         *
         * @return the number of devices in the list
         */
        std::size_t size() const
        {
            return size_;
        }

        /**
         * Returns a device from the list.
         *
         * @param[in] i the index of the device to return, counting from zero
         *
         * @return the device
         */
        Device operator[](const std::size_t i) const;

       private:
        std::size_t size_;
        libusb_device **devices;
    };
}  // namespace USB
