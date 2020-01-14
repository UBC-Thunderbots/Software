#pragma once
#include "software/multithreading/thread_safe_buffer.h"

/*
 * A TransferMedium is an abstraction around the medium used to communicate
 * between computers and robots. Any medium such as UART, SPI, Ethernet/WiFi
 * used must extend this class.
 *
 */
class TransferMedium
{
   public:
    TransferMedium()          = default;
    virtual ~TransferMedium() = default;

    /*
     * Packages the data and sends it through the medium
     *
     * @param data The data to be sent over the medium
     */
    virtual void send_data(const std::string& data)
    {
        throw std::logic_error("base TransferMedium send_data called");
    }

    /*
     * Calls the provided callback everytime a new packet is received
     *
     * NOTE: Depending on the implementation of send_data
     * this function may be triggered by a packet sent by the
     * same instance of the class. (example: UDP broadcast, the sender
     * also gets a packet)
     *
     * NOTE: this function does NOT block, it returns immediately after
     * registering the callback
     */
    virtual void receive_data_async(std::function<void(std::string)> receive_callback)
    {
        throw std::logic_error("base TransferMedium receive_data_async called");
    }
};
