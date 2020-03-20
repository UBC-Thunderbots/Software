#pragma once
#include <functional>
/*
 * A TransferMedium is an abstraction around the medium used to communicate
 * between computers and robots. Any medium such as UART, SPI, Ethernet/WiFi
 * used to transfer data to and from robots can extend this class.
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
    virtual void send_data(const std::string& data) = 0;

    /*
     * Calls the provided callback everytime a new packet is received
     *
     * NOTE: this function does NOT block, it returns immediately after
     * registering the callback
     */
    virtual void receive_data_async(
        std::function<void(std::string)> receive_callback) = 0;
};
