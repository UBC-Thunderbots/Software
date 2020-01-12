#pragma once

/*
 * A TransferMedium is an abstraction around the medium used to communicate
 * between computers and robots. Any medium such as UART, SPI, Ethernet/WiFi
 * used must extend this class.
 *
 */
class TransferMedium
{
   public:
    TransferMedium();
    virtual ~TransferMedium();

    /*
     * Connect to the medium
     *
     * @returns bool True if successful
     */
    virtual bool connect() = 0;

    /*
     * Disconnect from the medium
     *
     * @returns bool True if successful
     */
    virtual bool disconnect() = 0;

    /*
     * Packages the data and sends it through the medium
     *
     * @param data The data to be sent over the medium
     */
    virtual bool send_data(const std::string& data) = 0;

    /*
     * Returns when a data packet has arrived through the medium
     *
     * NOTE: Depending on the implementation of send_data
     * this function may be triggered by a packet sent by the
     * same instance of the class. (example: UDP broadcast, the sender
     * also gets a packet)
     *
     * @returns std::string The data that arrived through the medium
     */
    virtual std::string receive_data(std::function<void(std::string)>) = 0;
};
