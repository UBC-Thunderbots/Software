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
    virtual void connect();

    /*
     * Disconnect from the medium
     *
     * @returns bool True if successful
     */
    virtual void disconnect();

    /*
     * Packages the data and sends it through the medium
     *
     * @param data The data to be sent over the medium
     */
    virtual void send_data(const std::string& data);

    /*
     * Returns when a data packet has arrived through the medium
     *
     * NOTE: Depending on the implementation of send_data
     * this function may be triggered by a packet sent by the
     * same instance of the class. (example: UDP broadcast, the sender
     * also gets a packet)
     *
     * @param func The function to call when data is received
     */
    virtual void receive_data(std::function<void(std::string)>);
};
