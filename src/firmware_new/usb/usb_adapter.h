#pragma once

class USBMediator
{
   public:
    /*
     * Creates a usb mediator, given the port and the baud rate.
     * Used to send and recieve msgs to serial devices using boost and protobuf
     *
     * TODO need a better name
     *
     * @param port The serial port to send to, usuall /dev/tty*
     * @param baud_rate The baud rate for serial communication
     */
    USBMediator(const std::string& port, int baud_rate);

    virtual ~USBMediator();

    /*
     * Templated function to send proto over serial.
     *
     * This function returns immediately, the optional callback can run once the msg
     * was successfully run.
     *
     * @param proto_msg_to_send The msg to send to the serial device
     */
    template <class T>
    void send_proto_over_usb(const T& proto_msg_to_send);

    /*
     * Templated function to recieve proto over serial.
     *
     * NOTE: if there are multiple types of protomsgs coming over usb, only the
     * templated type will match the incoming msg
     *
     */
    template <class R>
    void receive_proto_over_usb();

   private:
    // in_buffer;
    // out_buffer;
};
