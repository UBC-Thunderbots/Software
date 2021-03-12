#pragma once

#include <boost/asio.hpp>
#include <boost/function.hpp>

/*
 * A boost::asio wrapper class that is used to synchronously communicate with a serial device via UART
 */

class UartCommunication
{
    using io_service = boost::asio::io_service;
    using serial_port_ptr =  std::shared_ptr<boost::asio::serial_port>;

public:

    enum flush_type
    {
        flush_receive = TCIFLUSH, //flushes data received but not read
        flush_send = TCOFLUSH,    //flushes data written but not transmitted
        flush_both = TCIOFLUSH    //flushes both data received but not read and data written but not transmitted
    };

    UartCommunication();

    /**
     * Attempts to open a serial connection with the given device port
     * setting used: No flow control, No parity, 1 stop bit
     *
     * @param ioService boost asio construct for managing io operations
     * @param baudRate the desired baud rate of the connection
     * @param port the serial port that we want to communicate with
     * @return true upon success, false otherwise
     */
    virtual bool openPort(io_service &ioService, int baudRate, std::string port);

    /**
     * Writes writeVal into serial port. Blocks current thread.
     *
     * @param write_val buffer that will be written to serial port
     * @return true upon success, false otherwise
     * @throws boost::exception upon error during write
     */
    virtual bool serialWrite(std::vector<unsigned char> &write_val);

    /**
     * Reads a given number of bytes from serial port until num_read_bytes is read or error occurs.
     * Blocks current thread.
     *
     * @param read_val buffer that will be set to the read data
     * @param num_read_bytes number of bytes that should be read
     * @return true upon success, false otherwise
     * @throws boost:exception upon error during read
     */
    virtual bool serialRead(std::vector<unsigned char> &read_val, size_t num_read_bytes);

    /**
     * Flushes serial port data
     * @param flushType enum that sets the data to be flushed (read, write, or both)
     * @return true upon success, false otherwise
     */
    virtual bool flushSerialPort(flush_type flushType);

    /**
     * closes serial port
     */
    virtual void closePort();

   private:
    serial_port_ptr serial_port;

};
