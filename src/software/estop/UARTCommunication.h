#pragma once

#include <boost/asio.hpp>
#include <boost/function.hpp>

/*
 * A boost::asio wrapper class that is used to synchronously communicate with a serial device via UART
 */
typedef boost::asio::io_service io_service;
typedef std::shared_ptr<boost::asio::serial_port> serial_port_ptr;

class UARTCommunication
{
   public:
    UARTCommunication();

    /**
     * Attempts to open a serial connection with the given device port
     *
     * @param ioService boost asio construct for managing io operations
     * @param baudRate the desired baud rate of the connection
     * @param port the serial port that we want to communicate with
     * @return 0 upon success, -1 otherwise
     */
    virtual int openPort(io_service &ioService, int baudRate, std::string port);

    /**
     * Writes writeVal into serial port. Blocks current thread.
     *
     * @param write_val value that will be written
     * @param ec boost asio error code holder
     * @return number of bytes written, or -1 if serial port was not open
     */
    virtual int serialWrite(unsigned char* write_val, boost::system::error_code& ec);

    /**
     * Reads a given number of bytes from serial port until numBytes is read or error occurs.
     * Blocks current thread.
     *
     * @param read_val buffer that store read data
     * @param num_bytes number of bytes that should be read
     * @param ec boost asio error code holder
     * @return number of bytes written, or -1 if serial port was not open
     */
    virtual int serialRead(unsigned char* read_val, int num_bytes, boost::system::error_code& ec);

    /**
     * closes serial port
     */
    virtual void closePort();

   private:
    serial_port_ptr serial_;

};
