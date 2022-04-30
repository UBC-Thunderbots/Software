#pragma once

#include <boost/asio.hpp>
#include <boost/function.hpp>

#include "uart_communication.h"

/*
 * A boost::asio wrapper class that is used to synchronously communicate with a serial
 * device via UART
 */

class BoostUartCommunication : public UartCommunication
{
    using IoService     = boost::asio::io_service;
    using SerialPortPtr = std::shared_ptr<boost::asio::serial_port>;

   public:
    /**
     * constructs and opens a serial connection with the given device port
     * settings used: 8 bit data, No flow control, No parity, 1 stop bit
     *
     * @param baud_rate the desired baud rate of the connection
     * @param device_serial_port the serial port that we want to communicate with
     * @throws boost::exception if port could not be opened
     */
    BoostUartCommunication(int baud_rate, std::string device_serial_port);

    BoostUartCommunication(const BoostUartCommunication &) = delete;

    BoostUartCommunication &operator=(const BoostUartCommunication &) = delete;

    ~BoostUartCommunication() override;


    /**
     * Stream output operator, writes to serial port, blocking thread
     * @param write_val
     * @return true if succeeded, false otherwise
     * @throws boost::exception upon error
     */
    bool operator<<(const std::vector<unsigned char> &write_val);


    /**
     * Writes into serial port. Blocks current thread.
     *
     * @param write_val buffer that will be written to serial port
     * @return true upon success, false otherwise
     * @throws boost::exception upon error during write
     */
    bool serialWrite(const std::vector<unsigned char> &write_val) override;

    /**
     * Reads a given number of bytes from serial port until num_read_bytes is read or
     * error occurs. Blocks current thread.
     *
     * @param num_read_bytes number of bytes that should be read
     * @return vector of size num_read_bytes with read data
     * @throws boost:exception upon error during read
     */
    std::vector<unsigned char> serialRead(size_t num_read_bytes) override;

    /**
     * Flushes serial port data
     * @param flush_type enum that sets the data to be flushed (read, write, or both)
     * @return true upon success, false otherwise
     */
    bool flushSerialPort(FlushType flush_type) override;

   private:
    /**
     * Attempts to open a serial connection with the given device port
     * setting used: No flow control, No parity, 1 stop bit
     *
     * @param io_service boost asio construct for managing io operations
     * @param baud_rate the desired baud rate of the connection
     * @param device_serial_port the serial port that we want to communicate with
     */
    void openPort(IoService &io_service, int baud_rate, std::string device_serial_port);

    /**
     * closes serial port
     */
    void closePort();

    SerialPortPtr serial_port;
    IoService io_service_;
};
