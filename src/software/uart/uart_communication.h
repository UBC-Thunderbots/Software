#pragma once

#include <boost/asio.hpp>
#include <boost/function.hpp>

/*
 * Interface for Uart Communication
 */
class UartCommunication
{
   public:
    enum FlushType
    {
        flush_receive = TCIFLUSH,  // flushes data received but not read
        flush_send    = TCOFLUSH,  // flushes data written but not transmitted
        flush_both    = TCIOFLUSH  // flushes both data received but not read and data
        // written but not transmitted
    };

    UartCommunication() = default;

    virtual ~UartCommunication() = default;

    UartCommunication(const UartCommunication &) = delete;

    UartCommunication &operator=(const UartCommunication &) = delete;

    /**
     * writes write_val into UART transmit buffer
     * @param write_val values to be written
     * @return true upon success, false otherwise
     */
    virtual bool serialWrite(const std::vector<unsigned char> &write_val) = 0;

    /**
     * reads a given number of bytes from UART receive buffer
     * @param num_read_bytes the number of bytes to read
     * @return the values read
     */
    virtual std::vector<unsigned char> serialRead(size_t num_read_bytes) = 0;

    /**
     * Flushes serial port data
     * @param flush_type enum that sets the data to be flushed (read, write, or both)
     * @return true upon success, false otherwise
     */
    virtual bool flushSerialPort(FlushType flush_type) = 0;
};
