
#include "boost_uart_communication.h"

#include <boost/asio.hpp>
#include <iostream>

BoostUartCommunication::BoostUartCommunication(int baud_rate,
                                               std::string device_serial_port)
    : io_service_()
{
    openPort(io_service_, baud_rate, device_serial_port);
}

std::vector<unsigned char> BoostUartCommunication::serialRead(size_t num_read_bytes)
{
    std::vector<unsigned char> read_buffer(num_read_bytes);

    boost::asio::read(*serial_port, boost::asio::buffer(read_buffer, num_read_bytes));

    return read_buffer;
}

bool BoostUartCommunication::serialWrite(const std::vector<unsigned char>& write_val)
{
    size_t write_size = boost::asio::write(
        *serial_port, boost::asio::buffer(write_val, write_val.size()));
    return write_size == write_val.size();
}

bool BoostUartCommunication::flushSerialPort(FlushType flush_type)
{
    int ret_val = tcflush(serial_port->lowest_layer().native_handle(), flush_type);
    return (ret_val == 0);
}

void BoostUartCommunication::openPort(IoService& ioService, int baud_rate,
                                      std::string device_serial_port)
{
    int uart_character_size_bits = 8;
    serial_port = SerialPortPtr(std::make_shared<boost::asio::serial_port>(
        boost::asio::serial_port(ioService, device_serial_port)));

    serial_port->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    serial_port->set_option(boost::asio::serial_port::flow_control(
        boost::asio::serial_port::flow_control::none));
    serial_port->set_option(
        boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    serial_port->set_option(
        boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    serial_port->set_option(boost::asio::serial_port::character_size(
        boost::asio::serial_port::character_size(uart_character_size_bits)));
}

void BoostUartCommunication::closePort()
{
    if (serial_port != nullptr && serial_port->is_open())
    {
        serial_port->close();
    }
}

bool BoostUartCommunication::operator<<(const std::vector<unsigned char>& write_val)
{
    return serialWrite(write_val);
}


BoostUartCommunication::~BoostUartCommunication()
{
    closePort();
}
