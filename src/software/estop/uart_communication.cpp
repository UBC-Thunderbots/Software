
#include "uart_communication.h"

#include <boost/asio.hpp>
#include <g3log/g3log.hpp>
#include <iostream>

UartCommunication::UartCommunication(io_service& ioService, int baudRate,
                                     std::string device_serial_port)
{
    openPort(ioService, baudRate, device_serial_port);
}

bool UartCommunication::serialRead(std::vector<unsigned char>& read_val,
                                   size_t num_read_bytes)
{
    std::vector<unsigned char> temp_buffer(num_read_bytes);
    size_t read_size =
        boost::asio::read(*serial_port, boost::asio::buffer(temp_buffer, num_read_bytes));
    read_val = temp_buffer;

    return read_size == num_read_bytes;
}

bool UartCommunication::serialWrite(std::vector<unsigned char>& write_val)
{
    size_t write_size = boost::asio::write(
        *serial_port, boost::asio::buffer(write_val, write_val.size()));
    return write_size == write_val.size();
}

bool UartCommunication::flushSerialPort(FlushType flushType)
{
    int ret_val = tcflush(serial_port->lowest_layer().native_handle(), flushType);
    return (ret_val == 0);
}

void UartCommunication::openPort(io_service& ioService, int baudRate,
                                 std::string device_serial_port)
{
    int uart_character_size_bits = 8;
    serial_port = serial_port_ptr(std::make_shared<boost::asio::serial_port>(
        boost::asio::serial_port(ioService, device_serial_port)));

    serial_port->set_option(boost::asio::serial_port_base::baud_rate(baudRate));
    serial_port->set_option(boost::asio::serial_port::flow_control(
        boost::asio::serial_port::flow_control::none));
    serial_port->set_option(
        boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    serial_port->set_option(
        boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    serial_port->set_option(boost::asio::serial_port::character_size(
        boost::asio::serial_port::character_size(uart_character_size_bits)));
}

void UartCommunication::closePort()
{
    if (serial_port != nullptr && serial_port->is_open())
    {
        serial_port->close();
    }
}

bool UartCommunication::operator<<(std::vector<unsigned char>& write_val)
{
    return serialWrite(write_val);
}

UartCommunication::~UartCommunication()
{
    closePort();
}
