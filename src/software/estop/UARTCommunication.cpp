
#include "UARTCommunication.h"

#include <boost/asio.hpp>
#include <g3log/loglevels.hpp>
#include <g3log/g3log.hpp>
#include <iostream>
#include <boost/function.hpp>
#include <boost/exception/all.hpp>

UARTCommunication::UARTCommunication(){}


int UARTCommunication::openPort(io_service &ioService, int baudRate, std::string port)
{
    int uartCharacterSize = 8;
    try
    {
        serial_ = serial_port_ptr(std::make_shared<boost::asio::serial_port>(boost::asio::serial_port(ioService, port)));

        serial_->set_option(boost::asio::serial_port_base::baud_rate(baudRate));
        serial_->set_option(boost::asio::serial_port::flow_control(
                boost::asio::serial_port::flow_control::none));
        serial_->set_option(
                boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        serial_->set_option(
                boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        serial_->set_option(boost::asio::serial_port::character_size(
                boost::asio::serial_port::character_size(uartCharacterSize)));
    }
    catch(boost::exception const&  ex)
    {
        LOG(WARNING) << "Unable to setup serial communication with port: " << port;
        LOG(WARNING) << "Boost exception: " <<diagnostic_information(ex);
        return -1;
    }

    return 0;
}

int UARTCommunication::serialRead(unsigned char *read_val, int num_bytes, boost::system::error_code& ec) {
    if(serial_->is_open()){
        long unsigned int read_size = boost::asio::read(*serial_, boost::asio::buffer(read_val, num_bytes), ec);
        return (int) read_size;
    } else{
        return -1;
    }
}

int UARTCommunication::serialWrite(unsigned char * write_val, boost::system::error_code& ec)
{
    if(serial_->is_open()){
        long unsigned int write_size = boost::asio::write(*serial_, boost::asio::buffer(write_val, sizeof(write_val)), ec);
        return (int) write_size;
    } else{
        return -1;
    }
}

void UARTCommunication::closePort()
{
    if (serial_->is_open())
    {
        serial_->close();
    }
}

