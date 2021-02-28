//
// Created by amr on 2020-12-01.
//

#include "arduinoSerialWrapper.h"

#include <boost/asio.hpp>
#include <g3log/loglevels.hpp>
#include <g3log/g3log.hpp>
#include <iostream>
#include <boost/function.hpp>

arduinoSerialWrapper::arduinoSerialWrapper(){
}


int arduinoSerialWrapper::openPort(io_service &ioService, int baudRate, std::string device)
{
    try
    {
        serial_ = serial_port_ptr(std::make_shared<boost::asio::serial_port>(boost::asio::serial_port(ioService, device)));

        serial_->set_option(boost::asio::serial_port_base::baud_rate(baudRate));
        serial_->set_option(boost::asio::serial_port::flow_control(
                boost::asio::serial_port::flow_control::none));
        serial_->set_option(
                boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        serial_->set_option(
                boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        serial_->set_option(boost::asio::serial_port::character_size(
                boost::asio::serial_port::character_size(8)));
    }
    catch(boost::exception const&  ex)
    {
        //LOG(WARNING) << "Unable to setup serial communication with device: " << device<< std::endl;
        std::cout<< "Unable to setup serial communication with device: " << device<< std::endl;
        return -1;
    }

    return 0;
}

int arduinoSerialWrapper::serialRead(unsigned char *readVal, int numBytes, boost::system::error_code& ec) {

    if(serial_->is_open()){
        long unsigned int sizeRead = boost::asio::read(*serial_, boost::asio::buffer(readVal, numBytes), ec);
        return (int) sizeRead;
    } else{
        return -1;
    }
}

int arduinoSerialWrapper::serialWrite(unsigned char * val)
{
    long unsigned int retVal = boost::asio::write(*serial_, boost::asio::buffer(val, sizeof(val)));
    return (int) retVal;
}

void arduinoSerialWrapper::closePort()
{
    if (serial_->is_open())
    {
        serial_->close();
    }
}

