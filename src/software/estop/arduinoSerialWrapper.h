#pragma once

#include <boost/asio.hpp>
#include <boost/function.hpp>

/*
 * A wrapper class that is used to communicate with an Arduino via UART
 */
typedef boost::asio::io_service io_service;
typedef std::shared_ptr<boost::asio::serial_port> serial_port_ptr;
typedef std::function<void(boost::system::error_code const&, const size_t)> handler;

class arduinoSerialWrapper
{
   public:
    arduinoSerialWrapper();

    virtual int openPort(io_service &ioService, int baudRate, std::string device);
    virtual int serialWrite(unsigned char* val);
    virtual int serialRead(unsigned char* readVal, int numBytes, boost::system::error_code& ec);
    virtual void closePort();

   private:
    serial_port_ptr serial_;

};
