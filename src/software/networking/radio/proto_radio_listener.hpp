#pragma once

#include <string>

#include "software/logger/logger.h"
#include "software/util/typename/typename.h"

template <class ReceiveProtoT>
class ProtoRadioListener
{
   public:
    /**
     * Creates an ProtoRadioListener that will listen for ReceiveProtoT packets over
     * radio. For every ReceiveProtoT packet received, the receive_callback will be called
     * to perform any operations desired by the caller
     *
     * @param receive_callback The function to run for every ReceiveProtoT packet received
     */
    ProtoRadioListener(std::function<void(ReceiveProtoT&)> receive_callback);

    virtual ~ProtoRadioListener();

   private:
    /**
     * Start listening for data
     */
    void startListen();

    // The function to call on every received packet of ReceiveProtoT data
    std::function<void(ReceiveProtoT&)> receive_callback;

    // Whether or not the listener is running
    bool running_ = true;
};

template <class ReceiveProtoT>
ProtoRadioListener<ReceiveProtoT>::ProtoRadioListener(
    std::function<void(ReceiveProtoT&)> receive_callback)
    : receive_callback(receive_callback)
{
    startListen();
}

template <class ReceiveProtoT>
void ProtoRadioListener<ReceiveProtoT>::startListen()
{
}

template <class ReceiveProtoT>
ProtoRadioListener<ReceiveProtoT>::~ProtoRadioListener()
{
    running_ = false;
}
