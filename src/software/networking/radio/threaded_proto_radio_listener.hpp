#pragma once

#include "software/networking/radio/proto_radio_listener.hpp"

/**
 * A threaded listener that receives serialized ReceiveProtoT Proto's over radio
 */
template <class ReceiveProtoT>
class ThreadedProtoRadioListener
{
   public:
    /**
     * Creates a ThreadedProtoRadioListener that will listen for ReceiveProtoT packets
     * over radio. For every ReceiveProtoT packet received, the receive_callback will be
     * called to perform any operations desired by the caller.
     *
     * @param receive_callback The function to run for every ReceiveProtoT packet received
     * from the network
     */
    ThreadedProtoRadioListener(std::function<void(ReceiveProtoT)> receive_callback);

    ~ThreadedProtoRadioListener();

   private:
    ProtoRadioListener<ReceiveProtoT> radio_listener_;
};

template <class ReceiveProtoT>
ThreadedProtoRadioListener<ReceiveProtoT>::ThreadedProtoRadioListener(
    std::function<void(ReceiveProtoT)> receive_callback)
    : radio_listener_(receive_callback)
{
}

template <class ReceiveProtoT>
ThreadedProtoRadioListener<ReceiveProtoT>::~ThreadedProtoRadioListener()
{
}
