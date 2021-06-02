#pragma once

template <class ReceiveProtoT>
ThreadedProtoUdpListener<ReceiveProtoT>::ThreadedProtoUdpListener(
    const std::string& ip_address, const unsigned short port,
    std::function<void(ReceiveProtoT)> receive_callback, bool multicast)
    : io_service(),
      work(io_service),
      receive_callback(receive_callback),
      udp_listener(io_service, ip_address, port, multicast)
{
    in_destructor = false;
    io_service_thread = std::thread([this]() {
    io_service.run();
    });
    networking_thread = std::thread([this]() {
        while (!in_destructor)
        {
            auto packet = udp_listener.receiveProto();
            if (packet.has_value())
            {
                this->receive_callback(packet.value());
                std::cerr<<"------"<<std::endl;
            }
        }
    });
}

template <class ReceiveProtoT>
ThreadedProtoUdpListener<ReceiveProtoT>::ThreadedProtoUdpListener(
    const unsigned short port, std::function<void(ReceiveProtoT)> receive_callback)
    : io_service(), receive_callback(receive_callback), udp_listener(io_service, port)
{
    in_destructor     = false;
    io_service_thread = std::thread([this]() {
    io_service.run();
    });
    networking_thread = std::thread([this]() {
        while (!in_destructor)
        {
            auto packet = udp_listener.receiveProto();
            if (packet.has_value())
            {
                this->receive_callback(packet.value());
                std::cerr<<"------"<<std::endl;
            }
        }
    });
}

template <class ReceiveProtoT>
ThreadedProtoUdpListener<ReceiveProtoT>::~ThreadedProtoUdpListener()
{
    in_destructor = true;

    // Stop the io_service. This is safe to call from another thread.
    // https://stackoverflow.com/questions/4808848/boost-asio-stopping-io-service
    // This MUST be done before attempting to join the thread because otherwise
    // the io_service will not stop and the thread will not join
    io_service.stop();

    // Join the networking_thread so that we wait for it to exit before
    // destructing the thread object. If we do not wait for the thread to finish
    // executing, it will call `std::terminate` when we deallocate the thread
    // object and kill our whole program
    io_service_thread.join();
    networking_thread.join();
}
