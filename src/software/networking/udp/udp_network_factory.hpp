#pragma once

class UdpNetworkFactory
{
    public:
        template <class ReceiveProtoT>
        static createThreadedProtoUdpListener(const std::string& ip_address, unsigned short port,
                    const std::string& interface, std::function<void(ReceiveProtoT)> receive_callback, bool multicast);

        template <class ReceiveProtoT>
            static createThreadedProtoUdpSender(const std::string& ip_address, unsigned short port,
                    const std::string& interface, std::function<void(ReceiveProtoT)> receive_callback, bool multicast);
};
