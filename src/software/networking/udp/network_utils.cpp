#include "software/networking/udp/network_utils.h"

#include <netinet/in.h>
#include <string>

bool getLocalIp(const std::string& interface, std::string& ip_address, bool ipv4)
{
    struct ifaddrs* ifAddrStruct = nullptr;
    struct ifaddrs* ifa = nullptr;

    getifaddrs(&ifAddrStruct);

    for (ifa = ifAddrStruct; ifa != nullptr; ifa = ifa->ifa_next)
    {
        if (ifa->ifa_name == interface)
        {
            if (ipv4 && ifa->ifa_addr->sa_family == AF_INET)
            {
                char addressBuffer[INET_ADDRSTRLEN];
                struct sockaddr_in* sa = (struct sockaddr_in*) ifa->ifa_addr;
                inet_ntop(AF_INET, &sa->sin_addr, addressBuffer, INET_ADDRSTRLEN);
                freeifaddrs(ifAddrStruct);
                ip_address = addressBuffer;
                return true;
            }
            else if (!ipv4 && ifa->ifa_addr->sa_family == AF_INET6)
            {
                char addressBuffer[INET6_ADDRSTRLEN];
                struct sockaddr_in6* sa = (struct sockaddr_in6*) ifa->ifa_addr;
                inet_ntop(AF_INET6, &sa->sin6_addr, addressBuffer, INET6_ADDRSTRLEN);
                freeifaddrs(ifAddrStruct);
                ip_address = addressBuffer;
                return true;
            }
        }
    }

    return false;
}

bool isIpv6(const std::string& ip_address)
{
    struct sockaddr_in6 sa;
    return inet_pton(AF_INET6, ip_address.c_str(), &(sa.sin6_addr)) != 0;
}
