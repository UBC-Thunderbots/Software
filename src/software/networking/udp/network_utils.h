#pragma once

#include <ifaddrs.h>
#include <string>

bool getLocalIp(const std::string& interface, std::string& ip_address, bool ipv4=true);

/**
 * @brief Check if the given string is a valid IPv6 address
 * 
 * @param ip_address The string to check
 * @return true if the string is a valid IPv6 address, false otherwise
 */
bool isIpv6(const std::string& ip_address);
