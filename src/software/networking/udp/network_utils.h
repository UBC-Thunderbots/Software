#pragma once

#include <ifaddrs.h>

#include <string>

/**
 * Given an interface, get the IP address associated with that interface
 *
 * The modified ip_address is valid only if the function returns true
 *
 * @param interface The interface to get the IP address from
 * @param ip_address A reference to the std::string that will store the IP address if
 * found
 * @param ipv4 If true, get the IPv4 address, otherwise get the IPv6 address
 *
 * @return true if the IP address was found, false otherwise
 */
bool getLocalIp(const std::string& interface, std::string& ip_address, bool ipv4 = true);

/**
 * Check if the given string follows the IPv6 address format
 *
 * Addresses that are actually an "embedded IPv4 address" are still considered as an IPv6
 * address since it follows the IPv6 address format
 *
 * @param ip_address The string to check
 * @return true if the string is a valid IPv6 address, false otherwise
 */
bool isIpv6(const std::string& ip_address);
