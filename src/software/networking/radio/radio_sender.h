#pragma once

#include <string>

class RadioSender
{
   public:
    /**
     * Creates a RadioSender that sends strings over radio.
     */
    RadioSender();

    ~RadioSender();

    /**
     * Sends a string message over radio.
     *
     * @param message The string message to send
     */
    void sendString(const std::string& message);
};
