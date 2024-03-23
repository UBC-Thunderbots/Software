#pragma once

#include <string>

#include "software/networking/radio/radio_sender.h"

class ThreadedRadioSender
{
   public:
    /**
     * Creates a ThreadedRadioSender that sends the sendString over radio.
     */
    ThreadedRadioSender();

    ~ThreadedRadioSender();

    /**
     * Sends a string message over radio.
     *
     * @param message The string message to send
     */
    void sendString(const std::string& message);

   private:
    RadioSender radio_sender_;
};
