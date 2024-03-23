#include "threaded_radio_sender.h"

#include <string>

ThreadedRadioSender::ThreadedRadioSender() : radio_sender_() {}

ThreadedRadioSender::~ThreadedRadioSender() {}

void ThreadedRadioSender::sendString(const std::string& message)
{
    radio_sender_.sendString(message);
}
