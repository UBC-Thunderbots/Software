#pragma once

#include "uart_communication.h"


/*
 * A mock of UartCommunication class used for testing purposes
 */

class MockUart : public UartCommunication
{
   public:
    MockUart(){};
    ~MockUart(){};
    MOCK_METHOD1(serialWrite, bool(const std::vector<unsigned char> &write_val));
    MOCK_METHOD1(serialRead, std::vector<unsigned char>(size_t num_read_bytes));
    MOCK_METHOD1(flushSerialPort, bool(FlushType flush_type));
};
