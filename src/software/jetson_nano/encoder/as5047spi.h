#ifndef AS5047_H
#define AS5047_H

#include <stdint.h>

#define ANGLEUNC_ADDR 0x3FFE

class AS5047 {
    private:
        char* m_spidev;
        int m_spifd;
        bool m_open;

        uint32_t speed_ = 100000;

        const uint8_t numSamples_zero = 10;
        uint16_t zeroOffset = 0;

        const struct {
            uint8_t readAngle[2] = {0x7F, 0xFE};
        } commands;

        uint8_t calcEvenParity(uint16_t msg);
        bool readSanityCheck(uint16_t msg);

    public:
        AS5047();
        AS5047(const char* p_spidev, uint32_t speed=100000);
        ~AS5047();

        void setPort(const char* p_spidev);
        void setSpeed(uint32_t speed);

        bool begin();

        uint16_t readAngle();
        uint16_t getRelAngle();
        void softZero();
};

#endif
