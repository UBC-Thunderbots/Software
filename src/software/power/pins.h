#pragma once

#include <Arduino.h>

// Chicker
const uint8_t KICKER_PIN  = 33;
const uint8_t CHIPPER_PIN = 32;

// Charger / LT3750
const uint8_t FLYBACK_FAULT = 27;
const uint8_t CHRG_DONE     = 25;  // LT3750 DONE, active-low/open-drain
const uint8_t CHRG          = 26;  // LT3750 CHARGE

// ADS7945 external ADC
const uint8_t ADC_MISO = 5;   // ADS7945 SDO, moved away from GPIO16
const uint8_t ADC_SCLK = 18;  // ADS7945 SCLK
const uint8_t ADC_CS   = 21;  // ADS7945 CS

// CH_SEL is intentionally not driven in this proof-of-concept.
// It should be pulled low in hardware so ADS7945 CH0 is selected.\
// CH_SEL could be used for temperature sensing, not a priority for now.

// Break Beam
const uint8_t BREAK_BEAM_PIN = 37;

// UART
const uint8_t RXD2 = 19;
const uint8_t TXD2 = 22;

// Power Monitor
const uint8_t PM_SDA = 13;
const uint8_t PM_SCL = 14;

// Dribbler
const uint8_t DRIBBLER_PIN = 4;

// Timers
const uint8_t CHICKER_PULSE_TIMER     = 0;
const uint8_t CHICKER_COOLDOWN_TIMER  = 3;
const uint32_t MICROSECONDS_IN_SECOND = 1000000;