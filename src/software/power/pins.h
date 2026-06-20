#pragma once

#include <Arduino.h>

// Geneva
const uint8_t DIR                  = 4;
const uint8_t PWM                  = 15;

// NOTE: On the new high-voltage sensing hardware, GPIO18 is also ADS7945 SCLK.
// Geneva is currently a stub in this firmware, but this conflict must be cleaned up
// before using Geneva encoder A again.
const uint8_t GENEVA_ENCODER_A_PIN = 18;

// NOTE: On the new high-voltage sensing hardware, GPIO5 is ADS7945 MISO.
// Geneva is currently a stub in this firmware, but this conflict must be cleaned up
// before using Geneva encoder B again.
const uint8_t GENEVA_ENCODER_B_PIN = 5;

const uint8_t LEFT_DIR  = 1;
const uint8_t RIGHT_DIR = -1;

// Chicker
const uint8_t KICKER_PIN  = 32;
const uint8_t CHIPPER_PIN = 33;

// Charger / LT3750
const uint8_t HV_SENSE      = 36;  // old ESP32 ADC path; no longer used
const uint8_t FLYBACK_FAULT = 27;
const uint8_t CHRG_DONE     = 25;  // LT3750 DONE, active-low/open-drain
const uint8_t CHRG          = 26;  // LT3750 CHARGE

// ADS7945 external ADC
const uint8_t ADC_MISO = 5;   // ADS7945 SDO, moved away from GPIO16
const uint8_t ADC_SCLK = 18;  // ADS7945 SCLK
const uint8_t ADC_CS   = 21;  // ADS7945 CS

// CH_SEL is intentionally not driven in this proof-of-concept.
// It should be pulled low in hardware so ADS7945 CH0 is selected.

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