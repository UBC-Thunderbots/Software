#pragma once

#include <Arduino.h>

// Geneva
const uint8_t DIR                  = 4;
const uint8_t PWM                  = 15;
const uint8_t GENEVA_ENCODER_A_PIN = 18;
const uint8_t GENEVA_ENCODER_B_PIN = 5;

// Chicker
const uint8_t KICKER_PIN  = 32;
const uint8_t CHIPPER_PIN = 33;

// Charger
const uint8_t HV_SENSE      = 36;
const uint8_t FLYBACK_FAULT = 27;
const uint8_t CHRG_DONE     = 26;
const uint8_t CHRG          = 25;

// Break Beam
const uint8_t BREAK_BEAM_PIN = 37;

// UART
const uint8_t RXD2 = 19;
const uint8_t TXD2 = 22;

// Power Monitor
const uint8_t PM_SDA = 13;
const uint8_t PM_SCL = 14;

// Timers
const uint8_t CHICKER_COOLDOWN_TIMER  = 0;
const uint8_t CHICKER_PULSE_TIMER     = 1;
const uint8_t GENEVA_TIMER            = 2;
const uint32_t MICROSECONDS_IN_SECOND = 1000000;
