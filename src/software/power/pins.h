#pragma once

#include <Arduino.h>

// Chicker
const uint8_t KICKER_PIN  = 33;
const uint8_t CHIPPER_PIN = 32;

// Charger
const uint8_t HV_SENSE  = 36;
const uint8_t CHRG_DONE = 25;
const uint8_t CHRG      = 26;

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
