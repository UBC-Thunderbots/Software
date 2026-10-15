#include "esp_hal.h"
#include "pins.h"

namespace
{
constexpr float ADC_RESOLUTION            = 4096.0F;
constexpr float ADC_REFERENCE_VOLTAGE     = 3.3F;
constexpr float CAPACITOR_VOLTAGE_DIVIDER = 1003.0F / 13.0F;
constexpr float CHARGE_DONE_THRESHOLD_V   = 0.5F;
}  // namespace

void Esp32Hal::initializeCharger()
{
    pinMode(HV_SENSE, INPUT);
    pinMode(CHRG_DONE, INPUT);
    pinMode(CHRG, OUTPUT);
    digitalWrite(CHRG, LOW);
}

void Esp32Hal::setCapacitorCharging(bool enabled)
{
    digitalWrite(CHRG, enabled ? HIGH : LOW);
}

bool Esp32Hal::isCapacitorChargeDone()
{
    const float done_pin_voltage =
        analogRead(CHRG_DONE) / ADC_RESOLUTION * ADC_REFERENCE_VOLTAGE;
    return done_pin_voltage <= CHARGE_DONE_THRESHOLD_V;
}

float Esp32Hal::getCapacitorVoltage()
{
    return analogRead(HV_SENSE) / ADC_RESOLUTION * ADC_REFERENCE_VOLTAGE *
           CAPACITOR_VOLTAGE_DIVIDER;
}
