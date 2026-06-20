#include "charger.h"

bool Charger::has_seen_done_since_last_charge_edge = false;
uint32_t Charger::last_charge_edge_ms              = 0;

Charger::Charger()
{
    pinMode(FLYBACK_FAULT, INPUT);

    // DONE is active-low, I think we already have a pullup but just incase
    pinMode(CHRG_DONE, INPUT_PULLUP);

    pinMode(CHRG, OUTPUT);
    digitalWrite(CHRG, LOW);

    // ADS7945 bit-banged SPI pins.
    pinMode(ADC_CS, OUTPUT);
    digitalWrite(ADC_CS, HIGH);

    pinMode(ADC_SCLK, OUTPUT);
    digitalWrite(ADC_SCLK, LOW);

    pinMode(ADC_MISO, INPUT);

    // Do not touch CH_SEL in this proof-of-concept.
    // Hardware should pull CH_SEL low to select CH0.
}

void Charger::chargeCapacitors()
{
    // LT3750 charging is initiated by a CHARGE rising edge.
    digitalWrite(CHRG, LOW);
    delayMicroseconds(LT3750_CHARGE_LOW_TIME_US);
    digitalWrite(CHRG, HIGH);

    // last_charge_edge_ms = millis();
    // has_seen_done_since_last_charge_edge = false;
}

void Charger::maintainCharge()
{
    const bool done = getChargeDone();

    // if (done)
    // {
    //     has_seen_done_since_last_charge_edge = true;
    //     return;
    // }

    // Minimal "continuous charging" approximation:
    // If the LT3750 previously reached DONE and later DONE deasserts, produce
    // another rising edge to start a new charge cycle.
    // if (has_seen_done_since_last_charge_edge &&
    //     (millis() - last_charge_edge_ms) > MIN_CHARGE_RETRIGGER_INTERVAL_MS)
    // {
    //     chargeCapacitors();
    // }
    
    if ((millis() - last_charge_edge_ms) > MIN_CHARGE_RETRIGGER_INTERVAL_MS )
    {
        chargeCapacitors();
    }
    
}

bool Charger::getChargeDone()
{
    // LT3750 DONE is active low.
    return digitalRead(CHRG_DONE) == LOW;
}

uint16_t Charger::readAds7945WordBitBang()
{
    uint16_t raw = 0;

    digitalWrite(ADC_SCLK, LOW);
    digitalWrite(ADC_CS, LOW);
    delayMicroseconds(ADC_CS_DELAY_US);

    for (int i = 0; i < 16; i++)
    {
        // ADS7945 MSB is available after CS falling edge; subsequent bits update
        // after SCLK falling edges. Sampling while SCLK is high is the intended
        // first simple mode.
        digitalWrite(ADC_SCLK, HIGH);
        delayMicroseconds(ADC_CLOCK_DELAY_US);

        raw <<= 1;
        raw |= digitalRead(ADC_MISO) ? 1 : 0;

        digitalWrite(ADC_SCLK, LOW);
        delayMicroseconds(ADC_CLOCK_DELAY_US);
    }

    digitalWrite(ADC_CS, HIGH);
    delayMicroseconds(ADC_CS_DELAY_US);

    return raw;
}

int16_t Charger::signExtend14(uint16_t raw14)
{
    raw14 &= 0x3FFF;

    if (raw14 & 0x2000)
    {
        return static_cast<int16_t>(raw14 | 0xC000);
    }

    return static_cast<int16_t>(raw14);
}

int16_t Charger::readAds7945SignedCode()
{
    // ADS7945 outputs the result of the previous conversion.
    // Since CH_SEL is fixed to CH0 and capacitor voltage changes slowly, this is fine.
    const uint16_t raw16 = readAds7945WordBitBang();
    const uint16_t raw14 = (raw16 >> ADC_RESULT_RIGHT_SHIFT) & 0x3FFF;

    return signExtend14(raw14);
}

float Charger::adcCodeToDifferentialVoltage(float code)
{
    return code * ADC_VREF / static_cast<float>(ADC_HALF_SCALE);
}

float Charger::adcDifferentialVoltageToCapacitorVoltage(float v_adc_diff)
{
    // ADS7945 measures ISO224 differential output.
    // ISO224: Vout_diff = Vin / 3, so Vin = Vout_diff / ISO224_GAIN.
    const float v_iso_input = v_adc_diff / ISO224_GAIN;

    // ISO224 input is the divided capacitor voltage.
    return v_iso_input / DIVIDER_RATIO;
}

float Charger::getCapacitorVoltage()
{
    static constexpr int NUM_SAMPLES = 8;

    // Calibration from scope measurements:
    // diagnostic 286.0 V corresponds to actual 196.44 V.
    // This linear fit is applied after converting the raw ADC code to a voltage which already 
    // accounts for the hardware gain and divider
    static constexpr float CAP_CAL_ANCHOR_DIAGNOSTIC_V = 286.0f;
    static constexpr float CAP_CAL_ANCHOR_ACTUAL_V     = 196.44f;
    static constexpr float CAP_CAL_SLOPE               = 0.7019289f;

    // ADS7945 returns the previous conversion, so discard one frame first.
    (void)readAds7945SignedCode();

    long code_sum = 0;

    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        code_sum += readAds7945SignedCode();
    }

    const float avg_code =
        static_cast<float>(code_sum) / static_cast<float>(NUM_SAMPLES);

    const float v_adc_diff = adcCodeToDifferentialVoltage(avg_code);

    // Uncalibrated diagnostic voltage.
    const float raw_capacitor_voltage =
        adcDifferentialVoltageToCapacitorVoltage(v_adc_diff);

    // Convert diagnostic voltage to calibrated physical capacitor voltage.
    const float calibrated_capacitor_voltage =
        CAP_CAL_ANCHOR_ACTUAL_V +
        CAP_CAL_SLOPE *
            (raw_capacitor_voltage - CAP_CAL_ANCHOR_DIAGNOSTIC_V);

    // Prevent a small negative reported voltage near 0 V.
    const float physical_capacitor_voltage =
        (calibrated_capacitor_voltage < 0.0f)
            ? 0.0f
            : calibrated_capacitor_voltage;

    // Initialize from the first real reading so startup does not ramp from 0 V.
    if (!capacitor_voltage_ema_initialized_)
    {
        capacitor_voltage_ema_             = physical_capacitor_voltage;
        capacitor_voltage_ema_initialized_ = true;
    }
    else
    {
        capacitor_voltage_ema_ +=
            CAP_VOLTAGE_EMA_ALPHA *
            (physical_capacitor_voltage - capacitor_voltage_ema_);
    }
    return capacitor_voltage_ema_;
}

bool Charger::getFlybackFault()
{
    return !digitalRead(FLYBACK_FAULT);
}