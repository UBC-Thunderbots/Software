#include "charger.h"

Charger::Charger()
{
    // Currently unused
    pinMode(FLYBACK_FAULT, INPUT);

    // DONE is active-low, also have hardware pullup on this pin.
    pinMode(CHRG_DONE, INPUT_PULLUP);

    pinMode(CHRG, OUTPUT);
    digitalWrite(CHRG, LOW);

    // ADS7945 bit-banged SPI pins.
    pinMode(ADC_CS, OUTPUT);
    digitalWrite(ADC_CS, HIGH);

    pinMode(ADC_SCLK, OUTPUT);
    digitalWrite(ADC_SCLK, LOW);

    pinMode(ADC_MISO, INPUT);

    // Hardware pulls CH_SEL low to select CH0.
    // Can be changed later to include temperature sensing.
}

void Charger::chargeCapacitors()
{
    // LT3750 charging is initiated by a CHARGE rising edge.
    
    if (charge_inhibited_)
    {
        return;
    }

    digitalWrite(CHRG, LOW);
    delayMicroseconds(LT3750_CHARGE_LOW_TIME_US);
    digitalWrite(CHRG, HIGH);

    charging_enabled_ = true;
    last_charge_edge_ms_ = millis();
}

void Charger::stopCharging()
{
    digitalWrite(CHRG, LOW);
    charging_enabled_ = false;
}

void Charger::setChargeInhibited(const bool inhibited)
{
    charge_inhibited_ = inhibited;

    if (charge_inhibited_)
    {
        stopCharging();
    }
}

void Charger::maintainCharge()
{
    // Do not enable charging until one valid capacitor-voltage measurement exists.
    if (!capacitor_voltage_control_initialized_)
    {
        return;
    }

    // During kick/chip guard and pulse time, do not charge.
    if (charge_inhibited_)
    {
        stopCharging();
        return;
    }

    const float voltage = capacitor_voltage_control_;
    const uint32_t now  = millis();

    if (voltage >= CHARGE_STOP_VOLTAGE_V)
    {
        stopCharging();
        return;
    }

    if (voltage <= CHARGE_RESTART_VOLTAGE_V)
    {
        const bool retry_due =
            (now - last_charge_edge_ms_) >= CHARGE_RETRY_INTERVAL_MS;

        if (!charging_enabled_ || retry_due)
        {
            chargeCapacitors();
        }
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
    // Consider modifying if temperature sensing is added.
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

    // Calibration from scope measurements
    // diagnostic 286.0 V corresponds to actual 196.44 V.
    // This linear fit is applied after converting the raw ADC code to a voltage which
    // already accounts for the hardware gain and divider
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

    const float avg_code = static_cast<float>(code_sum) / static_cast<float>(NUM_SAMPLES);

    const float v_adc_diff = adcCodeToDifferentialVoltage(avg_code);

    // Uncalibrated diagnostic voltage.
    const float raw_capacitor_voltage =
        adcDifferentialVoltageToCapacitorVoltage(v_adc_diff);

    // Convert diagnostic voltage to calibrated physical capacitor voltage.
    const float calibrated_capacitor_voltage =
        CAP_CAL_ANCHOR_ACTUAL_V +
        CAP_CAL_SLOPE * (raw_capacitor_voltage - CAP_CAL_ANCHOR_DIAGNOSTIC_V);

    // Prevent a small negative reported voltage near 0 V.
    const float physical_capacitor_voltage =
        (calibrated_capacitor_voltage < 0.0f) ? 0.0f : calibrated_capacitor_voltage;

    // Non EMA value used in maintainCharge() logic for hysteresis.
    capacitor_voltage_control_ = physical_capacitor_voltage;
    capacitor_voltage_control_initialized_ = true;

    // Initialize from the first real reading so startup does not ramp from 0 V.
    if (!capacitor_voltage_ema_initialized_)
    {
        capacitor_voltage_ema_             = physical_capacitor_voltage;
        capacitor_voltage_ema_initialized_ = true;
    }
    else
    {
        capacitor_voltage_ema_ +=
            CAP_VOLTAGE_EMA_ALPHA * (physical_capacitor_voltage - capacitor_voltage_ema_);
    }
    return capacitor_voltage_ema_;
}

bool Charger::getFlybackFault()
{
    return !digitalRead(FLYBACK_FAULT);
}
