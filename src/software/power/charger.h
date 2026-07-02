#pragma once

#include <pins.h>

/*
 * Represents the charger and capacitor voltage sensing on the power board.
 * capacitor voltage -> divider -> ISO224 -> ADS7945 CH0 -> ESP32 GPIO5/18/21
 */
class Charger
{
   public:
    /**
     * Creates a Charger, setting up pins.
     */
    Charger();

    /**
     * Starts/restarts an LT3750 charge cycle by creating a low -> high edge
     * on CHRG. Does nothing while charging is inhibited.
     *
     * New edges may be issued periodically while voltage remains below target threshold.
     */
    void chargeCapacitors();

    /**
     * Immediately disables the LT3750 by holding CHRG low.
     */
    void stopCharging();

    /**
     * Forces the charger off while true. Used before and during kicker pulses.
     */
    void setChargeInhibited(bool inhibited);

    /**
     * Maintains capacitor voltage using hysteresis and the latest unfiltered
     * calibrated voltage reading.
     */
    void maintainCharge();

    /**
     * Returns the EMA-smoothed capacitor voltage for telemetry.
     * Also updates the internal unfiltered control voltage used by maintainCharge().
     */
    float getCapacitorVoltage();

    /**
     * Returns whether the flyback fault is tripped.
     * Kept for compatibility with previous firmware, but unused.
     */
    bool getFlybackFault();

    /**
     * Returns true when LT3750 DONE is active.
     * Not supported by all boards, thus unused.
     */
    static bool getChargeDone();

   private:
    static uint16_t readAds7945WordBitBang();
    static int16_t readAds7945SignedCode();
    static int16_t signExtend14(uint16_t raw14);
    static float adcCodeToDifferentialVoltage(float code);
    static float adcDifferentialVoltageToCapacitorVoltage(float v_adc_diff);

    // ADS7945 / analog constants.
    //
    // 5.0f for REF5050.
    static constexpr float ADC_VREF = 5.0f;

    static constexpr int ADC_BITS       = 14;
    static constexpr int ADC_HALF_SCALE = 1 << (ADC_BITS - 1);  // 8192

    // ADS7945 14-bit result is expected to be left-aligned in the 16-clock frame:
    // raw16 bits [15:2] = D13..D0.
    static constexpr int ADC_RESULT_RIGHT_SHIFT = 2;

    // ISO224 nominal gain: VOUT_DIFF = VIN / 3.
    static constexpr float ISO224_GAIN = 1.0f / 3.0f;

    // High-voltage divider.
    static constexpr float DIVIDER_R_TOP_OHMS    = 230000.0f;
    static constexpr float DIVIDER_R_BOTTOM_OHMS = 8200.0f;
    static constexpr float DIVIDER_RATIO =
        DIVIDER_R_BOTTOM_OHMS / (DIVIDER_R_TOP_OHMS + DIVIDER_R_BOTTOM_OHMS);

    // Bit-banged SPI timing. Slow and conservative.
    static constexpr uint32_t ADC_CLOCK_DELAY_US = 2;
    static constexpr uint32_t ADC_CS_DELAY_US    = 2;

    // LT3750 timing.
    static constexpr uint32_t LT3750_CHARGE_LOW_TIME_US = 25;  // datasheet min is 20 us

    // Capacitor voltage control for maintainCharge() hysteresis.
    static constexpr float CHARGE_STOP_VOLTAGE_V    = 198.0f;
    static constexpr float CHARGE_RESTART_VOLTAGE_V = 190.0f;

    // EMA is used for telemetry smoothing, not used for maintainCharge() logic.
    static constexpr float CAP_VOLTAGE_EMA_ALPHA =
        0.02f;  // smaller is smoother but slower to respond

    float capacitor_voltage_control_            = 0.0f;
    bool capacitor_voltage_control_initialized_ = false;

    float capacitor_voltage_ema_            = 0.0f;
    bool capacitor_voltage_ema_initialized_ = false;

    bool charging_enabled_ = false;
    bool charge_inhibited_ = false;

    // Time of most recent CHRG edge
    uint32_t last_charge_edge_ms_ = 0;

    // While voltage is below CHARGE_RESTART_VOLTAGE_V,
    // retry charge edges at interval
    static constexpr uint32_t CHARGE_RETRY_INTERVAL_MS = 250;
};
