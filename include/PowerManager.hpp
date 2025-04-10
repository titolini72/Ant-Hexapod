/**
 * @file PowerManager.h
 * @author Pierre-Yves Mordret <titolini72@gmail.com>
 * @brief Defines the PowerManager class for monitoring and managing power (battery voltage) and alerting when below threshold.
 * @version 1.0
 * @date 2026-01-02
 * @copyright Copyright (c) 2026 Pierre-Yves Mordret
 */

#ifndef POWERMANAGER_H
#define POWERMANAGER_H

#include <Arduino.h>
#include "../config.h"

class PowerManager {
public:
    /**
     * @brief Construct a new PowerManager object and initialize the analog pin for voltage monitoring.
     * @param pin Analog pin number to read battery voltage from.
     * @param delay Minimum delay (ms) between voltage checks.
     * @param threshold Voltage threshold for alert (in volts).
     * @param ratio Voltage divider ratio to scale ADC reading to real voltage.
     */
    PowerManager(unsigned char pin, int delay, float threshold, float ratio);

    /**
     * @brief Set the analog pin for voltage monitoring.
     * @param pin Analog pin number.
     */
    void set_pin(unsigned char pin);

    /**
     * @brief Set the voltage threshold for alerting.
     * @param threshold Voltage threshold (in volts).
     */
    void set_threshold(float threshold);

    /**
     * @brief Set the voltage divider ratio.
     * @param ratio Ratio to convert ADC reading to real voltage.
     */
    void set_ratio(float ratio);

    /**
     * @brief Set the minimum delay between voltage checks.
     * @param delay Delay in milliseconds.
     */
    void set_delay(int delay);

    /**
     * @brief Main loop for monitoring battery voltage and updating alert status.
     *        Should be called regularly in the main program loop.
     *        Sets the alert flag if voltage is below the threshold.
     */
    void loop();

    /**
     * @brief Get the current alert status.
     * @return true if voltage is below threshold, false otherwise.
     */
    bool alert() const { return _alert; }

    /**
     * @brief Get the last measured voltage.
     * @return Last measured voltage value.
     */
    float last_voltage() const { return _last_voltage; }

private:
    unsigned char _pin;
    int _delay;
    float _threshold;
    float _ratio;
    unsigned long _now;
    bool _alert;
    float _last_voltage = 0.0;
};

#endif // POWERMANAGER_H
