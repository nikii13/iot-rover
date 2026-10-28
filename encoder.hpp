#pragma once

#include <atomic>
#include <cstdint>

#include <lgpio.h>

/*
 * Speed-oriented wheel encoder counter.
 *
 * Lab 7 needs reliable wheel speed. Direction is already known from the
 * motor command, so we do not require both quadrature channels to be alive.
 * We count rising edges independently on C1 and C2 and use whichever channel
 * has counted more pulses. If both channels are healthy they should agree
 * closely; if one channel is missing, the other still provides RPM.
 */
class wheel_counter {
public:
    const uint8_t pin1;
    const uint8_t pin2;

    wheel_counter(int chip, uint8_t pin1, uint8_t pin2);
    wheel_counter() = delete;
    wheel_counter(const wheel_counter&) = delete;
    wheel_counter& operator=(const wheel_counter&) = delete;

    int64_t channel1() const noexcept;
    int64_t channel2() const noexcept;
    int64_t pulses() const noexcept;
    void reset() noexcept;

private:
    std::atomic_int64_t count1_{0};
    std::atomic_int64_t count2_{0};

    friend void encoder_alert(int events, lgGpioAlert_p evt, void *data);
};
