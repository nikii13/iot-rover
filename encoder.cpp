#include <algorithm>
#include <stdexcept>
#include <string>

#include "encoder.hpp"

void encoder_alert(int events, lgGpioAlert_p evt, void *data)
{
    auto *wheel = static_cast<wheel_counter *>(data);

    for (int i = 0; i < events; ++i) {
        const lgGpioReport_t &report = evt[i].report;

        // Ignore watchdog/error reports. For speed we count one rising edge
        // per encoder pulse, matching the lab's ~700 pulses/revolution scale.
        if (report.flags != 0 || report.level != 1) {
            continue;
        }

        if (report.gpio == wheel->pin1) {
            wheel->count1_.fetch_add(1, std::memory_order_relaxed);
        } else if (report.gpio == wheel->pin2) {
            wheel->count2_.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

static void check_lgpio(int status, const char *operation, int gpio)
{
    if (status < 0) {
        throw std::runtime_error(
            std::string(operation) + " failed for GPIO" + std::to_string(gpio) +
            " (lgpio status " + std::to_string(status) + ")"
        );
    }
}

wheel_counter::wheel_counter(int chip, uint8_t p1, uint8_t p2)
    : pin1(p1), pin2(p2)
{
    /*
     * Do not force an internal pull direction here. The rover's encoder
     * interface drives the GPIO signal. This also avoids changing the logic
     * level compared with the wiring already validated on the robot.
     */
    check_lgpio(lgGpioClaimAlert(chip, 0, LG_BOTH_EDGES, pin1, -1),
                "lgGpioClaimAlert", pin1);
    check_lgpio(lgGpioClaimAlert(chip, 0, LG_BOTH_EDGES, pin2, -1),
                "lgGpioClaimAlert", pin2);

    check_lgpio(lgGpioSetAlertsFunc(chip, pin1, encoder_alert, this),
                "lgGpioSetAlertsFunc", pin1);
    check_lgpio(lgGpioSetAlertsFunc(chip, pin2, encoder_alert, this),
                "lgGpioSetAlertsFunc", pin2);
}

int64_t wheel_counter::channel1() const noexcept
{
    return count1_.load(std::memory_order_relaxed);
}

int64_t wheel_counter::channel2() const noexcept
{
    return count2_.load(std::memory_order_relaxed);
}

int64_t wheel_counter::pulses() const noexcept
{
    // If one quadrature channel is dead, the other still works for speed.
    // When both are healthy, use one channel's count rather than summing them,
    // so PPR stays on the ~700 rising-edges/revolution scale.
    return std::max(channel1(), channel2());
}

void wheel_counter::reset() noexcept
{
    count1_.store(0, std::memory_order_relaxed);
    count2_.store(0, std::memory_order_relaxed);
}
