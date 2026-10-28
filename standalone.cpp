#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <exception>

#include "encoder.hpp"

constexpr uint8_t GPIO_A1 = 17;
constexpr uint8_t GPIO_A2 = 27;
constexpr uint8_t GPIO_B1 = 22;
constexpr uint8_t GPIO_B2 = 23;

int main()
{
    const int chip = lgGpiochipOpen(0);
    if (chip < 0) {
        std::fprintf(stderr, "lgGpiochipOpen(0) failed: %d\n", chip);
        return EXIT_FAILURE;
    }

    try {
        wheel_counter motor_a{chip, GPIO_A1, GPIO_A2};
        wheel_counter motor_b{chip, GPIO_B1, GPIO_B2};

        std::puts("Counting encoder rising edges. Ctrl+C to stop.");
        std::puts("A/B uses the healthiest channel automatically.");

        for (;;) {
            lguSleep(0.5);
            std::printf(
                "A=%" PRId64 " (C1=%" PRId64 ", C2=%" PRId64 ")  "
                "B=%" PRId64 " (C1=%" PRId64 ", C2=%" PRId64 ")\n",
                motor_a.pulses(), motor_a.channel1(), motor_a.channel2(),
                motor_b.pulses(), motor_b.channel1(), motor_b.channel2()
            );
            std::fflush(stdout);
        }
    } catch (const std::exception &e) {
        std::fprintf(stderr, "encoder: %s\n", e.what());
        lgGpiochipClose(chip);
        return EXIT_FAILURE;
    }
}
