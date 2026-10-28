#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include "encoder.hpp"

#define SOCKET_NAME "/tmp/encoder.sock"

// NikiC rover - BCM GPIO numbers, verified against the physical header:
// Motor A: C1 physical 11 -> GPIO17, C2 physical 13 -> GPIO27
// Motor B: C1 physical 15 -> GPIO22, C2 physical 16 -> GPIO23
constexpr uint8_t GPIO_A1 = 17;
constexpr uint8_t GPIO_A2 = 27;
constexpr uint8_t GPIO_B1 = 22;
constexpr uint8_t GPIO_B2 = 23;

static int chip = -1;

static void cleanup()
{
    remove(SOCKET_NAME);
    if (chip >= 0) {
        const int status = lgGpiochipClose(chip);
        if (status < 0) {
            std::fprintf(stderr, "lgGpiochipClose failed: %d\n", status);
        }
        chip = -1;
    }
}

static int bind_server_socket()
{
    const int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd == -1) {
        std::perror("socket");
        std::exit(EXIT_FAILURE);
    }

    sockaddr_un name{};
    name.sun_family = AF_UNIX;
    std::strncpy(name.sun_path, SOCKET_NAME, sizeof(name.sun_path) - 1);

    remove(SOCKET_NAME);
    if (bind(fd, reinterpret_cast<const sockaddr *>(&name), sizeof(name)) == -1) {
        std::perror("bind");
        close(fd);
        std::exit(EXIT_FAILURE);
    }

    if (chmod(SOCKET_NAME,
              S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH) == -1) {
        std::perror("chmod");
        close(fd);
        std::exit(EXIT_FAILURE);
    }

    return fd;
}

int main()
{
    std::atexit(cleanup);

    chip = lgGpiochipOpen(0);
    if (chip < 0) {
        std::fprintf(stderr, "lgGpiochipOpen(0) failed: %d\n", chip);
        return EXIT_FAILURE;
    }

    try {
        wheel_counter motor_a{chip, GPIO_A1, GPIO_A2};
        wheel_counter motor_b{chip, GPIO_B1, GPIO_B2};

        std::fprintf(stderr,
            "encoderd: A GPIO%d/GPIO%d, B GPIO%d/GPIO%d, socket %s\n",
            GPIO_A1, GPIO_A2, GPIO_B1, GPIO_B2, SOCKET_NAME);

        const int connection = bind_server_socket();
        char buffer[256];

        for (;;) {
            sockaddr_un client{};
            socklen_t length = sizeof(client);

            const ssize_t received = recvfrom(
                connection, buffer, sizeof(buffer) - 1, 0,
                reinterpret_cast<sockaddr *>(&client), &length
            );
            if (received < 0) {
                if (errno == EINTR) continue;
                std::perror("recvfrom");
                return EXIT_FAILURE;
            }

            buffer[received] = '\0';

            int len = 0;
            if (std::strncmp(buffer, "diag", 4) == 0) {
                len = std::snprintf(
                    buffer, sizeof(buffer),
                    "A1=%" PRId64 ",A2=%" PRId64 ",B1=%" PRId64 ",B2=%" PRId64
                    ";A=%" PRId64 ",B=%" PRId64,
                    motor_a.channel1(), motor_a.channel2(),
                    motor_b.channel1(), motor_b.channel2(),
                    motor_a.pulses(), motor_b.pulses()
                );
            } else if (std::strncmp(buffer, "reset", 5) == 0) {
                motor_a.reset();
                motor_b.reset();
                len = std::snprintf(buffer, sizeof(buffer), "0,0");
            } else {
                // Backward compatible with the professor's server protocol.
                len = std::snprintf(
                    buffer, sizeof(buffer),
                    "%" PRId64 ",%" PRId64,
                    motor_a.pulses(), motor_b.pulses()
                );
            }

            if (sendto(connection, buffer, static_cast<size_t>(len), 0,
                       reinterpret_cast<const sockaddr *>(&client), length) == -1) {
                std::perror("sendto");
                return EXIT_FAILURE;
            }
        }
    } catch (const std::exception &e) {
        std::fprintf(stderr, "encoderd: %s\n", e.what());
        return EXIT_FAILURE;
    }
}
