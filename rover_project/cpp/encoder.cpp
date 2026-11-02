// encoder.cpp — fuente del servicio "encodered".
//
// Responsabilidad única: leer los 4 canales de encoder por GPIO y responder a
// peticiones locales por socket UNIX datagrama. NO controla motores.
//
// Protocolo (compatible con el original):
//   Socket servidor : AF_UNIX / SOCK_DGRAM, por defecto /tmp/encoder.sock
//   El cliente debe hacer bind() a su propia ruta (p.ej. /tmp/encoder_client_<pid>.sock)
//   para que el servidor pueda responder con sendto().
//   Petición cualquiera (p.ej. "GET" o vacía)  -> "MotorA_steps,MotorB_steps"
//   Petición "STATUS" (extensión de diagnóstico) ->
//       "A,B;mode=x1;a1=r/f;a2=r/f;b1=r/f;b2=r/f;inv=a,b;rep=a,b;lost=N"
//   Petición "PING" -> "PONG"
//
// GPIO: interfaz de carácter del kernel (uAPI v2, /dev/gpiochipN). No requiere
// root: basta con que el usuario pertenezca al grupo "gpio" (por defecto en
// Raspberry Pi OS). Un solo hilo con poll(): eventos GPIO y peticiones del
// socket se atienden en el mismo bucle, sin necesidad de mutex.
//
// Instancia única: flock() sobre /tmp/encodered.lock. Una segunda instancia
// termina con código 2 sin tocar el socket de la primera.
//
// Compilación: make -C cpp   (genera cpp/encodered)

#include "quadrature.hpp"

#include <linux/gpio.h>
#include <poll.h>
#include <signal.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

volatile sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

struct Options {
    std::string socket_path = "/tmp/encoder.sock";
    std::string lock_path = "/tmp/encodered.lock";
    std::string chip = "auto";
    int pins[4] = {17, 27, 22, 23};  // A1, A2, B1, B2 (BCM)
    DecodeMode mode = DecodeMode::X1;
    std::string bias = "pull-up";
    unsigned debounce_us = 0;
    bool no_gpio = false;
    bool verbose = false;
};

void usage(const char* argv0) {
    std::fprintf(stderr,
        "Uso: %s [opciones]\n"
        "  --socket RUTA        socket del servicio (def. /tmp/encoder.sock)\n"
        "  --lock RUTA          fichero de bloqueo (def. /tmp/encodered.lock)\n"
        "  --chip auto|/dev/gpiochipN\n"
        "  --pins A1,A2,B1,B2   GPIO BCM (def. 17,27,22,23)\n"
        "  --mode pulse|x1|x2|x4  decodificación (def. x1)\n"
        "  --bias pull-up|pull-down|disabled|as-is (def. pull-up)\n"
        "  --debounce-us N      antirrebote del kernel (0 = desactivado)\n"
        "  --no-gpio            no abre GPIO (solo pruebas de protocolo)\n"
        "  --verbose\n", argv0);
}

bool parse_args(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto need = [&](const char* name) -> const char* {
            if (i + 1 >= argc) { std::fprintf(stderr, "Falta valor para %s\n", name); std::exit(1); }
            return argv[++i];
        };
        if (a == "--socket") o.socket_path = need("--socket");
        else if (a == "--lock") o.lock_path = need("--lock");
        else if (a == "--chip") o.chip = need("--chip");
        else if (a == "--pins") {
            const char* v = need("--pins");
            if (std::sscanf(v, "%d,%d,%d,%d", &o.pins[0], &o.pins[1], &o.pins[2], &o.pins[3]) != 4) {
                std::fprintf(stderr, "--pins requiere 4 números separados por comas\n");
                return false;
            }
        } else if (a == "--mode") {
            std::string m = need("--mode");
            if (m == "pulse") o.mode = DecodeMode::PULSE;
            else if (m == "x1") o.mode = DecodeMode::X1;
            else if (m == "x2") o.mode = DecodeMode::X2;
            else if (m == "x4") o.mode = DecodeMode::X4;
            else { std::fprintf(stderr, "Modo desconocido: %s\n", m.c_str()); return false; }
        } else if (a == "--bias") o.bias = need("--bias");
        else if (a == "--debounce-us") o.debounce_us = (unsigned)std::strtoul(need("--debounce-us"), nullptr, 10);
        else if (a == "--no-gpio") o.no_gpio = true;
        else if (a == "--verbose") o.verbose = true;
        else if (a == "-h" || a == "--help") { usage(argv[0]); std::exit(0); }
        else { std::fprintf(stderr, "Opción desconocida: %s\n", a.c_str()); return false; }
    }
    return true;
}

// Busca el gpiochip que corresponde al conector de 40 pines.
std::string find_header_chip() {
    const char* known[] = {"pinctrl-rp1", "pinctrl-bcm2711", "pinctrl-bcm2835", "pinctrl-bcm2712"};
    for (int n = 0; n < 16; ++n) {
        std::string path = "/dev/gpiochip" + std::to_string(n);
        int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
        if (fd < 0) continue;
        gpiochip_info info{};
        if (ioctl(fd, GPIO_GET_CHIPINFO_IOCTL, &info) == 0) {
            for (const char* k : known) {
                if (std::strstr(info.label, k) != nullptr && info.lines >= 28) {
                    ::close(fd);
                    return path;
                }
            }
        }
        ::close(fd);
    }
    return "/dev/gpiochip0";
}

class GpioLines {
public:
    ~GpioLines() { if (fd_ >= 0) ::close(fd_); }

    bool open(const Options& o) {
        std::string chip = (o.chip == "auto") ? find_header_chip() : o.chip;
        int chip_fd = ::open(chip.c_str(), O_RDWR | O_CLOEXEC);
        if (chip_fd < 0) {
            std::fprintf(stderr, "[encodered] No se puede abrir %s: %s\n"
                                 "  ¿El usuario pertenece al grupo 'gpio'? (groups $USER)\n",
                         chip.c_str(), std::strerror(errno));
            return false;
        }
        gpio_v2_line_request req{};
        for (int i = 0; i < 4; ++i) req.offsets[i] = (uint32_t)o.pins[i];
        req.num_lines = 4;
        std::snprintf(req.consumer, sizeof(req.consumer), "encodered");
        req.event_buffer_size = 1024;  // evita desbordes a altas RPM
        uint64_t flags = GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_EDGE_RISING |
                         GPIO_V2_LINE_FLAG_EDGE_FALLING;
        if (o.bias == "pull-up") flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_UP;
        else if (o.bias == "pull-down") flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN;
        else if (o.bias == "disabled") flags |= GPIO_V2_LINE_FLAG_BIAS_DISABLED;
        req.config.flags = flags;
        if (o.debounce_us > 0) {
            req.config.num_attrs = 1;
            req.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_DEBOUNCE;
            req.config.attrs[0].attr.debounce_period_us = o.debounce_us;
            req.config.attrs[0].mask = 0xF;
        }
        if (ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
            std::fprintf(stderr, "[encodered] Error solicitando GPIO %d,%d,%d,%d en %s: %s\n"
                                 "  (¿líneas ocupadas por otro proceso o mal configuradas?)\n",
                         o.pins[0], o.pins[1], o.pins[2], o.pins[3], chip.c_str(), std::strerror(errno));
            ::close(chip_fd);
            return false;
        }
        ::close(chip_fd);
        fd_ = req.fd;
        chip_ = chip;
        return true;
    }

    bool read_levels(int levels[4]) const {
        gpio_v2_line_values v{};
        v.mask = 0xF;
        if (ioctl(fd_, GPIO_V2_LINE_GET_VALUES_IOCTL, &v) < 0) return false;
        for (int i = 0; i < 4; ++i) levels[i] = (v.bits >> i) & 1;
        return true;
    }

    int fd() const { return fd_; }
    const std::string& chip() const { return chip_; }

private:
    int fd_ = -1;
    std::string chip_;
};

class Service {
public:
    explicit Service(const Options& o) : o_(o), motors_{QuadratureDecoder(o.mode), QuadratureDecoder(o.mode)} {}

    ~Service() { cleanup(); }

    bool acquire_lock() {
        lock_fd_ = ::open(o_.lock_path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0666);
        if (lock_fd_ < 0) {
            std::fprintf(stderr, "[encodered] No se puede crear %s: %s\n", o_.lock_path.c_str(), std::strerror(errno));
            return false;
        }
        if (flock(lock_fd_, LOCK_EX | LOCK_NB) < 0) {
            std::fprintf(stderr, "[encodered] Ya hay otra instancia en ejecución (%s bloqueado).\n",
                         o_.lock_path.c_str());
            ::close(lock_fd_);
            lock_fd_ = -1;
            return false;
        }
        char buf[32];
        int n = std::snprintf(buf, sizeof(buf), "%d\n", (int)getpid());
        if (ftruncate(lock_fd_, 0) == 0) { ssize_t w = ::write(lock_fd_, buf, (size_t)n); (void)w; }
        return true;
    }

    bool open_socket() {
        sock_ = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        if (sock_ < 0) { std::perror("[encodered] socket"); return false; }
        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        if (o_.socket_path.size() >= sizeof(addr.sun_path)) {
            std::fprintf(stderr, "[encodered] Ruta de socket demasiado larga\n");
            return false;
        }
        std::strcpy(addr.sun_path, o_.socket_path.c_str());
        ::unlink(o_.socket_path.c_str());  // seguro: tenemos el lock, un socket existente es residual
        if (::bind(sock_, (sockaddr*)&addr, sizeof(addr)) < 0) {
            std::perror("[encodered] bind");
            return false;
        }
        socket_bound_ = true;
        ::chmod(o_.socket_path.c_str(), 0666);  // clientes sin root pueden enviar
        return true;
    }

    bool open_gpio() {
        if (o_.no_gpio) {
            std::fprintf(stderr, "[encodered] Modo --no-gpio: los contadores permanecerán en 0.\n");
            return true;
        }
        if (!gpio_.open(o_)) return false;
        int lv[4];
        if (gpio_.read_levels(lv)) {
            motors_[0].set_initial_levels(lv[0], lv[1]);
            motors_[1].set_initial_levels(lv[2], lv[3]);
            std::fprintf(stderr, "[encodered] Niveles iniciales A1=%d A2=%d B1=%d B2=%d\n", lv[0], lv[1], lv[2], lv[3]);
        }
        std::fprintf(stderr, "[encodered] GPIO %s pines A=%d,%d B=%d,%d modo=%s bias=%s\n",
                     gpio_.chip().c_str(), o_.pins[0], o_.pins[1], o_.pins[2], o_.pins[3],
                     mode_name(o_.mode), o_.bias.c_str());
        return true;
    }

    int run() {
        std::fprintf(stderr, "[encodered] Escuchando en %s (pid %d)\n", o_.socket_path.c_str(), (int)getpid());
        pollfd fds[2];
        int nfds = 0;
        fds[nfds++] = {sock_, POLLIN, 0};
        if (gpio_.fd() >= 0) fds[nfds++] = {gpio_.fd(), POLLIN, 0};

        while (!g_stop) {
            int r = ::poll(fds, (nfds_t)nfds, 200);
            if (r < 0) {
                if (errno == EINTR) continue;
                std::perror("[encodered] poll");
                return 1;
            }
            if (nfds > 1 && (fds[1].revents & POLLIN)) drain_events();
            if (fds[0].revents & POLLIN) handle_request();
        }
        std::fprintf(stderr, "[encodered] Señal recibida, terminando.\n");
        return 0;
    }

    void cleanup() {
        if (sock_ >= 0) { ::close(sock_); sock_ = -1; }
        if (socket_bound_) { ::unlink(o_.socket_path.c_str()); socket_bound_ = false; }
        if (lock_fd_ >= 0) { ::close(lock_fd_); lock_fd_ = -1; }
    }

private:
    void drain_events() {
        gpio_v2_line_event ev[64];
        ssize_t n = ::read(gpio_.fd(), ev, sizeof(ev));
        if (n <= 0) return;
        size_t count = (size_t)n / sizeof(gpio_v2_line_event);
        for (size_t i = 0; i < count; ++i) {
            if (last_seqno_ != 0 && ev[i].seqno != last_seqno_ + 1)
                lost_ += ev[i].seqno - last_seqno_ - 1;  // eventos perdidos por desborde
            last_seqno_ = ev[i].seqno;
            const bool rising = ev[i].id == GPIO_V2_LINE_EVENT_RISING_EDGE;
            const int off = (int)ev[i].offset;
            if (off == o_.pins[0]) motors_[0].on_edge(0, rising);
            else if (off == o_.pins[1]) motors_[0].on_edge(1, rising);
            else if (off == o_.pins[2]) motors_[1].on_edge(0, rising);
            else if (off == o_.pins[3]) motors_[1].on_edge(1, rising);
        }
    }

    void handle_request() {
        char buf[64];
        sockaddr_un from{};
        socklen_t fromlen = sizeof(from);
        ssize_t n = ::recvfrom(sock_, buf, sizeof(buf) - 1, 0, (sockaddr*)&from, &fromlen);
        if (n < 0) return;
        buf[n] = '\0';
        // Quitar salto de línea final si lo hubiera.
        while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r')) buf[--n] = '\0';

        if (fromlen <= (socklen_t)sizeof(sa_family_t)) {
            unaddressed_++;  // cliente sin bind(): no hay a quién responder
            if (o_.verbose) std::fprintf(stderr, "[encodered] Petición sin dirección de respuesta\n");
            return;
        }

        char out[256];
        int len;
        if (std::strcmp(buf, "STATUS") == 0) {
            const auto& a = motors_[0];
            const auto& b = motors_[1];
            len = std::snprintf(out, sizeof(out),
                "%lld,%lld;mode=%s;a1=%llu/%llu;a2=%llu/%llu;b1=%llu/%llu;b2=%llu/%llu;"
                "inv=%llu,%llu;rep=%llu,%llu;lost=%llu",
                (long long)a.count(), (long long)b.count(), mode_name(o_.mode),
                (unsigned long long)a.stats(0).rising, (unsigned long long)a.stats(0).falling,
                (unsigned long long)a.stats(1).rising, (unsigned long long)a.stats(1).falling,
                (unsigned long long)b.stats(0).rising, (unsigned long long)b.stats(0).falling,
                (unsigned long long)b.stats(1).rising, (unsigned long long)b.stats(1).falling,
                (unsigned long long)a.invalid_transitions(), (unsigned long long)b.invalid_transitions(),
                (unsigned long long)a.repeated_edges(), (unsigned long long)b.repeated_edges(),
                (unsigned long long)lost_);
        } else if (std::strcmp(buf, "PING") == 0) {
            len = std::snprintf(out, sizeof(out), "PONG");
        } else {
            len = std::snprintf(out, sizeof(out), "%lld,%lld",
                                (long long)motors_[0].count(), (long long)motors_[1].count());
        }
        if (::sendto(sock_, out, (size_t)len, MSG_DONTWAIT, (sockaddr*)&from, fromlen) < 0 && o_.verbose)
            std::fprintf(stderr, "[encodered] sendto: %s\n", std::strerror(errno));
    }

    Options o_;
    GpioLines gpio_;
    QuadratureDecoder motors_[2];
    int sock_ = -1;
    int lock_fd_ = -1;
    bool socket_bound_ = false;
    uint64_t last_seqno_ = 0;
    uint64_t lost_ = 0;
    uint64_t unaddressed_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
    Options o;
    if (!parse_args(argc, argv, o)) { usage(argv[0]); return 1; }

    struct sigaction sa{};
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    signal(SIGPIPE, SIG_IGN);

    Service svc(o);
    if (!svc.acquire_lock()) return 2;
    if (!svc.open_gpio()) return 3;
    if (!svc.open_socket()) return 4;
    int rc = svc.run();
    svc.cleanup();
    return rc;
}
