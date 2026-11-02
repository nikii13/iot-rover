// Pruebas unitarias del decodificador (sin hardware). make -C cpp test
#include "quadrature.hpp"
#include <cstdio>
#include <cstdlib>

static int failures = 0;
#define CHECK_EQ(a, b) do { long long _a = (long long)(a), _b = (long long)(b); \
    if (_a != _b) { std::printf("FALLO %s:%d  %s = %lld, esperado %lld\n", __FILE__, __LINE__, #a, _a, _b); ++failures; } } while (0)

// Genera N ciclos completos de cuadratura. forward: ch1 adelantado.
static void spin(QuadratureDecoder& d, int cycles, bool forward) {
    for (int i = 0; i < cycles; ++i) {
        if (forward) {
            d.on_edge(0, true); d.on_edge(1, true); d.on_edge(0, false); d.on_edge(1, false);
        } else {
            d.on_edge(1, true); d.on_edge(0, true); d.on_edge(1, false); d.on_edge(0, false);
        }
    }
}

// Solo el canal 1 cambia (canal 2 muerto, fijo en bajo).
static void spin_dead_ch2(QuadratureDecoder& d, int cycles) {
    for (int i = 0; i < cycles; ++i) { d.on_edge(0, true); d.on_edge(0, false); }
}

int main() {
    { QuadratureDecoder d(DecodeMode::X1); spin(d, 100, true);  CHECK_EQ(d.count(), 100); }
    { QuadratureDecoder d(DecodeMode::X1); spin(d, 100, false); CHECK_EQ(d.count(), -100); }
    { QuadratureDecoder d(DecodeMode::X2); spin(d, 100, true);  CHECK_EQ(d.count(), 200); }
    { QuadratureDecoder d(DecodeMode::X2); spin(d, 100, false); CHECK_EQ(d.count(), -200); }
    { QuadratureDecoder d(DecodeMode::X4); spin(d, 100, true);  CHECK_EQ(d.count(), 400); CHECK_EQ(d.invalid_transitions(), 0); }
    { QuadratureDecoder d(DecodeMode::X4); spin(d, 100, false); CHECK_EQ(d.count(), -400); }
    { QuadratureDecoder d(DecodeMode::PULSE); spin(d, 100, false); CHECK_EQ(d.count(), 100); }

    // Canal 2 muerto: PULSE y X1 siguen contando en módulo; X4 se anula (síntoma del "0 pulsos").
    { QuadratureDecoder d(DecodeMode::PULSE); spin_dead_ch2(d, 50); CHECK_EQ(d.count(), 50); }
    { QuadratureDecoder d(DecodeMode::X1); spin_dead_ch2(d, 50); CHECK_EQ(d.count(), 50); }
    { QuadratureDecoder d(DecodeMode::X4); spin_dead_ch2(d, 50); CHECK_EQ(d.count(), 0);
      CHECK_EQ(d.stats(0).edges(), 100); CHECK_EQ(d.stats(1).edges(), 0); }

    // Estadísticas por canal.
    { QuadratureDecoder d(DecodeMode::X1); spin(d, 10, true);
      CHECK_EQ(d.stats(0).rising, 10); CHECK_EQ(d.stats(1).falling, 10); }

    // Flanco repetido (evento perdido) se registra.
    { QuadratureDecoder d(DecodeMode::X4); d.on_edge(0, true); d.on_edge(0, true); CHECK_EQ(d.repeated_edges(), 1); }

    if (failures == 0) std::printf("test_quadrature: OK\n");
    return failures == 0 ? 0 : 1;
}
