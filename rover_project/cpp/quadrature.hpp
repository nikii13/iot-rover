// quadrature.hpp — Decodificador de encoder de un motor (sin dependencias de hardware).
//
// Se mantiene separado de encoder.cpp para poder probarlo sin Raspberry Pi
// (ver cpp/test_quadrature.cpp).
//
// Modos de decodificación:
//   PULSE : cuenta flancos de subida del canal 1, siempre +1. No da sentido de giro.
//           Es el modo más robusto si el canal 2 no funciona.
//   X1    : flanco de subida del canal 1; el sentido se toma del nivel del canal 2.
//           Mismo número de cuentas que PULSE. Con el canal 2 muerto, el sentido
//           sale constante (pero las cuentas siguen siendo correctas en módulo).
//   X2    : ambos flancos del canal 1; sentido con el canal 2. 2x cuentas que X1.
//   X4    : todos los flancos de ambos canales (cuadratura completa). 4x cuentas que X1.
//           ATENCIÓN: si un canal está muerto, X4 suma +1/-1 alternativamente y el
//           contador neto se queda en ~0 (síntoma compatible con "motor A = 0 pulsos").
//
// Convención de sentido: canal 1 adelantado respecto al canal 2 => +1.
#pragma once
#include <cstdint>

enum class DecodeMode { PULSE, X1, X2, X4 };

struct ChannelStats {
    uint64_t rising = 0;
    uint64_t falling = 0;
    uint64_t edges() const { return rising + falling; }
};

class QuadratureDecoder {
public:
    explicit QuadratureDecoder(DecodeMode mode = DecodeMode::X1) : mode_(mode) {}

    void set_initial_levels(int ch1, int ch2) {
        ch1_ = ch1 ? 1 : 0;
        ch2_ = ch2 ? 1 : 0;
    }

    // Procesa un flanco. channel = 0 (canal 1) o 1 (canal 2); rising = nuevo nivel alto.
    void on_edge(int channel, bool rising) {
        const int new_level = rising ? 1 : 0;
        const int prev_state = (ch1_ << 1) | ch2_;

        if (channel == 0) {
            if (rising) stats_[0].rising++; else stats_[0].falling++;
            if (ch1_ == new_level) repeated_++;  // flanco repetido: se perdió un evento
            ch1_ = new_level;
        } else {
            if (rising) stats_[1].rising++; else stats_[1].falling++;
            if (ch2_ == new_level) repeated_++;
            ch2_ = new_level;
        }

        switch (mode_) {
        case DecodeMode::PULSE:
            if (channel == 0 && rising) count_ += 1;
            break;
        case DecodeMode::X1:
            // Subida de ch1 con ch2 bajo => ch1 adelantado => +1.
            if (channel == 0 && rising) count_ += (ch2_ == 0) ? +1 : -1;
            break;
        case DecodeMode::X2:
            if (channel == 0) count_ += (ch1_ != ch2_) ? +1 : -1;
            break;
        case DecodeMode::X4: {
            const int new_state = (ch1_ << 1) | ch2_;
            // Tabla de transición estándar (estado anterior * 4 + nuevo).
            // Secuencia +1: 00 -> 10 -> 11 -> 01 -> 00 (ch1 adelantado).
            static const int8_t table[16] = {
                //       new: 00  01  10  11
                /*00*/        0,  -1, +1,  0,
                /*01*/       +1,   0,  0, -1,
                /*10*/       -1,   0,  0, +1,
                /*11*/        0,  +1, -1,  0,
            };
            const int8_t d = table[prev_state * 4 + new_state];
            if (d == 0 && prev_state != new_state) invalid_++;
            count_ += d;
            break;
        }
        }
    }

    int64_t count() const { return count_; }
    const ChannelStats& stats(int ch) const { return stats_[ch]; }
    uint64_t invalid_transitions() const { return invalid_; }
    uint64_t repeated_edges() const { return repeated_; }
    DecodeMode mode() const { return mode_; }

private:
    DecodeMode mode_;
    int ch1_ = 0, ch2_ = 0;
    int64_t count_ = 0;
    ChannelStats stats_[2];
    uint64_t invalid_ = 0;
    uint64_t repeated_ = 0;
};

inline const char* mode_name(DecodeMode m) {
    switch (m) {
    case DecodeMode::PULSE: return "pulse";
    case DecodeMode::X1: return "x1";
    case DecodeMode::X2: return "x2";
    case DecodeMode::X4: return "x4";
    }
    return "?";
}
