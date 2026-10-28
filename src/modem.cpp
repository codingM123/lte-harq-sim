#include "modem.hpp"

#include <cmath>
#include <stdexcept>

namespace harq {

Symbols qpsk_modulate(const Bits& bits) {
    if (bits.size() % 2 != 0) throw std::invalid_argument("qpsk_modulate: odd number of bits");
    const double a = 1.0 / std::sqrt(2.0);
    Symbols out;
    out.reserve(bits.size() / 2);
    for (std::size_t i = 0; i < bits.size(); i += 2) {
        out.emplace_back((1.0 - 2.0 * bits[i]) * a, (1.0 - 2.0 * bits[i + 1]) * a);
    }
    return out;
}

Llrs qpsk_demodulate(const Symbols& received, double noise_variance) {
    // Each axis carries one bit as +a or -a (a = 1/sqrt(2)) plus Gaussian noise of
    // variance N0/2, so LLR = ln p(y|0)/p(y|1) = 2*a*y/(N0/2) = 2*sqrt(2)*y/N0.
    const double scale = 2.0 * std::sqrt(2.0) / noise_variance;
    Llrs out;
    out.reserve(received.size() * 2);
    for (const Symbol& s : received) {
        out.push_back(scale * s.real());
        out.push_back(scale * s.imag());
    }
    return out;
}

Bits hard_decisions(const Llrs& llrs) {
    Bits out;
    out.reserve(llrs.size());
    for (double l : llrs) out.push_back(l < 0.0 ? 1 : 0);
    return out;
}

double noise_variance_for_snr(double snr_db) {
    return std::pow(10.0, -snr_db / 10.0);
}

void AwgnChannel::apply(Symbols& symbols, double noise_variance) {
    const double sigma = std::sqrt(noise_variance / 2.0);  // standard deviation per axis
    for (Symbol& s : symbols) {
        const double noise_i = sigma * gauss_(rng_);
        const double noise_q = sigma * gauss_(rng_);
        s += Symbol(noise_i, noise_q);
    }
}

}  // namespace harq
