// QPSK modulation with LTE's bit mapping (3GPP TS 36.211, section 7.1.2), a soft
// demodulator that outputs LLRs, and an additive white Gaussian noise (AWGN) channel.
#pragma once

#include <random>

#include "types.hpp"

namespace harq {

// Bit pair (b0, b1) -> ((1 - 2*b0) + j*(1 - 2*b1)) / sqrt(2). Every symbol has energy 1.
Symbols qpsk_modulate(const Bits& bits);

// Soft demodulator. noise_variance is N0, the total noise power per complex symbol
// (N0/2 on each of the I and Q axes). Returns two LLRs per symbol.
Llrs qpsk_demodulate(const Symbols& received, double noise_variance);

// Hard decisions from LLRs: 1 where the LLR is negative, 0 otherwise.
Bits hard_decisions(const Llrs& llrs);

// N0 for a given SNR. With unit symbol energy, SNR = Es/N0, so N0 = 10^(-SNR/10).
double noise_variance_for_snr(double snr_db);

class AwgnChannel {
public:
    explicit AwgnChannel(uint64_t seed) : rng_(seed) {}

    // Adds complex Gaussian noise with total variance noise_variance to every symbol.
    void apply(Symbols& symbols, double noise_variance);

private:
    std::mt19937_64 rng_;
    std::normal_distribution<double> gauss_{0.0, 1.0};
};

}  // namespace harq
