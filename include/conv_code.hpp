// LTE rate-1/3 convolutional code (3GPP TS 36.212, section 5.1.3.1) with a
// soft-decision Viterbi decoder.
//
// Constraint length 7 (6 memory bits, so 64 states) and generator polynomials
// 133, 171 and 165 (octal). LTE terminates this code by tail-biting; to keep the
// decoder simple this project uses zero-tail termination instead: 6 zero bits are
// appended so the encoder always finishes in state 0.
#pragma once

#include <array>
#include <cstddef>

#include "types.hpp"

namespace harq {

using CodedStreams = std::array<Bits, 3>;  // coded streams d0, d1, d2
using SoftStreams = std::array<Llrs, 3>;   // LLRs for d0, d1, d2

class ConvCode {
public:
    static constexpr int kMemory = 6;
    static constexpr int kStates = 1 << kMemory;  // 64

    ConvCode();

    // Encodes n bits into three streams of n + 6 bits each.
    CodedStreams encode(const Bits& input) const;

    // Soft-decision Viterbi decoder. Each stream holds n_info + 6 LLRs, with 0 for
    // bits that were never received. Returns the n_info most likely input bits.
    Bits decode(const SoftStreams& llrs, std::size_t n_info) const;

private:
    // For every (state, input bit) pair: the 3 output bits packed into 0..7, and the next state.
    std::array<uint8_t, kStates * 2> output_{};
    std::array<uint8_t, kStates * 2> next_state_{};
};

}  // namespace harq
