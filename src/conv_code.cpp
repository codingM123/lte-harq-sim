#include "conv_code.hpp"

#include <limits>
#include <stdexcept>

namespace harq {
namespace {

constexpr std::array<unsigned, 3> kGenerators = {0133, 0171, 0165};  // octal

int parity(unsigned value) {
    int p = 0;
    while (value != 0) {
        p ^= static_cast<int>(value & 1u);
        value >>= 1;
    }
    return p;
}

}  // namespace

ConvCode::ConvCode() {
    // The 7-bit register holds the current input bit (bit 6) and the 6 previous
    // inputs (bits 5..0). Each output bit is the XOR of the register bits selected
    // by one generator polynomial. The new state drops the oldest input.
    for (int state = 0; state < kStates; ++state) {
        for (int input = 0; input < 2; ++input) {
            const unsigned reg = (static_cast<unsigned>(input) << kMemory) | static_cast<unsigned>(state);
            unsigned out = 0;
            for (int i = 0; i < 3; ++i) {
                out |= static_cast<unsigned>(parity(reg & kGenerators[i])) << i;
            }
            output_[state * 2 + input] = static_cast<uint8_t>(out);
            next_state_[state * 2 + input] = static_cast<uint8_t>(reg >> 1);
        }
    }
}

CodedStreams ConvCode::encode(const Bits& input) const {
    const std::size_t n = input.size() + kMemory;
    CodedStreams streams;
    for (auto& s : streams) s.reserve(n);

    int state = 0;
    for (std::size_t k = 0; k < n; ++k) {
        const int bit = k < input.size() ? (input[k] & 1) : 0;  // zero tail after the data
        const unsigned out = output_[state * 2 + bit];
        for (int i = 0; i < 3; ++i) {
            streams[i].push_back(static_cast<uint8_t>((out >> i) & 1u));
        }
        state = next_state_[state * 2 + bit];
    }
    return streams;
}

Bits ConvCode::decode(const SoftStreams& llrs, std::size_t n_info) const {
    const std::size_t n = n_info + kMemory;
    for (const auto& s : llrs) {
        if (s.size() != n) throw std::invalid_argument("ConvCode::decode: wrong LLR length");
    }

    constexpr double kUnreachable = -std::numeric_limits<double>::infinity();
    std::array<double, kStates> metric;
    std::array<double, kStates> next_metric;
    metric.fill(kUnreachable);
    metric[0] = 0.0;  // the encoder always starts in state 0

    // decisions[k], bit s: which of the two possible previous states won for state s at step k.
    std::vector<uint64_t> decisions(n, 0);

    for (std::size_t k = 0; k < n; ++k) {
        // Score of each of the 8 possible output patterns: +LLR where the pattern
        // says 0 and -LLR where it says 1. A higher score is a better match.
        std::array<double, 8> branch;
        for (int pattern = 0; pattern < 8; ++pattern) {
            double score = 0.0;
            for (int i = 0; i < 3; ++i) {
                score += ((pattern >> i) & 1) ? -llrs[i][k] : llrs[i][k];
            }
            branch[pattern] = score;
        }

        next_metric.fill(kUnreachable);
        const int max_input = (k < n_info) ? 1 : 0;  // tail bits are known to be 0
        uint64_t step_decisions = 0;
        for (int state = 0; state < kStates; ++state) {
            if (metric[state] == kUnreachable) continue;
            for (int input = 0; input <= max_input; ++input) {
                const int idx = state * 2 + input;
                const int next = next_state_[idx];
                const double candidate = metric[state] + branch[output_[idx]];
                if (candidate > next_metric[next]) {  // add-compare-select
                    next_metric[next] = candidate;
                    // The two states that lead into `next` differ only in their lowest bit.
                    const uint64_t mask = uint64_t{1} << next;
                    step_decisions = (state & 1) ? (step_decisions | mask) : (step_decisions & ~mask);
                }
            }
        }
        decisions[k] = step_decisions;
        metric = next_metric;
    }

    // Trace back from state 0 (guaranteed by the zero tail). The input bit that
    // led into a state is that state's top bit.
    Bits decoded(n_info);
    int state = 0;
    for (std::size_t k = n; k-- > 0;) {
        if (k < n_info) decoded[k] = static_cast<uint8_t>((state >> (kMemory - 1)) & 1);
        const int lowest = static_cast<int>((decisions[k] >> state) & 1u);
        state = ((state << 1) & (kStates - 1)) | lowest;
    }
    return decoded;
}

}  // namespace harq
