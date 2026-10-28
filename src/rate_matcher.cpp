#include "rate_matcher.hpp"

#include <array>
#include <stdexcept>

namespace harq {
namespace {

constexpr std::size_t kColumns = 32;

// Inter-column permutation for convolutional codes (TS 36.212, Table 5.1.4-2).
// The odd-numbered columns come first, so the first half of each interleaved
// stream is spread evenly across the whole block instead of bunching up at the start.
constexpr std::array<int, kColumns> kColumnPermutation = {
    1, 17, 9, 25, 5, 21, 13, 29, 3, 19, 11, 27, 7, 23, 15, 31,
    0, 16, 8, 24, 4, 20, 12, 28, 2, 18, 10, 26, 6, 22, 14, 30};

}  // namespace

RateMatcher::RateMatcher(std::size_t stream_len)
    : stream_len_(stream_len), rows_((stream_len + kColumns - 1) / kColumns) {
    if (stream_len == 0) throw std::invalid_argument("RateMatcher: empty stream");
    const std::size_t per_stream = rows_ * kColumns;      // K_PI in the spec
    const std::size_t dummies = per_stream - stream_len;  // padding bits placed at the front

    buffer_map_.assign(3 * per_stream, kDummy);
    for (std::size_t stream = 0; stream < 3; ++stream) {
        for (std::size_t k = 0; k < per_stream; ++k) {
            // The stream is written into a rows x 32 matrix row by row, then read out
            // column by column, taking the columns in permuted order.
            const std::size_t row = k % rows_;
            const std::size_t column = static_cast<std::size_t>(kColumnPermutation[k / rows_]);
            const std::size_t written = row * kColumns + column;
            if (written >= dummies) {
                buffer_map_[stream * per_stream + k] =
                    static_cast<int>(stream * stream_len + (written - dummies));
            }
        }
    }
}

std::size_t RateMatcher::start_position(int rv) const {
    if (rv < 0 || rv > 3) throw std::invalid_argument("RateMatcher: rv must be 0..3");
    // Redundancy versions start a quarter of the buffer apart, similar to LTE's
    // turbo-code rule where k0 is roughly rv * buffer_length / 4.
    return static_cast<std::size_t>(rv) * buffer_map_.size() / 4;
}

template <typename Visit>
void RateMatcher::walk(std::size_t e, int rv, Visit visit) const {
    const std::size_t length = buffer_map_.size();
    std::size_t position = start_position(rv);
    std::size_t taken = 0;
    while (taken < e) {
        const int flat = buffer_map_[position];
        if (flat != kDummy) {  // padding bits are skipped and never sent
            visit(static_cast<std::size_t>(flat));
            ++taken;
        }
        position = (position + 1) % length;  // wrap around: this is the "circular" part
    }
}

Bits RateMatcher::select(const CodedStreams& coded, std::size_t e, int rv) const {
    for (const auto& s : coded) {
        if (s.size() != stream_len_) throw std::invalid_argument("RateMatcher::select: wrong stream length");
    }
    Bits out;
    out.reserve(e);
    walk(e, rv, [&](std::size_t flat) { out.push_back(coded[flat / stream_len_][flat % stream_len_]); });
    return out;
}

void RateMatcher::accumulate(const Llrs& received, int rv, SoftStreams& soft) const {
    for (const auto& s : soft) {
        if (s.size() != stream_len_) throw std::invalid_argument("RateMatcher::accumulate: wrong soft buffer size");
    }
    std::size_t i = 0;
    walk(received.size(), rv, [&](std::size_t flat) { soft[flat / stream_len_][flat % stream_len_] += received[i++]; });
}

SoftStreams RateMatcher::empty_soft_buffer() const {
    SoftStreams soft;
    for (auto& s : soft) s.assign(stream_len_, 0.0);
    return soft;
}

}  // namespace harq
