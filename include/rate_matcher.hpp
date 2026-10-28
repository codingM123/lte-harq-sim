// LTE-style rate matching for the convolutional code (3GPP TS 36.212, section 5.1.4.2),
// extended with redundancy versions so it can be used for HARQ.
//
// Transmitter: each coded stream goes through LTE's sub-block interleaver, and the
// three interleaved streams are placed one after another in a "circular buffer".
// One transmission reads E bits from the buffer, starting at the redundancy
// version's position and wrapping around at the end.
//
// Receiver: walking the buffer the same way tells it where every received LLR came
// from, so the LLR can be added into the soft buffer at the right place.
#pragma once

#include <cstddef>
#include <vector>

#include "conv_code.hpp"
#include "types.hpp"

namespace harq {

class RateMatcher {
public:
    // stream_len: length of each coded stream (payload + CRC + 6 tail bits).
    explicit RateMatcher(std::size_t stream_len);

    // The E coded bits sent in one transmission with redundancy version rv (0..3).
    Bits select(const CodedStreams& coded, std::size_t e, int rv) const;

    // Adds received LLRs into the soft buffer at the positions they were taken from.
    // Adding LLRs is how the receiver combines retransmissions.
    void accumulate(const Llrs& received, int rv, SoftStreams& soft) const;

    // A soft buffer with no information yet (all LLRs 0).
    SoftStreams empty_soft_buffer() const;

    // Where a redundancy version starts reading in the circular buffer.
    std::size_t start_position(int rv) const;

    std::size_t stream_length() const { return stream_len_; }
    std::size_t rows() const { return rows_; }
    std::size_t buffer_length() const { return buffer_map_.size(); }  // includes dummy bits

    static constexpr int kDummy = -1;

private:
    // Calls visit(flat_index) for each of the e coded bits read for this redundancy version.
    template <typename Visit>
    void walk(std::size_t e, int rv, Visit visit) const;

    std::size_t stream_len_;
    std::size_t rows_;
    // buffer_map_[w] = which coded bit sits at circular-buffer position w, stored as
    // stream * stream_len + index, or kDummy for a padding bit that is never sent.
    std::vector<int> buffer_map_;
};

}  // namespace harq
