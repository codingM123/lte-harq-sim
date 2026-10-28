#include "harq_process.hpp"

#include <stdexcept>
#include <utility>

namespace harq {

const char* scheme_name(Scheme scheme) {
    switch (scheme) {
        case Scheme::NoHarq: return "none";
        case Scheme::Arq: return "arq";
        case Scheme::ChaseCombining: return "cc";
        case Scheme::IncrementalRedundancy: return "ir";
    }
    return "unknown";
}

HarqProcess::HarqProcess(int id, Scheme scheme, int max_transmissions, std::size_t stream_len)
    : id_(id),
      scheme_(scheme),
      max_transmissions_(scheme == Scheme::NoHarq ? 1 : max_transmissions),
      stream_len_(stream_len) {
    if (max_transmissions < 1 || max_transmissions > 4) {
        throw std::invalid_argument("HarqProcess: max_transmissions must be 1..4");
    }
    for (auto& s : soft_) s.assign(stream_len_, 0.0);
}

void HarqProcess::start_block(Bits block_with_crc, CodedStreams coded, uint64_t now) {
    if (busy_) throw std::logic_error("HarqProcess::start_block: process is still busy");
    block_ = std::move(block_with_crc);
    coded_ = std::move(coded);
    busy_ = true;
    transmissions_ = 0;
    first_tx_subframe_ = now;
    for (auto& s : soft_) s.assign(stream_len_, 0.0);  // new data: forget the previous block
}

int HarqProcess::next_rv() const {
    // Only incremental redundancy changes which coded bits are sent each time.
    return scheme_ == Scheme::IncrementalRedundancy ? kRvSequence[transmissions_ % 4] : 0;
}

SoftStreams& HarqProcess::soft_buffer_for_new_copy() {
    const bool combines = scheme_ == Scheme::ChaseCombining || scheme_ == Scheme::IncrementalRedundancy;
    if (!combines) {
        for (auto& s : soft_) s.assign(stream_len_, 0.0);
    }
    return soft_;
}

HarqProcess::Feedback HarqProcess::finish_transmission(bool crc_ok, uint64_t now, int rtt) {
    if (!busy_) throw std::logic_error("HarqProcess::finish_transmission: no block in flight");
    ++transmissions_;
    // The ACK/NACK only reaches the transmitter rtt subframes later; until then this
    // process cannot send anything. This waiting is the "stop-and-wait" part.
    ready_at_ = now + static_cast<uint64_t>(rtt);

    Feedback fb{Outcome::Retransmit, transmissions_, first_tx_subframe_};
    if (crc_ok) {
        fb.outcome = Outcome::Delivered;  // ACK
        busy_ = false;
    } else if (transmissions_ >= max_transmissions_) {
        fb.outcome = Outcome::Dropped;  // NACK on the last allowed attempt: give up
        busy_ = false;
    }
    return fb;  // otherwise NACK: the block stays and is sent again
}

}  // namespace harq
