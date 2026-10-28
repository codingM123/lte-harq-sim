// One HARQ process: a stop-and-wait channel that carries one transport block at a time.
//
// In a real LTE network the base station keeps the transmit side (the block and how
// many times it has been sent) and the phone keeps the receive side (the soft buffer).
// This simulation runs both ends of each process in one object.
#pragma once

#include <cstddef>
#include <cstdint>

#include "conv_code.hpp"
#include "types.hpp"

namespace harq {

enum class Scheme {
    NoHarq,                 // send once; a failed block is lost
    Arq,                    // resend, but throw away failed copies (no combining)
    ChaseCombining,         // resend the same coded bits and add up the LLRs of every copy
    IncrementalRedundancy,  // resend different coded bits each time and add them in
};

const char* scheme_name(Scheme scheme);

// Redundancy version order LTE uses for a first transmission and its retransmissions.
constexpr int kRvSequence[4] = {0, 2, 3, 1};

class HarqProcess {
public:
    HarqProcess(int id, Scheme scheme, int max_transmissions, std::size_t stream_len);

    int id() const { return id_; }
    bool busy() const { return busy_; }                         // holding a block that is not finished
    bool ready(uint64_t now) const { return now >= ready_at_; }  // feedback for the last send has arrived

    // Transmit side
    void start_block(Bits block_with_crc, CodedStreams coded, uint64_t now);
    int next_rv() const;  // redundancy version for the next transmission
    const CodedStreams& coded() const { return coded_; }
    const Bits& block() const { return block_; }
    int transmissions() const { return transmissions_; }

    // Receive side: the soft buffer this transmission's LLRs should be added into.
    // For schemes that do not combine, the old copy is thrown away first.
    SoftStreams& soft_buffer_for_new_copy();

    // Feedback
    enum class Outcome { Delivered, Retransmit, Dropped };
    struct Feedback {
        Outcome outcome;
        int transmissions;           // times this block has been sent, including this one
        uint64_t first_tx_subframe;  // when the block was first sent
    };
    // Records one transmission and its CRC result. The process may send again once the
    // ACK/NACK has arrived, rtt subframes after this transmission.
    Feedback finish_transmission(bool crc_ok, uint64_t now, int rtt);

private:
    int id_;
    Scheme scheme_;
    int max_transmissions_;
    std::size_t stream_len_;

    bool busy_ = false;
    uint64_t ready_at_ = 0;
    int transmissions_ = 0;
    uint64_t first_tx_subframe_ = 0;
    Bits block_;
    CodedStreams coded_;
    SoftStreams soft_;
};

}  // namespace harq
