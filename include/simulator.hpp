// Runs an LTE-style link with several parallel HARQ processes over an AWGN channel
// and measures throughput, block error rate, transmissions per block and latency.
#pragma once

#include <cstddef>
#include <cstdint>

#include "harq_process.hpp"

namespace harq {

struct LinkConfig {
    std::size_t info_bits = 1000;  // payload bits per transport block (+24 CRC bits = 1024)
    std::size_t coded_bits = 1536;  // coded bits per transmission (768 QPSK symbols), code rate 2/3
    int max_transmissions = 4;      // first transmission + up to 3 retransmissions
    int processes = 8;              // parallel HARQ processes (LTE FDD uses 8)
    int rtt_subframes = 8;          // subframes until ACK/NACK is back (8 ms in LTE FDD)
    uint64_t subframes = 4000;      // simulated time; 1 subframe = 1 ms
    Scheme scheme = Scheme::IncrementalRedundancy;
    double snr_db = 0.0;  // Es/N0 per QPSK symbol, in dB
    uint64_t seed = 1;
};

struct LinkResult {
    Scheme scheme = Scheme::NoHarq;
    double snr_db = 0.0;
    int processes = 0;
    uint64_t subframes = 0;
    uint64_t busy_subframes = 0;  // subframes in which something was sent
    uint64_t blocks_started = 0;
    uint64_t blocks_delivered = 0;
    uint64_t blocks_dropped = 0;
    uint64_t first_tx_failures = 0;
    uint64_t undetected_errors = 0;  // CRC passed but the data was wrong (expected to be ~0)
    uint64_t delivered_bits = 0;
    uint64_t transmissions_used = 0;  // transmissions spent on blocks that finished
    uint64_t latency_sum_ms = 0;      // summed over delivered blocks

    double throughput_kbps() const;    // delivered payload bits per ms = kbit/s
    double bler_first_tx() const;      // share of blocks whose first transmission failed
    double residual_bler() const;      // share of finished blocks that were given up on
    double avg_transmissions() const;  // per finished block
    double avg_latency_ms() const;     // first transmission to successful decode
    double busy_fraction() const;      // share of subframes that carried data
};

LinkResult run_link(const LinkConfig& cfg);

}  // namespace harq
