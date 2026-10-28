#include "simulator.hpp"

#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

#include "conv_code.hpp"
#include "crc24a.hpp"
#include "modem.hpp"
#include "rate_matcher.hpp"

namespace harq {
namespace {

double ratio(uint64_t num, uint64_t den) {
    return den == 0 ? 0.0 : static_cast<double>(num) / static_cast<double>(den);
}

}  // namespace

double LinkResult::throughput_kbps() const { return ratio(delivered_bits, subframes); }
double LinkResult::bler_first_tx() const { return ratio(first_tx_failures, blocks_started); }
double LinkResult::residual_bler() const { return ratio(blocks_dropped, blocks_delivered + blocks_dropped); }
double LinkResult::avg_transmissions() const { return ratio(transmissions_used, blocks_delivered + blocks_dropped); }
double LinkResult::avg_latency_ms() const { return ratio(latency_sum_ms, blocks_delivered); }
double LinkResult::busy_fraction() const { return ratio(busy_subframes, subframes); }

LinkResult run_link(const LinkConfig& cfg) {
    if (cfg.coded_bits == 0 || cfg.coded_bits % 2 != 0) {
        throw std::invalid_argument("run_link: coded_bits must be a positive even number (QPSK)");
    }
    if (cfg.processes < 1 || cfg.rtt_subframes < 1 || cfg.info_bits == 0) {
        throw std::invalid_argument("run_link: processes, rtt and info_bits must be positive");
    }

    const std::size_t block_len = cfg.info_bits + kCrcBits;          // payload + CRC
    const std::size_t stream_len = block_len + ConvCode::kMemory;    // + zero tail
    const ConvCode code;
    const RateMatcher matcher(stream_len);
    const double n0 = noise_variance_for_snr(cfg.snr_db);

    std::mt19937_64 data_rng(cfg.seed * 2 + 1);
    std::bernoulli_distribution coin(0.5);
    AwgnChannel channel(cfg.seed * 2 + 2);

    std::vector<HarqProcess> processes;
    processes.reserve(static_cast<std::size_t>(cfg.processes));
    for (int i = 0; i < cfg.processes; ++i) {
        processes.emplace_back(i, cfg.scheme, cfg.max_transmissions, stream_len);
    }

    LinkResult r;
    r.scheme = cfg.scheme;
    r.snr_db = cfg.snr_db;
    r.processes = cfg.processes;
    r.subframes = cfg.subframes;

    for (uint64_t now = 0; now < cfg.subframes; ++now) {
        // Pick a process whose ACK/NACK has already come back, starting from a rotating
        // position so every process gets its turn.
        HarqProcess* proc = nullptr;
        for (int i = 0; i < cfg.processes; ++i) {
            HarqProcess& candidate = processes[(now + static_cast<uint64_t>(i)) % processes.size()];
            if (candidate.ready(now)) {
                proc = &candidate;
                break;
            }
        }
        if (proc == nullptr) continue;  // every process is waiting for feedback: nothing is sent
        ++r.busy_subframes;

        if (!proc->busy()) {  // the previous block finished, so this process takes new data
            Bits payload(cfg.info_bits);
            for (auto& b : payload) b = coin(data_rng) ? 1 : 0;
            Bits block = attach_crc24a(payload);
            CodedStreams coded = code.encode(block);
            proc->start_block(std::move(block), std::move(coded), now);
            ++r.blocks_started;
        }

        // Transmitter: pick this transmission's coded bits, modulate, send through the channel.
        const int rv = proc->next_rv();
        Symbols symbols = qpsk_modulate(matcher.select(proc->coded(), cfg.coded_bits, rv));
        channel.apply(symbols, n0);

        // Receiver: LLRs -> soft buffer (combining happens here) -> Viterbi -> CRC check.
        const Llrs llrs = qpsk_demodulate(symbols, n0);
        SoftStreams& soft = proc->soft_buffer_for_new_copy();
        matcher.accumulate(llrs, rv, soft);
        const Bits decoded = code.decode(soft, block_len);
        const bool crc_ok = check_crc24a(decoded);
        const bool data_correct = decoded == proc->block();
        if (crc_ok && !data_correct) ++r.undetected_errors;

        if (proc->transmissions() == 0 && !crc_ok) ++r.first_tx_failures;
        const HarqProcess::Feedback fb = proc->finish_transmission(crc_ok, now, cfg.rtt_subframes);

        switch (fb.outcome) {
            case HarqProcess::Outcome::Delivered:
                ++r.blocks_delivered;
                if (data_correct) r.delivered_bits += cfg.info_bits;
                r.transmissions_used += static_cast<uint64_t>(fb.transmissions);
                r.latency_sum_ms += (now - fb.first_tx_subframe) + 1;
                break;
            case HarqProcess::Outcome::Dropped:
                ++r.blocks_dropped;
                r.transmissions_used += static_cast<uint64_t>(fb.transmissions);
                break;
            case HarqProcess::Outcome::Retransmit:
                break;
        }
    }
    return r;
}

}  // namespace harq
