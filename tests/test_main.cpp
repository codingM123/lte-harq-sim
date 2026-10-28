// Unit tests for every block of the simulator. Run with "make test".
// No test framework is used: CHECK records a failure and the test run carries on.

#include <cmath>
#include <iostream>
#include <random>
#include <set>
#include <string>

#include "conv_code.hpp"
#include "crc24a.hpp"
#include "harq_process.hpp"
#include "modem.hpp"
#include "rate_matcher.hpp"
#include "simulator.hpp"

using namespace harq;

namespace {

int g_checks = 0;
int g_failures = 0;

#define CHECK(condition)                                                                   \
    do {                                                                                   \
        ++g_checks;                                                                        \
        if (!(condition)) {                                                                \
            ++g_failures;                                                                  \
            std::cerr << "  FAILED " << __FILE__ << ":" << __LINE__ << "  " #condition "\n"; \
        }                                                                                  \
    } while (0)

Bits random_bits(std::size_t n, uint64_t seed) {
    std::mt19937_64 rng(seed);
    std::bernoulli_distribution coin(0.5);
    Bits bits(n);
    for (auto& b : bits) b = coin(rng) ? 1 : 0;
    return bits;
}

// LLRs a perfect channel would give: +4 for a 0 bit, -4 for a 1 bit.
SoftStreams perfect_llrs(const CodedStreams& coded) {
    SoftStreams soft;
    for (int i = 0; i < 3; ++i) {
        for (uint8_t b : coded[i]) soft[i].push_back(b ? -4.0 : 4.0);
    }
    return soft;
}

void test_crc() {
    const Bits payload = random_bits(1000, 1);
    Bits block = attach_crc24a(payload);
    CHECK(block.size() == payload.size() + kCrcBits);
    CHECK(check_crc24a(block));

    // Every single-bit error must be caught.
    bool all_single_errors_caught = true;
    for (std::size_t i = 0; i < block.size(); ++i) {
        block[i] ^= 1;
        if (check_crc24a(block)) all_single_errors_caught = false;
        block[i] ^= 1;
    }
    CHECK(all_single_errors_caught);

    // A CRC of degree 24 catches every burst of 24 bits or fewer.
    bool all_bursts_caught = true;
    for (std::size_t start = 0; start + 24 <= block.size(); start += 37) {
        Bits corrupted = block;
        corrupted[start] ^= 1;
        corrupted[start + 23] ^= 1;
        corrupted[start + 11] ^= 1;
        if (check_crc24a(corrupted)) all_bursts_caught = false;
    }
    CHECK(all_bursts_caught);
}

void test_conv_code_generators() {
    // Encoding a single 1 bit gives each generator polynomial's taps as the output
    // (the impulse response): 133 = 1011011, 171 = 1111001, 165 = 1110101 (octal -> binary).
    const ConvCode code;
    const CodedStreams impulse = code.encode(Bits{1});
    CHECK(impulse[0] == (Bits{1, 0, 1, 1, 0, 1, 1}));
    CHECK(impulse[1] == (Bits{1, 1, 1, 1, 0, 0, 1}));
    CHECK(impulse[2] == (Bits{1, 1, 1, 0, 1, 0, 1}));
}

void test_conv_code_round_trip() {
    const ConvCode code;
    const Bits input = random_bits(300, 2);
    const CodedStreams coded = code.encode(input);
    for (const auto& s : coded) CHECK(s.size() == input.size() + ConvCode::kMemory);

    // The 6 tail bits must bring the encoder back to state 0, so the last coded bits
    // depend only on the end of the data. A noiseless decode must return the input.
    CHECK(code.decode(perfect_llrs(coded), input.size()) == input);
}

void test_conv_code_corrects_errors() {
    const ConvCode code;
    const Bits input = random_bits(400, 3);
    SoftStreams soft = perfect_llrs(code.encode(input));
    // Flip a few coded bits far apart from each other; the code's free distance is 15,
    // so isolated errors like these must be corrected.
    for (std::size_t k : {20u, 90u, 160u, 230u, 300u}) soft[k % 3][k] = -soft[k % 3][k];
    CHECK(code.decode(soft, input.size()) == input);

    // Erasures (LLR 0, a bit that was never sent) on one whole stream must also be fine.
    SoftStreams erased = perfect_llrs(code.encode(input));
    for (auto& l : erased[2]) l = 0.0;
    CHECK(code.decode(erased, input.size()) == input);
}

void test_rate_matcher_layout() {
    const std::size_t stream_len = 1030;
    const RateMatcher matcher(stream_len);
    CHECK(matcher.rows() == 33);
    CHECK(matcher.buffer_length() == 3 * 33 * 32);
    CHECK(matcher.start_position(0) == 0);
    CHECK(matcher.start_position(2) == matcher.buffer_length() / 2);

    // Reading every real bit once must hand back each coded bit exactly once.
    CodedStreams coded = {random_bits(stream_len, 4), random_bits(stream_len, 5), random_bits(stream_len, 6)};
    const Bits all = matcher.select(coded, 3 * stream_len, 0);
    CHECK(all.size() == 3 * stream_len);

    Llrs llrs;
    for (uint8_t b : all) llrs.push_back(b ? -1.0 : 1.0);
    SoftStreams soft = matcher.empty_soft_buffer();
    matcher.accumulate(llrs, 0, soft);
    bool every_bit_once_and_correct = true;
    for (int i = 0; i < 3; ++i) {
        for (std::size_t k = 0; k < stream_len; ++k) {
            const double expected = coded[i][k] ? -1.0 : 1.0;
            if (soft[i][k] != expected) every_bit_once_and_correct = false;
        }
    }
    CHECK(every_bit_once_and_correct);
}

void test_incremental_redundancy_coverage() {
    // With 1536 bits per transmission, redundancy versions 0 then 2 should send
    // almost every one of the 3090 coded bits, while chase combining (rv 0 twice)
    // only ever sends the same 1536.
    const std::size_t stream_len = 1030;
    const RateMatcher matcher(stream_len);
    const Llrs ones(1536, 1.0);

    auto count_covered = [&](int rv_a, int rv_b) {
        SoftStreams soft = matcher.empty_soft_buffer();
        matcher.accumulate(ones, rv_a, soft);
        matcher.accumulate(ones, rv_b, soft);
        std::size_t covered = 0;
        for (const auto& s : soft) {
            for (double l : s) covered += (l != 0.0) ? 1 : 0;
        }
        return covered;
    };
    CHECK(count_covered(0, 0) == 1536);
    CHECK(count_covered(0, 2) > 2900);
}

void test_qpsk() {
    const Bits bits = random_bits(2000, 7);
    const Symbols symbols = qpsk_modulate(bits);
    CHECK(symbols.size() == 1000);
    CHECK(std::abs(std::norm(symbols[0]) - 1.0) < 1e-12);  // unit energy
    CHECK(hard_decisions(qpsk_demodulate(symbols, 0.1)) == bits);

    // LTE mapping: bits 00 -> (+1 + j)/sqrt(2), bits 11 -> (-1 - j)/sqrt(2).
    const Symbols corners = qpsk_modulate(Bits{0, 0, 1, 1});
    CHECK(corners[0].real() > 0 && corners[0].imag() > 0);
    CHECK(corners[1].real() < 0 && corners[1].imag() < 0);

    // At 10 dB the noise per axis has variance 0.05, so the measured value should be close.
    Symbols noisy(200000, Symbol(0.0, 0.0));
    AwgnChannel channel(8);
    channel.apply(noisy, noise_variance_for_snr(10.0));
    double power = 0.0;
    for (const Symbol& s : noisy) power += std::norm(s);
    power /= static_cast<double>(noisy.size());
    CHECK(std::abs(power - 0.1) < 0.002);
}

void test_harq_process() {
    const std::size_t stream_len = 100;
    const Bits block = random_bits(94, 9);
    const CodedStreams coded = {random_bits(stream_len, 10), random_bits(stream_len, 11), random_bits(stream_len, 12)};

    // Incremental redundancy: rv 0, 2, 3, 1, then give up after 4 NACKs.
    HarqProcess ir(0, Scheme::IncrementalRedundancy, 4, stream_len);
    CHECK(!ir.busy() && ir.ready(0));
    ir.start_block(block, coded, 0);
    const int expected_rv[4] = {0, 2, 3, 1};
    const HarqProcess::Outcome expected_outcome[4] = {
        HarqProcess::Outcome::Retransmit, HarqProcess::Outcome::Retransmit,
        HarqProcess::Outcome::Retransmit, HarqProcess::Outcome::Dropped};
    uint64_t now = 0;
    for (int tx = 0; tx < 4; ++tx) {
        CHECK(ir.next_rv() == expected_rv[tx]);
        const auto fb = ir.finish_transmission(false, now, 8);
        CHECK(fb.outcome == expected_outcome[tx]);
        CHECK(fb.transmissions == tx + 1);
        CHECK(!ir.ready(now + 7));  // still waiting for the NACK
        CHECK(ir.ready(now + 8));
        now += 8;
    }
    CHECK(!ir.busy());

    // An ACK ends the block straight away.
    HarqProcess cc(1, Scheme::ChaseCombining, 4, stream_len);
    cc.start_block(block, coded, 3);
    CHECK(cc.next_rv() == 0);
    CHECK(cc.finish_transmission(false, 3, 8).outcome == HarqProcess::Outcome::Retransmit);
    CHECK(cc.next_rv() == 0);  // chase combining always resends the same bits
    const auto ack = cc.finish_transmission(true, 11, 8);
    CHECK(ack.outcome == HarqProcess::Outcome::Delivered);
    CHECK(ack.transmissions == 2);
    CHECK(ack.first_tx_subframe == 3);

    // Combining schemes keep earlier LLRs; plain ARQ throws them away.
    HarqProcess keep(2, Scheme::ChaseCombining, 4, stream_len);
    keep.start_block(block, coded, 0);
    keep.soft_buffer_for_new_copy()[0][0] = 5.0;
    CHECK(keep.soft_buffer_for_new_copy()[0][0] == 5.0);
    HarqProcess discard(3, Scheme::Arq, 4, stream_len);
    discard.start_block(block, coded, 0);
    discard.soft_buffer_for_new_copy()[0][0] = 5.0;
    CHECK(discard.soft_buffer_for_new_copy()[0][0] == 0.0);

    // No HARQ: a single failed attempt drops the block.
    HarqProcess none(4, Scheme::NoHarq, 4, stream_len);
    none.start_block(block, coded, 0);
    CHECK(none.finish_transmission(false, 0, 8).outcome == HarqProcess::Outcome::Dropped);
}

void test_link_end_to_end() {
    // At high SNR every block should get through on the first try, and with 8
    // processes every subframe carries a new block: 1000 bits per ms = 1000 kbit/s.
    LinkConfig cfg;
    cfg.snr_db = 12.0;
    cfg.subframes = 200;
    const LinkResult good = run_link(cfg);
    CHECK(good.first_tx_failures == 0);
    CHECK(good.blocks_dropped == 0);
    CHECK(good.undetected_errors == 0);
    CHECK(std::abs(good.throughput_kbps() - 1000.0) < 1e-9);
    CHECK(std::abs(good.avg_latency_ms() - 1.0) < 1e-9);

    // A single stop-and-wait process can only send once every 8 subframes.
    cfg.processes = 1;
    const LinkResult single = run_link(cfg);
    CHECK(single.busy_subframes == 25);
    CHECK(std::abs(single.throughput_kbps() - 125.0) < 1e-9);

    // At very low SNR nothing should get through.
    cfg.processes = 8;
    cfg.snr_db = -10.0;
    const LinkResult bad = run_link(cfg);
    CHECK(bad.blocks_delivered == 0);
    CHECK(bad.blocks_dropped > 0);
}

}  // namespace

int main() {
    const std::pair<const char*, void (*)()> tests[] = {
        {"CRC-24A", test_crc},
        {"convolutional code generators", test_conv_code_generators},
        {"convolutional code round trip", test_conv_code_round_trip},
        {"convolutional code corrects errors", test_conv_code_corrects_errors},
        {"rate matcher layout", test_rate_matcher_layout},
        {"incremental redundancy coverage", test_incremental_redundancy_coverage},
        {"QPSK and AWGN", test_qpsk},
        {"HARQ process state machine", test_harq_process},
        {"end-to-end link", test_link_end_to_end},
    };
    for (const auto& [name, fn] : tests) {
        const int before = g_failures;
        fn();
        std::cout << (g_failures == before ? "[pass] " : "[FAIL] ") << name << "\n";
    }
    std::cout << g_checks - g_failures << "/" << g_checks << " checks passed\n";
    return g_failures == 0 ? 0 : 1;
}
