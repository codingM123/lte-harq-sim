// harq_sim: command-line front end for the LTE HARQ link simulator.
//
// Modes
//   harq_sim                  SNR sweep for every scheme          -> <out>/scheme_<name>.csv
//   harq_sim --validate       uncoded QPSK BER vs theory          -> <out>/qpsk_ber_validation.csv
//   harq_sim --process-sweep  throughput vs number of HARQ processes at --snr
//                                                                 -> <out>/process_sweep.csv
//
// Run "harq_sim --help" for every option.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "modem.hpp"
#include "simulator.hpp"

namespace {

using namespace harq;

enum class Mode { Sweep, Validate, ProcessSweep };

const std::vector<Scheme> kAllSchemes = {Scheme::NoHarq, Scheme::Arq, Scheme::ChaseCombining,
                                         Scheme::IncrementalRedundancy};

struct Options {
    Mode mode = Mode::Sweep;
    std::vector<Scheme> schemes = kAllSchemes;
    double snr_start = -6.0;
    double snr_stop = 10.0;
    double snr_step = 1.0;
    double snr = 4.0;     // single SNR used by --process-sweep
    LinkConfig link;      // settings shared by every run
    unsigned threads = 0;  // 0 means one thread per CPU core
    std::string out_dir = "results";
};

void print_help() {
    std::cout <<
        "Usage: harq_sim [mode] [options]\n"
        "\n"
        "Modes:\n"
        "  (none)             SNR sweep for every scheme, writes <out>/scheme_<name>.csv\n"
        "  --validate         uncoded QPSK bit error rate vs theory, writes <out>/qpsk_ber_validation.csv\n"
        "  --process-sweep    throughput with 1, 2, 4 and 8 HARQ processes at --snr,\n"
        "                     writes <out>/process_sweep.csv\n"
        "\n"
        "Options (defaults in brackets):\n"
        "  --scheme S         all, none, arq, cc or ir [all]\n"
        "  --snr-start dB     first SNR of the sweep [-6]\n"
        "  --snr-stop dB      last SNR of the sweep [10]\n"
        "  --snr-step dB      SNR step [1]\n"
        "  --snr dB           SNR for --process-sweep [4]\n"
        "  --subframes N      simulated subframes (1 ms each) per run [4000]\n"
        "  --info-bits N      payload bits per transport block [1000]\n"
        "  --coded-bits N     coded bits per transmission, must be even [1536]\n"
        "  --max-tx N         maximum transmissions per block, 1..4 [4]\n"
        "  --processes N      parallel HARQ processes [8]\n"
        "  --rtt N            subframes until ACK/NACK arrives [8]\n"
        "  --seed N           random seed [1]\n"
        "  --threads N        worker threads, 0 = all cores [0]\n"
        "  --out DIR          output folder [results]\n";
}

[[noreturn]] void usage_error(const std::string& message) {
    std::cerr << "harq_sim: " << message << "\nRun harq_sim --help for the list of options.\n";
    std::exit(2);
}

Scheme parse_scheme(const std::string& name) {
    for (Scheme s : kAllSchemes) {
        if (name == scheme_name(s)) return s;
    }
    usage_error("unknown scheme '" + name + "' (use all, none, arq, cc or ir)");
}

double as_double(const std::string& option, const std::string& text) {
    try {
        std::size_t used = 0;
        const double v = std::stod(text, &used);
        if (used == text.size()) return v;
    } catch (const std::exception&) {
    }
    usage_error("invalid number '" + text + "' after " + option);
}

// Whole number that must be at least `minimum`.
long long as_count(const std::string& option, const std::string& text, long long minimum) {
    try {
        std::size_t used = 0;
        const long long v = std::stoll(text, &used);
        if (used == text.size() && v >= minimum) return v;
    } catch (const std::exception&) {
    }
    usage_error("expected a whole number of at least " + std::to_string(minimum) + " after " + option +
                ", got '" + text + "'");
}

Options parse_args(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto value = [&]() -> std::string {
            if (i + 1 >= argc) usage_error("missing value after " + arg);
            return argv[++i];
        };
        if (arg == "--help" || arg == "-h") {
            print_help();
            std::exit(0);
        } else if (arg == "--validate") {
            o.mode = Mode::Validate;
        } else if (arg == "--process-sweep") {
            o.mode = Mode::ProcessSweep;
        } else if (arg == "--scheme") {
            const std::string v = value();
            o.schemes = (v == "all") ? kAllSchemes : std::vector<Scheme>{parse_scheme(v)};
        } else if (arg == "--snr-start") {
            o.snr_start = as_double(arg, value());
        } else if (arg == "--snr-stop") {
            o.snr_stop = as_double(arg, value());
        } else if (arg == "--snr-step") {
            o.snr_step = as_double(arg, value());
        } else if (arg == "--snr") {
            o.snr = as_double(arg, value());
        } else if (arg == "--subframes") {
            o.link.subframes = static_cast<uint64_t>(as_count(arg, value(), 1));
        } else if (arg == "--info-bits") {
            o.link.info_bits = static_cast<std::size_t>(as_count(arg, value(), 1));
        } else if (arg == "--coded-bits") {
            o.link.coded_bits = static_cast<std::size_t>(as_count(arg, value(), 2));
        } else if (arg == "--max-tx") {
            o.link.max_transmissions = static_cast<int>(as_count(arg, value(), 1));
        } else if (arg == "--processes") {
            o.link.processes = static_cast<int>(as_count(arg, value(), 1));
        } else if (arg == "--rtt") {
            o.link.rtt_subframes = static_cast<int>(as_count(arg, value(), 1));
        } else if (arg == "--seed") {
            o.link.seed = static_cast<uint64_t>(as_count(arg, value(), 0));
        } else if (arg == "--threads") {
            o.threads = static_cast<unsigned>(as_count(arg, value(), 0));
        } else if (arg == "--out") {
            o.out_dir = value();
        } else {
            usage_error("unknown option " + arg);
        }
    }
    if (o.snr_step <= 0.0) usage_error("--snr-step must be positive");
    if (o.snr_stop < o.snr_start) usage_error("--snr-stop must not be below --snr-start");
    if (o.link.max_transmissions < 1 || o.link.max_transmissions > 4) usage_error("--max-tx must be 1..4");
    if (o.link.coded_bits == 0 || o.link.coded_bits % 2 != 0) usage_error("--coded-bits must be a positive even number");
    if (o.link.processes < 1 || o.link.rtt_subframes < 1) usage_error("--processes and --rtt must be at least 1");
    return o;
}

// Runs independent jobs on several threads. Each thread takes the next job number from
// an atomic counter, so no job runs twice, and writes only its own slot in the results
// vector, so the results need no lock. The mutex only keeps progress messages tidy.
template <typename Job, typename Work>
auto run_parallel(const std::vector<Job>& jobs, unsigned threads, Work work) {
    using Result = decltype(work(jobs.front()));
    std::vector<Result> results(jobs.size());
    std::atomic<std::size_t> next_job{0};
    std::mutex print_mutex;
    std::size_t finished = 0;  // guarded by print_mutex

    auto worker = [&]() {
        for (std::size_t i = next_job++; i < jobs.size(); i = next_job++) {
            results[i] = work(jobs[i]);
            const std::lock_guard<std::mutex> lock(print_mutex);
            ++finished;
            std::cerr << "\r  finished " << finished << "/" << jobs.size() << " runs" << std::flush;
        }
    };

    if (threads == 0) threads = std::max(1u, std::thread::hardware_concurrency());
    threads = static_cast<unsigned>(std::min<std::size_t>(threads, std::max<std::size_t>(jobs.size(), 1)));
    std::vector<std::thread> pool;
    for (unsigned t = 0; t < threads; ++t) pool.emplace_back(worker);
    for (auto& th : pool) th.join();
    std::cerr << "\n";
    return results;
}

std::vector<double> snr_points(const Options& o) {
    std::vector<double> points;
    const int count = static_cast<int>(std::floor((o.snr_stop - o.snr_start) / o.snr_step + 1e-9)) + 1;
    for (int i = 0; i < count; ++i) points.push_back(o.snr_start + i * o.snr_step);
    return points;
}

std::ofstream open_csv(const std::string& dir, const std::string& name) {
    const std::string path = (std::filesystem::path(dir) / name).string();
    std::ofstream file(path);
    if (!file) throw std::runtime_error("cannot write " + path);
    file << std::setprecision(8);
    std::cout << "  wrote " << path << "\n";
    return file;
}

int run_sweep(const Options& o) {
    const std::vector<double> snrs = snr_points(o);
    std::vector<LinkConfig> jobs;
    for (Scheme scheme : o.schemes) {
        for (std::size_t i = 0; i < snrs.size(); ++i) {
            LinkConfig cfg = o.link;
            cfg.scheme = scheme;
            cfg.snr_db = snrs[i];
            // The seed depends only on the base seed and the SNR value, so every scheme sees the
            // same random data at a given SNR, and a point gives the same result whatever range
            // or thread count it runs in.
            cfg.seed = o.link.seed * 1000000 + static_cast<uint64_t>(std::llround((snrs[i] + 1000.0) * 100.0));
            jobs.push_back(cfg);
        }
    }

    std::cout << "SNR sweep: " << o.schemes.size() << " scheme(s) x " << snrs.size() << " SNR points, "
              << o.link.subframes << " subframes each\n";
    const auto results = run_parallel(jobs, o.threads, run_link);

    for (std::size_t s = 0; s < o.schemes.size(); ++s) {
        std::ofstream csv = open_csv(o.out_dir, std::string("scheme_") + scheme_name(o.schemes[s]) + ".csv");
        csv << "snr_db,throughput_kbps,bler_first_tx,residual_bler,avg_transmissions,avg_latency_ms,"
               "blocks_started,blocks_delivered,blocks_dropped,undetected_errors\n";
        for (std::size_t i = 0; i < snrs.size(); ++i) {
            const LinkResult& r = results[s * snrs.size() + i];
            csv << r.snr_db << ',' << r.throughput_kbps() << ',' << r.bler_first_tx() << ',' << r.residual_bler() << ','
                << r.avg_transmissions() << ',' << r.avg_latency_ms() << ',' << r.blocks_started << ','
                << r.blocks_delivered << ',' << r.blocks_dropped << ',' << r.undetected_errors << '\n';
        }
    }

    std::cout << "\nThroughput (kbit/s; the peak is " << o.link.info_bits << ")\n  SNR(dB)";
    for (Scheme s : o.schemes) std::cout << std::setw(10) << scheme_name(s);
    std::cout << "\n" << std::fixed;
    for (std::size_t i = 0; i < snrs.size(); ++i) {
        std::cout << std::setw(9) << std::setprecision(1) << snrs[i];
        for (std::size_t s = 0; s < o.schemes.size(); ++s) {
            std::cout << std::setw(10) << std::setprecision(1) << results[s * snrs.size() + i].throughput_kbps();
        }
        std::cout << "\n";
    }
    uint64_t undetected = 0;
    for (const auto& r : results) undetected += r.undetected_errors;
    std::cout << "Undetected errors (CRC passed, data wrong): " << undetected << "\n";
    return 0;
}

struct BerPoint {
    double ebn0_db = 0.0;
    double measured = 0.0;
    double theory = 0.0;
    uint64_t bits = 0;
    uint64_t errors = 0;
};

// Uncoded QPSK over AWGN, compared with the textbook result BER = 0.5 * erfc(sqrt(Eb/N0)).
// QPSK carries 2 bits per symbol, so Es/N0 = Eb/N0 + 10*log10(2) dB.
BerPoint measure_qpsk_ber(double ebn0_db, uint64_t seed) {
    constexpr uint64_t kMinBits = 1000000;
    constexpr uint64_t kMaxBits = 20000000;
    constexpr uint64_t kTargetErrors = 2000;
    constexpr std::size_t kChunk = 20000;

    const double n0 = noise_variance_for_snr(ebn0_db + 10.0 * std::log10(2.0));
    std::mt19937_64 rng(seed);
    std::bernoulli_distribution coin(0.5);
    AwgnChannel channel(seed + 1);

    BerPoint p;
    p.ebn0_db = ebn0_db;
    while (p.bits < kMaxBits && !(p.errors >= kTargetErrors && p.bits >= kMinBits)) {
        Bits tx(kChunk);
        for (auto& b : tx) b = coin(rng) ? 1 : 0;
        Symbols symbols = qpsk_modulate(tx);
        channel.apply(symbols, n0);
        const Bits rx = hard_decisions(qpsk_demodulate(symbols, n0));
        for (std::size_t k = 0; k < kChunk; ++k) p.errors += (tx[k] != rx[k]) ? 1 : 0;
        p.bits += kChunk;
    }
    p.measured = static_cast<double>(p.errors) / static_cast<double>(p.bits);
    p.theory = 0.5 * std::erfc(std::sqrt(std::pow(10.0, ebn0_db / 10.0)));
    return p;
}

int run_validation(const Options& o) {
    std::vector<double> points;
    for (int db = 0; db <= 10; ++db) points.push_back(db);

    std::cout << "Uncoded QPSK BER check (Eb/N0 0..10 dB)\n";
    std::vector<std::size_t> jobs(points.size());
    for (std::size_t i = 0; i < jobs.size(); ++i) jobs[i] = i;
    const auto results = run_parallel(jobs, o.threads, [&](std::size_t i) {
        return measure_qpsk_ber(points[i], o.link.seed * 7919 + i);
    });

    std::ofstream csv = open_csv(o.out_dir, "qpsk_ber_validation.csv");
    csv << "ebn0_db,ber_measured,ber_theory,bits,errors\n";
    std::cout << "  Eb/N0(dB)   measured     theory   difference\n";
    for (const BerPoint& p : results) {
        csv << p.ebn0_db << ',' << p.measured << ',' << p.theory << ',' << p.bits << ',' << p.errors << '\n';
        std::cout << std::fixed << std::setprecision(1) << std::setw(11) << p.ebn0_db << std::scientific
                  << std::setprecision(3) << std::setw(11) << p.measured << std::setw(11) << p.theory
                  << std::fixed << std::setprecision(1) << std::setw(12)
                  << 100.0 * (p.measured - p.theory) / p.theory << "%\n";
    }
    return 0;
}

int run_process_sweep(const Options& o) {
    // With an 8-subframe round trip, one stop-and-wait process can only use 1 subframe in 8.
    // Running several processes in parallel fills the gaps.
    const Scheme scheme = o.schemes.size() == 1 ? o.schemes.front() : Scheme::IncrementalRedundancy;
    std::vector<LinkConfig> jobs;
    for (int count : {1, 2, 4, 8}) {
        LinkConfig cfg = o.link;
        cfg.scheme = scheme;
        cfg.snr_db = o.snr;
        cfg.processes = count;
        jobs.push_back(cfg);
    }

    std::cout << "HARQ process sweep at " << o.snr << " dB, scheme " << scheme_name(scheme) << "\n";
    const auto results = run_parallel(jobs, o.threads, run_link);

    std::ofstream csv = open_csv(o.out_dir, "process_sweep.csv");
    csv << "processes,throughput_kbps,busy_fraction,avg_transmissions,avg_latency_ms,residual_bler\n";
    std::cout << "  processes  throughput(kbit/s)  link busy  avg latency(ms)\n";
    for (const LinkResult& r : results) {
        csv << r.processes << ',' << r.throughput_kbps() << ',' << r.busy_fraction() << ',' << r.avg_transmissions()
            << ',' << r.avg_latency_ms() << ',' << r.residual_bler() << '\n';
        std::cout << std::fixed << std::setw(11) << r.processes << std::setprecision(1) << std::setw(20)
                  << r.throughput_kbps() << std::setw(10) << 100.0 * r.busy_fraction() << "%" << std::setw(17)
                  << r.avg_latency_ms() << "\n";
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options o = parse_args(argc, argv);
        std::filesystem::create_directories(o.out_dir);
        switch (o.mode) {
            case Mode::Sweep: return run_sweep(o);
            case Mode::Validate: return run_validation(o);
            case Mode::ProcessSweep: return run_process_sweep(o);
        }
    } catch (const std::exception& e) {
        std::cerr << "harq_sim: error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
