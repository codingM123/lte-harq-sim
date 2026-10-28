// Basic types shared by every block of the simulator.
#pragma once

#include <complex>
#include <cstdint>
#include <vector>

namespace harq {

// One bit per element, value 0 or 1.
using Bits = std::vector<uint8_t>;

// Log-likelihood ratios (LLRs), one per bit.
// LLR = ln(P(bit = 0) / P(bit = 1)): positive means "probably 0", negative means
// "probably 1", and the magnitude says how confident the receiver is.
// An LLR of 0 means "no information", for example a coded bit that was never sent.
using Llrs = std::vector<double>;

using Symbol = std::complex<double>;
using Symbols = std::vector<Symbol>;

}  // namespace harq
