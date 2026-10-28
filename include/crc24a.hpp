// LTE CRC-24A, attached to every transport block (3GPP TS 36.212, section 5.1.1).
// The receiver uses it to decide whether to send ACK or NACK.
#pragma once

#include <cstddef>

#include "types.hpp"

namespace harq {

// Generator polynomial g(D) = D^24 + D^23 + D^18 + D^17 + D^14 + D^11 + D^10
//                           + D^7 + D^6 + D^5 + D^4 + D^3 + D + 1,
// written without the D^24 term.
constexpr uint32_t kCrc24aPoly = 0x864CFB;
constexpr std::size_t kCrcBits = 24;

// Remainder of bits(D) * D^24 divided by g(D).
uint32_t crc24a(const Bits& bits);

// Returns the payload followed by its 24 CRC bits, most significant bit first.
Bits attach_crc24a(const Bits& payload);

// True if a block (payload followed by CRC) passes the check.
bool check_crc24a(const Bits& block);

}  // namespace harq
