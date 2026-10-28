#include "crc24a.hpp"

namespace harq {

uint32_t crc24a(const Bits& bits) {
    uint32_t reg = 0;  // 24-bit shift register
    for (uint8_t bit : bits) {
        const bool feedback = (((reg >> 23) & 1u) ^ (bit & 1u)) != 0;
        reg = (reg << 1) & 0xFFFFFFu;
        if (feedback) {
            reg ^= kCrc24aPoly;
        }
    }
    return reg;
}

Bits attach_crc24a(const Bits& payload) {
    Bits block = payload;
    const uint32_t crc = crc24a(payload);
    for (int i = static_cast<int>(kCrcBits) - 1; i >= 0; --i) {
        block.push_back(static_cast<uint8_t>((crc >> i) & 1u));
    }
    return block;
}

bool check_crc24a(const Bits& block) {
    // Running the CRC over payload + CRC leaves a remainder of 0 when nothing is corrupted.
    return block.size() >= kCrcBits && crc24a(block) == 0;
}

}  // namespace harq
