# How it works

A walkthrough of every block in the simulator, in the order a transport block travels through it. Section numbers refer to 3GPP TS 36.211 and TS 36.212.

## 1. The big picture

LTE sends data in **transport blocks**, one per **subframe** (1 ms). The receiver checks each block with a CRC and replies **ACK** (received correctly) or **NACK** (failed). On a NACK the transmitter sends the block again.

Two ideas make LTE's version of this, HARQ, efficient:

- **Soft combining.** The receiver does not throw away a failed copy. It keeps its soft information (LLRs, see section 5) and adds the next copy to it. Two weak copies together often decode when neither does alone.
- **Parallel stop-and-wait processes.** ACK/NACK takes 8 subframes to come back. One process would send, then sit idle for 7 subframes. LTE FDD runs 8 processes in turn, so while process 0 waits for its feedback, processes 1 to 7 use the next subframes.

`simulator.cpp` runs this loop once per subframe: pick a process whose feedback has arrived, give it a new block if its last one finished, send one transmission through the chain below, check the CRC, and record ACK or NACK.

## 2. CRC-24A (`crc24a.cpp`)

The receiver needs to know whether its decoded block is correct. The transmitter appends 24 check bits: the remainder of dividing the payload, treated as a polynomial over bits, by LTE's generator polynomial (0x864CFB, section 5.1.1).

The division is a 24-bit shift register: feed in one bit at a time, and whenever a 1 falls off the top, XOR the polynomial in. Running the same register over payload + CRC leaves 0 if nothing changed. A 24-bit CRC catches every error burst up to 24 bits long, and misses a random error pattern with probability about 2^-24, or 6 in 100 million.

## 3. Convolutional encoder (`conv_code.cpp`)

The encoder adds structured redundancy so the receiver can correct errors. It is a 6-bit shift register (constraint length 7, so 2^6 = 64 states). For each input bit it outputs 3 bits, each the XOR of the register positions picked by one generator polynomial: 133, 171 and 165 in octal (section 5.1.3.1). Three output bits per input bit gives code rate 1/3.

LTE ends this code by tail-biting. This project appends 6 zero bits instead (zero-tail), so the encoder always finishes in state 0, which keeps the decoder simple. A 1024-bit block therefore becomes three streams of 1030 bits each: 3090 coded bits.

A quick check that the encoder is right: encoding a single 1 bit outputs each generator's bit pattern (its impulse response). The unit tests check exactly this.

## 4. Rate matching and redundancy versions (`rate_matcher.cpp`)

The code produces 3090 bits, but one transmission carries only 1536 (768 QPSK symbols). Rate matching chooses which 1536 bits to send.

1. **Sub-block interleaver** (section 5.1.4.2.1). Each stream is written row by row into a matrix with 32 columns, padded at the front with dummy bits to fill the last row. It is then read out column by column, with the columns taken in LTE's permuted order: 1, 17, 9, 25, and so on. All the odd columns come first, so the first half of each interleaved stream is spread evenly across the whole block.
2. **Circular buffer.** The three interleaved streams are placed one after another: 3 × 1056 = 3168 positions, including 78 dummy bits that are never sent.
3. **Bit selection.** A transmission reads E = 1536 bits starting at its redundancy version's position, skipping dummies and wrapping around at the end of the buffer.

Redundancy version (RV) r starts at r × 3168 / 4. LTE retransmits in the order 0, 2, 3, 1:

- **RV 0** sends all of stream 0 plus half of stream 1, spread evenly: a rate-2/3 code.
- **RV 2** starts halfway through the buffer, so it sends mostly the bits RV 0 skipped. After two transmissions, almost all 3090 bits have been received: a rate-1/3 code.

Chase combining always uses RV 0 (the same bits every time). Incremental redundancy follows the RV sequence. That is the only difference between the two schemes.

The receiver walks the buffer in exactly the same way, so it knows which coded bit every received LLR belongs to.

## 5. QPSK, the channel and LLRs (`modem.cpp`)

**Modulation** (TS 36.211, section 7.1.2). Each pair of bits (b0, b1) becomes one complex symbol ((1 − 2·b0) + j(1 − 2·b1)) / √2. Every symbol has energy Es = 1.

**Channel.** Additive white Gaussian noise: each symbol gets random noise with variance N0/2 on each axis. SNR is Es/N0, so N0 = 10^(−SNR/10).

**Soft demodulation.** Instead of deciding "0" or "1", the receiver computes a log-likelihood ratio for each bit:

LLR = ln( P(y | bit = 0) / P(y | bit = 1) ) = 2√2 · y / N0

where y is the received value on that axis. The sign gives the likely bit, and the size gives the confidence: a value far from zero, or a quiet channel, gives a large LLR.

## 6. Soft combining (`harq_process.cpp`, `rate_matcher.cpp`)

Each HARQ process keeps a soft buffer: one LLR per coded bit, all starting at 0, which means "no information". Every received copy is **added** into it at the positions it came from.

Adding LLRs is the mathematically correct way to combine independent observations of the same bit, because log-likelihoods of independent observations add. In practice:

- **Chase combining:** the same 1536 bits are received again. Their LLRs roughly double, which is worth about +3 dB of SNR for each doubling of the number of copies.
- **Incremental redundancy:** new bits fill positions that were still 0. The decoder now sees a lower-rate, stronger code. This gains more than +3 dB, which is why IR wins at low SNR.
- **ARQ without combining:** the buffer is cleared before each copy, so every attempt starts from nothing.

## 7. Viterbi decoder (`conv_code.cpp`)

The decoder finds the input sequence whose encoded output best matches the soft buffer. It walks the trellis (64 states × 1030 steps) and, for every state, keeps only the best path into it:

- **Branch score:** for a hypothesised output pattern, add +LLR where the pattern says 0 and −LLR where it says 1. Unsent bits have LLR 0 and do not affect the score.
- **Add-compare-select:** each state can be reached from two previous states. Add each one's path score to its branch score, keep the larger, and store a 1-bit decision saying which predecessor won.
- **Traceback:** the zero tail guarantees the encoder ended in state 0. Start there and follow the stored decisions backwards. The input bit at each step is the top bit of the state it led to.

The cost grows linearly with block length (states × steps), which is why Viterbi decoding is practical in hardware.

## 8. HARQ processes and the subframe loop (`harq_process.cpp`, `simulator.cpp`)

Each process is a small state machine:

- **Idle:** ready for a new block.
- **Busy:** a block is in flight. After each transmission it waits `rtt` subframes for feedback.
  - ACK: the block is delivered, and the process becomes idle.
  - NACK with fewer than 4 transmissions: the block stays and is sent again with the next RV.
  - NACK on the 4th transmission: the block is dropped, and the process becomes idle.

Each subframe the simulator picks the next process whose feedback has arrived. With 8 processes and an 8-subframe round trip, exactly one is ready every subframe, so the link is always busy. With 1 process, only 1 subframe in 8 carries data. The process-sweep experiment measures this.

## 9. What is measured

- **Throughput:** delivered payload bits ÷ simulated milliseconds, in kbit/s.
- **First-transmission BLER:** the share of blocks whose first transmission failed. It is the same for every scheme.
- **Residual BLER:** the share of finished blocks that were dropped after the last allowed transmission.
- **Average transmissions:** per finished block.
- **Latency:** from first transmission to successful decode: 1 ms, then 9, 17 or 25 ms for one, two or three retransmissions.
- **Undetected errors:** blocks whose CRC passed but whose data was wrong (expected to be zero).

## 10. Running the sweep on several threads (`main.cpp`)

Every (scheme, SNR) pair is an independent job. Each worker thread takes the next job number from a `std::atomic` counter, so no job runs twice, and writes only into its own slot of the results vector, so the results need no lock. A `std::mutex` keeps the progress messages from different threads from mixing. Each job's random seed depends only on its SNR value, so the results are identical with 1 thread or 16, and a single SNR point matches the same point inside a full sweep.

## 11. How correctness is checked

- **Unit tests** (`tests/test_main.cpp`):
  - The CRC catches every single-bit error and short burst.
  - The encoder's impulse response equals the generator polynomials.
  - Noiseless decoding returns the input, and the decoder corrects isolated errors and a fully erased stream.
  - The rate matcher hands back every coded bit exactly once.
  - RV 0 + RV 2 cover almost the whole buffer, while RV 0 twice covers only 1536 bits.
  - The HARQ state machine follows the RV order, the timing and the drop rule.
  - The end-to-end link gives exactly 1000 kbit/s at high SNR, and 125 kbit/s with one process.
- **Theory check:** uncoded QPSK bit error rate is compared with 0.5·erfc(√(Eb/N0)) (`harq_sim --validate`).
- **Independent CRC check:** during development, the CRC was compared with plain polynomial long division.
