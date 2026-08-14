// Big-endian (network byte order) read/write helpers for DIS PDUs.
// DIS (IEEE 1278.1) transmits all multi-byte fields big-endian.
#pragma once
#include <cstdint>
#include <cstring>

namespace be {

inline uint8_t  rd_u8 (const uint8_t* p) { return p[0]; }
inline uint16_t rd_u16(const uint8_t* p) { return (uint16_t(p[0]) << 8) | p[1]; }
inline uint32_t rd_u32(const uint8_t* p) {
    return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) |
           (uint32_t(p[2]) << 8)  |  uint32_t(p[3]);
}
inline uint64_t rd_u64(const uint8_t* p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v = (v << 8) | p[i];
    return v;
}
inline float rd_f32(const uint8_t* p) {
    uint32_t u = rd_u32(p); float f; std::memcpy(&f, &u, 4); return f;
}
inline double rd_f64(const uint8_t* p) {
    uint64_t u = rd_u64(p); double d; std::memcpy(&d, &u, 8); return d;
}

inline void wr_u8 (uint8_t* p, uint8_t v)  { p[0] = v; }
inline void wr_u16(uint8_t* p, uint16_t v) { p[0] = uint8_t(v >> 8); p[1] = uint8_t(v); }
inline void wr_u32(uint8_t* p, uint32_t v) {
    p[0] = uint8_t(v >> 24); p[1] = uint8_t(v >> 16);
    p[2] = uint8_t(v >> 8);  p[3] = uint8_t(v);
}
inline void wr_u64(uint8_t* p, uint64_t v) {
    for (int i = 7; i >= 0; --i) { p[i] = uint8_t(v); v >>= 8; }
}
inline void wr_f32(uint8_t* p, float f)  { uint32_t u; std::memcpy(&u, &f, 4); wr_u32(p, u); }
inline void wr_f64(uint8_t* p, double d) { uint64_t u; std::memcpy(&u, &d, 8); wr_u64(p, u); }

} // namespace be
