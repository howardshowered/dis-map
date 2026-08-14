// Minimal DIS v7 (IEEE 1278.1-2012) definitions and Entity State PDU parser.
#pragma once
#include <cstdint>
#include <string>
#include <cstring>
#include "byteorder.h"

namespace dis {

constexpr uint8_t kProtocolVersionV7 = 7;   // DIS 7 / IEEE 1278.1-2012
constexpr uint8_t kPduTypeEntityState = 1;
constexpr uint8_t kFamilyEntityInfo   = 1;

// PDU header is 12 bytes. In DIS v7 offset 10 is the PDU Status field
// (padding in earlier revisions); we don't depend on it here.
constexpr int kHeaderLen = 12;

struct EntityId {
    uint16_t site = 0;
    uint16_t application = 0;
    uint16_t entity = 0;

    bool operator==(const EntityId& o) const {
        return site == o.site && application == o.application && entity == o.entity;
    }
    // 48-bit packed key for hashing / map storage.
    uint64_t key() const {
        return (uint64_t(site) << 32) | (uint64_t(application) << 16) | entity;
    }
};

// Force ID (IEEE 1278.1). Used for map colouring.
enum class ForceId : uint8_t {
    Other    = 0,
    Friendly = 1,
    Opposing = 2,
    Neutral  = 3,
};

struct EntityType {
    uint8_t  kind = 0;      // 1=Platform, 2=Munition, 3=Life form, ...
    uint8_t  domain = 0;    // 1=Land, 2=Air, 3=Surface, 4=Subsurface, 5=Space
    uint16_t country = 0;
    uint8_t  category = 0;
    uint8_t  subcategory = 0;
    uint8_t  specific = 0;
    uint8_t  extra = 0;
};

struct EntityStatePdu {
    EntityId    id;
    ForceId     force = ForceId::Other;
    EntityType  type;
    double      ecefX = 0, ecefY = 0, ecefZ = 0;   // geocentric location (m)
    float       velX = 0, velY = 0, velZ = 0;      // linear velocity (m/s, ECEF)
    float       psi = 0, theta = 0, phi = 0;       // orientation (rad)
    uint32_t    appearance = 0;
    std::string marking;                            // up to 11 chars
};

// Field offsets within an Entity State PDU (bytes from start of PDU).
namespace off {
    constexpr int kEntityId    = 12;   // 6 bytes: site, app, entity
    constexpr int kForceId     = 18;   // 1 byte
    constexpr int kNumArtic    = 19;   // 1 byte: number of articulation params
    constexpr int kEntityType  = 20;   // 8 bytes
    constexpr int kAltType     = 28;   // 8 bytes
    constexpr int kVelocity    = 36;   // 3 x float32
    constexpr int kLocation    = 48;   // 3 x float64 (ECEF)
    constexpr int kOrientation = 72;   // 3 x float32
    constexpr int kAppearance  = 84;   // uint32
    constexpr int kDeadReckon  = 88;   // 40 bytes
    constexpr int kMarking     = 128;  // 1 byte charset + 11 byte string
    constexpr int kCapabilities= 140;  // uint32
    constexpr int kMinLen      = 144;  // through end of capabilities
}

// Parse an Entity State PDU from a datagram. Returns false if the buffer is
// not a well-formed DIS v7 Entity State PDU. Non-ES PDUs are silently rejected.
inline bool parseEntityState(const uint8_t* buf, size_t len, EntityStatePdu& out) {
    if (len < off::kMinLen) return false;

    const uint8_t version = be::rd_u8(buf + 0);
    const uint8_t pduType = be::rd_u8(buf + 2);
    if (pduType != kPduTypeEntityState) return false;
    // Accept v6/v7 payloads; warn-worthy but not fatal if version differs.
    (void)version;

    out.id.site        = be::rd_u16(buf + off::kEntityId + 0);
    out.id.application = be::rd_u16(buf + off::kEntityId + 2);
    out.id.entity      = be::rd_u16(buf + off::kEntityId + 4);

    out.force = static_cast<ForceId>(be::rd_u8(buf + off::kForceId));

    out.type.kind        = be::rd_u8 (buf + off::kEntityType + 0);
    out.type.domain      = be::rd_u8 (buf + off::kEntityType + 1);
    out.type.country     = be::rd_u16(buf + off::kEntityType + 2);
    out.type.category    = be::rd_u8 (buf + off::kEntityType + 4);
    out.type.subcategory = be::rd_u8 (buf + off::kEntityType + 5);
    out.type.specific    = be::rd_u8 (buf + off::kEntityType + 6);
    out.type.extra       = be::rd_u8 (buf + off::kEntityType + 7);

    out.velX = be::rd_f32(buf + off::kVelocity + 0);
    out.velY = be::rd_f32(buf + off::kVelocity + 4);
    out.velZ = be::rd_f32(buf + off::kVelocity + 8);

    out.ecefX = be::rd_f64(buf + off::kLocation + 0);
    out.ecefY = be::rd_f64(buf + off::kLocation + 8);
    out.ecefZ = be::rd_f64(buf + off::kLocation + 16);

    out.psi   = be::rd_f32(buf + off::kOrientation + 0);
    out.theta = be::rd_f32(buf + off::kOrientation + 4);
    out.phi   = be::rd_f32(buf + off::kOrientation + 8);

    out.appearance = be::rd_u32(buf + off::kAppearance);

    // Entity marking: byte 0 = character set, bytes 1..11 = text (space/NUL padded).
    char m[12] = {0};
    std::memcpy(m, buf + off::kMarking + 1, 11);
    m[11] = '\0';
    std::string marking(m);
    // Trim trailing spaces / control chars.
    while (!marking.empty() &&
           (marking.back() == ' ' || (unsigned char)marking.back() < 0x20)) {
        marking.pop_back();
    }
    out.marking = marking;

    return true;
}

inline const char* forceName(ForceId f) {
    switch (f) {
        case ForceId::Friendly: return "Friendly";
        case ForceId::Opposing: return "Opposing";
        case ForceId::Neutral:  return "Neutral";
        default:                return "Other";
    }
}

} // namespace dis
