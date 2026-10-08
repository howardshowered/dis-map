// Minimal DIS v7 (IEEE 1278.1-2012) definitions and parsers for the PDUs this
// app understands: Entity State (type 1), Fire (type 2) and Detonation (type 3).
#pragma once
#include <cstdint>
#include <string>
#include <cstring>
#include "byteorder.h"

namespace dis {

constexpr uint8_t kProtocolVersionV7 = 7;   // DIS 7 / IEEE 1278.1-2012

// PDU types we decode.
constexpr uint8_t kPduTypeEntityState = 1;
constexpr uint8_t kPduTypeFire        = 2;
constexpr uint8_t kPduTypeDetonation  = 3;

// Protocol families.
constexpr uint8_t kFamilyEntityInfo = 1;
constexpr uint8_t kFamilyWarfare    = 2;

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
    // Entity ID 0:0:0 means "no entity"; 65535s are the NO_ENTITY / ALL wildcards.
    bool isNone() const { return site == 0 && application == 0 && entity == 0; }
};

// Event identifier (IEEE 1278.1 §5.2.9). The Fire and Detonation PDUs of one
// engagement carry the same Event ID, which is how the shot and the impact are
// paired up.
struct EventId {
    uint16_t site = 0;
    uint16_t application = 0;
    uint16_t number = 0;

    bool operator==(const EventId& o) const {
        return site == o.site && application == o.application && number == o.number;
    }
    uint64_t key() const {
        return (uint64_t(site) << 32) | (uint64_t(application) << 16) | number;
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

// Munition descriptor (IEEE 1278.1 §5.2.17) — 16 bytes, carried by both the
// Fire and the Detonation PDU.
struct MunitionDescriptor {
    EntityType munition;    // what was fired
    uint16_t   warhead  = 0;
    uint16_t   fuse     = 0;
    uint16_t   quantity = 0;   // rounds in the burst
    uint16_t   rate     = 0;   // rounds per minute
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

// Fire PDU (IEEE 1278.1 §7.3.2) — a weapon/munition was launched.
struct FirePdu {
    EntityId firing;        // who shot
    EntityId target;        // intended target (may be 0:0:0 = unknown)
    EntityId munitionId;    // ID the munition will use in its own Entity State
    EventId  event;
    uint32_t fireMissionIndex = 0;
    double   ecefX = 0, ecefY = 0, ecefZ = 0;   // launch point (m)
    MunitionDescriptor descriptor;
    float    velX = 0, velY = 0, velZ = 0;      // munition velocity (m/s, ECEF)
    float    range = 0;                          // m to the target, 0 if unknown
};

// Detonation PDU (IEEE 1278.1 §7.3.3) — a munition detonated or impacted.
struct DetonationPdu {
    EntityId firing;
    EntityId target;
    EntityId munitionId;
    EventId  event;
    float    velX = 0, velY = 0, velZ = 0;      // munition velocity at impact
    double   ecefX = 0, ecefY = 0, ecefZ = 0;   // detonation point (m)
    MunitionDescriptor descriptor;
    float    entX = 0, entY = 0, entZ = 0;      // impact point in target's frame
    uint8_t  result = 0;                         // see detonationResultName()
    uint8_t  numVariableParams = 0;              // trailing records, not decoded
};

// Field offsets in bytes from the start of the PDU, per PDU type.
namespace off {

// Entity State PDU (type 1): 144-byte fixed record, then articulation params.
namespace es {
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

// Fire PDU (type 2): 96-byte fixed record, no variable portion.
namespace fire {
    constexpr int kFiringId    = 12;   // 6 bytes
    constexpr int kTargetId    = 18;   // 6 bytes
    constexpr int kMunitionId  = 24;   // 6 bytes (munition/expendable ID)
    constexpr int kEventId     = 30;   // 6 bytes
    constexpr int kFireMission = 36;   // uint32
    constexpr int kLocation    = 40;   // 3 x float64 (ECEF)
    constexpr int kDescriptor  = 64;   // 16 bytes
    constexpr int kVelocity    = 80;   // 3 x float32
    constexpr int kRange       = 92;   // float32
    constexpr int kMinLen      = 96;
}

// Detonation PDU (type 3): 104-byte fixed record, then variable parameters.
namespace det {
    constexpr int kFiringId    = 12;   // 6 bytes
    constexpr int kTargetId    = 18;   // 6 bytes
    constexpr int kMunitionId  = 24;   // 6 bytes
    constexpr int kEventId     = 30;   // 6 bytes
    constexpr int kVelocity    = 36;   // 3 x float32
    constexpr int kLocation    = 48;   // 3 x float64 (ECEF)
    constexpr int kDescriptor  = 72;   // 16 bytes
    constexpr int kEntityLoc   = 88;   // 3 x float32 (target entity frame)
    constexpr int kResult      = 100;  // uint8
    constexpr int kNumVarParam = 101;  // uint8
    constexpr int kPad         = 102;  // 2 bytes
    constexpr int kMinLen      = 104;  // variable parameter records follow
}

constexpr int kEntityIdLen   = 6;
constexpr int kEventIdLen    = 6;
constexpr int kEntityTypeLen = 8;
constexpr int kDescriptorLen = 16;

} // namespace off

// ---------------------------------------------------------------------------
// Shared record readers / writers
// ---------------------------------------------------------------------------
inline EntityId readEntityId(const uint8_t* p) {
    EntityId id;
    id.site        = be::rd_u16(p + 0);
    id.application = be::rd_u16(p + 2);
    id.entity      = be::rd_u16(p + 4);
    return id;
}

inline EventId readEventId(const uint8_t* p) {
    EventId e;
    e.site        = be::rd_u16(p + 0);
    e.application = be::rd_u16(p + 2);
    e.number      = be::rd_u16(p + 4);
    return e;
}

inline EntityType readEntityType(const uint8_t* p) {
    EntityType t;
    t.kind        = be::rd_u8 (p + 0);
    t.domain      = be::rd_u8 (p + 1);
    t.country     = be::rd_u16(p + 2);
    t.category    = be::rd_u8 (p + 4);
    t.subcategory = be::rd_u8 (p + 5);
    t.specific    = be::rd_u8 (p + 6);
    t.extra       = be::rd_u8 (p + 7);
    return t;
}

inline MunitionDescriptor readDescriptor(const uint8_t* p) {
    MunitionDescriptor d;
    d.munition = readEntityType(p);
    d.warhead  = be::rd_u16(p + 8);
    d.fuse     = be::rd_u16(p + 10);
    d.quantity = be::rd_u16(p + 12);
    d.rate     = be::rd_u16(p + 14);
    return d;
}

inline void writeEntityId(uint8_t* p, const EntityId& id) {
    be::wr_u16(p + 0, id.site);
    be::wr_u16(p + 2, id.application);
    be::wr_u16(p + 4, id.entity);
}

inline void writeEventId(uint8_t* p, const EventId& e) {
    be::wr_u16(p + 0, e.site);
    be::wr_u16(p + 2, e.application);
    be::wr_u16(p + 4, e.number);
}

inline void writeEntityType(uint8_t* p, const EntityType& t) {
    be::wr_u8 (p + 0, t.kind);
    be::wr_u8 (p + 1, t.domain);
    be::wr_u16(p + 2, t.country);
    be::wr_u8 (p + 4, t.category);
    be::wr_u8 (p + 5, t.subcategory);
    be::wr_u8 (p + 6, t.specific);
    be::wr_u8 (p + 7, t.extra);
}

inline void writeDescriptor(uint8_t* p, const MunitionDescriptor& d) {
    writeEntityType(p, d.munition);
    be::wr_u16(p + 8,  d.warhead);
    be::wr_u16(p + 10, d.fuse);
    be::wr_u16(p + 12, d.quantity);
    be::wr_u16(p + 14, d.rate);
}

// PDU type from the header, or 0 if the buffer is too short to hold one.
inline uint8_t pduType(const uint8_t* buf, size_t len) {
    return (len >= size_t(kHeaderLen)) ? be::rd_u8(buf + 2) : 0;
}

// ---------------------------------------------------------------------------
// Parsers. Each returns false if the buffer is not a well-formed PDU of that
// type; PDUs of other types are silently rejected.
// ---------------------------------------------------------------------------
inline bool parseEntityState(const uint8_t* buf, size_t len, EntityStatePdu& out) {
    if (len < off::es::kMinLen) return false;

    const uint8_t version = be::rd_u8(buf + 0);
    const uint8_t pduType = be::rd_u8(buf + 2);
    if (pduType != kPduTypeEntityState) return false;
    // Accept v6/v7 payloads; warn-worthy but not fatal if version differs.
    (void)version;

    out.id    = readEntityId(buf + off::es::kEntityId);
    out.force = static_cast<ForceId>(be::rd_u8(buf + off::es::kForceId));
    out.type  = readEntityType(buf + off::es::kEntityType);

    out.velX = be::rd_f32(buf + off::es::kVelocity + 0);
    out.velY = be::rd_f32(buf + off::es::kVelocity + 4);
    out.velZ = be::rd_f32(buf + off::es::kVelocity + 8);

    out.ecefX = be::rd_f64(buf + off::es::kLocation + 0);
    out.ecefY = be::rd_f64(buf + off::es::kLocation + 8);
    out.ecefZ = be::rd_f64(buf + off::es::kLocation + 16);

    out.psi   = be::rd_f32(buf + off::es::kOrientation + 0);
    out.theta = be::rd_f32(buf + off::es::kOrientation + 4);
    out.phi   = be::rd_f32(buf + off::es::kOrientation + 8);

    out.appearance = be::rd_u32(buf + off::es::kAppearance);

    // Entity marking: byte 0 = character set, bytes 1..11 = text (space/NUL padded).
    char m[12] = {0};
    std::memcpy(m, buf + off::es::kMarking + 1, 11);
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

inline bool parseFire(const uint8_t* buf, size_t len, FirePdu& out) {
    if (len < off::fire::kMinLen) return false;
    if (be::rd_u8(buf + 2) != kPduTypeFire) return false;

    out.firing     = readEntityId(buf + off::fire::kFiringId);
    out.target     = readEntityId(buf + off::fire::kTargetId);
    out.munitionId = readEntityId(buf + off::fire::kMunitionId);
    out.event      = readEventId (buf + off::fire::kEventId);

    out.fireMissionIndex = be::rd_u32(buf + off::fire::kFireMission);

    out.ecefX = be::rd_f64(buf + off::fire::kLocation + 0);
    out.ecefY = be::rd_f64(buf + off::fire::kLocation + 8);
    out.ecefZ = be::rd_f64(buf + off::fire::kLocation + 16);

    out.descriptor = readDescriptor(buf + off::fire::kDescriptor);

    out.velX = be::rd_f32(buf + off::fire::kVelocity + 0);
    out.velY = be::rd_f32(buf + off::fire::kVelocity + 4);
    out.velZ = be::rd_f32(buf + off::fire::kVelocity + 8);

    out.range = be::rd_f32(buf + off::fire::kRange);
    return true;
}

inline bool parseDetonation(const uint8_t* buf, size_t len, DetonationPdu& out) {
    if (len < off::det::kMinLen) return false;
    if (be::rd_u8(buf + 2) != kPduTypeDetonation) return false;

    out.firing     = readEntityId(buf + off::det::kFiringId);
    out.target     = readEntityId(buf + off::det::kTargetId);
    out.munitionId = readEntityId(buf + off::det::kMunitionId);
    out.event      = readEventId (buf + off::det::kEventId);

    out.velX = be::rd_f32(buf + off::det::kVelocity + 0);
    out.velY = be::rd_f32(buf + off::det::kVelocity + 4);
    out.velZ = be::rd_f32(buf + off::det::kVelocity + 8);

    out.ecefX = be::rd_f64(buf + off::det::kLocation + 0);
    out.ecefY = be::rd_f64(buf + off::det::kLocation + 8);
    out.ecefZ = be::rd_f64(buf + off::det::kLocation + 16);

    out.descriptor = readDescriptor(buf + off::det::kDescriptor);

    out.entX = be::rd_f32(buf + off::det::kEntityLoc + 0);
    out.entY = be::rd_f32(buf + off::det::kEntityLoc + 4);
    out.entZ = be::rd_f32(buf + off::det::kEntityLoc + 8);

    out.result            = be::rd_u8(buf + off::det::kResult);
    out.numVariableParams = be::rd_u8(buf + off::det::kNumVarParam);
    return true;
}

// ---------------------------------------------------------------------------
// Enumeration names
// ---------------------------------------------------------------------------
inline const char* forceName(ForceId f) {
    switch (f) {
        case ForceId::Friendly: return "Friendly";
        case ForceId::Opposing: return "Opposing";
        case ForceId::Neutral:  return "Neutral";
        default:                return "Other";
    }
}

inline const char* pduTypeName(uint8_t t) {
    switch (t) {
        case kPduTypeEntityState: return "Entity State";
        case kPduTypeFire:        return "Fire";
        case kPduTypeDetonation:  return "Detonation";
        default:                  return "Other";
    }
}

// Detonation Result field (IEEE 1278.1-2012 Annex B, UID 62).
inline const char* detonationResultName(uint8_t r) {
    switch (r) {
        case 0:  return "Other";
        case 1:  return "Entity Impact";
        case 2:  return "Entity Proximate Detonation";
        case 3:  return "Ground Impact";
        case 4:  return "Ground Proximate Detonation";
        case 5:  return "Detonation";
        case 6:  return "None / Dud";
        case 7:  return "HE hit, small";
        case 8:  return "HE hit, medium";
        case 9:  return "HE hit, large";
        case 10: return "Armour-piercing hit";
        case 11: return "Dirt blast, small";
        case 12: return "Dirt blast, medium";
        case 13: return "Dirt blast, large";
        case 14: return "Water blast, small";
        case 15: return "Water blast, medium";
        case 16: return "Water blast, large";
        case 17: return "Air hit";
        case 18: return "Building hit, small";
        case 19: return "Building hit, medium";
        case 20: return "Building hit, large";
        case 21: return "Mine-clearing line charge";
        case 22: return "Environment object impact";
        case 23: return "Environment object proximate detonation";
        case 24: return "Water impact";
        case 25: return "Air burst";
        default: return "Unknown";
    }
}

// True when the detonation result indicates the round struck its target (as
// opposed to a near-miss, a ground/water impact, or a dud). Drives map colour.
inline bool detonationIsHit(uint8_t r) {
    switch (r) {
        case 1: case 2:                                 // entity impact / proximate
        case 7: case 8: case 9: case 10:                // HE / AP hits
        case 17:                                        // air hit
        case 18: case 19: case 20:                      // building hits
            return true;
        default:
            return false;
    }
}

// True for a round that failed to function.
inline bool detonationIsDud(uint8_t r) { return r == 6; }

} // namespace dis
