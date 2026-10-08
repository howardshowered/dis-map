// dis_sender — broadcasts a handful of moving DIS v7 Entity State PDUs over
// UDP multicast, plus a repeating script of Fire / Detonation PDUs, so you can
// exercise dis_map without a live simulation.
//
//   dis_sender [--group=239.1.2.3] [--port=3000] [--rate=10] [--warfare=0]
//
// Each entity flies a great-ish circle at a fixed lat, drifting in longitude.
// Every few seconds one entity shoots at another: a Fire PDU goes out at the
// shooter's position, then a matching Detonation PDU (same Event ID) after the
// munition's flight time.

#include <winsock2.h>
#include <ws2tcpip.h>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include <thread>
#include <chrono>
#include <vector>

#include "byteorder.h"
#include "geodetic.h"
#include "dis.h"

#pragma comment(lib, "ws2_32.lib")

struct SimEntity {
    uint16_t site, app, entity;
    dis::ForceId force;
    double lat, lon;      // current position
    double lonPerSec;     // eastward drift (deg/s)
    double altM;
    double speedMps;
    const char* marking;

    dis::EntityId id() const { return { site, app, entity }; }
};

// --- PDU header ------------------------------------------------------------
static void writeHeader(uint8_t* buf, uint8_t pduType, uint8_t family,
                        uint16_t length, uint32_t timestamp) {
    be::wr_u8 (buf + 0, dis::kProtocolVersionV7);   // protocol version = 7
    be::wr_u8 (buf + 1, 1);                          // exercise ID
    be::wr_u8 (buf + 2, pduType);
    be::wr_u8 (buf + 3, family);
    be::wr_u32(buf + 4, timestamp);                  // timestamp
    be::wr_u16(buf + 8, length);                     // PDU length
    be::wr_u8 (buf + 10, 0);                         // PDU status (v7)
    be::wr_u8 (buf + 11, 0);                         // padding
}

// DIS entity orientation is the Euler triple (psi about Z, then theta about Y,
// then phi about X) that rotates the *geocentric* axes onto the entity's body
// axes — not the local-tangent-plane attitude. Derive it for an entity at
// (lat, lon) flying a given local heading and pitch with wings level.
static void localAttitudeToEuler(double latDeg, double lonDeg,
                                 double headingDeg, double pitchDeg,
                                 float& psi, float& theta, float& phi) {
    constexpr double kDeg2Rad = 0.017453292519943295;
    const double lat = latDeg * kDeg2Rad, lon = lonDeg * kDeg2Rad;
    const double h = headingDeg * kDeg2Rad, p = pitchDeg * kDeg2Rad;
    const double sl = std::sin(lat), cl = std::cos(lat);
    const double so = std::sin(lon), co = std::cos(lon);

    // Local North / East / Down unit vectors expressed in ECEF.
    const double n[3] = { -sl * co, -sl * so,  cl  };
    const double e[3] = { -so,       co,       0.0 };
    const double d[3] = { -cl * co, -cl * so, -sl  };

    // Body axes in the local NED frame, wings level.
    const double ch = std::cos(h), sh = std::sin(h);
    const double cp = std::cos(p), sp = std::sin(p);
    const double fwdNed[3]   = {  cp * ch,  cp * sh, -sp };
    const double rightNed[3] = { -sh,       ch,       0.0 };
    const double downNed[3]  = {  sp * ch,  sp * sh,  cp };

    auto toEcef = [&](const double v[3], double out[3]) {
        for (int i = 0; i < 3; ++i)
            out[i] = v[0] * n[i] + v[1] * e[i] + v[2] * d[i];
    };
    double fwd[3], right[3], down[3];
    toEcef(fwdNed, fwd);
    toEcef(rightNed, right);
    toEcef(downNed, down);

    // Rows of the ECEF -> body matrix are those body axes, so the standard
    // 3-2-1 extraction recovers the DIS angles.
    theta = (float)std::asin(-fwd[2]);
    psi   = (float)std::atan2(fwd[1], fwd[0]);
    phi   = (float)std::atan2(right[2], down[2]);
}

// Build an Entity State PDU into buf (returns length written). Velocity is
// ECEF m/s; orientation is the DIS Euler triple in radians; appearance carries
// the DIS appearance bits (see dis::kAppearanceDeactivated).
static int buildEspdu(uint8_t* buf, const dis::EntityId& id, dis::ForceId force,
                      const dis::EntityType& type,
                      double lat, double lon, double alt,
                      float vx, float vy, float vz,
                      float psi, float theta, float phi,
                      uint32_t appearance, const char* marking,
                      uint32_t timestamp) {
    std::memset(buf, 0, dis::off::es::kMinLen);
    writeHeader(buf, dis::kPduTypeEntityState, dis::kFamilyEntityInfo,
                (uint16_t)dis::off::es::kMinLen, timestamp);

    dis::writeEntityId(buf + dis::off::es::kEntityId, id);

    be::wr_u8(buf + dis::off::es::kForceId, (uint8_t)force);
    be::wr_u8(buf + dis::off::es::kNumArtic, 0);

    dis::writeEntityType(buf + dis::off::es::kEntityType, type);

    be::wr_f32(buf + dis::off::es::kVelocity + 0, vx);
    be::wr_f32(buf + dis::off::es::kVelocity + 4, vy);
    be::wr_f32(buf + dis::off::es::kVelocity + 8, vz);

    // --- Location: geodetic -> ECEF ---
    double x, y, z;
    geo::llaToEcef(lat, lon, alt, x, y, z);
    be::wr_f64(buf + dis::off::es::kLocation + 0, x);
    be::wr_f64(buf + dis::off::es::kLocation + 8, y);
    be::wr_f64(buf + dis::off::es::kLocation + 16, z);

    // --- Orientation (rad) ---
    be::wr_f32(buf + dis::off::es::kOrientation + 0, psi);
    be::wr_f32(buf + dis::off::es::kOrientation + 4, theta);
    be::wr_f32(buf + dis::off::es::kOrientation + 8, phi);

    be::wr_u32(buf + dis::off::es::kAppearance, appearance);

    // --- Marking: charset (1) = ASCII, then up to 11 chars ---
    be::wr_u8(buf + dis::off::es::kMarking, 1);
    std::strncpy(reinterpret_cast<char*>(buf + dis::off::es::kMarking + 1), marking, 11);

    return dis::off::es::kMinLen;
}

// Entity State for one of the scripted platforms.
static int buildPlatformEspdu(uint8_t* buf, const SimEntity& e, uint32_t timestamp) {
    dis::EntityType type;
    type.kind   = dis::kEntityKindPlatform;
    type.domain = 2;   // air

    // Velocity (ECEF, m/s): true eastward motion so heading resolves to ~090
    // and on-map track vectors point along the direction of travel. The East
    // unit vector in ECEF at (lat,lon) is (-sin lon, cos lon, 0).
    const double lonRad = e.lon * 0.017453292519943295;
    const double dir = (e.lonPerSec >= 0) ? 1.0 : -1.0;   // E or W drift

    // Level flight, nose along the direction of travel.
    float psi, theta, phi;
    localAttitudeToEuler(e.lat, e.lon, (dir >= 0) ? 90.0 : 270.0, 0.0,
                         psi, theta, phi);

    return buildEspdu(buf, e.id(), e.force, type, e.lat, e.lon, e.altM,
                      (float)(dir * e.speedMps * -std::sin(lonRad)),
                      (float)(dir * e.speedMps *  std::cos(lonRad)),
                      0.0f, psi, theta, phi, 0, e.marking, timestamp);
}

// Build a Fire PDU into buf (returns length written).
static int buildFirePdu(uint8_t* buf, const dis::FirePdu& f, uint32_t timestamp) {
    std::memset(buf, 0, dis::off::fire::kMinLen);
    writeHeader(buf, dis::kPduTypeFire, dis::kFamilyWarfare,
                (uint16_t)dis::off::fire::kMinLen, timestamp);

    dis::writeEntityId(buf + dis::off::fire::kFiringId,   f.firing);
    dis::writeEntityId(buf + dis::off::fire::kTargetId,   f.target);
    dis::writeEntityId(buf + dis::off::fire::kMunitionId, f.munitionId);
    dis::writeEventId (buf + dis::off::fire::kEventId,    f.event);

    be::wr_u32(buf + dis::off::fire::kFireMission, f.fireMissionIndex);

    be::wr_f64(buf + dis::off::fire::kLocation + 0,  f.ecefX);
    be::wr_f64(buf + dis::off::fire::kLocation + 8,  f.ecefY);
    be::wr_f64(buf + dis::off::fire::kLocation + 16, f.ecefZ);

    dis::writeDescriptor(buf + dis::off::fire::kDescriptor, f.descriptor);

    be::wr_f32(buf + dis::off::fire::kVelocity + 0, f.velX);
    be::wr_f32(buf + dis::off::fire::kVelocity + 4, f.velY);
    be::wr_f32(buf + dis::off::fire::kVelocity + 8, f.velZ);

    be::wr_f32(buf + dis::off::fire::kRange, f.range);
    return dis::off::fire::kMinLen;
}

// Build a Detonation PDU into buf (returns length written). No variable
// parameter records are appended.
static int buildDetonationPdu(uint8_t* buf, const dis::DetonationPdu& d,
                              uint32_t timestamp) {
    std::memset(buf, 0, dis::off::det::kMinLen);
    writeHeader(buf, dis::kPduTypeDetonation, dis::kFamilyWarfare,
                (uint16_t)dis::off::det::kMinLen, timestamp);

    dis::writeEntityId(buf + dis::off::det::kFiringId,   d.firing);
    dis::writeEntityId(buf + dis::off::det::kTargetId,   d.target);
    dis::writeEntityId(buf + dis::off::det::kMunitionId, d.munitionId);
    dis::writeEventId (buf + dis::off::det::kEventId,    d.event);

    be::wr_f32(buf + dis::off::det::kVelocity + 0, d.velX);
    be::wr_f32(buf + dis::off::det::kVelocity + 4, d.velY);
    be::wr_f32(buf + dis::off::det::kVelocity + 8, d.velZ);

    be::wr_f64(buf + dis::off::det::kLocation + 0,  d.ecefX);
    be::wr_f64(buf + dis::off::det::kLocation + 8,  d.ecefY);
    be::wr_f64(buf + dis::off::det::kLocation + 16, d.ecefZ);

    dis::writeDescriptor(buf + dis::off::det::kDescriptor, d.descriptor);

    be::wr_f32(buf + dis::off::det::kEntityLoc + 0, d.entX);
    be::wr_f32(buf + dis::off::det::kEntityLoc + 4, d.entY);
    be::wr_f32(buf + dis::off::det::kEntityLoc + 8, d.entZ);

    be::wr_u8(buf + dis::off::det::kResult,      d.result);
    be::wr_u8(buf + dis::off::det::kNumVarParam, 0);
    return dis::off::det::kMinLen;
}

// ---------------------------------------------------------------------------
// Warfare script: a fixed rotation of engagements, replayed forever.
// ---------------------------------------------------------------------------
struct Shot {
    int     shooter;     // index into the entity list
    int     target;
    double  flightSec;   // delay between the Fire and the Detonation PDU
    uint8_t result;      // detonation result (see dis::detonationResultName)
    double  missM;       // lateral offset of the impact from the target (m)
};

static const Shot kShots[] = {
    { 0, 1, 2.0, 1,    0.0 },   // BLUE01 -> RED07   : entity impact
    { 1, 3, 2.5, 6,    0.0 },   // RED07  -> BLUE02  : dud, no detonation
    { 3, 4, 3.0, 3, 1500.0 },   // BLUE02 -> SHIP-A  : falls short, ground impact
    { 1, 0, 1.8, 2,  120.0 },   // RED07  -> BLUE01  : proximity near miss
};
constexpr int kNumShots = int(sizeof(kShots) / sizeof(kShots[0]));
constexpr double kFireIntervalSec = 4.0;
constexpr double kMunitionSpeed   = 850.0;   // m/s

// A Fire PDU that has gone out and is awaiting its Detonation PDU. While it is
// pending, the munition reports its own Entity State PDUs so the map shows the
// round in flight, not just the launch and the impact.
struct Pending {
    double   tLeft;      // seconds until the detonation is sent
    double   flightSec;  // total flight time, for interpolating the position
    uint16_t eventNum;
    const Shot* shot;
    dis::EntityId munitionId;
    dis::MunitionDescriptor desc;
    dis::ForceId force;                        // the shooter's force
    double launchLat, launchLon, launchAlt;
    char   marking[12];
};

// Shortest signed longitude difference, so a shot never interpolates the long
// way round the globe.
static double lonDelta(double from, double to) {
    double d = to - from;
    while (d > 180.0)  d -= 360.0;
    while (d < -180.0) d += 360.0;
    return d;
}

static double wrapLon(double lon) {
    if (lon > 180.0)  lon -= 360.0;
    if (lon < -180.0) lon += 360.0;
    return lon;
}

// Flat-earth offset, used to place a near-miss impact point off the target.
static void offsetLatLon(double lat, double lon, double rangeM, double bearingDeg,
                         double& outLat, double& outLon) {
    constexpr double kMetresPerDeg = 111320.0;
    constexpr double kDeg2Rad = 0.017453292519943295;
    const double rad = bearingDeg * kDeg2Rad;
    outLat = lat + (rangeM * std::cos(rad)) / kMetresPerDeg;
    double cosLat = std::cos(lat * kDeg2Rad);
    if (std::fabs(cosLat) < 1e-6) cosLat = (cosLat < 0 ? -1e-6 : 1e-6);
    outLon = lon + (rangeM * std::sin(rad)) / (kMetresPerDeg * cosLat);
}

// The munition every scripted shot uses: a generic US guided anti-air round
// with a high-explosive warhead and a contact fuse.
static dis::MunitionDescriptor makeDescriptor() {
    dis::MunitionDescriptor d;
    d.munition.kind        = 2;     // Munition
    d.munition.domain      = 1;     // Anti-air
    d.munition.country     = 225;   // United States
    d.munition.category    = 1;     // Guided
    d.munition.subcategory = 1;
    d.munition.specific    = 0;
    d.warhead  = 3000;              // High explosive
    d.fuse     = 1000;              // Contact
    d.quantity = 1;
    d.rate     = 0;
    return d;
}

int main(int argc, char** argv) {
    std::string group = "239.1.2.3";
    uint16_t    port  = 3000;
    int         rate  = 10;   // Hz
    bool        warfare = true;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a.rfind("--group=", 0) == 0)       group = a.substr(8);
        else if (a.rfind("--port=", 0) == 0)   port  = (uint16_t)std::atoi(a.c_str() + 7);
        else if (a.rfind("--rate=", 0) == 0)   rate  = std::atoi(a.c_str() + 7);
        else if (a.rfind("--warfare=", 0) == 0) warfare = (std::atoi(a.c_str() + 10) != 0);
    }
    if (rate < 1) rate = 1;

    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) { std::printf("WSAStartup failed\n"); return 1; }

    SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s == INVALID_SOCKET) { std::printf("socket failed\n"); return 1; }

    // Set multicast TTL so packets can leave the host if needed.
    int ttl = 1;
    setsockopt(s, IPPROTO_IP, IP_MULTICAST_TTL, (char*)&ttl, sizeof(ttl));

    sockaddr_in dest{};
    dest.sin_family = AF_INET;
    dest.sin_port   = htons(port);
    inet_pton(AF_INET, group.c_str(), &dest.sin_addr);

    std::vector<SimEntity> ents = {
        { 1, 1, 101, dis::ForceId::Friendly,  34.0,  -80.0,  0.20, 9000, 240, "BLUE01" },
        { 1, 1, 102, dis::ForceId::Opposing,  36.5,  -60.0, -0.15, 11000, 300, "RED07" },
        { 1, 1, 103, dis::ForceId::Neutral,    9.0,   20.0,  0.10, 500,   90, "NEUTRAL" },
        { 1, 1, 104, dis::ForceId::Friendly,  51.5,   -0.1,  0.05, 3000, 150, "BLUE02" },
        { 1, 1, 105, dis::ForceId::Other,    -33.9,  151.2, -0.08, 200,   40, "SHIP-A" },
    };

    std::printf("Sending %zu entities to %s:%u at %d Hz%s. Ctrl+C to stop.\n",
                ents.size(), group.c_str(), port, rate,
                warfare ? ", warfare events on" : "");

    const dis::MunitionDescriptor desc = makeDescriptor();
    std::vector<Pending> pending;
    double   fireTimer = 1.0;      // first shot shortly after start-up
    int      shotIdx   = 0;
    uint16_t eventNum  = 0;

    uint8_t buf[256];
    const double dt = 1.0 / rate;
    uint32_t ts = 0;

    // Entity State for a munition in flight: positioned at (lat,lon,alt) and
    // running at kMunitionSpeed toward the aim point.
    auto sendMunitionState = [&](const Pending& p,
                                 double lat, double lon, double alt,
                                 double aimLat, double aimLon, double aimAlt,
                                 uint32_t appearance) {
        double mx, my, mz, ax, ay, az;
        geo::llaToEcef(lat, lon, alt, mx, my, mz);
        geo::llaToEcef(aimLat, aimLon, aimAlt, ax, ay, az);
        const double dx = ax - mx, dy = ay - my, dz = az - mz;
        const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double inv = (d > 1.0) ? (kMunitionSpeed / d) : 0.0;

        // Nose along the flight vector: resolve it into the local frame to get
        // the heading and pitch the round is actually flying.
        double en, nn, up;
        geo::ecefVelToEnu(lat, lon, dx * inv, dy * inv, dz * inv, en, nn, up);
        const double ground = std::sqrt(en * en + nn * nn);
        const double hdgDeg = std::atan2(en, nn) * 57.29577951308232;
        const double pitchDeg = (ground > 1e-6 || std::fabs(up) > 1e-6)
                              ? std::atan2(up, ground) * 57.29577951308232 : 0.0;
        float psi, theta, phi;
        localAttitudeToEuler(lat, lon, hdgDeg, pitchDeg, psi, theta, phi);

        int len = buildEspdu(buf, p.munitionId, p.force, p.desc.munition,
                             lat, lon, alt,
                             (float)(dx * inv), (float)(dy * inv), (float)(dz * inv),
                             psi, theta, phi, appearance, p.marking, ts++);
        sendto(s, (const char*)buf, len, 0, (sockaddr*)&dest, sizeof(dest));
    };

    while (true) {
        for (auto& e : ents) {
            e.lon = wrapLon(e.lon + e.lonPerSec * dt);

            int len = buildPlatformEspdu(buf, e, ts++);
            sendto(s, (const char*)buf, len, 0, (sockaddr*)&dest, sizeof(dest));
        }

        if (warfare) {
            // --- Fire: launch the next scripted shot when the timer expires ---
            fireTimer -= dt;
            if (fireTimer <= 0.0) {
                fireTimer += kFireIntervalSec;
                const Shot& shot = kShots[shotIdx];
                shotIdx = (shotIdx + 1) % kNumShots;
                const SimEntity& sh = ents[shot.shooter];
                const SimEntity& tg = ents[shot.target];

                double sx, sy, sz, tx, ty, tz;
                geo::llaToEcef(sh.lat, sh.lon, sh.altM, sx, sy, sz);
                geo::llaToEcef(tg.lat, tg.lon, tg.altM, tx, ty, tz);
                const double dx = tx - sx, dy = ty - sy, dz = tz - sz;
                const double range = std::sqrt(dx * dx + dy * dy + dz * dz);

                // Munition velocity: straight at the target at ~850 m/s.
                const double inv = (range > 1.0) ? (kMunitionSpeed / range) : 0.0;

                dis::FirePdu f;
                f.firing     = sh.id();
                f.target     = tg.id();
                f.event      = { sh.site, sh.app, ++eventNum };
                f.munitionId = { sh.site, sh.app, uint16_t(1000 + eventNum) };
                f.ecefX = sx; f.ecefY = sy; f.ecefZ = sz;
                f.descriptor = desc;
                f.velX = (float)(dx * inv);
                f.velY = (float)(dy * inv);
                f.velZ = (float)(dz * inv);
                f.range = (float)range;

                int len = buildFirePdu(buf, f, ts++);
                sendto(s, (const char*)buf, len, 0, (sockaddr*)&dest, sizeof(dest));
                std::printf("FIRE  event %u  %s -> %s  range %.0f km\n",
                            eventNum, sh.marking, tg.marking, range / 1000.0);

                Pending p{};
                p.tLeft      = shot.flightSec;
                p.flightSec  = shot.flightSec;
                p.eventNum   = eventNum;
                p.shot       = &shot;
                p.munitionId = f.munitionId;
                p.desc       = desc;
                p.force      = sh.force;
                p.launchLat  = sh.lat;
                p.launchLon  = sh.lon;
                p.launchAlt  = sh.altM;
                std::snprintf(p.marking, sizeof(p.marking), "MSL-%u", eventNum);
                pending.push_back(p);
            }

            // --- Detonation: fire the matching impact once flight time elapses ---
            for (size_t i = 0; i < pending.size();) {
                pending[i].tLeft -= dt;
                if (pending[i].tLeft > 0.0) {
                    // Still in flight: report the round's own Entity State,
                    // homing from the launch point onto the target.
                    const Pending& fp = pending[i];
                    const SimEntity& tgt = ents[fp.shot->target];
                    const double prog = 1.0 - (fp.tLeft / fp.flightSec);
                    const double mlat = fp.launchLat + (tgt.lat - fp.launchLat) * prog;
                    const double mlon = wrapLon(fp.launchLon +
                                                lonDelta(fp.launchLon, tgt.lon) * prog);
                    const double malt = fp.launchAlt + (tgt.altM - fp.launchAlt) * prog;
                    sendMunitionState(fp, mlat, mlon, malt,
                                      tgt.lat, tgt.lon, tgt.altM, 0);
                    ++i;
                    continue;
                }

                const Pending& p = pending[i];
                const Shot& shot = *p.shot;
                const SimEntity& sh = ents[shot.shooter];
                const SimEntity& tg = ents[shot.target];

                // Impact point: the target's current position, displaced for a
                // miss; a ground impact is placed on the surface.
                double ilat = tg.lat, ilon = tg.lon;
                if (shot.missM > 0.0)
                    offsetLatLon(tg.lat, tg.lon, shot.missM, 90.0, ilat, ilon);
                const double ialt = (shot.result == 3) ? 0.0 : tg.altM;

                dis::DetonationPdu d;
                d.firing     = sh.id();
                d.target     = tg.id();
                d.munitionId = p.munitionId;
                d.event      = { sh.site, sh.app, p.eventNum };
                d.descriptor = p.desc;
                d.result     = shot.result;
                geo::llaToEcef(ilat, ilon, ialt, d.ecefX, d.ecefY, d.ecefZ);

                // Terminal velocity: still running at the shooter on impact.
                double lx, ly, lz;
                geo::llaToEcef(sh.lat, sh.lon, sh.altM, lx, ly, lz);
                const double vx = d.ecefX - lx, vy = d.ecefY - ly, vz = d.ecefZ - lz;
                const double vlen = std::sqrt(vx * vx + vy * vy + vz * vz);
                const double vinv = (vlen > 1.0) ? (850.0 / vlen) : 0.0;
                d.velX = (float)(vx * vinv);
                d.velY = (float)(vy * vinv);
                d.velZ = (float)(vz * vinv);

                int len = buildDetonationPdu(buf, d, ts++);
                sendto(s, (const char*)buf, len, 0, (sockaddr*)&dest, sizeof(dest));

                // Final Entity State for the round, flagged deactivated, so the
                // munition track is removed instead of lingering until stale.
                sendMunitionState(p, ilat, ilon, ialt, ilat, ilon, ialt,
                                  dis::kAppearanceDeactivated);

                std::printf("DET   event %u  %s  %s\n", p.eventNum, tg.marking,
                            dis::detonationResultName(shot.result));

                pending.erase(pending.begin() + i);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(int(1000.0 / rate)));
    }

    closesocket(s);
    WSACleanup();
    return 0;
}
