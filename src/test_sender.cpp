// dis_sender — broadcasts a handful of moving DIS v7 Entity State PDUs over
// UDP multicast so you can exercise dis_map without a live simulation.
//
//   dis_sender [--group=239.1.2.3] [--port=3000] [--rate=10]
//
// Each entity flies a great-ish circle at a fixed lat, drifting in longitude.

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
};

// Build an Entity State PDU into buf (returns length written).
static int buildEspdu(uint8_t* buf, const SimEntity& e, uint32_t timestamp) {
    std::memset(buf, 0, dis::off::kMinLen);

    // --- PDU header (12 bytes) ---
    be::wr_u8 (buf + 0, dis::kProtocolVersionV7);   // protocol version = 7
    be::wr_u8 (buf + 1, 1);                          // exercise ID
    be::wr_u8 (buf + 2, dis::kPduTypeEntityState);   // PDU type = 1
    be::wr_u8 (buf + 3, dis::kFamilyEntityInfo);     // protocol family = 1
    be::wr_u32(buf + 4, timestamp);                  // timestamp
    be::wr_u16(buf + 8, (uint16_t)dis::off::kMinLen);// PDU length
    be::wr_u8 (buf + 10, 0);                         // PDU status (v7)
    be::wr_u8 (buf + 11, 0);                         // padding

    // --- Entity ID ---
    be::wr_u16(buf + dis::off::kEntityId + 0, e.site);
    be::wr_u16(buf + dis::off::kEntityId + 2, e.app);
    be::wr_u16(buf + dis::off::kEntityId + 4, e.entity);

    be::wr_u8(buf + dis::off::kForceId, (uint8_t)e.force);
    be::wr_u8(buf + dis::off::kNumArtic, 0);

    // --- Entity type: platform / air / generic ---
    be::wr_u8 (buf + dis::off::kEntityType + 0, 1);  // kind = platform
    be::wr_u8 (buf + dis::off::kEntityType + 1, 2);  // domain = air

    // --- Velocity (ECEF, m/s): true eastward motion so heading resolves to
    //     ~090 and on-map track vectors point along the direction of travel.
    //     East unit vector in ECEF at (lat,lon) is (-sin lon, cos lon, 0).
    const double lonRad = e.lon * 0.017453292519943295;
    const double dir = (e.lonPerSec >= 0) ? 1.0 : -1.0;   // E or W drift
    be::wr_f32(buf + dis::off::kVelocity + 0, (float)(dir * e.speedMps * -std::sin(lonRad)));
    be::wr_f32(buf + dis::off::kVelocity + 4, (float)(dir * e.speedMps *  std::cos(lonRad)));
    be::wr_f32(buf + dis::off::kVelocity + 8, 0.0f);

    // --- Location: geodetic -> ECEF ---
    double x, y, z;
    geo::llaToEcef(e.lat, e.lon, e.altM, x, y, z);
    be::wr_f64(buf + dis::off::kLocation + 0, x);
    be::wr_f64(buf + dis::off::kLocation + 8, y);
    be::wr_f64(buf + dis::off::kLocation + 16, z);

    // --- Orientation (rad) ---
    be::wr_f32(buf + dis::off::kOrientation + 0, 0.0f);
    be::wr_f32(buf + dis::off::kOrientation + 4, 0.0f);
    be::wr_f32(buf + dis::off::kOrientation + 8, 0.0f);

    // --- Appearance ---
    be::wr_u32(buf + dis::off::kAppearance, 0);

    // --- Marking: charset (1) = ASCII, then up to 11 chars ---
    be::wr_u8(buf + dis::off::kMarking, 1);
    std::strncpy(reinterpret_cast<char*>(buf + dis::off::kMarking + 1), e.marking, 11);

    return dis::off::kMinLen;
}

int main(int argc, char** argv) {
    std::string group = "239.1.2.3";
    uint16_t    port  = 3000;
    int         rate  = 10;   // Hz

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a.rfind("--group=", 0) == 0)     group = a.substr(8);
        else if (a.rfind("--port=", 0) == 0) port  = (uint16_t)std::atoi(a.c_str() + 7);
        else if (a.rfind("--rate=", 0) == 0) rate  = std::atoi(a.c_str() + 7);
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

    std::printf("Sending %zu entities to %s:%u at %d Hz. Ctrl+C to stop.\n",
                ents.size(), group.c_str(), port, rate);

    uint8_t buf[256];
    const double dt = 1.0 / rate;
    uint32_t ts = 0;
    while (true) {
        for (auto& e : ents) {
            e.lon += e.lonPerSec * dt;
            if (e.lon > 180.0)  e.lon -= 360.0;
            if (e.lon < -180.0) e.lon += 360.0;

            int len = buildEspdu(buf, e, ts++);
            sendto(s, (const char*)buf, len, 0, (sockaddr*)&dest, sizeof(dest));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(int(1000.0 / rate)));
    }

    closesocket(s);
    WSACleanup();
    return 0;
}
