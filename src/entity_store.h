// Thread-safe store of the latest known state for every entity seen on the wire.
// The receiver thread writes; the UI thread snapshots for rendering.
#pragma once
#include <mutex>
#include <unordered_map>
#include <vector>
#include <chrono>
#include "dis.h"
#include "geodetic.h"

namespace store {

using Clock = std::chrono::steady_clock;

struct Track {
    dis::EntityId id;
    dis::ForceId  force = dis::ForceId::Other;
    dis::EntityType type;
    geo::LatLonAlt lla{0, 0, 0};
    // Raw PDU values, kept alongside the derived geodetic/ground-track figures
    // so the detail panel can show what actually came over the wire.
    double  ecefX = 0, ecefY = 0, ecefZ = 0;   // geocentric location (m)
    double  velX = 0, velY = 0, velZ = 0;      // linear velocity (m/s, ECEF)
    double  speed = 0.0;          // m/s, magnitude of linear velocity (3D)
    double  groundSpeed = 0.0;    // m/s, horizontal component
    double  heading = 0.0;        // degrees, true ground track (0=N, 90=E)
    std::string marking;
    Clock::time_point lastSeen;
    uint64_t updates = 0;
};

class EntityStore {
public:
    // Called from the receiver thread for every valid Entity State PDU.
    void update(const dis::EntityStatePdu& p) {
        // A deactivated entity has left the exercise — a munition that has
        // detonated, say. Drop it rather than letting it sit until it goes
        // stale; sims mark the round deactivated in its final Entity State.
        if (dis::isDeactivated(p.appearance)) {
            std::lock_guard<std::mutex> lk(mtx_);
            tracks_.erase(p.id.key());
            return;
        }

        Track t;
        t.id      = p.id;
        t.force   = p.force;
        t.type    = p.type;
        t.lla     = geo::ecefToLla(p.ecefX, p.ecefY, p.ecefZ);
        t.ecefX   = p.ecefX;
        t.ecefY   = p.ecefY;
        t.ecefZ   = p.ecefZ;
        t.velX    = p.velX;
        t.velY    = p.velY;
        t.velZ    = p.velZ;
        t.speed   = std::sqrt(double(p.velX) * p.velX +
                              double(p.velY) * p.velY +
                              double(p.velZ) * p.velZ);
        double e, n, u;
        geo::ecefVelToEnu(t.lla.lat, t.lla.lon, p.velX, p.velY, p.velZ, e, n, u);
        t.groundSpeed = std::sqrt(e * e + n * n);
        t.heading = std::atan2(e, n) * 57.29577951308232;   // 0=N, 90=E
        if (t.heading < 0) t.heading += 360.0;
        t.marking = p.marking;
        t.lastSeen = Clock::now();

        std::lock_guard<std::mutex> lk(mtx_);
        auto& slot = tracks_[p.id.key()];
        t.updates = slot.updates + 1;
        slot = t;
    }

    // Snapshot for the UI thread; drops tracks not heard from in staleMs.
    std::vector<Track> snapshot(int staleMs = 30000) {
        std::vector<Track> out;
        const auto now = Clock::now();
        std::lock_guard<std::mutex> lk(mtx_);
        for (auto it = tracks_.begin(); it != tracks_.end();) {
            const auto ageMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   now - it->second.lastSeen).count();
            if (ageMs > staleMs) {
                it = tracks_.erase(it);
            } else {
                out.push_back(it->second);
                ++it;
            }
        }
        return out;
    }

    size_t count() {
        std::lock_guard<std::mutex> lk(mtx_);
        return tracks_.size();
    }

private:
    std::mutex mtx_;
    std::unordered_map<uint64_t, Track> tracks_;
};

} // namespace store
