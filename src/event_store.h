// Thread-safe ring of recent warfare events (Fire / Detonation PDUs).
// The receiver thread appends; the UI thread snapshots for rendering.
//
// Unlike entities, engagements are instantaneous: each event is kept for a
// fixed retention window so the map can animate it and list it, then dropped.
#pragma once
#include <mutex>
#include <atomic>
#include <deque>
#include <vector>
#include <chrono>
#include <cmath>
#include "dis.h"
#include "geodetic.h"

namespace store {

// Must match entity_store.h — both stores timestamp against the same clock.
using Clock = std::chrono::steady_clock;

enum class EventKind : uint8_t { Fire, Detonation };

// How long an event is kept, and how long its map marker animates.
constexpr int kEventRetainMs = 20000;
constexpr int kFireAnimMs    = 4000;
constexpr int kDetAnimMs     = 3500;

struct Engagement {
    uint64_t  seq = 0;              // monotonic, unique — used as a selection key
    EventKind kind = EventKind::Fire;
    dis::EntityId firing;
    dis::EntityId target;
    dis::EntityId munitionId;
    dis::EventId  event;
    dis::MunitionDescriptor descriptor;

    geo::LatLonAlt lla{0, 0, 0};
    bool    locValid = false;       // false => PDU carried no usable world location
    double  range = 0.0;            // Fire: reported range to target (m)
    double  speed = 0.0;            // munition speed (m/s)
    double  heading = 0.0;          // munition ground track (deg, 0=N)
    uint8_t result = 0;             // Detonation only; see dis::detonationResultName
    Clock::time_point when;

    int animMs() const { return kind == EventKind::Fire ? kFireAnimMs : kDetAnimMs; }
    int64_t ageMs(Clock::time_point now) const {
        return std::chrono::duration_cast<std::chrono::milliseconds>(now - when).count();
    }
};

class EventStore {
public:
    // Called from the receiver thread for every valid Fire PDU.
    void addFire(const dis::FirePdu& p) {
        Engagement e;
        e.kind       = EventKind::Fire;
        e.firing     = p.firing;
        e.target     = p.target;
        e.munitionId = p.munitionId;
        e.event      = p.event;
        e.descriptor = p.descriptor;
        e.range      = p.range;
        fillGeometry(e, p.ecefX, p.ecefY, p.ecefZ, p.velX, p.velY, p.velZ);
        push(e);
    }

    // Called from the receiver thread for every valid Detonation PDU.
    void addDetonation(const dis::DetonationPdu& p) {
        Engagement e;
        e.kind       = EventKind::Detonation;
        e.firing     = p.firing;
        e.target     = p.target;
        e.munitionId = p.munitionId;
        e.event      = p.event;
        e.descriptor = p.descriptor;
        e.result     = p.result;
        fillGeometry(e, p.ecefX, p.ecefY, p.ecefZ, p.velX, p.velY, p.velZ);
        push(e);
    }

    // Snapshot for the UI thread, oldest first; drops events past retention.
    std::vector<Engagement> snapshot(int retainMs = kEventRetainMs) {
        std::vector<Engagement> out;
        const auto now = Clock::now();
        std::lock_guard<std::mutex> lk(mtx_);
        while (!events_.empty() && events_.front().ageMs(now) > retainMs)
            events_.pop_front();
        out.assign(events_.begin(), events_.end());
        return out;
    }

    uint64_t fireCount() const { return fires_; }
    uint64_t detonationCount() const { return dets_; }

private:
    // Max events retained regardless of age, so a burst of fire cannot grow
    // the deque without bound.
    static constexpr size_t kMaxEvents = 512;

    // Derive geodetic position plus munition speed/heading from the PDU's ECEF
    // location and velocity. A world location of (near) zero means the sender
    // did not supply one — common for entity-relative detonations — so the
    // renderer falls back to the target entity's last known position.
    static void fillGeometry(Engagement& e,
                             double x, double y, double z,
                             float vx, float vy, float vz) {
        const double r = std::sqrt(x * x + y * y + z * z);
        e.locValid = (r > 1.0e6);          // any point near Earth is ~6.37e6 m out
        if (e.locValid) e.lla = geo::ecefToLla(x, y, z);

        e.speed = std::sqrt(double(vx) * vx + double(vy) * vy + double(vz) * vz);
        if (e.locValid && e.speed > 0.0) {
            double en, nn, up;
            geo::ecefVelToEnu(e.lla.lat, e.lla.lon, vx, vy, vz, en, nn, up);
            e.heading = std::atan2(en, nn) * 57.29577951308232;
            if (e.heading < 0) e.heading += 360.0;
        }
    }

    void push(Engagement& e) {
        e.when = Clock::now();
        std::lock_guard<std::mutex> lk(mtx_);
        e.seq = ++nextSeq_;
        if (e.kind == EventKind::Fire) ++fires_; else ++dets_;
        events_.push_back(e);
        while (events_.size() > kMaxEvents) events_.pop_front();
    }

    std::mutex              mtx_;
    std::deque<Engagement>  events_;
    uint64_t                nextSeq_ = 0;
    std::atomic<uint64_t>   fires_{0};
    std::atomic<uint64_t>   dets_{0};
};

} // namespace store
