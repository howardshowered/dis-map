// DIS Map — Win32 + GDI+ realtime display of DIS v7 Entity State, Fire and
// Detonation PDUs received over UDP multicast.
//
//   Left-drag : pan     Mouse wheel : zoom     R : reset view
//   G : toggle graticule    E : toggle warfare events    Esc : quit
//
// Optional: drop an equirectangular "world.png" (full -180..180 lon,
// -90..90 lat) next to the exe to use it as a map background.

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <windowsx.h>   // GET_X_LPARAM / GET_Y_LPARAM
#include <objidl.h>     // IStream — needed by GDI+ under WIN32_LEAN_AND_MEAN
#include <shellapi.h>   // CommandLineToArgvW
#include <algorithm>
// GDI+ headers use unqualified min/max; NOMINMAX suppresses the Windows macros,
// so pull std::min/std::max into the Gdiplus namespace before including it.
namespace Gdiplus { using std::min; using std::max; }
#include <gdiplus.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <cstdint>
#include <cwchar>
#include <fstream>

#include "entity_store.h"
#include "event_store.h"
#include "receiver.h"
#include "coastline.h"

#pragma comment(lib, "gdiplus.lib")

using namespace Gdiplus;

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------
static store::EntityStore     g_store;
static store::EventStore      g_events;               // Fire / Detonation history
static net::MulticastReceiver* g_receiver = nullptr;
static ULONG_PTR              g_gdiToken = 0;
static Image*                 g_worldImg = nullptr;   // optional background
static bool                   g_showGrid = true;
static bool                   g_showCoast = true;     // built-in coastline layer
static bool                   g_showEvents = true;    // warfare event overlay + log

// Selection + hit-testing (all touched only on the UI thread). A track and an
// event are never selected at the same time.
static uint64_t               g_selectedKey = 0;      // 0 => no track selected
static uint64_t               g_selectedEventSeq = 0; // 0 => no event selected
struct HitEntry { uint64_t key; double sx, sy; };
static std::vector<HitEntry>  g_lastPositions;
static std::vector<HitEntry>  g_lastEventPositions;
// Clickable rows of the warfare event log. Event markers usually sit right on
// top of the shooter's or target's symbol, so the log is the reliable way to
// pick one.
struct LogRow { uint64_t seq; float x0, y0, x1, y1; };
static std::vector<LogRow>    g_lastLogRows;
static bool                   g_cursorValid = false;
static double                 g_cursorLat = 0, g_cursorLon = 0;

static std::mutex             g_statusMtx;
static std::wstring           g_statusLine = L"Starting...";

// Connection config + settings-dialog handles.
static std::string            g_group = "239.1.2.3";
static uint16_t               g_port  = 3000;
static std::string            g_iface = "0.0.0.0";   // local NIC for the join
static HINSTANCE              g_hInst = nullptr;
static HWND                   g_mainHwnd = nullptr;
static HWND                   g_settingsHwnd = nullptr;
static HWND                   g_hEditAddr = nullptr;
static HWND                   g_hEditPort = nullptr;
static HWND                   g_hEditIface = nullptr;
constexpr int                 kIdEditAddr  = 101;
constexpr int                 kIdEditPort  = 102;
constexpr int                 kIdEditIface = 103;

static void saveConfig();   // persists g_group / g_port (defined below)

// View transform: equirectangular normalized [0,1] * scale + offset.
struct View {
    double scale = 0;     // pixels across the full 360deg (0 => fit on first paint)
    double offX = 0;
    double offY = 0;
} g_view;

static bool   g_dragging = false;
static POINT  g_dragStart{};
static double g_dragOffX = 0, g_dragOffY = 0;

// ---------------------------------------------------------------------------
// Projection helpers (equirectangular)
// ---------------------------------------------------------------------------
static void worldToScreen(double lat, double lon, double& sx, double& sy) {
    const double nx = (lon + 180.0) / 360.0;   // [0,1]
    const double ny = (90.0 - lat) / 180.0;    // [0,1]
    sx = nx * g_view.scale + g_view.offX;
    sy = ny * (g_view.scale * 0.5) + g_view.offY;   // map is 2:1
}

static void screenToWorld(double sx, double sy, double& lat, double& lon) {
    const double nx = (sx - g_view.offX) / g_view.scale;
    const double ny = (sy - g_view.offY) / (g_view.scale * 0.5);
    lon = nx * 360.0 - 180.0;
    lat = 90.0 - ny * 180.0;
}

static void setStatus(const std::wstring& s) {
    std::lock_guard<std::mutex> lk(g_statusMtx);
    g_statusLine = s;
}

static void fitView(int w, int h) {
    // Fit a 2:1 equirectangular map inside the client area with a margin.
    const int margin = 8;
    double scaleW = double(w - 2 * margin);
    double scaleH = double(h - 2 * margin) * 2.0;   // height maps to scale*0.5
    g_view.scale = std::min(scaleW, scaleH);
    if (g_view.scale < 100) g_view.scale = 100;
    g_view.offX = (w - g_view.scale) / 2.0;
    g_view.offY = (h - g_view.scale * 0.5) / 2.0;
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------
static Color forceColor(dis::ForceId f) {
    switch (f) {
        case dis::ForceId::Friendly: return Color(255, 80, 150, 255);   // blue
        case dis::ForceId::Opposing: return Color(255, 240, 70, 70);    // red
        case dis::ForceId::Neutral:  return Color(255, 90, 210, 120);   // green
        default:                     return Color(255, 220, 220, 100);  // yellow
    }
}

// Map a point given in a track-local frame (fwd = along the ground track,
// right = 90° clockwise from it) to screen pixels. Screen y grows downward and
// heading 0 is north, so forward is (sin h, -cos h) and right is (cos h, sin h).
static PointF trackLocalPoint(double sx, double sy, double headingRad,
                              double fwd, double right) {
    const double s = std::sin(headingRad), c = std::cos(headingRad);
    return PointF((REAL)(sx + fwd * s + right * c),
                  (REAL)(sy - fwd * c + right * s));
}

// Symbol for one track. Munitions get a sharp dart pointing along their ground
// track so a round in flight reads differently from the platform that fired it;
// every other entity kind keeps the standard dot.
static void drawTrackSymbol(Graphics& g, const store::Track& t,
                            double sx, double sy, const Color& c) {
    SolidBrush fill(c);
    Pen ring(Color(200, 255, 255, 255), 1.0f);

    if (!dis::isMunition(t.type)) {
        const REAL r = 5.0f;
        g.FillEllipse(&fill, (REAL)(sx - r), (REAL)(sy - r), r * 2, r * 2);
        g.DrawEllipse(&ring, (REAL)(sx - r), (REAL)(sy - r), r * 2, r * 2);
        return;
    }

    if (t.groundSpeed <= 0.5) {
        // No usable ground track, so draw a diamond — it implies no direction.
        PointF d[4] = {
            PointF((REAL)sx, (REAL)(sy - 6)), PointF((REAL)(sx + 4), (REAL)sy),
            PointF((REAL)sx, (REAL)(sy + 6)), PointF((REAL)(sx - 4), (REAL)sy),
        };
        g.FillPolygon(&fill, d, 4);
        g.DrawPolygon(&ring, d, 4);
        return;
    }

    const double rad = t.heading * 0.017453292519943295;
    PointF dart[4] = {
        trackLocalPoint(sx, sy, rad,  8.0,  0.0),   // tip
        trackLocalPoint(sx, sy, rad, -4.0,  5.0),   // right barb
        trackLocalPoint(sx, sy, rad, -1.5,  0.0),   // tail notch
        trackLocalPoint(sx, sy, rad, -4.0, -5.0),   // left barb
    };
    g.FillPolygon(&fill, dart, 4);
    g.DrawPolygon(&ring, dart, 4);
}

static Color fireColor(BYTE a) { return Color(a, 255, 196, 64); }   // amber

static Color detColor(uint8_t result, BYTE a) {
    if (dis::detonationIsDud(result)) return Color(a, 165, 175, 185);  // grey — no function
    if (dis::detonationIsHit(result)) return Color(a, 255, 85, 55);    // red — target struck
    return Color(a, 255, 150, 70);                                      // orange — miss / ground
}

// Flat-earth offset: move (lat,lon) rangeM metres along a true heading. Only
// used to draw a shot line when the target entity's position is unknown.
static void offsetLatLon(double lat, double lon, double rangeM, double headingDeg,
                         double& outLat, double& outLon) {
    constexpr double kMetresPerDeg = 111320.0;
    constexpr double kDeg2Rad = 0.017453292519943295;
    const double rad = headingDeg * kDeg2Rad;
    outLat = lat + (rangeM * std::cos(rad)) / kMetresPerDeg;
    double cosLat = std::cos(lat * kDeg2Rad);
    if (std::fabs(cosLat) < 1e-6) cosLat = (cosLat < 0 ? -1e-6 : 1e-6);
    outLon = lon + (rangeM * std::sin(rad)) / (kMetresPerDeg * cosLat);
}

// Warfare event overlay: amber muzzle flash + shot line for Fire PDUs,
// expanding shockwave rings for Detonation PDUs. Each marker animates for a
// few seconds, then lingers as a faint residual until it ages out of the
// store — residuals stay clickable so you can inspect a shot after the fact.
static void drawEvents(Graphics& g,
                       const std::vector<store::Engagement>& events,
                       const std::unordered_map<uint64_t, const store::Track*>& byKey,
                       int w, int h) {
    g_lastEventPositions.clear();
    const auto now = store::Clock::now();

    for (const auto& e : events) {
        // Anchor the marker: prefer the PDU's own world location, else fall back
        // to the target entity (detonations may carry only entity-frame coords).
        double lat = e.lla.lat, lon = e.lla.lon;
        if (!e.locValid) {
            auto it = byKey.find(e.target.key());
            if (it == byKey.end()) continue;   // nothing to anchor to — skip
            lat = it->second->lla.lat;
            lon = it->second->lla.lon;
        }

        double sx, sy;
        worldToScreen(lat, lon, sx, sy);
        if (sx < -60 || sy < -60 || sx > w + 60 || sy > h + 60) continue;

        g_lastEventPositions.push_back({ e.seq, sx, sy });

        // k runs 1 -> 0 across the animation window, then goes negative.
        const double k = 1.0 - double(e.ageMs(now)) / e.animMs();
        const bool animating = (k > 0.0);
        const BYTE a = (BYTE)(animating ? (70.0 + 165.0 * k) : 55.0);

        if (e.seq == g_selectedEventSeq) {
            Pen sel(Color(255, 255, 230, 120), 2.0f);
            g.DrawEllipse(&sel, (REAL)(sx - 13), (REAL)(sy - 13), 26.0f, 26.0f);
        }

        if (e.kind == store::EventKind::Fire) {
            // Shot line toward the target's current position, or along the
            // munition's heading for the reported range if the target is unknown.
            double tx = 0, ty = 0;
            bool haveLine = false;
            auto it = byKey.find(e.target.key());
            if (!e.target.isNone() && it != byKey.end()) {
                worldToScreen(it->second->lla.lat, it->second->lla.lon, tx, ty);
                haveLine = true;
            } else if (e.range > 0.0 && e.speed > 0.0) {
                double tlat, tlon;
                offsetLatLon(lat, lon, e.range, e.heading, tlat, tlon);
                worldToScreen(tlat, tlon, tx, ty);
                haveLine = true;
            }
            if (haveLine) {
                Pen shot(Color((BYTE)(a * 0.75), 255, 210, 110), 1.6f);
                REAL dash[] = { 5.0f, 4.0f };
                shot.SetDashPattern(dash, 2);
                g.DrawLine(&shot, (REAL)sx, (REAL)sy, (REAL)tx, (REAL)ty);
                if (animating) {
                    // Tracer running from launch point to target over the window.
                    const double t = 1.0 - k;
                    const double px = sx + (tx - sx) * t, py = sy + (ty - sy) * t;
                    SolidBrush tracer(Color(235, 255, 235, 170));
                    g.FillEllipse(&tracer, (REAL)(px - 3), (REAL)(py - 3), 6.0f, 6.0f);
                }
            }

            // Muzzle flash: a four-ray star that shrinks as the event ages.
            static const double kRay[4][2] = {
                { 0.7071, 0.7071 }, { -0.7071, 0.7071 },
                { -0.7071, -0.7071 }, { 0.7071, -0.7071 }
            };
            const double rr = animating ? (9.0 + 8.0 * k) : 6.0;
            Pen ray(fireColor(a), animating ? 2.0f : 1.0f);
            for (const auto& d : kRay) {
                g.DrawLine(&ray,
                           (REAL)(sx + d[0] * rr * 0.35), (REAL)(sy + d[1] * rr * 0.35),
                           (REAL)(sx + d[0] * rr),        (REAL)(sy + d[1] * rr));
            }
            SolidBrush core(fireColor(a));
            g.FillEllipse(&core, (REAL)(sx - 3), (REAL)(sy - 3), 6.0f, 6.0f);
        } else {
            // Detonation: two trailing shockwave rings plus a bright core.
            constexpr double kMaxR = 26.0;
            for (int ring = 0; ring < 2; ++ring) {
                const double phase = (1.0 - k) - ring * 0.25;
                if (phase <= 0.0 || phase >= 1.0) continue;
                const double rr = 4.0 + kMaxR * phase;
                Pen wave(detColor(e.result, (BYTE)(a * (1.0 - phase))), 2.0f);
                g.DrawEllipse(&wave, (REAL)(sx - rr), (REAL)(sy - rr),
                              (REAL)(rr * 2), (REAL)(rr * 2));
            }
            if (dis::detonationIsDud(e.result)) {
                // A dud gets an X so it never reads as a live burst.
                Pen x(detColor(e.result, a), 2.0f);
                const REAL d = 5.0f;
                g.DrawLine(&x, (REAL)(sx - d), (REAL)(sy - d), (REAL)(sx + d), (REAL)(sy + d));
                g.DrawLine(&x, (REAL)(sx - d), (REAL)(sy + d), (REAL)(sx + d), (REAL)(sy - d));
            } else {
                const REAL cr = (REAL)(animating ? (3.0 + 4.0 * k) : 3.0);
                SolidBrush core(detColor(e.result, a));
                g.FillEllipse(&core, (REAL)(sx - cr), (REAL)(sy - cr), cr * 2, cr * 2);
            }
        }
    }
}

// Scrolling log of the most recent engagements, newest first (bottom-right).
// Rows are clickable — see g_lastLogRows.
static void drawEventLog(Graphics& g, Font& font,
                         const std::vector<store::Engagement>& events,
                         int w, int h) {
    g_lastLogRows.clear();
    if (events.empty()) return;
    const auto now = store::Clock::now();
    const int maxRows = 6;
    const int rows = (int)std::min<size_t>(maxRows, events.size());

    const REAL pw = 300, rowH = 16.0f, ph = rows * rowH + 24.0f;
    const REAL px = (REAL)(w - pw - 8), py = (REAL)(h - 28) - ph;

    SolidBrush bg(Color(170, 8, 12, 20));
    g.FillRectangle(&bg, px, py, pw, ph);
    Pen border(Color(90, 150, 170, 190), 1.0f);
    g.DrawRectangle(&border, px, py, pw, ph);
    SolidBrush hdr(Color(200, 180, 205, 225));
    g.DrawString(L"Warfare events", -1, &font, PointF(px + 8, py + 3), &hdr);

    for (int i = 0; i < rows; ++i) {
        const auto& e = events[events.size() - 1 - i];   // newest first
        const REAL ry = py + 20 + i * rowH;
        g_lastLogRows.push_back({ e.seq, px, ry, px + pw, ry + rowH });
        if (e.seq == g_selectedEventSeq) {
            SolidBrush hi(Color(60, 255, 230, 120));
            g.FillRectangle(&hi, px + 1, ry, pw - 2, rowH);
        }
        const double ageS = e.ageMs(now) / 1000.0;
        wchar_t line[128];
        if (e.kind == store::EventKind::Fire) {
            swprintf(line, 128, L"%4.1fs  FIRE  %u:%u:%u → %u:%u:%u",
                     ageS, e.firing.site, e.firing.application, e.firing.entity,
                     e.target.site, e.target.application, e.target.entity);
        } else {
            swprintf(line, 128, L"%4.1fs  DET   %u:%u:%u  %hs",
                     ageS, e.target.site, e.target.application, e.target.entity,
                     dis::detonationResultName(e.result));
        }
        SolidBrush txt(e.kind == store::EventKind::Fire ? fireColor(235)
                                                        : detColor(e.result, 235));
        g.DrawString(line, -1, &font, PointF(px + 8, ry), &txt);
    }
}

static void drawCoastline(Graphics& g) {
    SolidBrush land(Color(255, 33, 52, 60));
    Pen shore(Color(180, 90, 140, 150), 1.0f);
    for (const auto& poly : coast::world()) {
        std::vector<PointF> pts;
        pts.reserve(poly.size());
        for (const auto& p : poly) {
            double sx, sy;
            worldToScreen(p.lat, p.lon, sx, sy);
            pts.emplace_back((REAL)sx, (REAL)sy);
        }
        if (pts.size() >= 3) {
            g.FillPolygon(&land, pts.data(), (INT)pts.size());
            g.DrawPolygon(&shore, pts.data(), (INT)pts.size());
        }
    }
}

static void drawGraticule(Graphics& g, Font& font, int w, int h) {
    Pen grid(Color(50, 255, 255, 255));
    Pen equator(Color(110, 120, 200, 255), 1.5f);
    SolidBrush lblBrush(Color(150, 170, 190, 210));
    // meridians every 30 deg
    for (int lon = -180; lon <= 180; lon += 30) {
        double x0, y0, x1, y1;
        worldToScreen(90, lon, x0, y0);
        worldToScreen(-90, lon, x1, y1);
        g.DrawLine(&grid, (REAL)x0, (REAL)y0, (REAL)x1, (REAL)y1);
        if (x0 > 0 && x0 < w) {
            wchar_t b[16]; swprintf(b, 16, L"%d°", lon);
            g.DrawString(b, -1, &font, PointF((REAL)(x0 + 2), 26.0f), &lblBrush);
        }
    }
    // parallels every 30 deg
    for (int lat = -90; lat <= 90; lat += 30) {
        double x0, y0, x1, y1;
        worldToScreen(lat, -180, x0, y0);
        worldToScreen(lat, 180, x1, y1);
        g.DrawLine(lat == 0 ? &equator : &grid, (REAL)x0, (REAL)y0, (REAL)x1, (REAL)y1);
        if (y0 > 24 && y0 < h) {
            wchar_t b[16]; swprintf(b, 16, L"%d°", lat);
            g.DrawString(b, -1, &font, PointF(2.0f, (REAL)(y0 + 1)), &lblBrush);
        }
    }
}

static void render(HDC hdc, int w, int h) {
    // Double-buffer into a memory bitmap to avoid flicker.
    Bitmap backbuf(w, h, PixelFormat32bppPARGB);
    Graphics g(&backbuf);
    g.SetSmoothingMode(SmoothingModeAntiAlias);
    g.SetTextRenderingHint(TextRenderingHintClearTypeGridFit);

    g.Clear(Color(255, 12, 18, 28));   // dark ocean

    // Map background: image if present, else map extent box + graticule.
    double bx0, by0, bx1, by1;
    worldToScreen(90, -180, bx0, by0);
    worldToScreen(-90, 180, bx1, by1);
    if (g_worldImg) {
        g.DrawImage(g_worldImg, (REAL)bx0, (REAL)by0,
                    (REAL)(bx1 - bx0), (REAL)(by1 - by0));
    } else if (g_showCoast) {
        drawCoastline(g);
    }
    Pen frame(Color(150, 90, 110, 130));
    g.DrawRectangle(&frame, (REAL)bx0, (REAL)by0, (REAL)(bx1 - bx0), (REAL)(by1 - by0));

    FontFamily ff(L"Segoe UI");
    Font       font(&ff, 11, FontStyleRegular, UnitPixel);
    SolidBrush label(Color(235, 235, 235, 235));

    if (g_showGrid) drawGraticule(g, font, w, h);

    auto tracks = g_store.snapshot();

    // Warfare events go under the entity symbols so tracks stay readable. They
    // need the track table to resolve shot lines and detonations that carry no
    // world location of their own.
    std::vector<store::Engagement> events;
    if (g_showEvents) {
        events = g_events.snapshot();
        std::unordered_map<uint64_t, const store::Track*> byKey;
        byKey.reserve(tracks.size() * 2);
        for (const auto& t : tracks) byKey[t.id.key()] = &t;
        drawEvents(g, events, byKey, w, h);
    } else {
        g_lastEventPositions.clear();
        g_lastLogRows.clear();
    }

    // Entities.
    g_lastPositions.clear();
    const store::Track* selectedTrack = nullptr;
    for (const auto& t : tracks) {
        double sx, sy;
        worldToScreen(t.lla.lat, t.lla.lon, sx, sy);
        if (sx < -50 || sy < -50 || sx > w + 50 || sy > h + 50) continue;

        g_lastPositions.push_back({ t.id.key(), sx, sy });
        const bool selected = (t.id.key() == g_selectedKey);
        if (selected) selectedTrack = &t;

        Color c = forceColor(t.force);
        const bool munition = dis::isMunition(t.type);

        // Velocity cue, length scaled by speed (clamped). Platforms get a
        // heading vector ahead of the symbol; a munition gets a trail behind
        // instead, so its dart stays the clearest indication of where it is
        // headed rather than being speared by its own vector.
        if (t.groundSpeed > 0.5) {
            const double rad = t.heading * 0.017453292519943295;
            const double len = 12.0 + std::min(28.0, t.groundSpeed / 6.0);
            if (munition) {
                Pen trail(Color(150, c.GetR(), c.GetG(), c.GetB()), 1.5f);
                g.DrawLine(&trail, trackLocalPoint(sx, sy, rad, -4.0, 0.0),
                                   trackLocalPoint(sx, sy, rad, -len, 0.0));
            } else {
                Pen vec(c, 2.0f);
                g.DrawLine(&vec, PointF((REAL)sx, (REAL)sy),
                                 trackLocalPoint(sx, sy, rad, len, 0.0));
            }
        }

        if (selected) {
            Pen sel(Color(255, 255, 230, 120), 2.0f);
            g.DrawEllipse(&sel, (REAL)(sx - 10), (REAL)(sy - 10), 20.0f, 20.0f);
        }

        drawTrackSymbol(g, t, sx, sy, c);

        // Marking / callsign label.
        std::wstring txt;
        if (!t.marking.empty()) {
            txt.assign(t.marking.begin(), t.marking.end());
        } else {
            wchar_t b[64];
            swprintf(b, 64, L"%u:%u:%u", t.id.site, t.id.application, t.id.entity);
            txt = b;
        }
        wchar_t sub[64];
        swprintf(sub, 64, L"%.0f kt", t.groundSpeed * 1.94384);   // m/s -> knots
        PointF p1((REAL)(sx + 8), (REAL)(sy - 8));
        PointF p2((REAL)(sx + 8), (REAL)(sy + 5));
        g.DrawString(txt.c_str(), -1, &font, p1, &label);
        SolidBrush dim(Color(170, 200, 200, 210));
        g.DrawString(sub, -1, &font, p2, &dim);
    }

    // Detail panel for the selected track.
    if (selectedTrack) {
        const auto& t = *selectedTrack;
        const auto ageMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            store::Clock::now() - t.lastSeen).count();
        std::wstring mk(t.marking.begin(), t.marking.end());
        wchar_t lines[10][80];
        swprintf(lines[0], 80, L"%s", mk.empty() ? L"(no marking)" : mk.c_str());
        swprintf(lines[1], 80, L"ID  %u:%u:%u",
                 t.id.site, t.id.application, t.id.entity);
        swprintf(lines[2], 80, L"Force  %hs", dis::forceName(t.force));
        swprintf(lines[3], 80, L"Type  %hs  %u.%u.%u.%u.%u",
                 dis::entityKindName(t.type.kind),
                 t.type.kind, t.type.domain, t.type.category,
                 t.type.subcategory, t.type.specific);
        swprintf(lines[4], 80, L"Lat  %+.4f°", t.lla.lat);
        swprintf(lines[5], 80, L"Lon  %+.4f°", t.lla.lon);
        swprintf(lines[6], 80, L"Alt  %.0f m", t.lla.alt);
        swprintf(lines[7], 80, L"Spd  %.0f kt  (%.0f m/s)",
                 t.groundSpeed * 1.94384, t.groundSpeed);
        swprintf(lines[8], 80, L"Hdg  %.0f°", t.heading);
        swprintf(lines[9], 80, L"Updates %llu   age %lldms",
                 (unsigned long long)t.updates, (long long)ageMs);

        const REAL px = 8, py = 112, pw = 240, ph = 190;
        SolidBrush bg(Color(205, 10, 16, 24));
        g.FillRectangle(&bg, px, py, pw, ph);
        Pen border(forceColor(t.force), 1.5f);
        g.DrawRectangle(&border, px, py, pw, ph);
        FontFamily ffb(L"Segoe UI");
        Font title(&ffb, 14, FontStyleBold, UnitPixel);
        g.DrawString(lines[0], -1, &title, PointF(px + 10, py + 8), &label);
        for (int i = 1; i < 10; ++i)
            g.DrawString(lines[i], -1, &font,
                         PointF(px + 10, py + 8 + i * 17.0f), &label);
    }

    // Detail panel for the selected warfare event (shares the track panel's
    // slot — only one of the two can be selected at a time).
    const store::Engagement* selectedEvent = nullptr;
    for (const auto& e : events)
        if (e.seq == g_selectedEventSeq) { selectedEvent = &e; break; }
    if (g_selectedEventSeq && !selectedEvent)
        g_selectedEventSeq = 0;         // aged out of the store

    if (selectedEvent) {
        const auto& e = *selectedEvent;
        const bool isFire = (e.kind == store::EventKind::Fire);
        const auto& m = e.descriptor;
        wchar_t lines[13][80];
        swprintf(lines[0], 80, L"%s", isFire ? L"FIRE" : L"DETONATION");
        swprintf(lines[1], 80, L"Event  %u:%u:%u",
                 e.event.site, e.event.application, e.event.number);
        swprintf(lines[2], 80, L"Shooter  %u:%u:%u",
                 e.firing.site, e.firing.application, e.firing.entity);
        if (e.target.isNone())
            swprintf(lines[3], 80, L"Target  (none)");
        else
            swprintf(lines[3], 80, L"Target  %u:%u:%u",
                     e.target.site, e.target.application, e.target.entity);
        swprintf(lines[4], 80, L"Munition  %u.%u.%u.%u.%u",
                 m.munition.kind, m.munition.domain, m.munition.category,
                 m.munition.subcategory, m.munition.specific);
        swprintf(lines[5], 80, L"Warhead %u   Fuse %u", m.warhead, m.fuse);
        swprintf(lines[6], 80, L"Qty %u   Rate %u/min", m.quantity, m.rate);
        if (e.locValid) {
            swprintf(lines[7], 80, L"Lat  %+.4f°", e.lla.lat);
            swprintf(lines[8], 80, L"Lon  %+.4f°", e.lla.lon);
            swprintf(lines[9], 80, L"Alt  %.0f m", e.lla.alt);
        } else {
            swprintf(lines[7], 80, L"Lat  (target-relative)");
            swprintf(lines[8], 80, L"Lon  (target-relative)");
            swprintf(lines[9], 80, L"Alt  —");
        }
        if (isFire)
            swprintf(lines[10], 80, L"Range  %.1f km", e.range / 1000.0);
        else
            swprintf(lines[10], 80, L"Result  %hs", dis::detonationResultName(e.result));
        swprintf(lines[11], 80, L"Munition spd  %.0f m/s", e.speed);
        swprintf(lines[12], 80, L"Age  %.1f s",
                 e.ageMs(store::Clock::now()) / 1000.0);

        const REAL px = 8, py = 112, pw = 240, ph = 13 * 17.0f + 18;
        SolidBrush bg(Color(205, 10, 16, 24));
        g.FillRectangle(&bg, px, py, pw, ph);
        const Color accent = isFire ? fireColor(255) : detColor(e.result, 255);
        Pen border(accent, 1.5f);
        g.DrawRectangle(&border, px, py, pw, ph);
        FontFamily ffb(L"Segoe UI");
        Font title(&ffb, 14, FontStyleBold, UnitPixel);
        SolidBrush titleBrush(accent);
        g.DrawString(lines[0], -1, &title, PointF(px + 10, py + 8), &titleBrush);
        for (int i = 1; i < 13; ++i)
            g.DrawString(lines[i], -1, &font,
                         PointF(px + 10, py + 8 + i * 17.0f), &label);
    }

    if (g_showEvents) drawEventLog(g, font, events, w, h);

    // HUD.
    std::wstring status;
    { std::lock_guard<std::mutex> lk(g_statusMtx); status = g_statusLine; }
    wchar_t cur[48] = L"";
    if (g_cursorValid)
        swprintf(cur, 48, L"   cursor: %+.2f, %+.2f", g_cursorLat, g_cursorLon);
    wchar_t hud[384];
    swprintf(hud, 384, L"%s   |   tracks: %zu   fire: %llu   det: %llu   packets: %llu%s",
             status.c_str(), tracks.size(),
             (unsigned long long)g_events.fireCount(),
             (unsigned long long)g_events.detonationCount(),
             (unsigned long long)(g_receiver ? g_receiver->packetsReceived() : 0), cur);
    SolidBrush panelBg(Color(180, 0, 0, 0));
    g.FillRectangle(&panelBg, 0.0f, 0.0f, (REAL)w, 24.0f);
    SolidBrush hudBrush(Color(255, 210, 230, 255));
    g.DrawString(hud, -1, &font, PointF(8, 5), &hudBrush);

    SolidBrush legendBg(Color(150, 0, 0, 0));
    g.FillRectangle(&legendBg, (REAL)(w - 132), 28.0f, 124.0f, 150.0f);
    struct { const wchar_t* n; dis::ForceId f; } leg[] = {
        {L"Friendly", dis::ForceId::Friendly},
        {L"Opposing", dis::ForceId::Opposing},
        {L"Neutral",  dis::ForceId::Neutral},
        {L"Other",    dis::ForceId::Other},
    };
    for (int i = 0; i < 4; ++i) {
        SolidBrush b(forceColor(leg[i].f));
        REAL y = 34 + i * 18.0f;
        g.FillEllipse(&b, (REAL)(w - 124), y, 10.0f, 10.0f);
        g.DrawString(leg[i].n, -1, &font, PointF((REAL)(w - 108), y - 2), &label);
    }
    // Shape key: the dart marks a munition in flight (any force colour).
    {
        SolidBrush b(Color(235, 235, 235, 235));
        Pen outline(Color(200, 255, 255, 255), 1.0f);
        const double cx = w - 119, cy = 104;
        PointF dart[4] = {
            trackLocalPoint(cx, cy, 1.5707963267948966,  7.0,  0.0),
            trackLocalPoint(cx, cy, 1.5707963267948966, -4.0,  4.5),
            trackLocalPoint(cx, cy, 1.5707963267948966, -1.5,  0.0),
            trackLocalPoint(cx, cy, 1.5707963267948966, -4.0, -4.5),
        };
        g.FillPolygon(&b, dart, 4);
        g.DrawPolygon(&outline, dart, 4);
        g.DrawString(L"Munition", -1, &font, PointF((REAL)(w - 108), 97.0f), &label);
    }

    // Warfare event key, greyed out while the overlay is hidden.
    struct { const wchar_t* n; Color c; } evLeg[] = {
        {L"Fire",    fireColor(255)},
        {L"Det hit", detColor(1, 255)},   // 1 = Entity Impact
        {L"Det miss",detColor(3, 255)},   // 3 = Ground Impact
        {L"Dud",     detColor(6, 255)},   // 6 = None / Dud
    };
    for (int i = 0; i < 4; ++i) {
        const Color c = g_showEvents ? evLeg[i].c : Color(70, 150, 150, 150);
        SolidBrush b(c);
        REAL y = 122 + i * 14.0f;
        g.FillEllipse(&b, (REAL)(w - 124), y, 8.0f, 8.0f);
        SolidBrush t(g_showEvents ? Color(235, 235, 235, 235) : Color(110, 190, 190, 190));
        g.DrawString(evLeg[i].n, -1, &font, PointF((REAL)(w - 110), y - 4), &t);
    }

    // Key hint line.
    SolidBrush hint(Color(130, 170, 190, 205));
    g.DrawString(L"S: connection   G: grid   C: coastline   E: events   R: reset   wheel: zoom   click: select",
                 -1, &font, PointF(8, (REAL)(h - 20)), &hint);

    // Blit.
    Graphics screen(hdc);
    screen.DrawImage(&backbuf, 0, 0, w, h);
}

// ---------------------------------------------------------------------------
// Connection settings dialog (opened with 'S')
// ---------------------------------------------------------------------------
static std::string wideToNarrow(const std::wstring& w) {
    std::string s; for (wchar_t c : w) s += char(c); return s;
}

static void applySettings(HWND hwnd) {
    wchar_t a[64] = {0}, p[16] = {0}, ifc[64] = {0};
    GetWindowTextW(GetDlgItem(hwnd, kIdEditAddr),  a, 63);
    GetWindowTextW(GetDlgItem(hwnd, kIdEditPort),  p, 15);
    GetWindowTextW(GetDlgItem(hwnd, kIdEditIface), ifc, 63);
    std::string addr  = wideToNarrow(a);
    std::string iface = wideToNarrow(ifc);
    if (iface.empty()) iface = "0.0.0.0";   // blank => any interface
    int portNum = _wtoi(p);

    in_addr probe{};
    if (inet_pton(AF_INET, addr.c_str(), &probe) != 1) {
        MessageBoxW(hwnd, L"Enter a valid IPv4 address, e.g. 239.1.2.3.",
                    L"Connection settings", MB_OK | MB_ICONWARNING);
        return;
    }
    if (portNum < 1 || portNum > 65535) {
        MessageBoxW(hwnd, L"Port must be between 1 and 65535.",
                    L"Connection settings", MB_OK | MB_ICONWARNING);
        return;
    }
    if (inet_pton(AF_INET, iface.c_str(), &probe) != 1) {
        MessageBoxW(hwnd, L"Local interface must be a valid IPv4 address, "
                    L"or 0.0.0.0 for any interface.",
                    L"Connection settings", MB_OK | MB_ICONWARNING);
        return;
    }
    // Soft warning if outside the IPv4 multicast range (224.0.0.0-239.255.255.255).
    const unsigned firstOctet = (unsigned char)addr[0] ? (unsigned)std::atoi(addr.c_str()) : 0;
    if (firstOctet < 224 || firstOctet > 239) {
        if (MessageBoxW(hwnd,
                L"That address is not in the multicast range (224-239.x.x.x).\n"
                L"Joining the group will likely fail. Use it anyway?",
                L"Connection settings", MB_YESNO | MB_ICONQUESTION) != IDYES)
            return;
    }

    g_group = addr;
    g_port  = (uint16_t)portNum;
    g_iface = iface;
    if (g_receiver) g_receiver->restart(g_group, g_port, g_iface);
    saveConfig();                 // remember the last-used connection
    g_selectedKey = 0;
    DestroyWindow(hwnd);
}

static LRESULT CALLBACK SettingsProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
        case WM_CREATE: {
            HFONT f = (HFONT)GetStockObject(DEFAULT_GUI_FONT);
            auto mk = [&](const wchar_t* cls, const wchar_t* txt, DWORD style,
                          int x, int y, int w, int h, int id) {
                HWND c = CreateWindowExW(0, cls, txt, WS_CHILD | WS_VISIBLE | style,
                                         x, y, w, h, hwnd, (HMENU)(INT_PTR)id,
                                         g_hInst, nullptr);
                SendMessageW(c, WM_SETFONT, (WPARAM)f, TRUE);
                return c;
            };
            mk(L"STATIC", L"Multicast group:", 0, 14, 18, 110, 20, 0);
            g_hEditAddr = mk(L"EDIT", L"", WS_BORDER | ES_AUTOHSCROLL,
                             128, 15, 140, 22, kIdEditAddr);
            mk(L"STATIC", L"Port:", 0, 14, 50, 110, 20, 0);
            g_hEditPort = mk(L"EDIT", L"", WS_BORDER | ES_AUTOHSCROLL | ES_NUMBER,
                             128, 47, 80, 22, kIdEditPort);
            mk(L"STATIC", L"Local interface:", 0, 14, 82, 110, 20, 0);
            g_hEditIface = mk(L"EDIT", L"", WS_BORDER | ES_AUTOHSCROLL,
                              128, 79, 140, 22, kIdEditIface);
            mk(L"STATIC", L"(0.0.0.0 = any NIC)", 0, 14, 108, 254, 18, 0);
            mk(L"BUTTON", L"Connect", BS_DEFPUSHBUTTON, 54, 132, 90, 28, IDOK);
            mk(L"BUTTON", L"Cancel",  0,                154, 132, 90, 28, IDCANCEL);

            // Seed fields with the current connection.
            std::wstring wg(g_group.begin(), g_group.end());
            SetWindowTextW(g_hEditAddr, wg.c_str());
            wchar_t pb[16]; swprintf(pb, 16, L"%u", g_port);
            SetWindowTextW(g_hEditPort, pb);
            std::wstring wi(g_iface.begin(), g_iface.end());
            SetWindowTextW(g_hEditIface, wi.c_str());
            return 0;
        }

        case WM_COMMAND:
            if (LOWORD(wp) == IDOK)          applySettings(hwnd);
            else if (LOWORD(wp) == IDCANCEL) DestroyWindow(hwnd);
            return 0;

        case WM_CLOSE:
            DestroyWindow(hwnd);
            return 0;

        case WM_DESTROY:
            EnableWindow(g_mainHwnd, TRUE);
            SetForegroundWindow(g_mainHwnd);
            g_settingsHwnd = nullptr;
            g_hEditAddr = g_hEditPort = g_hEditIface = nullptr;
            return 0;
    }
    return DefWindowProc(hwnd, msg, wp, lp);
}

static void openSettings() {
    if (g_settingsHwnd) { SetForegroundWindow(g_settingsHwnd); return; }
    RECT pr; GetWindowRect(g_mainHwnd, &pr);
    const int w = 300, h = 210;
    const int x = pr.left + ((pr.right - pr.left) - w) / 2;
    const int y = pr.top + ((pr.bottom - pr.top) - h) / 2;
    g_settingsHwnd = CreateWindowExW(
        WS_EX_DLGMODALFRAME, L"DisMapSettings", L"Connection settings",
        WS_POPUP | WS_CAPTION | WS_SYSMENU, x, y, w, h,
        g_mainHwnd, nullptr, g_hInst, nullptr);
    if (!g_settingsHwnd) return;
    EnableWindow(g_mainHwnd, FALSE);   // simple modal behaviour
    ShowWindow(g_settingsHwnd, SW_SHOW);
    SetFocus(g_hEditAddr);
}

// ---------------------------------------------------------------------------
// Window procedure
// ---------------------------------------------------------------------------
static LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
        case WM_CREATE:
            SetTimer(hwnd, 1, 33, nullptr);   // ~30 fps repaint
            return 0;

        case WM_TIMER:
            InvalidateRect(hwnd, nullptr, FALSE);
            return 0;

        case WM_SIZE:
            if (g_view.scale <= 0)
                fitView(LOWORD(lp), HIWORD(lp));
            return 0;

        case WM_ERASEBKGND:
            return 1;   // we paint the whole client area

        case WM_PAINT: {
            PAINTSTRUCT ps;
            HDC hdc = BeginPaint(hwnd, &ps);
            RECT rc; GetClientRect(hwnd, &rc);
            if (g_view.scale <= 0) fitView(rc.right, rc.bottom);
            render(hdc, rc.right, rc.bottom);
            EndPaint(hwnd, &ps);
            return 0;
        }

        case WM_LBUTTONDOWN: {
            const int mx = GET_X_LPARAM(lp), my = GET_Y_LPARAM(lp);
            // A click in the event log selects that engagement and never pans.
            for (const auto& r : g_lastLogRows) {
                if (mx >= r.x0 && mx <= r.x1 && my >= r.y0 && my <= r.y1) {
                    g_selectedEventSeq = (g_selectedEventSeq == r.seq) ? 0 : r.seq;
                    g_selectedKey = 0;
                    return 0;
                }
            }
            // Hit-test tracks first (within 9 px), then warfare events (12 px);
            // tracks win ties because their symbols are smaller. No hit => pan.
            double best = 9.0 * 9.0; uint64_t hit = 0;
            for (const auto& e : g_lastPositions) {
                const double dx = e.sx - mx, dy = e.sy - my;
                const double d2 = dx * dx + dy * dy;
                if (d2 <= best) { best = d2; hit = e.key; }
            }
            uint64_t evHit = 0;
            if (!hit) {
                double evBest = 12.0 * 12.0;
                for (const auto& e : g_lastEventPositions) {
                    const double dx = e.sx - mx, dy = e.sy - my;
                    const double d2 = dx * dx + dy * dy;
                    if (d2 <= evBest) { evBest = d2; evHit = e.key; }
                }
            }
            if (hit) {
                g_selectedKey = hit;        // selecting a track clears any event
                g_selectedEventSeq = 0;
            } else if (evHit) {
                g_selectedEventSeq = evHit;
                g_selectedKey = 0;
            } else {
                g_selectedKey = 0;     // click empty space clears selection
                g_selectedEventSeq = 0;
                g_dragging = true;
                g_dragStart.x = mx; g_dragStart.y = my;
                g_dragOffX = g_view.offX;
                g_dragOffY = g_view.offY;
                SetCapture(hwnd);
            }
            return 0;
        }

        case WM_MOUSEMOVE: {
            const int mx = GET_X_LPARAM(lp), my = GET_Y_LPARAM(lp);
            if (g_dragging) {
                g_view.offX = g_dragOffX + (mx - g_dragStart.x);
                g_view.offY = g_dragOffY + (my - g_dragStart.y);
            }
            if (g_view.scale > 0) {
                screenToWorld(mx, my, g_cursorLat, g_cursorLon);
                g_cursorValid = (g_cursorLat >= -90 && g_cursorLat <= 90 &&
                                 g_cursorLon >= -180 && g_cursorLon <= 180);
            }
            return 0;
        }

        case WM_LBUTTONUP:
            g_dragging = false;
            ReleaseCapture();
            return 0;

        case WM_MOUSEWHEEL: {
            POINT pt{ GET_X_LPARAM(lp), GET_Y_LPARAM(lp) };
            ScreenToClient(hwnd, &pt);
            const double factor = (GET_WHEEL_DELTA_WPARAM(wp) > 0) ? 1.15 : 1.0 / 1.15;
            // Zoom about the cursor: keep the world point under the cursor fixed.
            const double newScale = g_view.scale * factor;
            g_view.offX = pt.x - (pt.x - g_view.offX) * (newScale / g_view.scale);
            g_view.offY = pt.y - (pt.y - g_view.offY) * (newScale / g_view.scale);
            g_view.scale = newScale;
            return 0;
        }

        case WM_KEYDOWN:
            if (wp == VK_ESCAPE) { DestroyWindow(hwnd); }
            else if (wp == 'R') {
                RECT rc; GetClientRect(hwnd, &rc);
                fitView(rc.right, rc.bottom);
            } else if (wp == 'G') {
                g_showGrid = !g_showGrid;
            } else if (wp == 'C') {
                g_showCoast = !g_showCoast;
            } else if (wp == 'E') {
                g_showEvents = !g_showEvents;
            } else if (wp == 'S') {
                openSettings();
            }
            return 0;

        case WM_DESTROY:
            KillTimer(hwnd, 1);
            PostQuitMessage(0);
            return 0;
    }
    return DefWindowProc(hwnd, msg, wp, lp);
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
static void loadWorldImage() {
    wchar_t path[MAX_PATH];
    GetModuleFileNameW(nullptr, path, MAX_PATH);
    std::wstring p(path);
    size_t slash = p.find_last_of(L"\\/");
    std::wstring dir = (slash == std::wstring::npos) ? L"" : p.substr(0, slash + 1);
    std::wstring img = dir + L"world.png";
    Image* i = Image::FromFile(img.c_str());
    if (i && i->GetLastStatus() == Ok) g_worldImg = i;
    else delete i;
}

// Config file lives in %APPDATA%\dis-map\config.ini (always user-writable).
static std::wstring configPath() {
    wchar_t appdata[MAX_PATH];
    DWORD n = GetEnvironmentVariableW(L"APPDATA", appdata, MAX_PATH);
    std::wstring dir = (n > 0 && n < MAX_PATH) ? std::wstring(appdata) : L".";
    dir += L"\\dis-map";
    CreateDirectoryW(dir.c_str(), nullptr);   // no-op if it already exists
    return dir + L"\\config.ini";
}

// Load persisted connection into the globals (overrides built-in defaults).
static void loadConfig() {
    std::ifstream f(configPath());
    if (!f) return;
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = line.substr(0, eq);
        const std::string val = line.substr(eq + 1);
        if (key == "group" && !val.empty()) {
            g_group = val;
        } else if (key == "port") {
            const int p = std::atoi(val.c_str());
            if (p >= 1 && p <= 65535) g_port = (uint16_t)p;
        } else if (key == "iface" && !val.empty()) {
            g_iface = val;
        }
    }
}

// Persist the current connection so it is restored on next launch.
static void saveConfig() {
    std::ofstream f(configPath(), std::ios::trunc);
    if (!f) return;
    f << "group=" << g_group << "\n";
    f << "port="  << g_port  << "\n";
    f << "iface=" << g_iface << "\n";
}

// Parse command-line overrides: --group=, --port=, --iface=.
static void parseArgs(std::string& group, uint16_t& port, std::string& iface) {
    int argc = 0;
    LPWSTR* argv = CommandLineToArgvW(GetCommandLineW(), &argc);
    for (int i = 1; i < argc; ++i) {
        std::wstring a = argv[i];
        auto toA = [](const std::wstring& w) {
            std::string s; for (wchar_t c : w) s += char(c); return s;
        };
        if (a.rfind(L"--group=", 0) == 0)      group = toA(a.substr(8));
        else if (a.rfind(L"--port=", 0) == 0)  port = (uint16_t)_wtoi(a.substr(7).c_str());
        else if (a.rfind(L"--iface=", 0) == 0) iface = toA(a.substr(8));
    }
    if (argv) LocalFree(argv);
}

int WINAPI wWinMain(HINSTANCE hInst, HINSTANCE, LPWSTR, int nShow) {
    GdiplusStartupInput gsi;
    GdiplusStartup(&g_gdiToken, &gsi, nullptr);
    loadWorldImage();
    g_hInst = hInst;

    loadConfig();                          // persisted last-used connection (if any)
    parseArgs(g_group, g_port, g_iface);   // command-line still overrides the config

    net::MulticastReceiver receiver(g_store, g_events, g_group, g_port, g_iface);
    receiver.onStatus = [](const std::string& s) {
        std::wstring w(s.begin(), s.end());
        setStatus(w);
    };
    g_receiver = &receiver;
    if (!receiver.start())
        setStatus(L"Receiver failed to start (see console).");
    saveConfig();                  // record the connection we started with

    WNDCLASSW wc{};
    wc.lpfnWndProc   = WndProc;
    wc.hInstance     = hInst;
    wc.hCursor       = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = nullptr;
    wc.lpszClassName = L"DisMapWindow";
    RegisterClassW(&wc);

    WNDCLASSW sc{};
    sc.lpfnWndProc   = SettingsProc;
    sc.hInstance     = hInst;
    sc.hCursor       = LoadCursor(nullptr, IDC_ARROW);
    sc.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
    sc.lpszClassName = L"DisMapSettings";
    RegisterClassW(&sc);

    HWND hwnd = CreateWindowExW(
        0, wc.lpszClassName, L"DIS Map — Entity State / Fire / Detonation (multicast)",
        WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, 1100, 640,
        nullptr, nullptr, hInst, nullptr);
    g_mainHwnd = hwnd;

    ShowWindow(hwnd, nShow);
    UpdateWindow(hwnd);

    MSG msg;
    while (GetMessage(&msg, nullptr, 0, 0)) {
        // Route dialog navigation (Tab/Enter/Esc) to the settings window.
        if (g_settingsHwnd && IsDialogMessage(g_settingsHwnd, &msg)) continue;
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }

    receiver.stop();
    g_receiver = nullptr;
    delete g_worldImg;
    GdiplusShutdown(g_gdiToken);
    return (int)msg.wParam;
}
