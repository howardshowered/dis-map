# DIS Map

A Windows desktop app (C++ / Win32 / GDI+) that listens for **DIS v7
(IEEE 1278.1-2012) Entity State PDUs** on a **UDP multicast** group and plots
each entity on a world map in realtime.

![screenshot](build/smoke.png)

## Features

- Joins a UDP multicast group and parses Entity State PDUs (big-endian) on a
  background thread.
- Converts DIS geocentric **ECEF (WGS84)** entity locations to geodetic
  lat/lon and projects them onto an equirectangular map.
- Realtime rendering (~30 fps), double-buffered, with per-track marking,
  ground speed, and **force-based colouring** (Friendly / Opposing / Neutral /
  Other).
- **Ground-track heading vectors** derived from each entity's ECEF velocity
  (rotated into the local East/North frame), length scaled by speed.
- **Click a track to select it** — a detail panel shows Entity ID, force,
  entity type, lat/lon/alt, ground speed, heading, update count and data age.
- Built-in **schematic coastline** reference layer, plus a labelled graticule
  and a live **cursor lat/lon readout**.
- **In-app connection settings** (`S`): change the multicast group and port at
  runtime and rejoin live, without restarting the app.
- Stale tracks (no update for 30 s) are dropped automatically.
- Optional `world.png` background (equirectangular, full -180..180 / -90..90);
  when present it replaces the built-in coastline.
- Includes a `dis_sender` test tool that broadcasts moving entities.

## Controls

| Input | Action |
|-------|--------|
| Left-drag        | Pan |
| Mouse wheel      | Zoom about the cursor |
| Click a track    | Select / show detail panel (click empty space to clear) |
| `S`              | Connection settings — change multicast group / port and reconnect |
| `R`              | Reset view to fit |
| `G`              | Toggle graticule |
| `C`              | Toggle coastline layer |
| `Esc`            | Quit |

## Build

Requires Visual Studio 2022 (MSVC) and CMake.

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

Binaries land in `build/bin/Release/`.

## Run

```powershell
# Terminal 1 — the map
build\bin\Release\dis_map.exe                       # defaults: 239.1.2.3:3000
build\bin\Release\dis_map.exe --group=239.1.2.3 --port=3000

# Terminal 2 — the test source (moving entities)
build\bin\Release\dis_sender.exe --rate=10
```

Drop an equirectangular `world.png` next to `dis_map.exe` for a real map
background; otherwise a graticule is drawn.

## Layout

| File | Purpose |
|------|---------|
| `src/byteorder.h`    | Big-endian read/write helpers (DIS is network byte order). |
| `src/dis.h`          | DIS v7 constants, field offsets, Entity State PDU parser. |
| `src/geodetic.h`     | ECEF ⇄ geodetic (WGS84) conversion. |
| `src/entity_store.h` | Thread-safe latest-state track table (incl. heading/ground speed). |
| `src/coastline.h`    | Coarse embedded world coastline for the reference layer. |
| `src/receiver.h`     | Winsock multicast receiver thread. |
| `src/main.cpp`       | Win32 window, GDI+ map rendering, input. |
| `src/test_sender.cpp`| Multicast test transmitter. |

## Notes / limitations

- Only **Entity State PDUs** (PDU type 1) are decoded; other PDU families are
  counted but ignored.
- The parser reads through the fixed portion of the PDU (144 bytes); trailing
  articulation parameters are not decoded.
- Multicast on a single host relies on default loopback being enabled (it is on
  Windows). Across hosts, ensure the sender's `IP_MULTICAST_TTL` and any router
  IGMP config allow the group through. To bind a specific NIC on a multi-homed
  host, extend `MulticastReceiver`'s `iface` argument.
- The equirectangular projection is intentionally simple; entity heading and
  altitude are received but not drawn as vectors.
