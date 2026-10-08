# DIS Map

A Windows desktop app (C++ / Win32 / GDI+) that listens for **DIS v7
(IEEE 1278.1-2012)** PDUs on a **UDP multicast** group and plots them on a world
map in realtime: **Entity State** tracks plus **Fire** and **Detonation**
warfare events.

![screenshot](build/smoke.png)

## Features

- Joins a UDP multicast group and parses Entity State (type 1), Fire (type 2)
  and Detonation (type 3) PDUs (big-endian) on a background thread.
- Converts DIS geocentric **ECEF (WGS84)** entity locations to geodetic
  lat/lon and projects them onto an equirectangular map.
- Realtime rendering (~30 fps), double-buffered, with per-track marking,
  ground speed, and **force-based colouring** (Friendly / Opposing / Neutral /
  Other).
- **Ground-track heading vectors** derived from each entity's ECEF velocity
  (rotated into the local East/North frame), length scaled by speed.
- **Click a track to select it** — a detail panel shows Entity ID, force,
  entity type, lat/lon/alt, ground speed, heading, update count and data age.
- **Warfare events** (`E` to toggle): a Fire PDU draws an amber muzzle flash at
  the launch point plus a dashed shot line with a travelling tracer — aimed at
  the target entity's current position, or along the munition's heading for the
  PDU's reported range when the target is unknown. A Detonation PDU draws
  expanding shockwave rings, coloured by result: **red** for a hit, **orange**
  for a near miss / ground impact, **grey X** for a dud. Markers animate for a
  few seconds, then linger faintly until they age out (20 s).
- A **warfare event log** (bottom right) lists the most recent engagements.
  **Click a row** to open a detail panel with the Event ID, shooter, target,
  munition type, warhead / fuse / quantity / rate, impact position, and either
  the reported range (Fire) or the detonation result (Detonation). The log is
  the reliable way to pick an event, since event markers usually sit directly on
  top of the shooter's or target's symbol.
- Fire and Detonation PDUs of one engagement share an **Event ID**, so a shot
  and its impact can be matched up in the log.
- Detonations that carry no world location (entity-relative impacts) are
  anchored to the target entity's last known position.
- Built-in **schematic coastline** reference layer, plus a labelled graticule
  and a live **cursor lat/lon readout**.
- **In-app connection settings** (`S`): change the multicast group, port, and
  **local interface** at runtime and rejoin live, without restarting the app.
  These are **persisted** to `%APPDATA%\dis-map\config.ini` and restored on next
  launch (command-line `--group` / `--port` / `--iface` still override the saved
  values). The local interface selects which NIC joins the group on multi-homed
  hosts (`0.0.0.0` = let the OS choose). Note: for a multicast receiver the local
  *port* is necessarily the group's port, so it is not separately configurable.
- Stale tracks (no update for 30 s) are dropped automatically.
- Optional `world.png` background (equirectangular, full -180..180 / -90..90);
  when present it replaces the built-in coastline.
- Includes a `dis_sender` test tool that broadcasts moving entities and a
  repeating script of engagements (hit, near miss, ground impact, dud).

## Controls

| Input | Action |
|-------|--------|
| Left-drag        | Pan |
| Mouse wheel      | Zoom about the cursor |
| Click a track    | Select / show detail panel (click empty space to clear) |
| Click an event / log row | Select a Fire or Detonation event and show its detail panel |
| `S`              | Connection settings — change multicast group / port / local interface and reconnect |
| `R`              | Reset view to fit |
| `G`              | Toggle graticule |
| `C`              | Toggle coastline layer |
| `E`              | Toggle warfare event overlay and log |
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
build\bin\Release\dis_map.exe --iface=192.168.1.50  # bind the join to a NIC

# Terminal 2 — the test source (moving entities + scripted engagements)
build\bin\Release\dis_sender.exe --rate=10
build\bin\Release\dis_sender.exe --warfare=0   # entities only, no Fire/Detonation
```

The sender fires a scripted shot every 4 s, cycling through an entity impact, a
dud, a ground impact short of the target, and a proximity near miss, and prints
each `FIRE` / `DET` to the console.

Drop an equirectangular `world.png` next to `dis_map.exe` for a real map
background; otherwise a graticule is drawn.

## Layout

| File | Purpose |
|------|---------|
| `src/byteorder.h`    | Big-endian read/write helpers (DIS is network byte order). |
| `src/dis.h`          | DIS v7 constants, per-type field offsets, Entity State / Fire / Detonation parsers and record writers. |
| `src/geodetic.h`     | ECEF ⇄ geodetic (WGS84) conversion. |
| `src/entity_store.h` | Thread-safe latest-state track table (incl. heading/ground speed). |
| `src/event_store.h`  | Thread-safe ring of recent Fire / Detonation events. |
| `src/coastline.h`    | Coarse embedded world coastline for the reference layer. |
| `src/receiver.h`     | Winsock multicast receiver thread. |
| `src/main.cpp`       | Win32 window, GDI+ map rendering, input. |
| `src/test_sender.cpp`| Multicast test transmitter. |

## Notes / limitations

- Only **Entity State** (type 1), **Fire** (type 2) and **Detonation** (type 3)
  PDUs are decoded; other PDU families are counted but ignored.
- Parsers read the fixed portion of each PDU only (144 / 96 / 104 bytes).
  Trailing articulation parameters on Entity State and variable parameter
  records on Detonation are not decoded.
- Warhead and fuse are shown as raw enumeration values rather than names.
- Multicast on a single host relies on default loopback being enabled (it is on
  Windows). Across hosts, ensure the sender's `IP_MULTICAST_TTL` and any router
  IGMP config allow the group through. To bind a specific NIC on a multi-homed
  host, extend `MulticastReceiver`'s `iface` argument.
- The equirectangular projection is intentionally simple; entity heading and
  altitude are received but not drawn as vectors.
