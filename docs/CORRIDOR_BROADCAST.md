# Corridor-Scoped Flood Delivery

This document explains the geo-corridor feature for MeshCore flood packets and shows
how to run the bundled L-shape corridor test in the simulator.

## Concept

A **corridor** is a chain of overlapping circles (waypoints) laid along a geographic
path. When a companion sends a `ROUTE_TYPE_TRANSPORT_FLOOD` channel message with a
corridor attached, every corridor-aware repeater checks its own GPS position against
that chain before forwarding:

- **Inside** the corridor → forward as normal
- **Outside** the corridor → drop the packet silently

Repeaters that do not understand corridor headers forward unconditionally, preserving
full backward compatibility (soft-hint design).

### Corridor geometry

Each waypoint is encoded as a **`lat, lon, radius_km`** triple. The corridor is the
union of N circles plus the capsule-shaped convex hull between consecutive circles.
A cos(latitude) correction is applied so that longitude distances are accurate at all
latitudes.

**Fail-open rules:**

| Condition | Behaviour |
|-----------|-----------|
| N = 0 (no corridor in packet) | Always forward |
| Repeater position unknown (lat=0, lon=0) | Always forward |
| Radius code = 15 (unlimited) | Always inside for that waypoint |
| Packet shorter than expected corridor bytes | Drop (malformed) |

### Wire encoding

Corridor data is appended to the packet payload (N × 4 bytes). The count N (0–8) is
stored in bits 15–12 of `transport_codes[1]`. Each 4-byte triple encodes:

| Bits | Field | Encoding |
|------|-------|----------|
| 31–18 | lat | 14-bit offset-binary: `(lat + 90) / 180 × 16383` |
| 17–4 | lon | 14-bit offset-binary: `(lon + 180) / 360 × 16383` |
| 3–0 | radius_code | 4-bit index into `CORRIDOR_RADIUS_KM[]` (code 15 = unlimited) |

Maximum corridor triples per packet: **8** (max overhead: 32 bytes).

## Simulator support

### Topology file: `agent/channel/corridor`

The `corridor` property on a node's channel agent accepts a list of waypoint strings
in `"lat,lon,radius_km"` format.  When the list is non-empty the simulator uses
`CMD_SEND_CHANNEL_TXT_MSG_CORRIDOR` instead of the plain channel-message command.

```yaml
nodes:
  - name: "Alice"
    agent:
      channel:
        enabled: true
        message_count: 1
        corridor:
          - "47.6100,-122.4000,3"   # WP1
          - "47.6100,-122.3500,3"   # WP2
          - "47.6600,-122.3500,3"   # WP3
```

### Repeater forwarding check

Simulated repeater firmware calls `allowPacketForward()` for every received flood
packet.  The corridor check block extracts the corridor triples from the end of the
payload and calls `isPointInCorridor()` with the repeater's configured lat/lon. A
repeater with an unknown position (default `0.0, 0.0`) always forwards.

## L-shape corridor test

The example files below exercise the full corridor forwarding path using a Seattle
L-shaped route.

### Files

| File | Role |
|------|------|
| [examples/topologies/corridor_l_shape.yaml](https://github.com/usrflo/mcsim/blob/corridor-approach/examples/topologies/corridor_l_shape.yaml) | Network topology — 2 companions, 4 repeaters, explicit radio edges |
| [examples/behaviors/corridor_broadcast.yaml](https://github.com/usrflo/mcsim/blob/corridor-approach/examples/behaviors/corridor_broadcast.yaml) | Behavior overlay — Alice sends one corridor-scoped channel message |

### Network layout

```
WP1 (47.6100, -122.4000) ──── WP2 (47.6100, -122.3500)
  Alice (sender)                 │
  R_out_SW  (outside, SW)        │   ← vertical leg
  R_out_N   (outside, N)         │
  R_in_H    (inside, H midpoint) WP3 (47.6600, -122.3500)
                                   Bob (receiver)
                                   R_in_V (inside, V midpoint)
```

The corridor has three waypoints, each with a 3 km radius:

| Waypoint | Lat | Lon | Role |
|----------|-----|-----|------|
| WP1 | 47.6100 | -122.4000 | West end (Alice's position) |
| WP2 | 47.6100 | -122.3500 | 90° corner |
| WP3 | 47.6600 | -122.3500 | North end (Bob's position) |

**Expected forwarding behaviour:**

| Node | Position | Forwards? |
|------|----------|-----------|
| R_in_H | midpoint of horizontal leg | Yes |
| R_in_V | midpoint of vertical leg | Yes |
| R_out_SW | ≈5 km SW of WP1 | **No** |
| R_out_N | ≈3.9 km north of horizontal leg | **No** |

Bob receives the message via `R_in_V` (and optionally `R_in_H`).

### Running the test

```bash
cargo run -- run \
  examples/topologies/corridor_l_shape.yaml \
  examples/behaviors/corridor_broadcast.yaml \
  --duration 30s
```

A shorter run (30 s) is sufficient because Alice sends only one message.

### What to look for in the output

- Alice sends one channel message tagged with the 3-waypoint corridor.
- `R_in_H` and `R_in_V` each log a forwarding decision and re-broadcast.
- `R_out_SW` and `R_out_N` log **"position outside corridor, dropping flood"** and
  do **not** re-transmit.
- Bob receives the message (delivery event logged for Bob).

### Enabling debug output

```bash
RUST_LOG=debug cargo run -- run \
  examples/topologies/corridor_l_shape.yaml \
  examples/behaviors/corridor_broadcast.yaml \
  --duration 30s 2>&1 | grep -E "corridor|forward|deliver"
```

## Further reading

- [MeshCore issue #1451 – Proposal: Limit Floods by a geo-coord-defined Corridor Approach](https://github.com/meshcore-dev/MeshCore/issues/1451)
- [plan_corridor_approach.md](plan_corridor_approach.md) — full design document
- [src/helpers/CorridorCheck.h](../src/helpers/CorridorCheck.h) — header-only implementation
