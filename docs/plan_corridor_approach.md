# Corridor Approach for Flood Packet Routing

> **GitHub Issue:** [#1451 – Proposal: Limit Floods by a geo-coord-defined Corridor Approach](https://github.com/meshcore-dev/MeshCore/issues/1451)

## Overview

This document describes the design and implementation plan for embedding corridor geo-data into `ROUTE_TYPE_TRANSPORT_FLOOD` packets, allowing corridor-aware repeaters to selectively forward flood packets based on their own geographic position.

**Core principle:** Corridor-unaware repeaters continue forwarding as today (soft-hint, backward-compatible). Corridor-aware repeaters skip forwarding if their position is outside the defined corridor.

## Design Decisions

| Decision | Value |
|----------|-------|
| Applicable route type | `ROUTE_TYPE_TRANSPORT_FLOOD` only |
| Max corridor triples | 8 (N ≤ 8) |
| Corridor overhead | N × 4 bytes (max 32 bytes) at end of payload |
| Min remaining payload | `MAX_PACKET_PAYLOAD` − 32 = 152 bytes |
| Enforcement | Soft-hint only; old repeaters forward unconditionally |
| Fail-open | Own position unknown (`node_lat == 0.0 && node_lon == 0.0`) → always forward |
| Position source | `_prefs.node_lat`, `_prefs.node_lon` (`double`, already persisted) |
| New helper | Header-only `CorridorCheck.h`, no new `.cpp`, no build system changes |

## Packet Layout

The byte order of the on-wire packet is unchanged for all fields before and including `path`. Corridor triples are **appended after** the typed payload content so that old firmware's path-length parsing at a fixed offset is never disturbed.

```
Byte offset (TRANSPORT_FLOOD with N > 0):

 0        header          [1B]  route=00, payload_type, version (unchanged)
 1–2      transport_codes[0]   [2B]  region scope (unchanged)
 3–4      transport_codes[1]   [2B]  NEW: bits 15-12 = N (0..8), bits 11-0 = 0 (reserved)
 5        path_length          [1B]  UNCHANGED POSITION — critical for backward compat
 6..      path                 [var] hop_count × hash_size bytes
 ..       payload_content      [M B] typed payload as usual
 ..       corridor_triples     [N×4B] appended at end of payload area
```

### transport_codes[1] Bit Layout

| Bits  | Field | Notes |
|-------|-------|-------|
| 15–12 | N     | Number of corridor triples, 0..8; values 9–15 reserved |
| 11–0  | —     | Reserved, must be 0 |

## Corridor Triple Encoding (4 bytes)

```
Bit 31      Bit 0
├──────────────────────────────────┤
│ lat_encoded[13:0] │ lon_encoded[13:0] │ radius_code[3:0] │
│    bits 31–18     │    bits 17–4      │    bits 3–0      │
```

```c
lat_encoded = (uint16_t) roundf((lat  +  90.0f) / 180.0f * 16383.0f);  // 14-bit
lon_encoded = (uint16_t) roundf((lon  + 180.0f) / 360.0f * 16383.0f);  // 14-bit
uint32_t triple = ((uint32_t)lat_encoded << 18)
               | ((uint32_t)lon_encoded  <<  4)
               |  (uint32_t)(radius_code & 0x0F);
```

Precision: ≈1.2 km in latitude, ≈2.4 km in longitude at the equator (improves toward higher latitudes — exactly where most MeshCore users are). Encoding error is smaller than the minimum useful radius.

### Radius Lookup Table

| Code | km  | Code | km  | Code | km  | Code |      |
|------|-----|------|-----|------|-----|------|------|
| 0    | 1   | 4    | 8   | 8    | 50  | 12   | 300  |
| 1    | 2   | 5    | 12  | 9    | 80  | 13   | 500  |
| 2    | 3   | 6    | 20  | 10   | 120 | 14   | 800  |
| 3    | 5   | 7    | 30  | 11   | 200 | 15   | ∞ (no limit) |

Code 15 (`∞`) can mark an anchor point without a spatial constraint (e.g., the last point of a chain where only the direction matters).

## Files

### New File

**`MeshCore/src/helpers/CorridorCheck.h`** — Header-only helper (no `.cpp`, no build system change needed)

Public API:

```cpp
#define MAX_CORRIDOR_TRIPLES  8

struct CorridorTriple {
    float lat;        // decimal degrees
    float lon;        // decimal degrees
    float radius_km;  // decoded from radius_code
};

// Encode one triple to 4-byte wire format
uint32_t encodeCorridorTriple(float lat, float lon, uint8_t radius_code);

// Decode one 4-byte wire triple; returns false if radius_code == 15 (infinite)
bool decodeCorridorTriple(uint32_t encoded, CorridorTriple& out);

// Extract N from transport_codes[1]
inline uint8_t  getCorridorCount(uint16_t tc1)    { return (tc1 >> 12) & 0x0F; }

// Build transport_codes[1] value for N triples
inline uint16_t makeCorridorHeader(uint8_t n)     { return (uint16_t)(n & 0x0F) << 12; }

// Radius code → km (returns FLT_MAX for code 15)
float radiusCodeToKm(uint8_t code);

// Returns true if point (px, py) is inside the corridor defined by 'count' connected circles.
// Applies cos(lat) correction for accurate spherical distance approximation.
bool isPointInCorridor(float px, float py, const CorridorTriple* circles, int count);
```

#### `isPointInCorridor` — Algorithm Notes

Based on the connected-circles algorithm from Issue #1451, with one fix:

**cos(lat) correction** — lat/lon is not a flat Cartesian plane. Without correction, corridors are ellipses in geographic space. Fix applied per segment:

```cpp
float lat_mid  = (c1.lat + c2.lat) * 0.5f;
float cos_lat  = cosf(lat_mid * (float)(M_PI / 180.0));
float vx = (c2.lon - c1.lon) * cos_lat;   // lon delta scaled to true distance
float wx = (px     - c1.lon) * cos_lat;
float vy = c2.lat - c1.lat;               // lat delta unchanged
float wy = py     - c1.lat;
```

This makes corridor circles geometrically correct across all latitudes. Only one `cosf()` call per segment (N−1 ≤ 7 calls total per packet) — negligible cost since the check runs once per received flood packet, never in the radio ISR.

The single-circle special case also applies cos(lat) to the longitude distance.

### Modified Files

#### `MeshCore/examples/simple_repeater/MyMesh.h`

- Add `#include <helpers/CorridorCheck.h>` alongside existing helpers.
- Add declaration for the new `sendFloodScoped` overload:

```cpp
void sendFloodScoped(const TransportKey& scope, mesh::Packet* pkt,
                     const CorridorTriple* corridor, uint8_t corridor_count,
                     uint32_t delay_millis, uint8_t path_hash_size);
```

#### `MeshCore/examples/simple_repeater/MyMesh.cpp`

**1. `allowPacketForward()` — relay-side corridor check**

Insert after the existing loop-detection block, before `return true`:

```cpp
// Corridor check (soft-hint): only applies to TRANSPORT_FLOOD packets
if (packet->getRouteType() == ROUTE_TYPE_TRANSPORT_FLOOD) {
    uint8_t n = getCorridorCount(packet->transport_codes[1]);
    if (n > 0 && (_prefs.node_lat != 0.0 || _prefs.node_lon != 0.0)) {
        const uint8_t* corridor_data = packet->payload + packet->payload_len - (uint16_t)n * 4;
        CorridorTriple triples[MAX_CORRIDOR_TRIPLES];
        uint8_t decoded = 0;
        for (uint8_t i = 0; i < n; i++) {
            uint32_t raw;
            memcpy(&raw, corridor_data + i * 4, 4);
            if (!decodeCorridorTriple(raw, triples[decoded])) {
                // radius_code == 15 (infinite) counts as always inside
                decoded++;
                continue;
            }
            decoded++;
        }
        if (!isPointInCorridor((float)_prefs.node_lat, (float)_prefs.node_lon, triples, decoded)) {
            MESH_DEBUG_PRINTLN("allowPacketForward: position outside corridor, dropping flood");
            return false;
        }
    }
}
```

**2. New `sendFloodScoped` overload — sender-side API**

```cpp
void MyMesh::sendFloodScoped(const TransportKey& scope, mesh::Packet* pkt,
                              const CorridorTriple* corridor, uint8_t corridor_count,
                              uint32_t delay_millis, uint8_t path_hash_size)
{
    // Clamp to maximum
    if (corridor_count > MAX_CORRIDOR_TRIPLES) corridor_count = MAX_CORRIDOR_TRIPLES;

    // Append encoded triples at end of payload
    uint8_t* dst = pkt->payload + pkt->payload_len;
    for (uint8_t i = 0; i < corridor_count; i++) {
        uint8_t radius_code = 15; // default: infinite
        for (uint8_t r = 0; r < 15; r++) {
            if (radiusCodeToKm(r) >= corridor[i].radius_km) { radius_code = r; break; }
        }
        uint32_t encoded = encodeCorridorTriple(corridor[i].lat, corridor[i].lon, radius_code);
        memcpy(dst, &encoded, 4);
        dst += 4;
    }
    pkt->payload_len += (uint16_t)corridor_count * 4;

    // Build transport codes
    uint16_t codes[2];
    codes[0] = scope.isNull() ? 0 : scope.calcTransportCode(pkt);
    codes[1] = makeCorridorHeader(corridor_count);

    if (scope.isNull()) {
        sendFlood(pkt, delay_millis, path_hash_size);          // no region scope
    } else {
        sendFlood(pkt, codes, delay_millis, path_hash_size);   // with region scope + corridor
    }
}
```

#### `MeshCore/docs/packet_format.md`

Update the `transport_codes` field description to document the new `transport_codes[1]` layout. Add a new **Corridor Extension** section describing the triple encoding, radius table, and the appended payload tail.

## Implementation Phases

### Phase 1 — `CorridorCheck.h` helper (independent)

1. Create `MeshCore/src/helpers/CorridorCheck.h`
2. Implement `CORRIDOR_RADIUS_KM[16]` as `constexpr float` table
3. Implement `encodeCorridorTriple()` and `decodeCorridorTriple()`
4. Implement `getCorridorCount()` / `makeCorridorHeader()` as inline functions
5. Implement `isPointInCorridor()` with cos(lat) correction

### Phase 2 — Relay check in `allowPacketForward()` *(depends on Phase 1)*

6. Add `#include <helpers/CorridorCheck.h>` to `MyMesh.h`
7. Insert corridor check block into `MyMesh::allowPacketForward()`

### Phase 3 — Sender API *(depends on Phase 1, parallel with Phase 2)*

8. Add new `sendFloodScoped` overload declaration to `MyMesh.h`
9. Implement it in `MyMesh.cpp`

### Phase 4 — Documentation

10. Update `MeshCore/docs/packet_format.md`

## Backward Compatibility

| Stage | Old firmware behavior | Impact |
|-------|----------------------|--------|
| Parse `transport_codes[1]` | Reads 2 bytes, value is stored but unused | ✅ No effect |
| Parse `path_length` | Byte position **unchanged** | ✅ Correct |
| Relay decision | Based on region + hop count + dedup only | ✅ Unchanged |
| Relay action | Copies raw bytes, overwrites only the path section | ✅ Corridor tail preserved |
| Destination payload parsing | Fixed-format types read expected bytes; trailing bytes ignored | ✅ No effect |
| `PAYLOAD_TYPE_TXT_MSG` | Null-terminated; corridor bytes are post-null trailing data | ✅ No effect |

## Verification

1. **Compilation:** `simple_repeater` builds without errors or warnings on all supported platforms (nRF52, RP2040, ESP32, STM32)
2. **Unit test:** Standalone `.cpp` test for `isPointInCorridor()` using the four test points from Issue #1451:
   - `(52.0, 13.0)` → INSIDE circle 1
   - `(52.4, 13.15)` → INSIDE corridor between circles 1–2
   - `(54.1, 14.6)` → OUTSIDE all circles
   - `(52.75, 13.35)` → INSIDE corridor between circles 2–3
3. **Relay behavior:** Send a `TRANSPORT_FLOOD` packet with N=2 triples; confirm that a repeater inside the corridor forwards and one outside does not
4. **Backward compatibility:** Confirm that a repeater built from pre-corridor firmware forwards all `TRANSPORT_FLOOD` packets as before (N=0 in `transport_codes[1]`)
5. **Payload budget:** Confirm that with N=8 triples (32 bytes overhead) and a typical `PAYLOAD_TYPE_TXT_MSG`, the total wire length stays within `MAX_TRANS_UNIT` (255 bytes)

## Out of Scope

- Simulator (`crates/mcsim-firmware/`) — would need separate work to generate corridor packets and validate relay behavior end-to-end
- `ROUTE_TYPE_FLOOD` (without transport codes) — no field available for corridor header without a different signaling mechanism
- Hard enforcement (forcing old repeaters to drop) — not achievable without breaking backward compatibility via a new payload version (`PH_VER_*`)
- Auto-corridor generation from contact list / path history — client-side logic (MeshCore companion app)