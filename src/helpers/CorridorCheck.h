#pragma once

// Flood Corridor helper for ROUTE_TYPE_TRANSPORT_FLOOD packets.
//
// A geo-corridor is a union of up to MAX_CORRIDOR_TRIPLES circles connected
// along a path.  It lets corridor-aware repeaters forward a flood only when
// their own position lies inside the corridor, giving the sender per-packet
// geographic scoping without pre-provisioned regions.
//
// Wire layout (Flood Corridor, "Model X"):
//   code_1 (transport_codes[0]) = transport code of the well-known pseudo-region
//                                 "corridor" (auto-key from "#corridor").  Old
//                                 repeaters don't know it → drop at the region
//                                 gate (graceful, rolling deployment).
//   code_2 (transport_codes[1]) bits 15-12 = N (triple count, 0..8); 0 = none.
//   corridor region: N * CORRIDOR_TRIPLE_BYTES, carried in Packet::corridor[]
//                     (serialized between path and payload).  The payload stays
//                     the standard [channel_hash][MAC][ciphertext] — receivers
//                     and the code_1 HMAC are unaware of the corridor.
//
// Wire encoding per triple (4 bytes / 32 bits):
//   Bits 31-18  lat_encoded  14-bit offset-binary  (lat +90)/180 × 16383
//   Bits 17- 4  lon_encoded  14-bit offset-binary  (lon+180)/360 × 16383
//   Bits  3- 0  radius_code  4-bit index into CORRIDOR_RADIUS_KM[]

#include <stdint.h>
#include <string.h>
#include <math.h>

#include <Packet.h>
#include <SHA256.h>
#include "TransportKeyStore.h"

// MAX_CORRIDOR_TRIPLES / CORRIDOR_TRIPLE_BYTES are defined in MeshCore.h
// (pulled in via Packet.h).

// Radius lookup table indexed by the 4-bit radius_code field.
// Code 15 means "unlimited" (FLT_MAX) — an always-inside anchor.
static const float CORRIDOR_RADIUS_KM[16] = {
    1.0f,   // 0
    2.0f,   // 1
    3.0f,   // 2
    5.0f,   // 3
    8.0f,   // 4
   12.0f,   // 5
   20.0f,   // 6
   30.0f,   // 7
   50.0f,   // 8
   80.0f,   // 9
  120.0f,   // 10
  200.0f,   // 11
  300.0f,   // 12
  500.0f,   // 13
  800.0f,   // 14
  3.4028235e+38f  // 15 — unlimited (FLT_MAX without including <float.h>)
};

#define CORRIDOR_RADIUS_UNLIMITED_KM  3.4028235e+38f

// --- Decoded representation ---

struct CorridorTriple {
    float lat;        // decimal degrees, -90..+90
    float lon;        // decimal degrees, -180..+180
    float radius_km;  // kilometres; CORRIDOR_RADIUS_UNLIMITED_KM means unlimited
};

// --- Encoding / Decoding ---

inline uint32_t encodeCorridorTriple(float lat, float lon, uint8_t radius_code) {
    uint16_t lat_enc = (uint16_t)((lat +  90.0f) / 180.0f * 16383.0f + 0.5f);
    uint16_t lon_enc = (uint16_t)((lon + 180.0f) / 360.0f * 16383.0f + 0.5f);
    if (lat_enc > 16383) lat_enc = 16383;
    if (lon_enc > 16383) lon_enc = 16383;
    return ((uint32_t)lat_enc << 18)
         | ((uint32_t)lon_enc <<  4)
         | ((uint32_t)(radius_code & 0x0F));
}

inline void decodeCorridorTriple(uint32_t encoded, CorridorTriple& out) {
    uint16_t lat_enc    = (uint16_t)((encoded >> 18) & 0x3FFF);
    uint16_t lon_enc    = (uint16_t)((encoded >>  4) & 0x3FFF);
    uint8_t  radius_code = (uint8_t)(encoded & 0x0F);
    out.lat       = (float)lat_enc / 16383.0f * 180.0f - 90.0f;
    out.lon       = (float)lon_enc / 16383.0f * 360.0f - 180.0f;
    out.radius_km = CORRIDOR_RADIUS_KM[radius_code];
}

// --- code_2 helpers ---

// Extract the corridor triple count N from a raw code_2 value.
inline uint8_t getCorridorCount(uint16_t code2) {
    return (uint8_t)((code2 >> 12) & 0x0F);
}

// Build the code_2 value for N corridor triples (N clamped to MAX_CORRIDOR_TRIPLES).
inline uint16_t makeCorridorHeader(uint8_t n) {
    if (n > MAX_CORRIDOR_TRIPLES) n = MAX_CORRIDOR_TRIPLES;
    return (uint16_t)((uint16_t)n << 12);
}

inline float radiusCodeToKm(uint8_t code) {
    return CORRIDOR_RADIUS_KM[code & 0x0F];
}

// --- Pseudo-region "corridor" (Model X) ---

// The well-known corridor pseudo-region key, derived deterministically from the
// name "#corridor" exactly like an auto-hashtag region (SHA256 → 16 bytes).
// Sender and every corridor-aware repeater compute the same key → same code_1
// for a given payload.  No secret distribution; the key is public on purpose
// (code_1 is a routing tag, not access control).
inline const TransportKey& corridorPseudoKey() {
    static TransportKey key;
    static bool init = false;
    if (!init) {
        SHA256 sha;
        sha.update("#corridor", 9);
        sha.finalize(key.key, sizeof(key.key));
        init = true;
    }
    return key;
}

// True iff code_1 matches the corridor pseudo-region for this packet's payload.
inline bool matchesCorridorRegion(const mesh::Packet* pkt) {
    return pkt->transport_codes[0] == corridorPseudoKey().calcTransportCode(pkt);
}

// --- Packet region helpers ---

// Encode the given triples into pkt->corridor[] and write N into code_2.
// The packet's payload is left untouched (corridor is a separate region).
// n is clamped to MAX_CORRIDOR_TRIPLES.  Each radius is rounded UP to the
// nearest supported table entry (so the encoded corridor always covers the
// requested one); use radius_km >= CORRIDOR_RADIUS_UNLIMITED_KM for "always inside".
inline void fillCorridor(mesh::Packet* pkt, const CorridorTriple* corridor, uint8_t n) {
    if (pkt == nullptr) return;
    if (n > MAX_CORRIDOR_TRIPLES) n = MAX_CORRIDOR_TRIPLES;
    for (uint8_t i = 0; i < n; i++) {
        uint8_t radius_code = 15;  // default: unlimited
        if (corridor[i].radius_km < CORRIDOR_RADIUS_UNLIMITED_KM) {
            for (uint8_t r = 0; r < 15; r++) {
                if (CORRIDOR_RADIUS_KM[r] >= corridor[i].radius_km) { radius_code = r; break; }
            }
        }
        uint32_t encoded = encodeCorridorTriple(corridor[i].lat, corridor[i].lon, radius_code);
        memcpy(&pkt->corridor[i * CORRIDOR_TRIPLE_BYTES], &encoded, CORRIDOR_TRIPLE_BYTES);
    }
    pkt->transport_codes[1] = makeCorridorHeader(n);
}

// Decode pkt->corridor[] into up to max triples.  Returns the count decoded
// (= pkt->getCorridorCount(), clamped to max).  No allocation.
inline uint8_t decodePacketCorridor(const mesh::Packet* pkt, CorridorTriple* out, uint8_t max) {
    uint8_t n = pkt->getCorridorCount();
    if (n > max) n = max;
    for (uint8_t i = 0; i < n; i++) {
        uint32_t encoded;
        memcpy(&encoded, &pkt->corridor[i * CORRIDOR_TRIPLE_BYTES], CORRIDOR_TRIPLE_BYTES);
        decodeCorridorTriple(encoded, out[i]);
    }
    return n;
}

// --- Spatial check ---

// True if the point lies inside the capsule between two circle centres, using
// the perpendicular distance to the segment and the linearly interpolated
// radius.  A cos(lat) correction scales longitude to ground distance.  An
// unlimited radius on either endpoint makes the whole segment always-inside.
// dist² is compared against r² (no sqrt).
inline bool pointInCorridorSegment(float lat, float lon,
                                   float c1lat, float c1lon, float r1,
                                   float c2lat, float c2lon, float r2)
{
    if (r1 >= CORRIDOR_RADIUS_UNLIMITED_KM || r2 >= CORRIDOR_RADIUS_UNLIMITED_KM) return true;
    float lat_mid = (c1lat + c2lat) * 0.5f;
    float cos_lat = cosf(lat_mid * (float)(M_PI / 180.0));
    float scale = 111.0f;  // 1 degree lat ≈ 111 km
    float vx = (c2lat - c1lat) * scale;
    float vy = (c2lon - c1lon) * cos_lat * scale;
    float wx = (lat - c1lat) * scale;
    float wy = (lon - c1lon) * cos_lat * scale;
    float len_sq = vx * vx + vy * vy;
    float t;
    if (len_sq < 1e-9f) {
        t = 0.0f;  // degenerate segment → treat as circle at c1
    } else {
        t = (wx * vx + wy * vy) / len_sq;
        if (t < 0.0f) t = 0.0f;
        else if (t > 1.0f) t = 1.0f;
    }
    float ax = c1lat * scale + t * vx;
    float ay = c1lon * cos_lat * scale + t * vy;
    float dx = lat * scale - ax;
    float dy = lon * cos_lat * scale - ay;
    float r = r1 + t * (r2 - r1);
    return (dx * dx + dy * dy) <= (r * r);
}

// True if point (lat, lon) lies inside the corridor = union of capsules along
// the polyline through the circle centres.
inline bool isPointInCorridor(float lat, float lon,
                              const CorridorTriple* circles, int count)
{
    if (count <= 0) return false;
    if (count == 1) {
        if (circles[0].radius_km >= CORRIDOR_RADIUS_UNLIMITED_KM) return true;
        return pointInCorridorSegment(lat, lon,
                                      circles[0].lat, circles[0].lon, circles[0].radius_km,
                                      circles[0].lat, circles[0].lon, circles[0].radius_km);
    }
    for (int i = 0; i < count - 1; ++i) {
        if (pointInCorridorSegment(lat, lon,
                                   circles[i].lat, circles[i].lon, circles[i].radius_km,
                                   circles[i + 1].lat, circles[i + 1].lon, circles[i + 1].radius_km)) {
            return true;
        }
    }
    return false;
}
