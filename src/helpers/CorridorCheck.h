#pragma once

// Corridor Check helper for ROUTE_TYPE_TRANSPORT_FLOOD packets
//
// Design:  transport_codes[1] bits 15-12 carry N (number of corridor triples, 0..8).
//          N×4 bytes are appended at the end of the packet payload after the typed
//          payload content.  Corridor-unaware repeaters (transport_codes[1] == 0)
//          forward as before — full backward compatibility.
//
// Wire encoding per triple (4 bytes / 32 bits):
//   Bits 31-18  lat_encoded  14-bit offset-binary  (lat+90)/180 × 16383
//   Bits 17- 4  lon_encoded  14-bit offset-binary  (lon+180)/360 × 16383
//   Bits  3- 0  radius_code  4-bit index into CORRIDOR_RADIUS_KM[]
//
// See: https://github.com/meshcore-dev/MeshCore/issues/1451

#include <stdint.h>
#include <string.h>
#include <math.h>

// Maximum number of corridor triples per packet
#define MAX_CORRIDOR_TRIPLES  8

// Byte size of one encoded triple on the wire
#define CORRIDOR_TRIPLE_BYTES  4

// Radius lookup table indexed by the 4-bit radius_code field.
// Code 15 means "unlimited" (FLT_MAX).
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

// --- Decoded representation ---

struct CorridorTriple {
    float lat;        // decimal degrees, -90..+90
    float lon;        // decimal degrees, -180..+180
    float radius_km;  // kilometres; FLT_MAX means unlimited
};

// --- Encoding / Decoding ---

/**
 * Encode one (lat, lon, radius_code) triple into 4-byte wire format.
 * radius_code must be 0..15; use 15 for unlimited.
 */
inline uint32_t encodeCorridorTriple(float lat, float lon, uint8_t radius_code) {
    uint16_t lat_enc = (uint16_t)((lat  +  90.0f) / 180.0f * 16383.0f + 0.5f);
    uint16_t lon_enc = (uint16_t)((lon  + 180.0f) / 360.0f * 16383.0f + 0.5f);
    if (lat_enc > 16383) lat_enc = 16383;
    if (lon_enc > 16383) lon_enc = 16383;
    return ((uint32_t)lat_enc   << 18)
         | ((uint32_t)lon_enc   <<  4)
         | ((uint32_t)(radius_code & 0x0F));
}

/**
 * Decode a 4-byte wire triple into a CorridorTriple.
 * Returns true normally.  When radius_code == 15 (unlimited) the triple is
 * decoded with radius_km == FLT_MAX — the caller should treat this as
 * "always inside" for this anchor point.
 */
inline bool decodeCorridorTriple(uint32_t encoded, CorridorTriple& out) {
    uint16_t lat_enc    = (uint16_t)((encoded >> 18) & 0x3FFF);
    uint16_t lon_enc    = (uint16_t)((encoded >>  4) & 0x3FFF);
    uint8_t  radius_code = (uint8_t)(encoded & 0x0F);

    out.lat       = (float)lat_enc / 16383.0f * 180.0f - 90.0f;
    out.lon       = (float)lon_enc / 16383.0f * 360.0f - 180.0f;
    out.radius_km = CORRIDOR_RADIUS_KM[radius_code];
    return true;
}

// --- transport_codes[1] helpers ---

/**
 * Extract the corridor triple count N from transport_codes[1].
 * Returns 0 when no corridor is present.
 */
inline uint8_t getCorridorCount(uint16_t tc1) {
    return (uint8_t)((tc1 >> 12) & 0x0F);
}

/**
 * Build the transport_codes[1] value for N corridor triples.
 * N must be 0..8; values above 8 are clamped.
 */
inline uint16_t makeCorridorHeader(uint8_t n) {
    if (n > MAX_CORRIDOR_TRIPLES) n = MAX_CORRIDOR_TRIPLES;
    return (uint16_t)((uint16_t)n << 12);
}

/**
 * Find the exact radius code for a given radius in km.
 *
 * Returns true and sets *code_out if radius_km exactly matches (within 0.001 km)
 * one of the 16 supported values.  When no exact match is found, sets *lower_out
 * and *upper_out to the nearest supported values below / above (FLT_MAX when
 * no bound exists on that side) and returns false.
 */
inline bool findExactRadiusCode(float radius_km,
                                 uint8_t* code_out,
                                 float*   lower_out = nullptr,
                                 float*   upper_out = nullptr) {
    for (uint8_t r = 0; r < 16; r++) {
        float tabval = CORRIDOR_RADIUS_KM[r];
        bool match = (tabval >= 3.4e38f)                      // unlimited sentinel
                     ? (radius_km >= 3.4e38f)
                     : (fabsf(radius_km - tabval) < 0.001f);
        if (match) {
            if (code_out) *code_out = r;
            return true;
        }
    }
    // Collect nearest lower (largest table value < radius_km, excluding unlimited)
    float lo = -1.0f;
    float hi = -1.0f;
    for (uint8_t r = 0; r < 15; r++) {  // skip index 15 (unlimited)
        float tabval = CORRIDOR_RADIUS_KM[r];
        if (tabval < radius_km && (lo < 0.0f || tabval > lo)) lo = tabval;
        if (tabval > radius_km && (hi < 0.0f || tabval < hi)) hi = tabval;
    }
    if (lower_out) *lower_out = (lo >= 0.0f) ? lo : -3.4028235e+38f;  // -FLT_MAX when no bound below
    if (upper_out) *upper_out = (hi >= 0.0f) ? hi : 3.4028235e+38f;   // FLT_MAX when no bound above
    return false;
}

/**
 * Look up radius in km for a given 4-bit code.
 * Returns FLT_MAX equivalent for code 15 (unlimited).
 */
inline float radiusCodeToKm(uint8_t code) {
    return CORRIDOR_RADIUS_KM[code & 0x0F];
}

// --- Spatial check ---

/**
 * Returns true if the point (px=latitude, py=longitude) lies inside the
 * corridor defined by the array of connected circles.
 *
 * The algorithm projects the point onto each line segment between consecutive
 * circle centres and checks whether the perpendicular distance from the point
 * to the nearest position on the segment is within the linearly interpolated
 * radius at that position.
 *
 * A cos(lat) correction is applied per segment so that longitude differences
 * are scaled to their true ground distance, producing circular (not elliptic)
 * corridors at all latitudes.
 *
 * Circles with radius_km == FLT_MAX (code 15) are treated as always-inside
 * anchor points: any segment adjacent to such a circle is skipped in the
 * distance check, and the single-circle special case returns true immediately.
 *
 * Reference: https://github.com/meshcore-dev/MeshCore/issues/1451
 */
inline bool isPointInCorridor(float px, float py,
                               const CorridorTriple* circles, int count)
{
    if (count <= 0) return false;

    // Special case: single circle
    if (count == 1) {
        if (circles[0].radius_km >= CORRIDOR_RADIUS_KM[15]) return true; // unlimited
        float lat_mid = circles[0].lat;
        float cos_lat = cosf(lat_mid * (float)(M_PI / 180.0));
        float dx = (px - circles[0].lat);
        float dy = (py - circles[0].lon) * cos_lat;
        // Convert degrees to km: 1 degree lat ≈ 111.0 km
        float scale = 111.0f;
        dx *= scale;
        dy *= scale;
        float r = circles[0].radius_km;
        return (dx * dx + dy * dy) <= (r * r);
    }

    // General case: iterate over segments
    float scale = 111.0f; // degrees to km

    for (int i = 0; i < count - 1; ++i) {
        const CorridorTriple& c1 = circles[i];
        const CorridorTriple& c2 = circles[i + 1];

        // cos(lat) correction using midpoint latitude of the segment
        float lat_mid = (c1.lat + c2.lat) * 0.5f;
        float cos_lat = cosf(lat_mid * (float)(M_PI / 180.0));

        // Segment vector (in km-equivalent units)
        float vx = (c2.lat - c1.lat) * scale;
        float vy = (c2.lon - c1.lon) * cos_lat * scale;

        // Vector from c1 to point
        float wx = (px - c1.lat) * scale;
        float wy = (py - c1.lon) * cos_lat * scale;

        float len_sq = vx * vx + vy * vy;

        float t;
        if (len_sq < 1e-9f) {
            // Degenerate segment: treat as single circle at c1
            t = 0.0f;
        } else {
            t = (wx * vx + wy * vy) / len_sq;
            if (t < 0.0f) t = 0.0f;
            else if (t > 1.0f) t = 1.0f;
        }

        // Nearest point on segment axis
        float ax = c1.lat  * scale + t * vx;
        float ay = c1.lon  * cos_lat * scale + t * vy;

        // Distance from point to axis
        float qx = px * scale - ax;
        float qy = py * cos_lat * scale - ay;
        float dist_sq = qx * qx + qy * qy;

        // Interpolated radius at the nearest point
        float r1 = c1.radius_km;
        float r2 = c2.radius_km;

        // Unlimited radius (code 15) on either endpoint → always inside this segment
        if (r1 >= 3.4028234e+38f || r2 >= 3.4028234e+38f) return true;

        float r = r1 + t * (r2 - r1);
        if (dist_sq <= r * r) return true;
    }

    return false;
}

// --- Packet mutation helper ---
//
// Requires <Packet.h> (pulled in via <MeshCore.h>).  Include this header
// after any header that already brings in Packet.h, or let the #pragma once
// in Packet.h handle de-duplication.

#include <Packet.h>

/**
 * Append encoded corridor triples to a packet's payload and write the
 * corridor count into transport_codes[1].
 *
 * Call this on a fully-built packet *before* sending.  Afterwards pass
 * pkt->transport_codes to sendFlood(pkt, codes, ...) so the corridor header
 * is transmitted.
 *
 * @param pkt            Target packet (payload must have room for n×4 bytes).
 * @param corridor       Array of CorridorTriple values to encode.
 * @param n              Number of triples (clamped to MAX_CORRIDOR_TRIPLES).
 * @return true on success; false when the payload would overflow
 *         (MAX_PACKET_PAYLOAD exceeded) — packet is unchanged on failure.
 */
inline bool appendCorridorToPacket(mesh::Packet* pkt,
                                    const CorridorTriple* corridor,
                                    uint8_t n) {
    if (pkt == nullptr) return false;
    if (n > MAX_CORRIDOR_TRIPLES) n = MAX_CORRIDOR_TRIPLES;
    uint16_t added = (uint16_t)n * CORRIDOR_TRIPLE_BYTES;
    if (pkt->payload_len + added > MAX_PACKET_PAYLOAD) return false;

    uint8_t* dst = pkt->payload + pkt->payload_len;
    for (uint8_t i = 0; i < n; i++) {
        uint8_t radius_code;
        float lower_bound, upper_bound;
        if (!findExactRadiusCode(corridor[i].radius_km, &radius_code, &lower_bound, &upper_bound)) {
            // Radius is not in the supported set — log an error and abort.
            if (lower_bound >= 0.0f && upper_bound >= 0.0f) {
                MESH_DEBUG_PRINTLN("appendCorridorToPacket: unsupported radius %.3f km. "
                                   "Nearest supported: lower=%.0f km, upper=%.0f km.",
                                   corridor[i].radius_km, lower_bound, upper_bound);
            } else if (lower_bound >= 0.0f) {
                MESH_DEBUG_PRINTLN("appendCorridorToPacket: unsupported radius %.3f km. "
                                   "Nearest supported lower: %.0f km. Above that is unlimited.",
                                   corridor[i].radius_km, lower_bound);
            } else if (upper_bound >= 0.0f) {
                MESH_DEBUG_PRINTLN("appendCorridorToPacket: unsupported radius %.3f km. "
                                   "Nearest supported upper: %.0f km. No smaller value exists.",
                                   corridor[i].radius_km, upper_bound);
            } else {
                MESH_DEBUG_PRINTLN("appendCorridorToPacket: unsupported radius %.3f km.",
                                   corridor[i].radius_km);
            }
            return false;
        }
        uint32_t encoded = encodeCorridorTriple(corridor[i].lat, corridor[i].lon, radius_code);
        memcpy(dst, &encoded, CORRIDOR_TRIPLE_BYTES);
        dst += CORRIDOR_TRIPLE_BYTES;
    }

    pkt->payload_len += added;
    pkt->transport_codes[1] = makeCorridorHeader(n);
    return true;
}
