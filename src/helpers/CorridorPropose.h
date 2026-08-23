#pragma once

// On-node flood-corridor proposal generator (Phase C, AP1).
//
// Suggests a corridor (waypoint chain) from the node's own position to a
// target, bending towards known repeater positions (Tier 1) or falling back
// to a straight diamond (Tier 0).  Pure geometry: no heap, no mesh state,
// fully deterministic — so a companion app can fetch a proposal (opcode 66)
// and preview/edit it before sending via opcode 53.

#include <stdint.h>
#include <string.h>
#include <math.h>

#include "CorridorRadius.h"

struct CorridorCandidate {
  float lat, lon;   // decimal degrees (from contact adverts)
};

struct CorridorGenParams {
  uint8_t tier0_end_code;      // radius code at A/B ends of the Tier-0 diamond
  uint8_t tier0_mid_code;      // radius code at the Tier-0 midpoint bead
  uint8_t width_floor_code;    // smallest radius code the D9 width pass may pick
  uint8_t width_ceiling_code;  // largest radius code the D9 width pass may pick
  uint8_t n_target;            // candidates that must lie inside a bead circle
  float r_hop_km;              // max distance between consecutive beads
};

static inline CorridorGenParams defaultCorridorGenParams() {
  CorridorGenParams p;
  p.tier0_end_code = 4;      // 8 km
  p.tier0_mid_code = 6;      // 20 km
  p.width_floor_code = 3;    // 5 km
  p.width_ceiling_code = 7;  // 30 km
  p.n_target = 2;
  p.r_hop_km = 12.0f;
  return p;
}

struct CorridorProposal {
  float lats[8];
  float lons[8];
  uint8_t radius_codes[8];  // direct codes, so callers encode without a reverse lookup
  uint8_t count;            // 1 (nearby) or 3 (Tier 0) or 2..8 (Tier 1)
  uint8_t reason;           // bits 0-2 tier, 0x08 thin stretch, 0x80 horizon-limited
};

// Equirectangular distance in km, same scaling as pointInCorridorSegment().
static inline float corridorDistKm(float lat1, float lon1, float lat2, float lon2) {
  float cos_lat = cosf((lat1 + lat2) * 0.5f * (float)(M_PI / 180.0));
  float dy = (lat2 - lat1) * 111.0f;
  float dx = (lon2 - lon1) * cos_lat * 111.0f;
  return sqrtf(dx * dx + dy * dy);
}

// Linear midpoint — curvature error is negligible for the <~1000 km hops this serves.
static inline void corridorMidpoint(float lat1, float lon1, float lat2, float lon2,
                                    float& lat, float& lon) {
  lat = (lat1 + lat2) * 0.5f;
  lon = (lon1 + lon2) * 0.5f;
}

// Build a proposal from src to dst via up to 32 candidates.  Returns out.count.
static inline uint8_t proposeCorridor(float src_lat, float src_lon,
                                      float dst_lat, float dst_lon,
                                      const CorridorCandidate* cand, uint8_t n_cand,
                                      const CorridorGenParams& p, CorridorProposal& out) {
  out.count = 0;
  out.reason = 0;
  if (n_cand > 32) n_cand = 32;  // visited bitmask holds 32 slots; extras are ignored

  // Tier-1 greedy walk: first bead is always src, last bead always dst.
  float cur_lat = src_lat, cur_lon = src_lon;
  out.lats[0] = src_lat;
  out.lons[0] = src_lon;
  out.count = 1;
  uint32_t visited = 0;
  bool reached = false;
  while (out.count < 8 - 1) {  // keep the last slot for the dst bead
    float d_cur = corridorDistKm(cur_lat, cur_lon, dst_lat, dst_lon);
    if (d_cur <= p.r_hop_km) {  // target within one hop → close the chain
      reached = true;
      break;
    }
    int best = -1;
    float best_progress = 0.0f;  // require strictly positive progress
    for (uint8_t i = 0; i < n_cand; i++) {
      if (visited & (1u << i)) continue;
      if (corridorDistKm(cur_lat, cur_lon, cand[i].lat, cand[i].lon) > p.r_hop_km) continue;
      float progress = d_cur - corridorDistKm(cand[i].lat, cand[i].lon, dst_lat, dst_lon);
      if (progress > best_progress) {  // ties keep the lowest index (deterministic)
        best_progress = progress;
        best = i;
      }
    }
    if (best < 0) break;  // dead end — dst gets appended as a horizon bead below
    out.lats[out.count] = cand[best].lat;
    out.lons[out.count] = cand[best].lon;
    out.count++;
    visited |= (1u << best);
    cur_lat = cand[best].lat;
    cur_lon = cand[best].lon;
  }
  out.lats[out.count] = dst_lat;
  out.lons[out.count] = dst_lon;
  out.count++;
  out.reason |= (reached ? 2 : 1) | (reached ? 0 : 0x80);

  if (out.count == 2) {  // walk added no intermediate bead → discard, emit Tier 0
    out.count = 0;
    out.reason = 0;
    float mlat, mlon;
    corridorMidpoint(src_lat, src_lon, dst_lat, dst_lon, mlat, mlon);
    if (corridorDistKm(src_lat, src_lon, dst_lat, dst_lon) < 2.0f) {
      out.lats[0] = mlat;
      out.lons[0] = mlon;
      out.radius_codes[0] = p.tier0_mid_code;
      out.count = 1;
    } else {
      out.lats[0] = src_lat;
      out.lons[0] = src_lon;
      out.radius_codes[0] = p.tier0_end_code;
      out.lats[1] = mlat;
      out.lons[1] = mlon;
      out.radius_codes[1] = p.tier0_mid_code;
      out.lats[2] = dst_lat;
      out.lons[2] = dst_lon;
      out.radius_codes[2] = p.tier0_end_code;
      out.count = 3;
    }
    return out.count;
  }

  // D9 width pass: per bead, smallest code in [floor..ceiling] whose circle
  // covers >= n_target candidates.  Plain circle test — intentionally cheaper
  // than the capsule test, good enough for a proposal.
  for (uint8_t b = 0; b < out.count; b++) {
    uint8_t code = p.width_ceiling_code;
    bool covered = false;
    for (uint8_t c = p.width_floor_code; c <= p.width_ceiling_code && !covered; c++) {
      uint8_t inside = 0;
      for (uint8_t i = 0; i < n_cand; i++) {
        if (corridorDistKm(out.lats[b], out.lons[b], cand[i].lat, cand[i].lon) <= CORRIDOR_RADIUS_KM[c]) inside++;
      }
      if (inside >= p.n_target) {
        code = c;
        covered = true;
      }
    }
    if (!covered) out.reason |= 0x08;  // thin stretch — no code covers enough candidates
    out.radius_codes[b] = code;
  }
  return out.count;
}
