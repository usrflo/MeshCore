#pragma once

// On-node flood-corridor proposal generator.
//
// Suggests a corridor (waypoint chain) from the node's own position to a
// target, bending towards known repeater positions (Tier 1) or falling back
// to a straight diamond (Tier 0).  Pure geometry: no heap, no mesh state,
// fully deterministic — the companion calls this to auto-scope contact
// floods (see MyMesh::sendFloodScoped), and opcode 67 exposes it for
// app-side preview.  Pulls CorridorCheck.h for the capsule test so the width
// pass uses the very same geometry the forwarding check applies.

#include <stdint.h>
#include <string.h>
#include <math.h>

#include "CorridorRadius.h"
#include "CorridorCheck.h"

struct CorridorCandidate {
  float lat, lon;   // decimal degrees (from contact adverts)
};

struct CorridorGenParams {
  uint8_t tier0_end_code;      // radius code at A/B ends of the Tier-0 diamond
  uint8_t tier0_mid_code;      // radius code at the Tier-0 midpoint bead
  uint8_t width_floor_code;    // smallest radius code the width pass may pick
  uint8_t width_ceiling_code;  // largest radius code the width pass may pick
  uint8_t n_target;            // density candidates that must lie inside a segment
  float r_hop_km;              // max distance between consecutive beads
  float detour_factor;         // candidate relevance: ellipse widening around src→dst
};

static inline CorridorGenParams defaultCorridorGenParams() {
  CorridorGenParams p;
  p.tier0_end_code = 4;      // 8 km
  p.tier0_mid_code = 6;      // 20 km
  p.width_floor_code = 3;    // 5 km
  p.width_ceiling_code = 7;  // 30 km
  p.n_target = 2;
  p.r_hop_km = 12.0f;
  p.detour_factor = 1.4f;
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

// --- Candidate relevance selection ---
//
// Keeps only candidates inside the src→dst ellipse
//   dist(cand,src) + dist(cand,dst) <= detour_factor * dist(src,dst) + r_hop_km
// then the max_out with the smallest detour overhead
//   rel = dist(cand,src) + dist(cand,dst) - dist(src,dst)   (>= 0)
// Output ascending by (rel, source index) — ties keep the LOWER input index.
// n_all may exceed 32; only the output is capped.  No heap.
// Returns the number written; out_idx (optional) maps out[k] -> all-index.
static inline uint8_t selectCorridorCandidates(float src_lat, float src_lon,
                                               float dst_lat, float dst_lon,
                                               const CorridorCandidate* all, uint8_t n_all,
                                               CorridorCandidate* out, uint8_t* out_idx,
                                               uint8_t max_out,
                                               const CorridorGenParams& p) {
  if (out == NULL || max_out == 0) return 0;
  if (max_out > 32) max_out = 32;  // fixed scratch arrays
  float rel[32];
  uint8_t idx[32];
  uint8_t n = 0;
  float d_sd = corridorDistKm(src_lat, src_lon, dst_lat, dst_lon);
  float limit = p.detour_factor * d_sd + p.r_hop_km;
  for (uint8_t i = 0; i < n_all; i++) {
    float d_s = corridorDistKm(src_lat, src_lon, all[i].lat, all[i].lon);
    float d_d = corridorDistKm(all[i].lat, all[i].lon, dst_lat, dst_lon);
    if (d_s + d_d > limit) continue;  // outside the ellipse — irrelevant for this target
    float r = d_s + d_d - d_sd;
    // insertion position in the ascending (rel, index) top-list
    uint8_t pos = n;
    while (pos > 0 && (rel[pos - 1] > r || (rel[pos - 1] == r && idx[pos - 1] > i))) pos--;
    if (pos == max_out) continue;  // worse than everything kept
    if (n < max_out) n++;
    for (uint8_t j = n - 1; j > pos; j--) { rel[j] = rel[j - 1]; idx[j] = idx[j - 1]; }
    rel[pos] = r;
    idx[pos] = i;
  }
  for (uint8_t k = 0; k < n; k++) {
    out[k] = all[idx[k]];
    if (out_idx) out_idx[k] = idx[k];
  }
  return n;
}

// --- Beam walk (Tier 1) ---

#define CORRIDOR_MAX_INTERMEDIATE_BEADS 6  // 8 beads total incl. src/dst
#define CORRIDOR_BEAM_WIDTH 2

struct CorridorBeamState {
  uint8_t bead[CORRIDOR_MAX_INTERMEDIATE_BEADS];  // waypoint indices, walk order
  uint8_t n_beads;
  uint32_t visited;  // bitmask over the waypoint list (max 32)
  float d_dst;       // dist(cur, dst); cur = (n_beads ? wp[bead[n_beads-1]] : src)
  bool reached;      // d_dst <= r_hop at cur
};

// Element-wise comparison; shorter prefix wins on full match (deterministic tie-break).
static inline bool corridorBeadSeqLess(const uint8_t* a, uint8_t na, const uint8_t* b, uint8_t nb) {
  uint8_t m = (na < nb) ? na : nb;
  for (uint8_t k = 0; k < m; k++) {
    if (a[k] != b[k]) return a[k] < b[k];
  }
  return na < nb;
}

// Level ranking: reached first, then closer to dst, then fewer beads, then lex.
static inline bool corridorStateBetter(const CorridorBeamState& a, const CorridorBeamState& b) {
  if (a.reached != b.reached) return a.reached;
  if (a.d_dst != b.d_dst) return a.d_dst < b.d_dst;
  if (a.n_beads != b.n_beads) return a.n_beads < b.n_beads;
  return corridorBeadSeqLess(a.bead, a.n_beads, b.bead, b.n_beads);
}

// Final pick among unreached states: closest approach, then fewer beads, then lex.
static inline bool corridorUnreachedBetter(const CorridorBeamState& a, const CorridorBeamState& b) {
  if (a.d_dst != b.d_dst) return a.d_dst < b.d_dst;
  if (a.n_beads != b.n_beads) return a.n_beads < b.n_beads;
  return corridorBeadSeqLess(a.bead, a.n_beads, b.bead, b.n_beads);
}

// Deterministic beam search from src toward dst over the waypoint candidates.
// Expansion: unvisited waypoints within r_hop with STRICTLY positive progress.
// Returns the intermediate bead count (0..6) and fills out_bead; out_reached
// tells whether the chain got within r_hop of dst (tier 2 vs tier 1 + 0x80).
// Greedy parity: whenever each level has a single viable child (or the greedy
// chain keeps the smallest d_dst), the output equals the old greedy walk's.
static inline uint8_t corridorBeamWalk(float src_lat, float src_lon,
                                       float dst_lat, float dst_lon,
                                       const CorridorCandidate* wp, uint8_t n_wp,
                                       const CorridorGenParams& p,
                                       uint8_t out_bead[CORRIDOR_MAX_INTERMEDIATE_BEADS],
                                       bool& out_reached) {
  out_reached = false;
  if (wp == NULL) n_wp = 0;
  if (n_wp > 32) n_wp = 32;  // visited bitmask holds 32 slots

  CorridorBeamState init;
  memset(&init, 0, sizeof(init));
  init.d_dst = corridorDistKm(src_lat, src_lon, dst_lat, dst_lon);
  init.reached = (init.d_dst <= p.r_hop_km);
  if (init.reached || n_wp == 0) return 0;  // target within one hop, or nothing to bend to → Tier 0

  CorridorBeamState best_unreached = init;
  CorridorBeamState beam[CORRIDOR_BEAM_WIDTH];
  beam[0] = init;
  uint8_t n_beam = 1;

  for (uint8_t level = 0; level < CORRIDOR_MAX_INTERMEDIATE_BEADS; level++) {
    CorridorBeamState child_a, child_b;
    bool has_a = false, has_b = false;
    for (uint8_t s = 0; s < n_beam; s++) {
      float cur_lat = (beam[s].n_beads > 0) ? wp[beam[s].bead[beam[s].n_beads - 1]].lat : src_lat;
      float cur_lon = (beam[s].n_beads > 0) ? wp[beam[s].bead[beam[s].n_beads - 1]].lon : src_lon;
      for (uint8_t i = 0; i < n_wp; i++) {
        if (beam[s].visited & (1u << i)) continue;
        if (corridorDistKm(cur_lat, cur_lon, wp[i].lat, wp[i].lon) > p.r_hop_km) continue;
        float d_new = corridorDistKm(wp[i].lat, wp[i].lon, dst_lat, dst_lon);
        if (d_new >= beam[s].d_dst) continue;  // require strictly positive progress
        CorridorBeamState child = beam[s];
        child.bead[child.n_beads] = i;
        child.n_beads++;
        child.visited |= (1u << i);
        child.d_dst = d_new;
        child.reached = (d_new <= p.r_hop_km);
        // top-2 insertion; strict-< keeps the earlier-generated child on exact ties
        if (!has_a || corridorStateBetter(child, child_a)) {
          child_b = child_a; has_b = has_a;
          child_a = child; has_a = true;
        } else if (!has_b || corridorStateBetter(child, child_b)) {
          child_b = child; has_b = true;
        }
      }
    }
    if (!has_a) break;  // dead end everywhere → best_unreached stands
    beam[0] = child_a;
    n_beam = 1;
    if (has_b) { beam[1] = child_b; n_beam = 2; }
    for (uint8_t s = 0; s < n_beam; s++) {
      if (!beam[s].reached && corridorUnreachedBetter(beam[s], best_unreached)) best_unreached = beam[s];
    }
    if (beam[0].reached) {  // earliest reach wins (greedy parity)
      memcpy(out_bead, beam[0].bead, beam[0].n_beads);
      out_reached = true;
      return beam[0].n_beads;
    }
  }
  memcpy(out_bead, best_unreached.bead, best_unreached.n_beads);
  return best_unreached.n_beads;
}

// --- Capsule width pass ---
//
// Per segment i (bead i → bead i+1): smallest code in [floor..ceiling] whose
// CAPSULE (both endpoint radii = CORRIDOR_RADIUS_KM[code]) covers >= n_target
// density candidates — the same pointInCorridorSegment() geometry the
// forwarding check uses.  None → ceiling + reason 0x08.  Bead code = max of
// its adjacent segment codes, so every capsule's smaller endpoint radius is
// still >= its segment code → coverage holds by construction.
static inline void corridorWidthPassCapsule(CorridorProposal& out,
                                            const CorridorCandidate* dens, uint8_t n_dens,
                                            const CorridorGenParams& p) {
  if (out.count == 0) return;
  if (out.count == 1) {  // single bead: plain circle test
    uint8_t code = p.width_ceiling_code;
    bool covered = false;
    for (uint8_t c = p.width_floor_code; c <= p.width_ceiling_code && !covered; c++) {
      uint8_t inside = 0;
      for (uint8_t i = 0; i < n_dens; i++) {
        if (corridorDistKm(out.lats[0], out.lons[0], dens[i].lat, dens[i].lon) <= CORRIDOR_RADIUS_KM[c]) inside++;
      }
      if (inside >= p.n_target) { code = c; covered = true; }
    }
    if (!covered) out.reason |= 0x08;
    out.radius_codes[0] = code;
    return;
  }
  uint8_t seg[CORRIDOR_MAX_INTERMEDIATE_BEADS + 1];  // count-1 segments, count <= 8
  for (uint8_t i = 0; i + 1 < out.count; i++) {
    uint8_t code = p.width_ceiling_code;
    bool covered = false;
    for (uint8_t c = p.width_floor_code; c <= p.width_ceiling_code && !covered; c++) {
      float r = CORRIDOR_RADIUS_KM[c];
      uint8_t inside = 0;
      for (uint8_t k = 0; k < n_dens; k++) {
        if (pointInCorridorSegment(dens[k].lat, dens[k].lon,
                                   out.lats[i], out.lons[i], r,
                                   out.lats[i + 1], out.lons[i + 1], r)) inside++;
      }
      if (inside >= p.n_target) { code = c; covered = true; }
    }
    if (!covered) out.reason |= 0x08;  // thin stretch
    seg[i] = code;
  }
  out.radius_codes[0] = seg[0];
  for (uint8_t b = 1; b + 1 < out.count; b++) out.radius_codes[b] = (seg[b - 1] > seg[b]) ? seg[b - 1] : seg[b];
  out.radius_codes[out.count - 1] = seg[out.count - 2];
}

// Build a proposal from src to dst.  The Tier-1 beam walk bends through `wp`
// (waypoint candidates: fresh fixed infrastructure); the width pass sizes the
// beads from `dens` (density candidates: any positioned contact).  Passing
// the same list for both reproduces the pre-split behaviour.  Returns
// out.count.
static inline uint8_t proposeCorridor(float src_lat, float src_lon,
                                      float dst_lat, float dst_lon,
                                      const CorridorCandidate* wp, uint8_t n_wp,
                                      const CorridorCandidate* dens, uint8_t n_dens,
                                      const CorridorGenParams& p, CorridorProposal& out) {
  out.count = 0;
  out.reason = 0;

  uint8_t beads[CORRIDOR_MAX_INTERMEDIATE_BEADS];
  bool reached;
  uint8_t n_beads = corridorBeamWalk(src_lat, src_lon, dst_lat, dst_lon, wp, n_wp, p, beads, reached);

  if (n_beads == 0) {
    // Walk contributed nothing → Tier-0 straight diamond (unchanged emit).
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

  out.lats[0] = src_lat;
  out.lons[0] = src_lon;
  for (uint8_t b = 0; b < n_beads; b++) {
    out.lats[1 + b] = wp[beads[b]].lat;
    out.lons[1 + b] = wp[beads[b]].lon;
  }
  out.lats[1 + n_beads] = dst_lat;
  out.lons[1 + n_beads] = dst_lon;
  out.count = 2 + n_beads;
  out.reason |= (reached ? 2 : 1) | (reached ? 0 : 0x80);

  corridorWidthPassCapsule(out, dens, n_dens, p);
  return out.count;
}
