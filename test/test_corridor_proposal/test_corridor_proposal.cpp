#include <gtest/gtest.h>
#include "helpers/CorridorPropose.h"

// On-node corridor proposal generator (helpers/CorridorPropose.h).
// reason bits: 0-2 tier (0 = diamond, 1 = advert-bent, 2 = full chain),
// 0x08 thin stretch, 0x80 horizon-limited.
//
// proposeCorridor() takes separate waypoint (walk) and density (width pass)
// candidate lists; the single-list run() overload below reproduces the
// pre-split behaviour.  Freshness/type filtering of the two lists happens in
// MyMesh::buildCorridorProposal (needs contacts + RTC — covered by the mcsim
// E2E corridor scenarios, not by these pure-geometry tests).

static CorridorCandidate makeCand(float lat, float lon) {
    CorridorCandidate c;
    c.lat = lat;
    c.lon = lon;
    return c;
}

static CorridorProposal run(const CorridorCandidate* wp, uint8_t n_wp,
                            const CorridorCandidate* dens, uint8_t n_dens,
                            float src_lat, float src_lon,
                            float dst_lat, float dst_lon,
                            const CorridorGenParams& p = defaultCorridorGenParams()) {
    CorridorProposal out;
    proposeCorridor(src_lat, src_lon, dst_lat, dst_lon, wp, n_wp, dens, n_dens, p, out);
    return out;
}

// Compat overload: the same list serves as waypoints and density candidates.
static CorridorProposal run(const CorridorCandidate* cand, uint8_t n,
                            float src_lat, float src_lon,
                            float dst_lat, float dst_lon,
                            const CorridorGenParams& p = defaultCorridorGenParams()) {
    return run(cand, n, cand, n, src_lat, src_lon, dst_lat, dst_lon, p);
}

// No candidates → straight 3-bead diamond A-M-B with codes {4, 6, 4}.
TEST(CorridorProposal, ZeroCandidatesGivesTier0Diamond) {
    CorridorProposal out = run(nullptr, 0, 47.6100f, -122.4000f, 47.6600f, -122.3500f);
    ASSERT_EQ(3u, out.count);
    EXPECT_EQ(0, out.reason);
    EXPECT_FLOAT_EQ(47.6100f, out.lats[0]);
    EXPECT_FLOAT_EQ(-122.4000f, out.lons[0]);
    EXPECT_NEAR(47.6350f, out.lats[1], 1e-4);   // M = midpoint
    EXPECT_NEAR(-122.3750f, out.lons[1], 1e-4);
    EXPECT_FLOAT_EQ(47.6600f, out.lats[2]);
    EXPECT_FLOAT_EQ(-122.3500f, out.lons[2]);
    EXPECT_EQ(4, out.radius_codes[0]);
    EXPECT_EQ(6, out.radius_codes[1]);
    EXPECT_EQ(4, out.radius_codes[2]);
}

// src and dst < 2 km apart → collapses to a single midpoint bead.
TEST(CorridorProposal, Tier0NearbyTargetCollapsesToOneBead) {
    CorridorProposal out = run(nullptr, 0, 47.6100f, -122.4000f, 47.6150f, -122.4000f);
    ASSERT_EQ(1u, out.count);
    EXPECT_EQ(0, out.reason);
    EXPECT_NEAR(47.6125f, out.lats[0], 1e-4);
    EXPECT_FLOAT_EQ(-122.4000f, out.lons[0]);
    EXPECT_EQ(6, out.radius_codes[0]);
}

// Candidates off the straight line pull the beads onto their positions (tier 1).
TEST(CorridorProposal, GreedyBendsTowardRepeaters) {
    CorridorCandidate cand[5] = {
        makeCand(47.3000f, -122.4300f), makeCand(47.4000f, -122.4150f),
        makeCand(47.5000f, -122.4000f), makeCand(47.6000f, -122.3850f),
        makeCand(47.7000f, -122.3700f),
    };
    CorridorProposal out = run(cand, 5, 47.2000f, -122.4000f, 48.0000f, -122.4000f);
    ASSERT_EQ(7u, out.count);   // src + 5 candidates + dst (dst out of r_hop reach)
    EXPECT_EQ(1, out.reason & 0x03);
    EXPECT_FLOAT_EQ(47.2000f, out.lats[0]);
    bool bent = false;
    for (int i = 1; i <= 5; i++) {   // intermediate beads sit exactly on candidates
        EXPECT_FLOAT_EQ(cand[i - 1].lat, out.lats[i]);
        EXPECT_FLOAT_EQ(cand[i - 1].lon, out.lons[i]);
        if (out.lons[i] != -122.4000f) bent = true;   // ... which are off the direct line
    }
    EXPECT_TRUE(bent);
    EXPECT_FLOAT_EQ(48.0000f, out.lats[6]);
}

// Evenly spaced candidates let the walk reach the target → tier 2, dst last.
TEST(CorridorProposal, FullChainReachesTarget) {
    CorridorCandidate cand[5] = {
        makeCand(47.3f, -122.4f), makeCand(47.4f, -122.4f), makeCand(47.5f, -122.4f),
        makeCand(47.6f, -122.4f), makeCand(47.7f, -122.4f),
    };
    CorridorProposal out = run(cand, 5, 47.2000f, -122.4000f, 47.8000f, -122.4000f);
    ASSERT_EQ(7u, out.count);
    EXPECT_EQ(2, out.reason & 0x03);
    EXPECT_EQ(0, out.reason & 0x80);
    EXPECT_NEAR(47.8000f, out.lats[out.count - 1], 1e-4);
    EXPECT_NEAR(-122.4000f, out.lons[out.count - 1], 1e-4);
}

// A chain that dead-ends before the target sets the horizon flag 0x80.
TEST(CorridorProposal, UnreachableTargetSetsHorizonFlag) {
    CorridorCandidate cand[2] = {
        makeCand(47.2800f, -122.4000f),   // reachable from src
        makeCand(47.2950f, -122.4000f),   // slightly further, closer to dst
    };
    CorridorProposal out = run(cand, 2, 47.2000f, -122.4000f, 48.1000f, -122.4000f);
    EXPECT_EQ(1, out.reason & 0x03);
    EXPECT_NE(0, out.reason & 0x80);   // dst ~100 km away, no candidate near it
    ASSERT_EQ(3u, out.count);          // src + best candidate + dst
    EXPECT_FLOAT_EQ(47.2950f, out.lats[1]);
}

// A dense field forcing a long chain never exceeds 8 beads, dst always present.
TEST(CorridorProposal, NeverExceedsEightBeads) {
    CorridorCandidate cand[15];
    for (int i = 0; i < 15; i++) cand[i] = makeCand(47.25f + 0.05f * i, -122.4f);
    CorridorProposal out = run(cand, 15, 47.2000f, -122.4000f, 48.0000f, -122.4000f);
    EXPECT_LE(out.count, 8u);
    EXPECT_GE(out.count, 2u);
    EXPECT_FLOAT_EQ(48.0000f, out.lats[out.count - 1]);
    EXPECT_FLOAT_EQ(-122.4000f, out.lons[out.count - 1]);
}

// Width pass (capsule-based): sparse → ceiling code + 0x08; a cluster ≥ n_target
// → floor code.  The pair straddles both segments' capsules, so with capsules
// every bead narrows to the floor code (the old circle pass only narrowed the
// bead sitting on the cluster).
TEST(CorridorProposal, WidthFollowsDensity) {
    CorridorGenParams p = defaultCorridorGenParams();

    CorridorCandidate sparse[1] = { makeCand(47.3000f, -122.4000f) };
    CorridorProposal out = run(sparse, 1, 47.2000f, -122.4000f, 47.5000f, -122.4000f, p);
    ASSERT_EQ(3u, out.count);
    EXPECT_NE(0, out.reason & 0x08);
    for (uint8_t b = 0; b < out.count; b++) EXPECT_EQ(7, out.radius_codes[b]);

    CorridorCandidate cluster[2] = {
        makeCand(47.3000f, -122.4000f),
        makeCand(47.3020f, -122.4000f),   // 0.25 km next to the first
    };
    out = run(cluster, 2, 47.2000f, -122.4000f, 47.5000f, -122.4000f, p);
    ASSERT_EQ(3u, out.count);
    for (uint8_t b = 0; b < out.count; b++) EXPECT_EQ(3, out.radius_codes[b]);
    EXPECT_EQ(0, out.reason & 0x08);   // both capsules cover the pair at the floor code
}

// Same inputs → byte-identical proposals.
TEST(CorridorProposal, DeterministicAcrossCalls) {
    CorridorCandidate cand[5] = {
        makeCand(47.3000f, -122.4300f), makeCand(47.4000f, -122.4150f),
        makeCand(47.5000f, -122.4000f), makeCand(47.6000f, -122.3850f),
        makeCand(47.7000f, -122.3700f),
    };
    CorridorProposal a = run(cand, 5, 47.2000f, -122.4000f, 48.0000f, -122.4000f);
    CorridorProposal b = run(cand, 5, 47.2000f, -122.4000f, 48.0000f, -122.4000f);
    ASSERT_EQ(a.count, b.count);
    EXPECT_EQ(0, memcmp(a.lats, b.lats, a.count * sizeof(float)));
    EXPECT_EQ(0, memcmp(a.lons, b.lons, a.count * sizeof(float)));
    EXPECT_EQ(0, memcmp(a.radius_codes, b.radius_codes, a.count));
    EXPECT_EQ(a.reason, b.reason);
}

// D6.1 invariant: distance to the target never increases along the chain.
TEST(CorridorProposal, WaypointsOrderedAlongPath) {
    CorridorCandidate cand[5] = {
        makeCand(47.3000f, -122.4300f), makeCand(47.4000f, -122.4150f),
        makeCand(47.5000f, -122.4000f), makeCand(47.6000f, -122.3850f),
        makeCand(47.7000f, -122.3700f),
    };
    const float dst_lat = 48.0000f, dst_lon = -122.4000f;
    CorridorProposal out = run(cand, 5, 47.2000f, -122.4000f, dst_lat, dst_lon);
    ASSERT_GT(out.count, 1u);
    for (uint8_t i = 1; i < out.count; i++) {
        float prev = corridorDistKm(out.lats[i - 1], out.lons[i - 1], dst_lat, dst_lon);
        float curr = corridorDistKm(out.lats[i], out.lons[i], dst_lat, dst_lon);
        EXPECT_LE(curr, prev + 1e-3f) << "bead " << (int)i;
    }
}

// All codes are valid table indices; Tier-0 pins the exact {4, 6, 4} pattern.
TEST(CorridorProposal, RadiusCodesWithinBounds) {
    CorridorCandidate cand[5] = {
        makeCand(47.3f, -122.4f), makeCand(47.4f, -122.4f), makeCand(47.5f, -122.4f),
        makeCand(47.6f, -122.4f), makeCand(47.7f, -122.4f),
    };
    CorridorProposal out = run(cand, 5, 47.2000f, -122.4000f, 47.8000f, -122.4000f);
    for (uint8_t b = 0; b < out.count; b++) EXPECT_LE(out.radius_codes[b], 15);

    out = run(nullptr, 0, 47.6100f, -122.4000f, 47.6600f, -122.3500f);
    ASSERT_EQ(3u, out.count);
    EXPECT_EQ(4, out.radius_codes[0]);
    EXPECT_EQ(6, out.radius_codes[1]);
    EXPECT_EQ(4, out.radius_codes[2]);
}

// --- selectCorridorCandidates ---

// Candidates outside the src→dst ellipse (detour_factor 1.4 + r_hop) are
// dropped even when they would fit the 32-slot output.
TEST(CorridorProposal, SelectCorridorCandidatesEllipseFilter) {
    CorridorCandidate all[3] = {
        makeCand(47.4000f, -122.4000f),   // on-axis
        makeCand(47.4000f, -122.0500f),   // ~26 km east: inside the default ellipse
        makeCand(47.4000f, -121.9000f),   // ~38 km east: outside (sum ~87 km > 1.4*44.4+12)
    };
    CorridorCandidate out[3];
    uint8_t out_idx[3];
    uint8_t n = selectCorridorCandidates(47.2000f, -122.4000f, 47.6000f, -122.4000f,
                                         all, 3, out, out_idx, 3, defaultCorridorGenParams());
    ASSERT_EQ(2u, n);
    EXPECT_FLOAT_EQ(all[0].lon, out[0].lon);
    EXPECT_FLOAT_EQ(all[1].lon, out[1].lon);
    EXPECT_EQ(0, out_idx[0]);
    EXPECT_EQ(1, out_idx[1]);
}

// 40 relevant candidates, max_out 32: output capped, ascending by detour
// overhead, out_idx maps back to the input.
TEST(CorridorProposal, SelectCorridorCandidatesCapAndOrder) {
    CorridorCandidate all[40];
    for (int i = 0; i < 40; i++) all[i] = makeCand(47.4000f, -122.4000f + 0.001f * i);
    CorridorCandidate out[32];
    uint8_t out_idx[32];
    uint8_t n = selectCorridorCandidates(47.2000f, -122.4000f, 47.6000f, -122.4000f,
                                         all, 40, out, out_idx, 32, defaultCorridorGenParams());
    ASSERT_EQ(32u, n);
    for (uint8_t k = 0; k < n; k++) {
        EXPECT_EQ(k, out_idx[k]);                          // detour grows with i
        EXPECT_FLOAT_EQ(all[k].lon, out[k].lon);
        if (k > 0) EXPECT_LE(out[k - 1].lon, out[k].lon);  // ascending relevance
    }
}

// detour_factor sharpens the ellipse: a mid-band candidate survives 1.4 but
// not 1.1.
TEST(CorridorProposal, SelectCorridorCandidatesDetourFactor) {
    CorridorCandidate all[1] = { makeCand(47.4000f, -122.1000f) };   // sum ~63 km
    CorridorCandidate out[1];
    uint8_t out_idx[1];

    CorridorGenParams p = defaultCorridorGenParams();
    EXPECT_EQ(1u, selectCorridorCandidates(47.2000f, -122.4000f, 47.6000f, -122.4000f,
                                           all, 1, out, out_idx, 1, p));
    p.detour_factor = 1.1f;   // limit 1.1*44.4+12 = 60.8 km < 63 km
    EXPECT_EQ(0u, selectCorridorCandidates(47.2000f, -122.4000f, 47.6000f, -122.4000f,
                                           all, 1, out, out_idx, 1, p));
}

// --- beam walk (V4a) ---

// The greedy-best first hop leads into a pocket with no onward progress; the
// beam keeps the second-best state alive and reaches the target around it.
// (The pure greedy walk dead-ends here → tier 1 + 0x80.)
TEST(CorridorProposal, BeamRecoversFromGreedyDeadEnd) {
    CorridorCandidate cand[4] = {
        makeCand(47.0991f, -122.3602f),   // G: pocket, best first-hop progress (~27.2 km to dst)
        makeCand(47.0901f, -122.4530f),   // W:  west rim, slightly worse (~28.4 km)
        makeCand(47.1802f, -122.4928f),   // W2: west rim, next hop
        makeCand(47.2703f, -122.4530f),   // W3: west rim, reaches dst
    };
    CorridorProposal out = run(cand, 4, nullptr, 0, 47.0000f, -122.4000f, 47.3423f, -122.4000f);
    ASSERT_EQ(5u, out.count);   // src + W + W2 + W3 + dst
    EXPECT_EQ(2, out.reason & 0x03);
    EXPECT_EQ(0, out.reason & 0x80);
    EXPECT_FLOAT_EQ(cand[1].lat, out.lats[1]);
    EXPECT_FLOAT_EQ(cand[2].lat, out.lats[2]);
    EXPECT_FLOAT_EQ(cand[3].lat, out.lats[3]);
    EXPECT_NEAR(47.3423f, out.lats[4], 1e-4);
}

// Exactly tied branches resolve to the lower candidate index, identically on
// every call.
TEST(CorridorProposal, BeamDeterministicWithBranching) {
    CorridorCandidate cand[2] = {
        makeCand(47.2000f, -122.3900f),   // east of the axis
        makeCand(47.2000f, -122.4100f),   // west of the axis — exact float tie
    };
    CorridorProposal a = run(cand, 2, nullptr, 0, 47.1000f, -122.4000f, 47.5000f, -122.4000f);
    ASSERT_EQ(3u, a.count);
    EXPECT_FLOAT_EQ(cand[0].lat, a.lats[1]);   // lower index wins the tie
    EXPECT_FLOAT_EQ(cand[0].lon, a.lons[1]);

    CorridorProposal b = run(cand, 2, nullptr, 0, 47.1000f, -122.4000f, 47.5000f, -122.4000f);
    ASSERT_EQ(a.count, b.count);
    EXPECT_EQ(0, memcmp(a.lats, b.lats, a.count * sizeof(float)));
    EXPECT_EQ(0, memcmp(a.lons, b.lons, a.count * sizeof(float)));
    EXPECT_EQ(a.reason, b.reason);
}

// --- capsule width pass (V4b) ---

// A density pair straddling a segment midpoint lies outside both endpoint
// floor-radius CIRCLES but inside the floor-radius CAPSULE — the capsule pass
// (the geometry relays actually check) narrows the segment to code 3 where
// the old circle pass returned the ceiling.
TEST(CorridorProposal, CapsuleWidthMatchesForwardCheck) {
    CorridorCandidate wp[1] = { makeCand(47.3000f, -122.4000f) };   // midpoint bead
    CorridorCandidate dens[2] = {
        makeCand(47.2500f, -122.3800f),   // 1.5 km east of the first segment's midpoint
        makeCand(47.2500f, -122.4200f),   // 1.5 km west of it
    };
    CorridorProposal out = run(wp, 1, dens, 2, 47.2000f, -122.4000f, 47.4000f, -122.4000f);
    ASSERT_EQ(3u, out.count);   // src + midpoint bead + dst (bead reaches dst)
    EXPECT_EQ(0, out.reason & 0x08);
    EXPECT_EQ(3, out.radius_codes[0]);                   // first capsule covers the pair at 5 km
    EXPECT_EQ(4, out.radius_codes[1]);                   // max(seg0=3, seg1=4)
    EXPECT_EQ(4, out.radius_codes[2]);                   // second capsule needs 8 km
}

// The walk depends only on wp, the width pass only on dens: same waypoints,
// empty density → identical beads, ceiling codes + 0x08; clustered density →
// identical beads, narrower codes.
TEST(CorridorProposal, WaypointAndDensitySeparation) {
    CorridorCandidate wp[5] = {
        makeCand(47.3f, -122.4f), makeCand(47.4f, -122.4f), makeCand(47.5f, -122.4f),
        makeCand(47.6f, -122.4f), makeCand(47.7f, -122.4f),
    };
    CorridorCandidate dens[2] = {
        makeCand(47.4990f, -122.4000f),
        makeCand(47.5010f, -122.4000f),   // pair at the middle of the chain
    };
    CorridorProposal thin = run(wp, 5, nullptr, 0, 47.2000f, -122.4000f, 47.8000f, -122.4000f);
    CorridorProposal wide = run(wp, 5, dens, 2, 47.2000f, -122.4000f, 47.8000f, -122.4000f);
    ASSERT_EQ(thin.count, wide.count);
    ASSERT_EQ(7u, thin.count);
    EXPECT_EQ(0, memcmp(thin.lats, wide.lats, thin.count * sizeof(float)));
    EXPECT_EQ(0, memcmp(thin.lons, wide.lons, thin.count * sizeof(float)));
    EXPECT_NE(0, thin.reason & 0x08);
    for (uint8_t b = 0; b < thin.count; b++) EXPECT_EQ(7, thin.radius_codes[b]);
    bool narrowed = false;
    for (uint8_t b = 0; b < wide.count; b++) {
        if (wide.radius_codes[b] < 7) narrowed = true;
    }
    EXPECT_TRUE(narrowed);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
