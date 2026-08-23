#include <gtest/gtest.h>
#include "helpers/CorridorPropose.h"

// On-node corridor proposal generator (helpers/CorridorPropose.h).
// reason bits: 0-2 tier (0 = diamond, 1 = advert-bent, 2 = full chain),
// 0x08 thin stretch, 0x80 horizon-limited.

static CorridorCandidate makeCand(float lat, float lon) {
    CorridorCandidate c;
    c.lat = lat;
    c.lon = lon;
    return c;
}

static CorridorProposal run(const CorridorCandidate* cand, uint8_t n,
                            float src_lat, float src_lon,
                            float dst_lat, float dst_lon,
                            const CorridorGenParams& p = defaultCorridorGenParams()) {
    CorridorProposal out;
    proposeCorridor(src_lat, src_lon, dst_lat, dst_lon, cand, n, p, out);
    return out;
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

// D9 width pass: sparse → ceiling code + 0x08; a cluster ≥ n_target → floor code.
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
    bool has_floor = false;
    for (uint8_t b = 0; b < out.count; b++) {
        if (out.radius_codes[b] == 3) has_floor = true;
    }
    EXPECT_TRUE(has_floor);   // the bead sitting on the cluster widens only to 5 km
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

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
