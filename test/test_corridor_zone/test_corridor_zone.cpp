#include <gtest/gtest.h>
#include "Packet.h"
#include <helpers/CorridorCheck.h>

using namespace mesh;

// Destination-zone (DZ) semantics of the corridor extension word and the
// reply-reversal helper.  Geometry: a corridor of three 8 km circles from
// Munich-ish (48.10, 11.40) up north-east to (48.60, 11.90).

static const float LAT0 = 48.10f, LON0 = 11.40f;
static const float LAT2 = 48.60f, LON2 = 11.90f;
static const float R_KM = 8.0f;

static void makeTestCorridor(CorridorTriple* t, int count) {
    for (int i = 0; i < count; i++) {
        float f = count > 1 ? (float)i / (float)(count - 1) : 0.0f;
        t[i].lat = LAT0 + f * (LAT2 - LAT0);
        t[i].lon = LON0 + f * (LON2 - LON0);
        t[i].radius_km = R_KM;
    }
}

static Packet makeZonePacket(uint8_t n, uint8_t flags) {
    Packet p;
    p.header = ROUTE_TYPE_TRANSPORT_FLOOD | (PAYLOAD_TYPE_GRP_TXT << PH_TYPE_SHIFT);
    p.path_len = 0;
    p.payload[0] = 0x11;
    p.payload_len = 1;
    CorridorTriple t[MAX_CORRIDOR_TRIPLES];
    makeTestCorridor(t, n);
    fillCorridor(&p, t, n, flags);
    p.transport_codes[0] = 0x1234;
    return p;
}

TEST(CorridorZone, WholeCorridorIsDestinationZone) {
    CorridorTriple t[3];
    makeTestCorridor(t, 3);
    // DZ=0: every in-corridor point is in the destination zone
    EXPECT_TRUE(isPointInDestinationZone(LAT0 + 0.01f, LON0, t, 3, false));            // first circle
    EXPECT_TRUE(isPointInDestinationZone((LAT0 + LAT2) / 2, (LON0 + LON2) / 2, t, 3, false));  // mid capsule
    EXPECT_TRUE(isPointInDestinationZone(LAT2 + 0.01f, LON2, t, 3, false));            // last circle
    EXPECT_FALSE(isPointInDestinationZone(50.0f, 13.0f, t, 3, false));                 // far outside
}

TEST(CorridorZone, LastTripleIsDestinationZone) {
    CorridorTriple t[3];
    makeTestCorridor(t, 3);
    // DZ=1: only the last circle counts — transport section is NOT the zone
    EXPECT_TRUE(isPointInDestinationZone(LAT2 + 0.01f, LON2, t, 3, true));             // last circle
    EXPECT_FALSE(isPointInDestinationZone(LAT0 + 0.01f, LON0, t, 3, true));            // first circle = transport
    EXPECT_FALSE(isPointInDestinationZone((LAT0 + LAT2) / 2, (LON0 + LON2) / 2, t, 3, true));  // mid capsule
}

TEST(CorridorZone, UnlimitedLastRadiusAlwaysInside) {
    CorridorTriple t[2];
    makeTestCorridor(t, 2);
    t[1].radius_km = CORRIDOR_RADIUS_UNLIMITED_KM;
    EXPECT_TRUE(isPointInDestinationZone(60.0f, 20.0f, t, 2, true));   // radius code 15 = unlimited
}

TEST(CorridorZone, SingleTripleDZDegenerates) {
    CorridorTriple t[1];
    makeTestCorridor(t, 1);
    // N=1: the transport section is empty, DZ=1 equals DZ=0
    EXPECT_EQ(isPointInDestinationZone(LAT0 + 0.01f, LON0, t, 1, false),
              isPointInDestinationZone(LAT0 + 0.01f, LON0, t, 1, true));
    EXPECT_EQ(isPointInDestinationZone(50.0f, 13.0f, t, 1, false),
              isPointInDestinationZone(50.0f, 13.0f, t, 1, true));
}

TEST(CorridorZone, NeverSuppressMatrix) {
    // DZ=0: never suppress anywhere in (or regardless of knowledge of) the corridor
    Packet whole = makeZonePacket(3, 0);
    EXPECT_TRUE(corridorNeverSuppress(&whole, LAT0 + 0.01f, LON0));       // inside
    EXPECT_TRUE(corridorNeverSuppress(&whole, 50.0f, 13.0f));             // outside (can't know → exempt)
    EXPECT_TRUE(corridorNeverSuppress(&whole, 0.0f, 0.0f));               // position unknown → exempt

    // DZ=1: exempt only with known position inside the last circle
    Packet transport = makeZonePacket(3, CORRIDOR_FLAG_DZ);
    EXPECT_TRUE(corridorNeverSuppress(&transport, LAT2 + 0.01f, LON2));   // destination zone
    EXPECT_FALSE(corridorNeverSuppress(&transport, LAT0 + 0.01f, LON0));  // transport section
    EXPECT_FALSE(corridorNeverSuppress(&transport, 0.0f, 0.0f));          // position unknown = transport

    // non-corridor packets are never exempt via this helper
    Packet plain;
    plain.header = ROUTE_TYPE_TRANSPORT_FLOOD | (PAYLOAD_TYPE_GRP_TXT << PH_TYPE_SHIFT);
    plain.path_len = 0;
    plain.payload[0] = 0x11;
    plain.payload_len = 1;
    plain.transport_codes[0] = 0x1234;
    plain.transport_codes[1] = 0;
    EXPECT_FALSE(corridorNeverSuppress(&plain, LAT0, LON0));

    // foreign code_2 extension: opaque, not a corridor
    Packet foreign = plain;
    foreign.transport_codes[1] = 0x5678;
    EXPECT_FALSE(corridorNeverSuppress(&foreign, LAT0, LON0));
}

TEST(CorridorZone, ReverseSwapsDestinationEnd) {
    CorridorTriple t[3], r[3];
    makeTestCorridor(t, 3);
    memcpy(r, t, sizeof(t));
    reverseCorridor(r, 3);
    EXPECT_NE(r[0].lat, t[0].lat);
    EXPECT_EQ(r[0].lat, t[2].lat);

    // capsule geometry is reversal-symmetric: the pipe is identical
    const float samples[][2] = {
        { LAT0 + 0.01f, LON0 },                       // first circle
        { LAT2 + 0.01f, LON2 },                       // last circle
        { (LAT0 + LAT2) / 2, (LON0 + LON2) / 2 },     // mid capsule
        { 50.0f, 13.0f },                              // outside
    };
    for (const auto& s : samples) {
        EXPECT_EQ(isPointInCorridor(s[0], s[1], t, 3), isPointInCorridor(s[0], s[1], r, 3))
            << "sample lat=" << s[0];
    }

    // the destination zone (DZ=1) swaps ends: the original first circle
    EXPECT_FALSE(isPointInDestinationZone(LAT0 + 0.01f, LON0, t, 3, true));   // before: transport
    EXPECT_TRUE(isPointInDestinationZone(LAT0 + 0.01f, LON0, r, 3, true));    // after: destination
    EXPECT_TRUE(isPointInDestinationZone(LAT2 + 0.01f, LON2, t, 3, true));    // before: destination
    EXPECT_FALSE(isPointInDestinationZone(LAT2 + 0.01f, LON2, r, 3, true));   // after: transport
}

TEST(CorridorZone, FillCorridorEncodesFlags) {
    Packet p = makeZonePacket(4, CORRIDOR_FLAG_AU | CORRIDOR_FLAG_DZ);
    EXPECT_TRUE(p.hasCorridorExt());
    EXPECT_EQ(0, p.getCorridorVer());
    EXPECT_EQ(4, p.getCorridorCount());
    EXPECT_FALSE(p.isCorridorFailClosed());
    EXPECT_TRUE(p.isCorridorAuto());
    EXPECT_TRUE(p.isCorridorDestLastTriple());
    EXPECT_EQ(makeCorridorHeader(4, CORRIDOR_FLAG_AU | CORRIDOR_FLAG_DZ), p.transport_codes[1]);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
