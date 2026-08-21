#include <gtest/gtest.h>
#include "Packet.h"

using namespace mesh;

// Flood Corridor wire layout for transport-coded packets (Packet::writeTo):
//   [header(1)][code_1(2)][code_2(2)][path_len(1)][path][corridor N*4][payload]
// code_2 bits 15-12 carry the corridor triple count N (0..15 on the wire,
// but corridor[] only holds MAX_CORRIDOR_TRIPLES triples).

static Packet makeCorridorPacket(uint8_t n_triples, uint8_t payload_len = 1) {
    Packet p;
    p.header = ROUTE_TYPE_TRANSPORT_FLOOD | (PAYLOAD_TYPE_ACK << PH_TYPE_SHIFT);
    p.path_len = 0;
    for (uint8_t b = 0; b < payload_len; b++) p.payload[b] = 0xA5;
    p.payload_len = payload_len;
    uint8_t n = n_triples > MAX_CORRIDOR_TRIPLES ? MAX_CORRIDOR_TRIPLES : n_triples;
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t b = 0; b < CORRIDOR_TRIPLE_BYTES; b++) {
            p.corridor[i * CORRIDOR_TRIPLE_BYTES + b] = 0x10 * i + b;   // recognizable pattern
        }
    }
    p.transport_codes[0] = 0x1234;
    p.transport_codes[1] = (uint16_t)((uint16_t)n_triples << 12);
    return p;
}

// getCorridorByteLen must never exceed the fixed corridor[] buffer, whatever
// code_2 says (regression: N in 9..15 used to yield 36..60 bytes against the
// 32-byte buffer, overflowing the memcpy in Dispatcher/Packet::readFrom).
TEST(PacketCorridor, ByteLenClampedToBuffer) {
    Packet p = makeCorridorPacket(0);
    for (uint8_t n = 0; n <= MAX_CORRIDOR_TRIPLES; n++) {
        p.transport_codes[1] = (uint16_t)((uint16_t)n << 12);
        EXPECT_FALSE(p.hasOversizedCorridor()) << "n=" << (int)n;
        EXPECT_EQ(n * CORRIDOR_TRIPLE_BYTES, p.getCorridorByteLen()) << "n=" << (int)n;
    }
    for (uint8_t n = MAX_CORRIDOR_TRIPLES + 1; n <= 15; n++) {
        p.transport_codes[1] = (uint16_t)((uint16_t)n << 12);
        EXPECT_TRUE(p.hasOversizedCorridor()) << "n=" << (int)n;
        EXPECT_EQ(0u, p.getCorridorByteLen()) << "n=" << (int)n;   // clamped: cannot overflow corridor[]
    }
    // without transport codes the nibble is meaningless
    p.header = ROUTE_TYPE_FLOOD | (PAYLOAD_TYPE_ACK << PH_TYPE_SHIFT);
    p.transport_codes[1] = (uint16_t)(15u << 12);
    EXPECT_FALSE(p.hasOversizedCorridor());
    EXPECT_EQ(0u, p.getCorridorByteLen());
}

// readFrom must reject a serialized packet whose code_2 advertises more
// triples than corridor[] holds — crafted on the wire by tampering byte 4
// (code_2 high byte; bits 15-12 = count).  The payload is long enough that
// the pre-fix length check passed and the memcpy ran past corridor[].
TEST(PacketCorridor, ReadFromRejectsOversizedCount) {
    for (uint8_t tampered = MAX_CORRIDOR_TRIPLES + 1; tampered <= 15; tampered++) {
        Packet p = makeCorridorPacket(MAX_CORRIDOR_TRIPLES, 80);
        uint8_t buf[255];
        uint8_t len = p.writeTo(buf);
        ASSERT_EQ(1 + 4 + 1 + MAX_CORRIDOR_TRIPLES * CORRIDOR_TRIPLE_BYTES + 80, len);
        buf[4] = (uint8_t)((buf[4] & 0x0F) | (tampered << 4));   // code_2 bits 15-12 := tampered
        Packet q;
        EXPECT_FALSE(q.readFrom(buf, len)) << "tampered N=" << (int)tampered;
    }
}

// Valid corridor packets round-trip through writeTo/readFrom unchanged.
TEST(PacketCorridor, RoundTripValidCount) {
    for (uint8_t n = 0; n <= MAX_CORRIDOR_TRIPLES; n++) {
        Packet p = makeCorridorPacket(n);
        uint8_t buf[255];
        uint8_t len = p.writeTo(buf);
        EXPECT_EQ(p.getRawLength(), len) << "n=" << (int)n;

        Packet q;
        ASSERT_TRUE(q.readFrom(buf, len)) << "n=" << (int)n;
        EXPECT_EQ(n, q.getCorridorCount());
        EXPECT_EQ(n * CORRIDOR_TRIPLE_BYTES, q.getCorridorByteLen());
        EXPECT_EQ(0, memcmp(q.corridor, p.corridor, n * CORRIDOR_TRIPLE_BYTES)) << "n=" << (int)n;
        EXPECT_EQ(1u, q.payload_len);
        EXPECT_EQ(0xA5, q.payload[0]);
    }
}

// Non-transport packets (no codes) must stay untouched by the corridor logic.
TEST(PacketCorridor, PlainFloodUnaffected) {
    Packet p;
    p.header = ROUTE_TYPE_FLOOD | (PAYLOAD_TYPE_ACK << PH_TYPE_SHIFT);
    p.path_len = 0;
    p.payload[0] = 0x5A;
    p.payload_len = 1;
    uint8_t buf[255];
    uint8_t len = p.writeTo(buf);
    Packet q;
    ASSERT_TRUE(q.readFrom(buf, len));
    EXPECT_EQ(0u, q.getCorridorByteLen());
    EXPECT_EQ(1u, q.payload_len);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
