#include <gtest/gtest.h>
#include "Packet.h"
#include <helpers/TransportKeyStore.h>
#include <helpers/CorridorCheck.h>

using namespace mesh;

// Flood Corridor wire layout for transport-coded packets (Packet::writeTo):
//   [header(1)][code_1(2)][code_2(2)][path_len(1)][path][corridor N*4][payload]
// code_2 is an extension registry word (little-endian on the wire):
//   bits 15-12 = extension type (0xC = corridor), 11-10 = version (0 = v0),
//   9 = FC, 8 = AU, 7 = DZ, 6-4 reserved, 3-0 = triple count N (0..8).

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
    p.transport_codes[1] = makeCorridorHeader(n_triples);
    return p;
}

// Extension registry word: type nibble, version, policy flags and count must
// round-trip through makeCorridorHeader() and the raw decoders.
TEST(PacketCorridor, RegistryLayoutAndFlags) {
    EXPECT_EQ(0xC006u, makeCorridorHeader(6));
    EXPECT_EQ(0xC206u, makeCorridorHeader(6, CORRIDOR_FLAG_FC));
    EXPECT_EQ(0xC106u, makeCorridorHeader(6, CORRIDOR_FLAG_AU));
    EXPECT_EQ(0xC086u, makeCorridorHeader(6, CORRIDOR_FLAG_DZ));
    EXPECT_EQ(0xC386u, makeCorridorHeader(6, CORRIDOR_FLAG_FC | CORRIDOR_FLAG_AU | CORRIDOR_FLAG_DZ));
    EXPECT_EQ(0xC008u, makeCorridorHeader(200));   // clamped to MAX_CORRIDOR_TRIPLES

    EXPECT_EQ(0xC, getCorridorExtType(0xC386));
    EXPECT_TRUE(isCorridorExt(0xC006));
    EXPECT_FALSE(isCorridorExt(0x0000));
    EXPECT_FALSE(isCorridorExt(0x5678));      // foreign extension
    EXPECT_FALSE(isCorridorExt(0xF123));      // experimental/reserved
    EXPECT_EQ(6, getCorridorCount(0xC386));
    EXPECT_EQ(0, getCorridorCount(0x5678));   // non-corridor ext decodes no count

    uint16_t with_flags = makeCorridorHeader(3, CORRIDOR_FLAG_FC | CORRIDOR_FLAG_DZ);
    EXPECT_EQ(0, getCorridorVer(with_flags));
    EXPECT_TRUE(isCorridorFailClosed(with_flags));
    EXPECT_FALSE(isCorridorAuto(with_flags));
    EXPECT_TRUE(isCorridorDestLastTriple(with_flags));
}

// getCorridorByteLen must never exceed the fixed corridor[] buffer, whatever
// code_2 says (regression: N in 9..15 used to yield 36..60 bytes against the
// 32-byte buffer, overflowing the memcpy in Dispatcher/Packet::readFrom).
TEST(PacketCorridor, ByteLenClampedToBuffer) {
    Packet p = makeCorridorPacket(0);
    for (uint8_t n = 0; n <= MAX_CORRIDOR_TRIPLES; n++) {
        p.transport_codes[1] = makeCorridorHeader(n);
        EXPECT_FALSE(p.hasOversizedCorridor()) << "n=" << (int)n;
        EXPECT_EQ(n * CORRIDOR_TRIPLE_BYTES, p.getCorridorByteLen()) << "n=" << (int)n;
    }
    for (uint8_t n = MAX_CORRIDOR_TRIPLES + 1; n <= 15; n++) {
        p.transport_codes[1] = (uint16_t)(0xC000 | n);   // oversized count in bits 3-0
        EXPECT_TRUE(p.hasOversizedCorridor()) << "n=" << (int)n;
        EXPECT_EQ(0u, p.getCorridorByteLen()) << "n=" << (int)n;   // clamped: cannot overflow corridor[]
    }
    // unknown corridor version: not oversized, but no parseable region either
    p.transport_codes[1] = (uint16_t)(0xC400 | 3);
    EXPECT_FALSE(p.hasOversizedCorridor());
    EXPECT_TRUE(p.hasUnknownCorridorVer());
    EXPECT_EQ(0u, p.getCorridorByteLen());
    // without transport codes the extension word is meaningless
    p.header = ROUTE_TYPE_FLOOD | (PAYLOAD_TYPE_ACK << PH_TYPE_SHIFT);
    p.transport_codes[1] = 0xC00F;
    EXPECT_FALSE(p.hasCorridorExt());
    EXPECT_FALSE(p.hasOversizedCorridor());
    EXPECT_EQ(0u, p.getCorridorByteLen());
}

// readFrom must reject a serialized packet whose code_2 advertises more
// triples than corridor[] holds — crafted on the wire by tampering the low
// nibble of byte 3 (code_2 low byte on little-endian; bits 3-0 = count).
TEST(PacketCorridor, ReadFromRejectsOversizedCount) {
    for (uint8_t tampered = MAX_CORRIDOR_TRIPLES + 1; tampered <= 15; tampered++) {
        Packet p = makeCorridorPacket(MAX_CORRIDOR_TRIPLES, 80);
        uint8_t buf[255];
        uint8_t len = p.writeTo(buf);
        ASSERT_EQ(1 + 4 + 1 + MAX_CORRIDOR_TRIPLES * CORRIDOR_TRIPLE_BYTES + 80, len);
        buf[3] = (uint8_t)((buf[3] & 0xF0) | tampered);   // code_2 bits 3-0 := tampered
        Packet q;
        EXPECT_FALSE(q.readFrom(buf, len)) << "tampered N=" << (int)tampered;
    }
}

// An unknown corridor encoding version must be rejected cleanly — its triple
// size (and with it the payload offset) cannot be determined, so silently
// forwarding or mis-parsing is worse than dropping.
TEST(PacketCorridor, ReadFromRejectsUnknownVersion) {
    for (uint8_t ver = 1; ver <= 3; ver++) {
        Packet p = makeCorridorPacket(3, 40);
        uint8_t buf[255];
        uint8_t len = p.writeTo(buf);
        buf[4] = (uint8_t)((buf[4] & 0xF3) | (ver << 2));   // code_2 bits 11-10 := ver (high byte)
        Packet q;
        EXPECT_FALSE(q.readFrom(buf, len)) << "ver=" << (int)ver;
    }
}

// Foreign code_2 extensions (type nibble != 0xC) must be carried opaquely:
// no corridor bytes are parsed, the payload sits directly after the path,
// and the packet stays valid for normal region/forward processing.
TEST(PacketCorridor, ForeignExtensionIsOpaque) {
    const uint16_t foreign[] = { 0x5678, 0xF123, 0x1006, 0x8000 };
    for (uint16_t code2 : foreign) {
        Packet p;
        p.header = ROUTE_TYPE_TRANSPORT_FLOOD | (PAYLOAD_TYPE_ACK << PH_TYPE_SHIFT);
        p.path_len = 0;
        p.transport_codes[0] = 0x1234;
        p.transport_codes[1] = code2;
        p.payload[0] = 0x5A;
        p.payload[1] = 0xA5;
        p.payload_len = 2;
        uint8_t buf[255];
        uint8_t len = p.writeTo(buf);
        Packet q;
        ASSERT_TRUE(q.readFrom(buf, len)) << "code2=" << std::hex << code2;
        EXPECT_FALSE(q.hasCorridorExt());
        EXPECT_EQ(0u, q.getCorridorByteLen());
        EXPECT_EQ(2u, q.payload_len);
        EXPECT_EQ(0x5A, q.payload[0]);
        EXPECT_EQ(0xA5, q.payload[1]);
    }
}

// Reserved bits 6-4 are ignored tolerantly: the corridor parses normally.
TEST(PacketCorridor, ReservedBitsTolerated) {
    Packet p = makeCorridorPacket(6);
    p.transport_codes[1] = (uint16_t)(0xC016);   // bit 4 set (reserved), N = 6
    uint8_t buf[255];
    uint8_t len = p.writeTo(buf);
    Packet q;
    ASSERT_TRUE(q.readFrom(buf, len));
    EXPECT_TRUE(q.hasCorridorExt());
    EXPECT_EQ(6, q.getCorridorCount());
    EXPECT_EQ(6u * CORRIDOR_TRIPLE_BYTES, q.getCorridorByteLen());
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

// ---------------------------------------------------------------------------
// Backward compatibility ("rolling deployment"):
//
// Corridor-UNAWARE firmware (upstream dev without Flood Corridor) has no
// corridor region — its tryParsePacket() makes everything after the path one
// flat payload, with the corridor bytes at the front, and its
// TransportKey::calcTransportCode() hashes type + that flat view.  The
// corridor-aware implementation hashes type + corridor + payload — the very
// same byte string — so an old repeater configured with the auto-hashtag
// region "corridor" (`region def corridor` + `region allowf corridor`, same
// SHA256("#corridor") key, no firmware update) matches code_1 in findMatch()
// and forwards corridor packets verbatim.  Old firmware never evaluates
// code_2, so the extension registry word is invisible to it.
// ---------------------------------------------------------------------------

// Replicates Dispatcher::tryParsePacket() of corridor-unaware firmware:
// header, transport codes, path — then a single flat payload (corridor
// absorbed at the front, no code_2 interpretation).
static Packet parseCorridorUnaware(const uint8_t* raw, int len) {
    Packet p;
    int i = 0;
    p.header = raw[i++];
    if (p.hasTransportCodes()) {
        memcpy(&p.transport_codes[0], &raw[i], 2); i += 2;
        memcpy(&p.transport_codes[1], &raw[i], 2); i += 2;
    } else {
        p.transport_codes[0] = p.transport_codes[1] = 0;
    }
    p.path_len = raw[i++];
    int path_byte_len = (p.path_len & 63) * p.getPathHashSize();
    memcpy(p.path, &raw[i], path_byte_len); i += path_byte_len;
    p.payload_len = len - i;   // flat: corridor bytes + actual payload
    memcpy(p.payload, &raw[i], p.payload_len);
    return p;
}

static void checkTransportCodeMatchesCorridorUnawareView(const TransportKey& key) {
    for (uint8_t n = 0; n <= MAX_CORRIDOR_TRIPLES; n++) {
        Packet p = makeCorridorPacket(n, 40);
        // sender (corridor-aware): code_1 over the wire view
        uint16_t code1 = key.calcTransportCode(&p);
        EXPECT_NE(0u, code1) << "n=" << (int)n;

        uint8_t buf[255];
        uint8_t len = p.writeTo(buf);

        Packet old = parseCorridorUnaware(buf, len);
        // Old firmware never interprets code_2 — clearing the type nibble
        // keeps getCorridorByteLen() at 0 so calcTransportCode() hashes
        // exactly the old firmware's byte string (type + flat payload).
        old.transport_codes[1] &= 0x0FFF;

        EXPECT_EQ(0u, old.getCorridorByteLen()) << "n=" << (int)n;
        ASSERT_EQ(p.getCorridorByteLen() + p.payload_len, old.payload_len) << "n=" << (int)n;
        uint16_t code_old = key.calcTransportCode(&old);
        EXPECT_EQ(code1, code_old) << "n=" << (int)n;
    }
}

TEST(PacketCorridor, TransportCodeMatchesCorridorUnawareView) {
    checkTransportCodeMatchesCorridorUnawareView(corridorPseudoKey());
}

// The compat path works for ANY region key a sender names in code_1 (e.g.
// "de-by"): an old repeater configured with that region derives the same
// auto-hashtag key ("#de-by") and matches over the identical wire view.
TEST(PacketCorridor, TransportCodeMatchesCorridorUnawareViewWithRegionKey) {
    TransportKeyStore store;
    TransportKey region_key;
    store.getAutoKeyFor(1, "#de-by", region_key);
    ASSERT_FALSE(region_key.isNull());
    checkTransportCodeMatchesCorridorUnawareView(region_key);
}

TEST(PacketCorridor, AutoHashtagRegionKeyMatchesPseudoKey) {
    // An old repeater configured with region "corridor" derives its transport
    // key from the implicit auto-hashtag name "#corridor" (RegionMap::
    // getTransportKeysFor() prefixes '#', TransportKeyStore::getAutoKeyFor()
    // hashes the name) — which must be byte-identical to the corridor
    // pseudo-region key.
    TransportKeyStore store;
    TransportKey region_key;
    store.getAutoKeyFor(1, "#corridor", region_key);
    EXPECT_EQ(0, memcmp(region_key.key, corridorPseudoKey().key, sizeof(region_key.key)));
}

TEST(PacketCorridor, TransportCodeCoversCorridorBytes) {
    // Guard against a silent regression to payload-only hashing: if the
    // corridor bytes dropped out of the HMAC input, the code would no longer
    // match the corridor-unaware view (test above) — old repeaters with the
    // "corridor" region configured would mismatch and drop again.
    Packet a = makeCorridorPacket(3, 40);
    Packet b = makeCorridorPacket(3, 40);
    b.corridor[0] ^= 0xFF;   // same payload, different corridor geometry
    uint16_t ca = corridorPseudoKey().calcTransportCode(&a);
    uint16_t cb = corridorPseudoKey().calcTransportCode(&b);
    EXPECT_NE(ca, cb);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
