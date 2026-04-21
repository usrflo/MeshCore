#pragma once

#include <Mesh.h>

#ifdef ESP32
  #include <FS.h>
#endif

#define MAX_PACKET_HASHES  128
#define MAX_PACKET_ACKS     64
#define MAX_RECENT_REPEATERS  64
#define MAX_ROUTE_HASH_BYTES   3

class SimpleMeshTables : public mesh::MeshTables {
public:
  struct RecentRepeaterInfo {
    // Just enough identity to match a next-hop path prefix plus the SNR that heard it.
    uint8_t prefix[MAX_ROUTE_HASH_BYTES];
    uint8_t prefix_len;
    int8_t snr_x4;
  };

private:
  uint8_t _hashes[MAX_PACKET_HASHES*MAX_HASH_SIZE];
  int _next_idx;
  uint32_t _acks[MAX_PACKET_ACKS];
  int _next_ack_idx;
  uint32_t _direct_dups, _flood_dups;
  RecentRepeaterInfo _recent_repeaters[MAX_RECENT_REPEATERS];
  int _next_recent_repeater_idx;

  bool hasSeenAck(uint32_t ack) const {
    for (int i = 0; i < MAX_PACKET_ACKS; i++) {
      if (ack == _acks[i]) {
        return true;
      }
    }
    return false;
  }

  void storeAck(uint32_t ack) {
    _acks[_next_ack_idx] = ack;
    _next_ack_idx = (_next_ack_idx + 1) % MAX_PACKET_ACKS;
  }

  bool hasSeenHash(const uint8_t* hash) const {
    const uint8_t* sp = _hashes;
    for (int i = 0; i < MAX_PACKET_HASHES; i++, sp += MAX_HASH_SIZE) {
      if (memcmp(hash, sp, MAX_HASH_SIZE) == 0) {
        return true;
      }
    }
    return false;
  }

  void storeHash(const uint8_t* hash) {
    memcpy(&_hashes[_next_idx*MAX_HASH_SIZE], hash, MAX_HASH_SIZE);
    _next_idx = (_next_idx + 1) % MAX_PACKET_HASHES;
  }

  bool extractRecentRepeater(const mesh::Packet* packet, uint8_t* prefix, uint8_t& prefix_len) const {
    // Learn repeater prefixes only from packet shapes that expose a trustworthy repeater ID.
    if (packet->getPayloadType() == PAYLOAD_TYPE_ADVERT && packet->payload_len >= PUB_KEY_SIZE) {
      memcpy(prefix, packet->payload, MAX_ROUTE_HASH_BYTES);
      prefix_len = MAX_ROUTE_HASH_BYTES;
      return true;
    }

    if (packet->getPayloadType() == PAYLOAD_TYPE_CONTROL
        && packet->isRouteDirect()
        && packet->getPathHashCount() == 0
        && packet->payload_len >= 6 + MAX_ROUTE_HASH_BYTES
        && (packet->payload[0] & 0xF0) == 0x90) {
      memcpy(prefix, &packet->payload[6], MAX_ROUTE_HASH_BYTES);
      prefix_len = MAX_ROUTE_HASH_BYTES;
      return true;
    }

    if (packet->isRouteFlood() && packet->getPathHashCount() > 0) {
      prefix_len = packet->getPathHashSize();
      if (prefix_len > MAX_ROUTE_HASH_BYTES) {
        prefix_len = MAX_ROUTE_HASH_BYTES;
      }

      const uint8_t* last_hop = &packet->path[(packet->getPathHashCount() - 1) * packet->getPathHashSize()];
      memcpy(prefix, last_hop, prefix_len);
      return true;
    }

    return false;
  }

  void recordRecentRepeater(const mesh::Packet* packet) {
    uint8_t prefix[MAX_ROUTE_HASH_BYTES] = {0};
    uint8_t prefix_len = 0;
    if (!extractRecentRepeater(packet, prefix, prefix_len) || prefix_len == 0) {
      return;
    }

    // Ring buffer is enough here; retry fallback only needs a recent prefix->SNR observation.
    RecentRepeaterInfo& slot = _recent_repeaters[_next_recent_repeater_idx];
    memset(slot.prefix, 0, sizeof(slot.prefix));
    memcpy(slot.prefix, prefix, prefix_len);
    slot.prefix_len = prefix_len;
    slot.snr_x4 = packet->_snr;
    _next_recent_repeater_idx = (_next_recent_repeater_idx + 1) % MAX_RECENT_REPEATERS;
  }

public:
  SimpleMeshTables() { 
    memset(_hashes, 0, sizeof(_hashes));
    _next_idx = 0;
    memset(_acks, 0, sizeof(_acks));
    _next_ack_idx = 0;
    _direct_dups = _flood_dups = 0;
    memset(_recent_repeaters, 0, sizeof(_recent_repeaters));
    _next_recent_repeater_idx = 0;
  }

#ifdef ESP32
  void restoreFrom(File f) {
    f.read(_hashes, sizeof(_hashes));
    f.read((uint8_t *) &_next_idx, sizeof(_next_idx));
    f.read((uint8_t *) &_acks[0], sizeof(_acks));
    f.read((uint8_t *) &_next_ack_idx, sizeof(_next_ack_idx));
    f.read((uint8_t *) &_recent_repeaters[0], sizeof(_recent_repeaters));
    f.read((uint8_t *) &_next_recent_repeater_idx, sizeof(_next_recent_repeater_idx));
  }
  void saveTo(File f) {
    f.write(_hashes, sizeof(_hashes));
    f.write((const uint8_t *) &_next_idx, sizeof(_next_idx));
    f.write((const uint8_t *) &_acks[0], sizeof(_acks));
    f.write((const uint8_t *) &_next_ack_idx, sizeof(_next_ack_idx));
    f.write((const uint8_t *) &_recent_repeaters[0], sizeof(_recent_repeaters));
    f.write((const uint8_t *) &_next_recent_repeater_idx, sizeof(_next_recent_repeater_idx));
  }
#endif

  bool hasSeen(const mesh::Packet* packet) override {
    if (packet->getPayloadType() == PAYLOAD_TYPE_ACK) {
      uint32_t ack;
      memcpy(&ack, packet->payload, 4);

      if (hasSeenAck(ack)) {
        if (packet->isRouteDirect()) {
          _direct_dups++;   // keep some stats
        } else {
          _flood_dups++;
        }
        return true;
      }

      storeAck(ack);
      return false;
    }

    uint8_t hash[MAX_HASH_SIZE];
    packet->calculatePacketHash(hash);

    if (hasSeenHash(hash)) {
      if (packet->isRouteDirect()) {
        _direct_dups++;   // keep some stats
      } else {
        _flood_dups++;
      }
      return true;
    }

    storeHash(hash);
    recordRecentRepeater(packet);
    return false;
  }

  void markSent(const mesh::Packet* packet) override {
    // Outbound packets must be marked as already-sent without teaching the recent-heard cache about ourselves.
    if (packet->getPayloadType() == PAYLOAD_TYPE_ACK) {
      uint32_t ack;
      memcpy(&ack, packet->payload, 4);
      if (!hasSeenAck(ack)) {
        storeAck(ack);
      }
      return;
    }

    uint8_t hash[MAX_HASH_SIZE];
    packet->calculatePacketHash(hash);
    if (!hasSeenHash(hash)) {
      storeHash(hash);
    }
  }

  void clear(const mesh::Packet* packet) override {
    if (packet->getPayloadType() == PAYLOAD_TYPE_ACK) {
      uint32_t ack;
      memcpy(&ack, packet->payload, 4);
      for (int i = 0; i < MAX_PACKET_ACKS; i++) {
        if (ack == _acks[i]) { 
          _acks[i] = 0;
          break;
        }
      }
    } else {
      uint8_t hash[MAX_HASH_SIZE];
      packet->calculatePacketHash(hash);

      uint8_t* sp = _hashes;
      for (int i = 0; i < MAX_PACKET_HASHES; i++, sp += MAX_HASH_SIZE) {
        if (memcmp(hash, sp, MAX_HASH_SIZE) == 0) { 
          memset(sp, 0, MAX_HASH_SIZE);
          break;
        }
      }
    }
  }

  uint32_t getNumDirectDups() const { return _direct_dups; }
  uint32_t getNumFloodDups() const { return _flood_dups; }

  const RecentRepeaterInfo* findRecentRepeaterByHash(const uint8_t* hash, uint8_t hash_len) const {
    if (hash == NULL || hash_len == 0) {
      return NULL;
    }

    // Search newest-to-oldest so the retry gate prefers the freshest SNR sample for a prefix.
    for (int i = 0; i < MAX_RECENT_REPEATERS; i++) {
      int idx = (_next_recent_repeater_idx - 1 - i + MAX_RECENT_REPEATERS) % MAX_RECENT_REPEATERS;
      const RecentRepeaterInfo* info = &_recent_repeaters[idx];
      if (info->prefix_len < hash_len || info->prefix_len == 0) {
        continue;
      }
      if (memcmp(info->prefix, hash, hash_len) == 0) {
        return info;
      }
    }
    return NULL;
  }

  void resetStats() { _direct_dups = _flood_dups = 0; }
};
