#pragma once

#include <stdint.h>
#include <stddef.h>

// Mock SHA256 for native testing — deterministic but not cryptographic.
// finalize() writes real (non-garbage) output so calculatePacketHash() produces
// distinguishable results for packets with different payloads.
#include <string.h>

class SHA256 {
  uint8_t _state[32];
  size_t _len;
public:
  SHA256() : _len(0) { memset(_state, 0, sizeof(_state)); }

  void update(const void* data, size_t len) {
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < len; i++) {
      uint8_t b = bytes[i];
      _state[_len % 32] ^= b;
      _state[(_len + 1) % 32] += (uint8_t)((b >> 1) | (b << 7));
      _len++;
    }
  }

  void finalize(void* hashVoid, size_t hashLen) {
    uint8_t* hash = static_cast<uint8_t*>(hashVoid);
    for (size_t i = 0; i < hashLen; i++) {
      hash[i] = _state[i % 32];
    }
  }

  void resetHMAC(const void* key, size_t keyLen) { (void)key; (void)keyLen; }

  // Mock HMAC: fold the key in at finalize time so the result is a
  // deterministic function of (key, data).  Real HMAC semantics are not
  // required — but TransportKey::calcTransportCode() equality checks in tests
  // need key- and data-sensitive output (the previous no-op left the
  // destination uninitialized).
  void finalizeHMAC(const void* keyVoid, size_t keyLen, void* hashVoid, size_t hashLen) {
    SHA256 tmp;
    memcpy(tmp._state, _state, sizeof(_state));
    tmp._len = _len;
    tmp.update(keyVoid, keyLen);
    tmp.finalize(hashVoid, hashLen);
  }
};
