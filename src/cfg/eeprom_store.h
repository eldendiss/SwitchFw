// cfg/eeprom_store.h
#pragma once
#include <Arduino.h>
#include <EEPROM.h>
#include <string.h>
#include <stdint.h>

/*
  Wear-leveling EEPROM store (header-only).
  - Layout per slot (SLOT_SZ bytes):
    [magic u16][version u8][seq u32][len u16][payload len bytes][crc u16]
  - Number of slots = EEPROM.length() / SLOT_SZ  (computed at runtime)
  - SLOT_SZ must be >= 9 + sizeof(T) + 2
*/

template<typename T, uint16_t SLOT_SZ = 128>
class WearLevelStore {
  static_assert(SLOT_SZ >= 9 + sizeof(T) + 2, "SLOT_SZ too small for payload");
public:
  bool load(T& out) {
    int32_t bestIdx = -1; uint32_t bestSeq = 0;
    const uint16_t slots = e2len() / SLOT_SZ;
    if (slots == 0) return false;

    for (uint16_t i = 0; i < slots; ++i) {
      const uint16_t a = slot_addr(i);
      if (!slot_valid(a)) continue;

      uint32_t seq; uint16_t len;
      read_u32(a + 3, seq);
      read_u16(a + 7, len);
      if (len != sizeof(T)) continue;
      if (9 + len + 2 > SLOT_SZ) continue;

      T tmp;
      eeread(a + 9, (uint8_t*)&tmp, len);
      const uint16_t crc = read_u16_ret(a + 9 + len);
      if (crc != crc16((const uint8_t*)&tmp, len)) continue;

      if (bestIdx < 0 || seq > bestSeq) { bestIdx = i; bestSeq = seq; out = tmp; }
    }
    return bestIdx >= 0;
  }

  bool save(const T& in) {
    // Find latest slot & sequence, then write next slot.
    const uint16_t slots = e2len() / SLOT_SZ;
    if (slots == 0) return false;

    int32_t bestIdx = -1; uint32_t bestSeq = 0;
    for (uint16_t i = 0; i < slots; ++i) {
      const uint16_t a = slot_addr(i);
      if (!slot_valid(a)) continue;
      uint32_t seq; read_u32(a + 3, seq);
      if (bestIdx < 0 || seq > bestSeq) { bestIdx = i; bestSeq = seq; }
    }
    const uint16_t target = (bestIdx < 0) ? 0 : (uint16_t)((bestIdx + 1) % slots);
    const uint16_t a = slot_addr(target);

    const uint16_t len = sizeof(T);
    const uint16_t crc = crc16((const uint8_t*)&in, len);

    // Program header
    write_u16(a + 0, MAGIC);
    EEPROM.update(a + 2, VERSION);
    write_u32(a + 3, bestSeq + 1);
    write_u16(a + 7, len);
    // Payload
    eewrite(a + 9, (const uint8_t*)&in, len);
    // CRC
    write_u16(a + 9 + len, crc);
    return true;
  }

private:
  static constexpr uint16_t MAGIC   = 0xC0DE;
  static constexpr uint8_t  VERSION = 1;

  static inline uint16_t e2len() { return (uint16_t)EEPROM.length(); }
  static inline uint16_t slot_addr(uint16_t idx) { return (uint16_t)(idx * SLOT_SZ); }

  static inline void eeread(uint16_t addr, uint8_t* dst, uint16_t n) {
    while (n--) { *dst++ = EEPROM.read(addr++); }
  }
  static inline void eewrite(uint16_t addr, const uint8_t* src, uint16_t n) {
    while (n--) { EEPROM.update(addr++, *src++); }
  }

  static inline bool slot_valid(uint16_t a) {
    const uint16_t magic = read_u16_ret(a + 0);
    if (magic != MAGIC) return false;
    const uint8_t ver = EEPROM.read(a + 2);
    return ver == VERSION;
  }

  static inline void write_u16(uint16_t a, uint16_t v) {
    EEPROM.update(a + 0, (uint8_t)(v & 0xFF));
    EEPROM.update(a + 1, (uint8_t)(v >> 8));
  }
  static inline uint16_t read_u16_ret(uint16_t a) {
    uint16_t v = EEPROM.read(a + 0);
    v |= (uint16_t)EEPROM.read(a + 1) << 8;
    return v;
  }
  static inline void write_u32(uint16_t a, uint32_t v) {
    EEPROM.update(a + 0, (uint8_t)(v & 0xFF));
    EEPROM.update(a + 1, (uint8_t)((v >> 8) & 0xFF));
    EEPROM.update(a + 2, (uint8_t)((v >> 16) & 0xFF));
    EEPROM.update(a + 3, (uint8_t)((v >> 24) & 0xFF));
  }
  static inline void read_u32(uint16_t a, uint32_t& v) {
    v  = (uint32_t)EEPROM.read(a + 0);
    v |= (uint32_t)EEPROM.read(a + 1) << 8;
    v |= (uint32_t)EEPROM.read(a + 2) << 16;
    v |= (uint32_t)EEPROM.read(a + 3) << 24;
  }
  static inline void read_u16(uint16_t a, uint16_t& v) {
    v  = (uint16_t)EEPROM.read(a + 0);
    v |= (uint16_t)EEPROM.read(a + 1) << 8;
  }

  static uint16_t crc16(const uint8_t* p, uint16_t n) {
    uint16_t c = 0xFFFF;
    while (n--) {
      c ^= *p++;
      for (uint8_t i = 0; i < 8; i++) c = (c & 1) ? (c >> 1) ^ 0xA001 : (c >> 1);
    }
    return c;
  }
};
