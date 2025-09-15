// cfg/eeprom_store.h
#pragma once
#include <Arduino.h>
#include <EEPROM.h>
#include <string.h>
#include <stdint.h>

/**
 * \file
 * \brief Header-only wear-leveling EEPROM store.
 * \details
 * Stores typed payloads across multiple fixed-size slots to distribute wear.
 * Slot layout (little-endian fields), for SLOT_SZ bytes:
 * \code
 * [0..1]   magic   : u16  (0xC0DE)
 * [2]      version : u8   (1)
 * [3..6]   seq     : u32  (monotonic sequence)
 * [7..8]   len     : u16  (payload length in bytes, must equal sizeof(T))
 * [9..]    payload : len  (raw bytes of T)
 * [9+len..9+len+1] crc    : u16  (CRC-16/IBM, poly 0xA001, init 0xFFFF)
 * \endcode
 * Runtime slot count is \c EEPROM.length() / SLOT_SZ. Any remainder is unused.
 *
 * Power-loss behavior: a partially-written "next" slot is harmless; it will fail
 * CRC and be skipped on load. Sequence continues from the highest valid \c seq.
 *
 * \warning SLOT_SZ must satisfy \c SLOT_SZ >= 9 + sizeof(T) + 2.
 * \note Uses \c EEPROM.update() to avoid rewrites of identical bytes.
 */

/**
 * \tparam T       POD-like type to store (trivially copyable raw bytes).
 * \tparam SLOT_SZ Slot size in bytes (compile-time constant, default 128).
 */
template <typename T, uint16_t SLOT_SZ = 128>
class WearLevelStore
{
  static_assert(SLOT_SZ >= 9 + sizeof(T) + 2, "SLOT_SZ too small for payload");

public:
  /**
   * \brief Load the most recent valid record from EEPROM.
   * \param[out] out Destination for the decoded payload.
   * \retval true  A valid slot was found (CRC ok, len matches sizeof(T)).
   * \retval false No valid slot found or no slots available.
   * \note Scans all slots, tracking the highest \c seq among valid entries.
   */
  bool load(T &out)
  {
    int32_t bestIdx = -1;
    uint32_t bestSeq = 0;
    const uint16_t slots = e2len() / SLOT_SZ;
    if (slots == 0)
      return false;

    for (uint16_t i = 0; i < slots; ++i)
    {
      const uint16_t a = slot_addr(i);
      if (!slot_valid(a))
        continue;

      uint32_t seq;
      uint16_t len;
      read_u32(a + 3, seq);
      read_u16(a + 7, len);
      if (len != sizeof(T))
        continue;
      if (9 + len + 2 > SLOT_SZ)
        continue;

      T tmp;
      eeread(a + 9, (uint8_t *)&tmp, len);
      const uint16_t crc = read_u16_ret(a + 9 + len);
      if (crc != crc16((const uint8_t *)&tmp, len))
        continue;

      if (bestIdx < 0 || seq > bestSeq)
      {
        bestIdx = i;
        bestSeq = seq;
        out = tmp;
      }
    }
    return bestIdx >= 0;
  }

  /**
   * \brief Save a new record to the next wear-level slot.
   * \param in Payload to persist.
   * \retval true  Save sequence completed (header, payload, CRC written).
   * \retval false No slots available (EEPROM too small or SLOT_SZ too large).
   * \details
   * Finds the latest valid slot (by seq), then writes to (latest+1) % slots.
   * Header is written first (incl. next \c seq), then payload, then CRC.
   */
  bool save(const T &in)
  {
    // Find latest slot & sequence, then write next slot.
    const uint16_t slots = e2len() / SLOT_SZ;
    if (slots == 0)
      return false;

    int32_t bestIdx = -1;
    uint32_t bestSeq = 0;
    for (uint16_t i = 0; i < slots; ++i)
    {
      const uint16_t a = slot_addr(i);
      if (!slot_valid(a))
        continue;
      uint32_t seq;
      read_u32(a + 3, seq);
      if (bestIdx < 0 || seq > bestSeq)
      {
        bestIdx = i;
        bestSeq = seq;
      }
    }
    const uint16_t target = (bestIdx < 0) ? 0 : (uint16_t)((bestIdx + 1) % slots);
    const uint16_t a = slot_addr(target);

    const uint16_t len = sizeof(T);
    const uint16_t crc = crc16((const uint8_t *)&in, len);

    // Program header
    write_u16(a + 0, MAGIC);
    EEPROM.update(a + 2, VERSION);
    write_u32(a + 3, bestSeq + 1);
    write_u16(a + 7, len);
    // Payload
    eewrite(a + 9, (const uint8_t *)&in, len);
    // CRC
    write_u16(a + 9 + len, crc);
    return true;
  }

private:
  /// \internal Magic constant to identify valid slots.
  static constexpr uint16_t MAGIC = 0xC0DE;
  /// \internal Version byte for slot format evolution.
  static constexpr uint8_t VERSION = 1;

  /// \internal Total EEPROM length in bytes (uint16_t-cast).
  static inline uint16_t e2len() { return (uint16_t)EEPROM.length(); }
  /// \internal Byte address of a slot.
  static inline uint16_t slot_addr(uint16_t idx) { return (uint16_t)(idx * SLOT_SZ); }

  /// \internal Read raw bytes from EEPROM.
  static inline void eeread(uint16_t addr, uint8_t *dst, uint16_t n)
  {
    while (n--)
    {
      *dst++ = EEPROM.read(addr++);
    }
  }
  /// \internal Write bytes using EEPROM.update().
  static inline void eewrite(uint16_t addr, const uint8_t *src, uint16_t n)
  {
    while (n--)
    {
      EEPROM.update(addr++, *src++);
    }
  }

  /// \internal Quick header check (magic + version).
  static inline bool slot_valid(uint16_t a)
  {
    const uint16_t magic = read_u16_ret(a + 0);
    if (magic != MAGIC)
      return false;
    const uint8_t ver = EEPROM.read(a + 2);
    return ver == VERSION;
  }

  /// \internal Little-endian u16 write.
  static inline void write_u16(uint16_t a, uint16_t v)
  {
    EEPROM.update(a + 0, (uint8_t)(v & 0xFF));
    EEPROM.update(a + 1, (uint8_t)(v >> 8));
  }
  /// \internal Little-endian u16 read (returns value).
  static inline uint16_t read_u16_ret(uint16_t a)
  {
    uint16_t v = EEPROM.read(a + 0);
    v |= (uint16_t)EEPROM.read(a + 1) << 8;
    return v;
  }
  /// \internal Little-endian u32 write.
  static inline void write_u32(uint16_t a, uint32_t v)
  {
    EEPROM.update(a + 0, (uint8_t)(v & 0xFF));
    EEPROM.update(a + 1, (uint8_t)((v >> 8) & 0xFF));
    EEPROM.update(a + 2, (uint8_t)((v >> 16) & 0xFF));
    EEPROM.update(a + 3, (uint8_t)((v >> 24) & 0xFF));
  }
  /// \internal Little-endian u32 read (by reference).
  static inline void read_u32(uint16_t a, uint32_t &v)
  {
    v = (uint32_t)EEPROM.read(a + 0);
    v |= (uint32_t)EEPROM.read(a + 1) << 8;
    v |= (uint32_t)EEPROM.read(a + 2) << 16;
    v |= (uint32_t)EEPROM.read(a + 3) << 24;
  }
  /// \internal Little-endian u16 read (by reference).
  static inline void read_u16(uint16_t a, uint16_t &v)
  {
    v = (uint16_t)EEPROM.read(a + 0);
    v |= (uint16_t)EEPROM.read(a + 1) << 8;
  }

  /**
   * \internal
   * \brief CRC-16/IBM (MODBUS) over payload bytes.
   * \param p Pointer to input buffer.
   * \param n Number of bytes.
   * \return CRC with init 0xFFFF, reflected algorithm, poly 0xA001.
   */
  static uint16_t crc16(const uint8_t *p, uint16_t n)
  {
    uint16_t c = 0xFFFF;
    while (n--)
    {
      c ^= *p++;
      for (uint8_t i = 0; i < 8; i++)
        c = (c & 1) ? (c >> 1) ^ 0xA001 : (c >> 1);
    }
    return c;
  }
};
