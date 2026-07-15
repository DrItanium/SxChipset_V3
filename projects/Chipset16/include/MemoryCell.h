/*
SxChipset_v3
Copyright (c) 2025, Joshua Scoggins
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef CHIPSET_MEMORY_CELL_H__
#define CHIPSET_MEMORY_CELL_H__
#include <cstdint>
#include <concepts>
#include <type_traits>


enum class ActionKind : uint8_t {
    Full16,
    Low8,
    Hi8,
};
using WriteActionKind = ActionKind;
// A high speed interface that we can abstract contents of memory 
template<typename T>
concept MemoryCell = requires(T a) {
    // only operate on 16-bit words
    { a.getWord(0) } -> std::same_as<uint16_t>;
    { a.setWord(0, 0, ActionKind::Full16) };
    { a.setWord32(0, 0) };
    { a.setWord64(0, 0) };
    { a.getWord32(0) } -> std::same_as<uint32_t>;
    { a.getWord64(0) } -> std::same_as<uint64_t>;
    { a.getBuffer() } -> std::same_as<const uint8_t*>;
    { a.length() } -> std::same_as<size_t>;
};

union MemoryCellBlock {
  constexpr MemoryCellBlock(uint32_t a = 0, uint32_t b = 0, uint32_t c = 0, uint32_t d = 0) : words{a, b, c, d} { }
private:
  uint8_t bytes[16];
  uint16_t shorts[8];
  uint32_t words[4];
  uint64_t longWords[2];
public:
  void clear() noexcept {
      longWords[0] = 0;
      longWords[1] = 0;
  }
  [[nodiscard]] inline constexpr uint16_t getWord(uint8_t offset) const noexcept { return shorts[offset & 0b111]; }
  inline void setWord(uint8_t offset, uint16_t value, ActionKind kind) noexcept {
      switch (kind) {
          case ActionKind::Full16:
              shorts[offset & 0b111] = value;
              break;
          case ActionKind::Low8:
              bytes[(offset << 1) & 0b1110] = static_cast<uint8_t>(value);
              break;
          case ActionKind::Hi8:
              bytes[((offset << 1) & 0b1110)+1] = static_cast<uint8_t>(value >> 8);
              break;
          default:
              break;
      }
  }
  inline void setWord32(uint8_t offset, uint32_t value) noexcept { words[offset & 0b11] = value; }
  inline void setWord64(uint8_t offset, uint64_t value) noexcept { longWords[offset & 0b1] = value; }
  [[nodiscard]] inline constexpr uint32_t getWord32(uint8_t offset) const noexcept { return words[offset & 0b11]; }
  [[nodiscard]] inline constexpr auto getWord64(uint8_t offset) const noexcept { return longWords[offset & 0b1]; }
  [[nodiscard]] inline const uint8_t* getBuffer() const noexcept { return bytes; }
  [[nodiscard]] inline constexpr size_t length() const noexcept { return sizeof(bytes); }
};

struct NullBlock final {
    void clear() { }
    [[nodiscard]] uint16_t getWord(uint8_t) const noexcept { return 0; }
    void setWord(uint8_t, uint16_t, ActionKind) noexcept { }
    void setWord32(uint8_t, uint32_t) noexcept { }
    void setWord64(uint8_t, uint64_t) noexcept { }
    [[nodiscard]] constexpr uint32_t getWord32(uint8_t) const noexcept { return 0; }
    [[nodiscard]] constexpr uint64_t getWord64(uint8_t) const noexcept { return 0; }
    [[nodiscard]] const uint8_t* getBuffer() const noexcept { return nullptr; }
    [[nodiscard]] constexpr size_t length() const noexcept { return 0; }
};

#endif // end !defined CHIPSET_MEMORY_CELL_H__
