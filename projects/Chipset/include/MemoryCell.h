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

// A high speed interface that we can abstract contents of memory 
template<typename T>
concept MemoryCell = requires(T a) {
    { a.update() };
    // only operate on 16-bit words
    { a.getWord(0) } -> std::same_as<uint16_t>;
    { a.setWord(0, 0) };
    { a.setWord(0, 0, true, true) };
    { a.onFinish() };
};

union MemoryCellBlock {
  constexpr MemoryCellBlock(uint32_t a = 0, uint32_t b = 0, uint32_t c = 0, uint32_t d = 0) : words{a, b, c, d} { }
private:
  uint8_t bytes[16];
  uint16_t shorts[8];
  uint32_t words[4];
public:
  void update() noexcept {

  }
  void clear() noexcept {
      for (auto& a : words) {
          a = 0;
      }
  }
  [[nodiscard]] inline constexpr uint16_t getWord(uint8_t offset) const noexcept { return shorts[offset & 0b111]; }
  inline void setWord(uint8_t offset, uint16_t value) noexcept { shorts[offset & 0b111] = value; }
  inline void setWord(uint8_t offset, uint16_t value, bool updateLo, bool updateHi) noexcept {
    // convert to an 8-bit setup so we can do conversions as needed
    uint8_t baseOffset = (offset << 1) & 0b1110;
    if (updateLo) {
        bytes[baseOffset] = static_cast<uint8_t>(value);
    }
    if (updateHi) {
        bytes[baseOffset+1] = static_cast<uint8_t>(value >> 8);
    }
  }
  inline void setWord32(uint8_t offset, uint32_t value) noexcept {
      words[offset & 0b11] = value;
  }
  void onFinish() noexcept { }
};

#endif // end !defined CHIPSET_MEMORY_CELL_H__
