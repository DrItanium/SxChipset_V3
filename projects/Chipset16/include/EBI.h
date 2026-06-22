/*
SxChipset_v3
Copyright (c) 2026, Joshua Scoggins
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

// Header for providing access to FlexIO components cleanly
#ifndef CHIPSET_EBI_H__
#define CHIPSET_EBI_H__
#include <cstdint>
#include <Arduino.h>
#include "Pinout.h"
#include <FlexIO_t4.h>
#include <array>

constexpr uint32_t makeAddress(uint8_t value) noexcept {
    return static_cast<uint32_t>(value & 0b111111) << 16;
}
static_assert(makeAddress(0b00'01'00'11) == 0b00'01'00'11'0000'0000'0000'0000);
static_assert(makeAddress(0b00'00'00'01) == 0b00'00'00'01'0000'0000'0000'0000);

struct EBIWrapperInterface {
public:
  EBIWrapperInterface() = delete;
  ~EBIWrapperInterface() = delete;
  EBIWrapperInterface(const EBIWrapperInterface&) = delete;
  EBIWrapperInterface(EBIWrapperInterface&&) = delete;
  EBIWrapperInterface& operator=(const EBIWrapperInterface&) = delete;
  EBIWrapperInterface& operator=(EBIWrapperInterface&&) = delete;
  static bool begin() noexcept;
  // in the 16-bit bus design, we can easily just assign data without needing
  // to have a strange lookup table for all of this
  static void
  setDataLines(uint16_t value) noexcept {
      GPIO6_DR_CLEAR = 0xFFFF'0000;
      GPIO6_DR_SET = (static_cast<uint32_t>(value) << 16);
  }
  static uint16_t
  readDataLines() noexcept {
      return static_cast<uint16_t>((GPIO6_PSR) >> 16);
  }
  static void setAddress(uint8_t address) noexcept {
      // unlike the 8-bit bus, the address lines are stashed in the GPIO7 in
      // the upper half of the bus
      GPIO7_DR_CLEAR = 0x0007'0000;
      GPIO7_DR_SET = (static_cast<uint32_t>(address & 0b111) << 16);
  }
  template<uint8_t address>
  static inline void setAddress() noexcept {
      constexpr auto convertedAddress = address & 0b111;
      if constexpr (convertedAddress == 0b000) {
          GPIO7_DR_CLEAR = 0x0007'0000;
      } else if constexpr (convertedAddress == 0b111) {
          GPIO7_DR_SET = 0x0007'0000;
      } else {
          GPIO7_DR_CLEAR = 0x0007'0000;
          constexpr uint32_t computedAddress = static_cast<uint32_t>(convertedAddress) << 16;
          GPIO7_DR_SET = computedAddress;
      }
  }

  template<PinDirection direction>
  static void
  setDataLinesDirection() noexcept {
      static_assert (direction == OUTPUT || direction == INPUT, "Invalid direction design");
      if constexpr (direction == OUTPUT) {
          GPIO6_GDIR = (GPIO6_GDIR | 0xFFFF'0000);
      } else {
          GPIO6_GDIR = (GPIO6_GDIR & 0x0000'FFFF);
      }
  }
};
using EBIInterface = EBIWrapperInterface;

#endif // end !defined(CHIPSET_EBI_H__)
