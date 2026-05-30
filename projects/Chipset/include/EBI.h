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
  static constexpr uint32_t EBIAddressTable[256] {
#define X(value) makeAddress(value), 
#include "Entry255.def"
#undef X
  };

  static constexpr uint32_t EBIOutputTransformation[256] {
#define X(value) ((static_cast<uint32_t>(value) << 24) & 0xFF00'0000),
#include "Entry255.def"
#undef X
  };
  static void begin() noexcept;
  static void 
  setAddress(uint8_t address) noexcept {
      // the address table lookup is necessary because the address bits are
      // backwards compared to the GPIO index
      // 0: A5
      // 1: A4
      // 2: A3
      // 3: A2
      // 4: A1
      // 5: A0
      //
      // This layout is taken from the Raspberry pi 0-4's SMI alternate
      // mode. It allows me to leverage a PCB I made with the two CH351s
      // meant for a raspberry pi 4.
      GPIO6_DR_CLEAR = EBIAddressTable[0xFF];
      GPIO6_DR_SET = EBIAddressTable[address];
  }
  static void 
  setDataLines(uint8_t value) noexcept {
      GPIO6_DR_CLEAR = EBIOutputTransformation[0xFF];
      GPIO6_DR_SET = EBIOutputTransformation[value];
  }
  static uint8_t
  readDataLines() noexcept {
        return static_cast<uint8_t>((GPIO6_PSR) >> 24);
  }
  template<PinDirection direction>
  static void
  setDataLinesDirection() noexcept {
      // I get a warning from the compiler if I do &= and |= directly
      // on GPIO6_GDIR. It warning states that doing that with a
      // volatile variable is deprecated. This form, however, is
      // supported.
      auto value = GPIO6_GDIR & ~EBIOutputTransformation[0xff];
      if constexpr (direction == OUTPUT) {
          GPIO6_GDIR = (value | EBIOutputTransformation[0xff]);
      } else {
          GPIO6_GDIR = value;
      }
  }

};
using EBIInterface = EBIWrapperInterface;

#endif // end !defined(CHIPSET_EBI_H__)
