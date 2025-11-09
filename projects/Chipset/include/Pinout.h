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
#ifndef CHIPSET_PINOUT_H__
#define CHIPSET_PINOUT_H__
#include <Arduino.h>
#include <type_traits>
enum class Pin : uint8_t {
  RPI_D0 = 19,
  RPI_D1 = 18,
  RPI_D2 = 14,
  RPI_D3 = 15,
  RPI_D4 = 40,
  RPI_D5 = 41,
  RPI_D6 = 17,
  RPI_D7 = 16,
  RPI_D8 = 22,
  RPI_D9 = 23,
  RPI_D10 = 20,
  RPI_D11 = 21,
  RPI_D12 = 38,
  RPI_D13 = 39,
  RPI_D14 = 26,
  RPI_D15 = 27,
  RPI_D16 = 8, // XBAR_INOUT14
  RPI_D17 = 7, // XBOAR_INOUT15
  RPI_D18 = 36, // XBAR_INOUT16
  RPI_D19 = 37, // XBAR_INOUT17
  RPI_D20 = 32, // XBAR_INOUT10
  RPI_D21 = 9, 
  RPI_D22 = 6,
  RPI_D23 = 2, // XBAR_INOUT6
  RPI_D24 = 3, // XBAR_INOUT7
  RPI_D25 = 4, // XBAR_INOUT8
  RPI_D26 = 33, // XBAR_INOUT9
  RPI_D27 = 5, // XBAR_INOUT17
#if 0
  RPI_D28 = 28,
  RPI_D29 = 29,
  RPI_D30 = 30, // XBAR_INOUT23
  RPI_D31 = 31, // XBAR_INOUT22
#else
  RPI_D28 = 30, // XBAR_INOUT23
  RPI_D29 = 31, // XBAR_INOUT22
  RPI_D30 = 28,
  RPI_D31 = 29,
#endif
  RPI_D32 = 34,
  RPI_D33 = 35,
  RPI_D34 = 10,
  RPI_D35 = 11,
  RPI_D36 = 12,
  RPI_D37 = 13,
  RPI_D38 = 0, // XBAR_INOUT16
  RPI_D39 = 1, // XBAR_INOUT17
  RPI_D40 = 24,
  RPI_D41 = 25,
  SIDECOMM_SDA = RPI_D41,
  SIDECOMM_SCL = RPI_D40,
  EBI_A5 = RPI_D0, // AD_B0_03
  EBI_A4 = RPI_D1, // AD_B0_02
  EBI_A3 = RPI_D2, // EMC_04
  EBI_A2 = RPI_D3, // EMC_05
  EBI_A1 = RPI_D4, // EMC_06
  EBI_A0 = RPI_D5, // EMC_08
  EBI_RD = RPI_D6, // B0_10
  EBI_WR = RPI_D7, // B1_01
  EBI_D0 = RPI_D8, // B1_00
  EBI_D1 = RPI_D9, // B0_11
  EBI_D2 = RPI_D10, // B0_00
  EBI_D3 = RPI_D11, // B0_02
  EBI_D4 = RPI_D12, // B0_01
  EBI_D5 = RPI_D13, // B0_03
  EBI_D6 = RPI_D14, // AD_B1_02
  EBI_D7 = RPI_D15, // AD_B1_03
  ADS = RPI_D16,
  BLAST = RPI_D17,
  // RPI_D18 ???
  WR = RPI_D19,
  BE0 = RPI_D20,
  BE1 = RPI_D21,
  INT960_0 = RPI_D22,  
  // RPI_D23
  // RPI_D24
  // RPI_D25
  // RPI_D26
  // RPI_D27
  // RPI_D28
  // RPI_D29
  READY = RPI_D30,
  READY_SYNC = RPI_D31,
  // RPI_D32
};


constexpr std::underlying_type_t<Pin> pinIndexConvert(Pin value) noexcept {
  return static_cast<std::underlying_type_t<Pin>>(value);
}
using PinDirection = decltype(OUTPUT);
using PinValue = decltype(HIGH);
inline void pinMode(Pin pin, PinDirection direction) noexcept {
  ::pinMode(pinIndexConvert(pin), direction);
}

inline void digitalWriteFast(Pin pin, PinValue value) noexcept {
  ::digitalWriteFast(pinIndexConvert(pin), value);
}
inline void digitalWrite(Pin pin, PinValue value) noexcept {
  ::digitalWrite(pinIndexConvert(pin), value);
}
inline decltype(auto) digitalReadFast(Pin pin) noexcept {
  return ::digitalReadFast(pinIndexConvert(pin));
}
inline decltype(auto) digitalRead(Pin pin) noexcept {
  return ::digitalRead(pinIndexConvert(pin));
}
inline void attachInterrupt(Pin pin, void (*function)(void), decltype(RISING) mode) noexcept {
  ::attachInterrupt(digitalPinToInterrupt(pinIndexConvert(pin)), function, mode);
}

inline void digitalToggle(Pin pin) noexcept {
  ::digitalToggle(pinIndexConvert(pin));
}
template<Pin p>
inline void digitalToggle() noexcept {
  digitalToggle(p);
}

inline void digitalToggleFast(Pin pin) noexcept {
  ::digitalToggleFast(pinIndexConvert(pin));
}
template<Pin p>
inline void digitalToggleFast() noexcept {
  digitalToggleFast(p);
}
inline void outputPin(Pin p, PinValue initialValue) noexcept {
  pinMode(p, OUTPUT);
  digitalWrite(p, initialValue);
}
inline void inputPin(Pin p) noexcept {
  pinMode(p, INPUT);
}
#endif // end !defined CHIPSET_PINOUT_H__
