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

enum class Pin {
#define X(index, value) INDIRECT_D ## index = value 
    X(0, 19),
    X(1, 18),
    X(2, 14),
    X(3, 15),
    X(4, 40),
    X(5, 41),
    X(6, 17),
    X(7, 16),
    X(8, 22),
    X(9, 23),
    X(10, 20),
    X(11, 21),
    X(12, 38),
    X(13, 39),
    X(14, 26),
    X(15, 27),
    X(16, 8),
    X(17, 7),
    X(18, 36),
    X(19, 37),
    X(20, 32),
    X(21, 9),
    X(22, 6),
    X(23, 2),
    X(24, 3),
    X(25, 4),
    X(26, 33),
    X(27, 5),
    X(28, 28),
    X(29, 29),
    X(30, 30),
    X(31, 31),
    X(32, 34),
    X(33, 35),
    X(34, 10),
    X(35, 11),
    X(36, 12),
    X(37, 13),
    X(38, 0),
    X(39, 1),
    X(40, 24),
    X(41, 25),
#undef X
    EBI_A0 = INDIRECT_D0,
    EBI_A1 = INDIRECT_D1,
    EBI_A2 = INDIRECT_D2,
    EBI_A3 = INDIRECT_D3,
    EBI_A4 = INDIRECT_D4,
    EBI_A5 = INDIRECT_D5,
    EBI_RD = INDIRECT_D6,
    EBI_WR = INDIRECT_D7,
    EBI_D0 = INDIRECT_D8,
    EBI_D1 = INDIRECT_D9,
    EBI_D2 = INDIRECT_D10,
    EBI_D3 = INDIRECT_D11,
    EBI_D4 = INDIRECT_D12,
    EBI_D5 = INDIRECT_D13,
    EBI_D6 = INDIRECT_D14,
    EBI_D7 = INDIRECT_D15,
    ADS = INDIRECT_D16,
    BLAST = INDIRECT_D17,
    WR = INDIRECT_D18,
    BE0 = INDIRECT_D19,
    BE1 = INDIRECT_D20,
    INT960_0 = INDIRECT_D21,  
    INT960_1 = INDIRECT_D22,  
    // INDIRECT_D23
    // INDIRECT_D24
    // INDIRECT_D25
    // INDIRECT_D26
    // INDIRECT_D27
    READY_LEVEL_IN = INDIRECT_D28,
    DEN = INDIRECT_D29,
    READY = INDIRECT_D30,
    READY_SYNC = INDIRECT_D31,
    INT960_2 = INDIRECT_D32,
    INT960_3 = INDIRECT_D33,
    // INDIRECT_D34 
    // INDIRECT_D35 (MOSI)
    // INDIRECT_D36 (MISO)
    // INDIRECT_D37 (SCK)
    TEENSY_AVR_RX = INDIRECT_D38,
    TEENSY_AVR_TX = INDIRECT_D39,
    TEENSY_AVR_SCL = INDIRECT_D40,
    TEENSY_AVR_SDA = INDIRECT_D41,

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
