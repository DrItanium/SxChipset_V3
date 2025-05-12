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

using PinIndex = decltype(PIN_PA02);
enum class Pin : PinIndex {
    PB25,
    PB24,
    PC18,
    PC19,
    PC20,
    PC21,
    PD20,
    PD21,
    PB18,
    PB02,
    PB22,
    PB23,
    PB00,
    PB01,
    PB16,
    PB17,
    PC22,
    PC23,
    PB12,
    PB13,
    PB20,
    PB21,
    PD12,
    PA15,
    PC17,
    PC16,
    PA12,
    PA13,
    PA14,
    PB19,
    PA23,
    PA22,
    PA21,
    PA20,
    PA19,
    PA18,
    PA17,
    PA16,
    PB15,
    PB14,
    PC13,
    PC12,
    PC15,
    PC14,
    PC11,
    PC10,
    PC06,
    PC07,
    PC04,
    PC05,
    PD11,
    PD08,
    PD09,
    PD10,
    PB05,
    PB06,
    PB07,
    PB08,
    PB09,
    PA04,
    PA06,
    PA07,
    // there is a hole in ids here
    PA02 = 67,
    PA05,
    PB03,
    PC00,
    PC01,
    PC02,
    PC03,
    PB04,
    Neopixel = PIN_NEOPIXEL,
    SD_Detect = 95,
    // custom signals (with comments about the digital index as well, mostly
    // sorted based on the digital connector block but not fully)
    READY = PD12, // 22
    BLAST = PA15, // 23
    WR = PB19, // 29
    BE1 = PB15, // 38
    BE0 = PB14, // 39
    Data0 = PC00, // 70/A3
    Data1 = PC01, // 71/A4
    Data2 = PC02, // 72/A5
    Data3 = PC03, // 73/A6
    Data4 = PC04, // 48
    Data5 = PC05, // 49
    Data6 = PC06, // 46
    Data7 = PC07, // 47
    Data8 = PC10, // 45
    Data9 = PC11, // 44
    Data10 = PC12, // 41
    Data11 = PC13, // 40
    Data12 = PC14, // 43
    Data13 = PC15, // 42
    Data15 = PC17, // 24
    Data14 = PC16, // 25
    ADS = PB04, // 74/A7
    READY_SYNC = PB05, // 54/A8
    AddressCapture_SPI_EN = PD10, // 53
};


[[gnu::always_inline]]
inline void 
pinMode(Pin targetPin, decltype(OUTPUT) direction) noexcept {
    ::pinMode(static_cast<std::underlying_type_t<Pin>>(targetPin), direction);
}

template<Pin p, decltype(OUTPUT) direction> 
[[gnu::always_inline]]
inline void 
pinMode() noexcept {
    pinMode(p, direction);
}

[[gnu::always_inline]]
inline void
digitalWrite(Pin targetPin, decltype(LOW) value) noexcept {
    ::digitalWrite(static_cast<std::underlying_type_t<Pin>>(targetPin), value);
}
template<Pin p>
[[gnu::always_inline]]
inline void
digitalWrite(decltype(LOW) value) noexcept {
    digitalWrite(p, value);
}
template<Pin p, decltype(LOW) value>
[[gnu::always_inline]]
inline void
digitalWrite() noexcept {
    digitalWrite(p, value);
}

[[gnu::always_inline]]
inline auto
digitalRead(Pin targetPin) noexcept {
    return ::digitalRead(static_cast<std::underlying_type_t<Pin>>(targetPin));
}
template<Pin p>
[[gnu::always_inline]]
inline decltype(auto)
digitalRead() noexcept {
    return digitalRead(p);
}
[[gnu::always_inline]]
inline void
outputPin(Pin targetPin, decltype(LOW) initialValue = HIGH) noexcept {
    pinMode(targetPin, OUTPUT);
    digitalWrite(targetPin, initialValue);
}
template<Pin p, decltype(LOW) initialValue = HIGH>
[[gnu::always_inline]]
inline void 
outputPin() noexcept {
    pinMode<p, OUTPUT>();
    digitalWrite<p, initialValue>();
}
template<Pin p>
decltype(auto) 
getCorrespondingPort() noexcept {
    return digitalPinToPort(static_cast<std::underlying_type_t<Pin>>(p));
}

#endif // end !defined CHIPSET_PINOUT_H__
