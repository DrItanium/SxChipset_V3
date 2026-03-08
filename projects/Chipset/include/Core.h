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
#ifndef CHIPSET_CORE_H__
#define CHIPSET_CORE_H__
#include <stdint.h>
template<typename T>
struct TreatAs final {
    using underlying_type = T;
};
union SplitWord32 {
    uint8_t bytes[4];
    uint16_t shorts[2];
    uint32_t value;
    
    inline void setWord(uint8_t offset, uint16_t value, bool updateLo, bool updateHi) noexcept {
        switch (offset & 0b1) {
            case 0:
                if (updateLo) {
                    bytes[0] = static_cast<uint8_t>(value);
                } 
                if (updateHi) {
                    bytes[1] = static_cast<uint8_t>(value >> 8);
                }
                break;
            case 1:
                if (updateLo) {
                    bytes[2] = static_cast<uint8_t>(value);
                } 
                if (updateHi) {
                    bytes[3] = static_cast<uint8_t>(value >> 8);
                }
                break;
            default:
                break;
        }
    }
    inline void setWord(uint8_t offset, uint16_t value) noexcept {
        shorts[offset & 0b1] = value;
    }
    [[nodiscard]] inline constexpr uint16_t getWord(uint8_t offset) const noexcept { return shorts[offset & 0b1]; }
    void update() noexcept {

    }
    void clear() noexcept { value = 0; }
    void onFinish() noexcept { }
};
union SplitWord64 {
    uint8_t bytes[sizeof(uint64_t) / sizeof(uint8_t)];
    uint16_t shorts[sizeof(uint64_t) / sizeof(uint16_t)];
    uint32_t words[sizeof(uint64_t) / sizeof(uint32_t)];
    uint64_t value;
    void clear() noexcept { value = 0; }
    void update() noexcept { }
    void onFinish() noexcept { }
    inline void setWord(uint8_t offset, uint16_t value, bool updateLo, bool updateHi) noexcept {
        switch (offset & 0b11) {
            case 0:
                if (updateLo) {
                    bytes[0] = static_cast<uint8_t>(value);
                } 
                if (updateHi) {
                    bytes[1] = static_cast<uint8_t>(value >> 8);
                }
                break;
            case 1:
                if (updateLo) {
                    bytes[2] = static_cast<uint8_t>(value);
                } 
                if (updateHi) {
                    bytes[3] = static_cast<uint8_t>(value >> 8);
                }
                break;
            case 2:
                if (updateLo) {
                    bytes[4] = static_cast<uint8_t>(value);
                } 
                if (updateHi) {
                    bytes[5] = static_cast<uint8_t>(value >> 8);
                }
                break;
            case 3:
                if (updateLo) {
                    bytes[6] = static_cast<uint8_t>(value);
                } 
                if (updateHi) {
                    bytes[7] = static_cast<uint8_t>(value >> 8);
                }
                break;
            default:
                break;
        }
    }
    inline void setWord(uint8_t offset, uint16_t value) noexcept {
        shorts[offset & 0b11] = value;
    }
    [[nodiscard]] inline constexpr uint16_t getWord(uint8_t offset) const noexcept { return shorts[offset & 0b11]; }
};


#endif // end !defined CHIPSET_CORE_H__
