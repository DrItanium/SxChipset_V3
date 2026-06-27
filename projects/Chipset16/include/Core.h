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
union SplitWord16 {
    uint16_t value;
    uint8_t bytes[2];
    constexpr SplitWord16(uint16_t v= 0) noexcept : value(v) { }
};
union SplitWord32 {
    uint8_t bytes[4];
    uint16_t shorts[2];
    uint32_t value;
    uint8_t lineOffset : 4;
    struct {
        uint8_t offset : 4;
        uint32_t targetCellBlock : 20;
        uint8_t targetBlock; // 16M section
    } components;
    struct {
        uint32_t offsetAddress : 24;
        uint8_t blockIndex; 
    } blockAddress;
    struct {
        uint8_t offset : 4;
        uint32_t index : 12;
    } sramAddress;
    // simple conversion operators
    explicit operator uint32_t() const noexcept { return value; }
    explicit operator uint8_t() const noexcept { return bytes[0]; }
    explicit operator uint16_t() const noexcept { return shorts[0]; }
};
static_assert(sizeof(SplitWord32) == sizeof(uint32_t));
union SplitWord64 {
    uint8_t bytes[sizeof(uint64_t) / sizeof(uint8_t)];
    uint16_t shorts[sizeof(uint64_t) / sizeof(uint16_t)];
    uint32_t words[sizeof(uint64_t) / sizeof(uint32_t)];
    uint64_t value;
};

inline uint32_t getCurrentCycleCount() noexcept {
    return ARM_DWT_CYCCNT;
}

inline uint32_t nanosecondsToCycles(uint32_t nsec) noexcept {
    return ((F_CPU_ACTUAL>>16) * nsec) / (1000000000UL>>16);
}

template<typename From, typename To>
union Converter {
    From from;
    To to;
};

/**
 * @brief Track how many cycles elapsed for the given scope
 */
template<bool E>
struct TimeTracker final {
    TimeTracker(const char*) { }
    ~TimeTracker() { }
};

#ifdef USB_TRIPLE_SERIAL
template<>
struct TimeTracker<true> final {
    TimeTracker(const char* prefix) : _prefix(prefix), _startTime(getCurrentCycleCount()) { }
    ~TimeTracker() {
        auto endTime = getCurrentCycleCount();
        SerialUSB1.printf("%s, cycleCount: %d cycles\n", _prefix, endTime - _startTime);
    }
    private:
        const char* _prefix;
        uint32_t _startTime;
};
#endif

#endif // end !defined CHIPSET_CORE_H__
