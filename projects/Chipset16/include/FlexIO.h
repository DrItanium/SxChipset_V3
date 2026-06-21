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
#ifndef CHIPSET_FLEXIO_H__
#define CHIPSET_FLEXIO_H__
#include <cstdint>
#include <functional>
#include <FlexIO_t4.h>
#include "Core.h"
#include "Pinout.h"

/**
 * @brief Set to true to turn on debugging output for the lifetime of the program
 */
constexpr auto FlexIODebugging = false;
constexpr bool 
validFlexIOResult(uint8_t input) noexcept {
    return input != 0xff;
}
constexpr bool 
uniqueValues(int a, int b, int c) noexcept {
    return (a != b) && (b != c) && (a != c) ;
}
uint32_t computeStateMachineBuffer(uint8_t outputs, std::function<uint8_t(bool, bool, bool)> fn) noexcept;

template<typename T>
concept FlexIODevice = requires(T a) {
    { a.begin() } -> std::same_as<bool>;
};
// The ready pulse handler engine is important for the purposes of execution
// but also makes sure that 
template<typename T>
concept ReadyPulseHandlerEngine = requires (T a) {
    { a.wait() };
} && FlexIODevice<T>;

struct FlexIOReadyPulseToLevelConverter final {
    public:
        using TriggerOperation = void(*)(void);
        constexpr FlexIOReadyPulseToLevelConverter(uint8_t pulseIn, uint8_t levelOut) : _in(pulseIn), _out(levelOut) { }
        constexpr FlexIOReadyPulseToLevelConverter(Pin pulseIn, Pin levelOut) : FlexIOReadyPulseToLevelConverter(static_cast<uint8_t>(pulseIn), static_cast<uint8_t>(levelOut)) { }
        [[nodiscard]] bool begin() noexcept;
        [[nodiscard]] uint32_t currentState() const noexcept { return _ioDevice->port().SHIFTSTATE; }
        [[nodiscard]] uint32_t input() const noexcept { return _ioDevice->port().PIN; }
        [[nodiscard]] uint32_t getReadyLevel() const noexcept { return input() & (1 << _outFlexPin); }
        void wait() noexcept {
            while (getReadyLevel() == _lastReadyState);
            _lastReadyState = getReadyLevel();
        }
    private:
        uint8_t _in, _inFlexPin= 0xff;
        uint8_t _out, _outFlexPin= 0xff;
        FlexIOHandler* _ioDevice = nullptr;
        uint8_t _stateMachineTimer = 0xff;
        uint8_t _state0 = 0xff;
        uint8_t _state1 = 0xff;
        uint8_t _state2 = 0xff;
        uint8_t _state3 = 0xff;
        uint32_t _lastReadyState = 0;
};

struct FlexIOTransactionDetector final {
    public:
        constexpr FlexIOTransactionDetector(uint8_t adsPin, uint8_t denPin) : _ads(adsPin), _den(denPin) { }
        constexpr FlexIOTransactionDetector(Pin ads, Pin den) 
            : FlexIOTransactionDetector(
                static_cast<uint8_t>(ads),
                static_cast<uint8_t>(den)) { }
        [[nodiscard]] bool begin() noexcept;
        [[nodiscard]] uint32_t currentState() const noexcept { return _ioDevice->port().SHIFTSTATE; }
        [[nodiscard]] bool inTransaction() const noexcept { 
            // we know that if the current state is actually state2 then we are in the proper transaction state
            return currentState() == _state2; 
        }
    private:
        uint8_t _ads, _adsFlexPin = 0xff;
        uint8_t _den, _denFlexPin = 0xff;
        FlexIOHandler* _ioDevice = nullptr;
        uint8_t _stateMachineTimer = 0xff;
        uint8_t _state0 = 0xff;
        uint8_t _state1 = 0xff;
        uint8_t _state2 = 0xff;
};

#endif // end !defined CHIPSET_FLEXIO_H__
