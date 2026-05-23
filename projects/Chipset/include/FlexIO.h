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

struct FlexIOReadyPulseToLevelConverter: public FlexIOHandlerCallback {
    public:
        bool call_back(FlexIOHandler *pflex) override { return false; }
        FlexIOReadyPulseToLevelConverter(uint8_t pulseIn, uint8_t levelOut) : _in(pulseIn), _out(levelOut) { }
        FlexIOReadyPulseToLevelConverter(Pin pulseIn, Pin levelOut) : FlexIOReadyPulseToLevelConverter(static_cast<uint8_t>(pulseIn), static_cast<uint8_t>(levelOut)) { }
        virtual ~FlexIOReadyPulseToLevelConverter() { }
        bool begin();
        void end() { }
    private:
        uint8_t _in, _inFlexPin= 0xff;
        uint8_t _out, _outFlexPin= 0xff;
        FlexIOHandler* _ioDevice = nullptr;
        uint8_t _stateMachineTimer = 0xff;
        uint8_t _state0 = 0xff;
        uint8_t _state1 = 0xff;
        uint8_t _state2 = 0xff;
        uint8_t _state3 = 0xff;
};

struct FlexIOTransactionDetector : public FlexIOHandlerCallback {
    public:
        bool call_back(FlexIOHandler *pflex) override { return false; }
        FlexIOTransactionDetector(uint8_t adsPin, uint8_t denPin, uint8_t inTransactionPin) : _ads(adsPin), _den(denPin), _transactionPin(inTransactionPin) { }
        FlexIOTransactionDetector(Pin ads, Pin den, Pin inTransaction) 
            : FlexIOTransactionDetector(
                static_cast<uint8_t>(ads),
                static_cast<uint8_t>(den),
                static_cast<uint8_t>(inTransaction)) { }
        virtual ~FlexIOTransactionDetector() { }
        bool begin();
        void end() { }
    private:
        uint8_t _ads, _adsFlexPin = 0xff;
        uint8_t _den, _denFlexPin = 0xff;
        uint8_t _transactionPin, _transactionFlexPin = 0xff;
        FlexIOHandler* _ioDevice = nullptr;
        uint8_t _stateMachineTimer = 0xff;
        uint8_t _state0 = 0xff;
        uint8_t _state1 = 0xff;
        uint8_t _state2 = 0xff;
};

struct FlexIOAddressAssigner : public FlexIOHandlerCallback {
    public:
        bool call_back(FlexIOHandler *pflex) override { return false; }
        FlexIOAddressAssigner(uint8_t a0, uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4, uint8_t a5) : _addressLines{a0, a1, a2, a3, a4, a5} { }
        FlexIOAddressAssigner(
                Pin a0Pin,
                Pin a1Pin,
                Pin a2Pin,
                Pin a3Pin,
                Pin a4Pin,
                Pin a5Pin
                ) : FlexIOAddressAssigner(
                static_cast<uint8_t>(a0Pin),
                static_cast<uint8_t>(a1Pin),
                static_cast<uint8_t>(a2Pin),
                static_cast<uint8_t>(a3Pin),
                static_cast<uint8_t>(a4Pin),
                static_cast<uint8_t>(a5Pin)
                ) { }
        virtual ~FlexIOAddressAssigner() { }
        void end() { }
        bool begin();
    private:
        uint8_t _addressLines[6];
        uint8_t _addressFlexLines[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
        FlexIOHandler* _ioDevice = nullptr;
        uint8_t _shifter = 0xff;
        uint8_t _timer = 0xff;
};

#endif // end !defined CHIPSET_FLEXIO_H__
