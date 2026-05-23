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
#include <FlexIO_t4.h>
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
#endif // end !defined CHIPSET_FLEXIO_H__
