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
#ifndef CHIPSET_FPU_CELL_H__
#define CHIPSET_FPU_CELL_H__
#include <cstdint>
#include "MemoryCell.h"

/**
 * Takes advantage of the FPU built into the teensy to assist the i960
 */
class FPUCell {
    public:

        void update() noexcept {
            clearEnableWord();
        }
        uint16_t getWord(uint8_t offset) const noexcept {
            return _words[offset & 0b1111];
        }
        void setWord(uint8_t offset, uint16_t value, bool enableLo = true, bool enableHi = true) noexcept {
            // ignore the ByteEnable bits since this thing only operates on
            // 16-bit values
            _words[offset & 0b1111] = value;
        }
        void onFinish() noexcept {
            if (doOperation()) {
                // do something with the arguments
                switch (getOpcode()) {
                    default:
                        break;
                }
            }
        }
    private:
        void clearEnableWord() noexcept {
            _words[0] = 0;
        }
        constexpr bool doOperation() const noexcept {
            return _words[0] != 0;
        }
        constexpr uint16_t getOpcode() const noexcept {
            return _words[1];
        }
    private:
        union {
            uint16_t _words[16];
            float _f32s[8];
            double _f64s[4];
        };
};
#endif // end ! defined CHIPSET_FPU_CELL_H__

