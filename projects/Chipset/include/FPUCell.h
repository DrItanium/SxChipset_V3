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
        enum class FPUOperations : uint16_t {
            // lowest quarter is 32-bit floating point operations 
            Nothing = 0x00,
            Add32,
            Sub32,
            Mul32,
            Div32,
            Equals32,
            NotEquals32,
            GreaterThan32,
            LessThan32,
            GreaterThanOrEqual32,
            LessThanOrEqual32,
            Nothing64 = 0x40, // next quarter is 64-bit fpu operations
            Add64,
            Sub64,
            Mul64,
            Div64,
            Equals64,
            NotEquals64,
            GreaterThan64,
            LessThan64,
            GreaterThanOrEqual64,
            LessThanOrEqual64,
        };

        void update() noexcept {
            clearEnableWord();
        }
        uint16_t getWord(uint8_t offset) const noexcept {
            switch (offset) {
                case 0 ... 15:
                    return _words[offset];
                default:
                    return 0;
            }
        }
        void setWord(uint8_t offset, uint16_t value, bool enableLo = true, bool enableHi = true) noexcept {
            // ignore the ByteEnable bits since this thing only operates on
            // 16-bit values
            switch (offset) {
                case 0 ... 15:
                    _words[offset] = value;
                    break;
                default:
                    break;
            }
        }
        void onFinish() noexcept {
            if (doOperation()) {
                // do something with the arguments
                switch (getOpcode()) {
#define X(code, op, field, destination) case FPUOperations:: code : field . destination = field .src2 op field .src1; break
                    X(Add32, +, float32View, dest);
                    X(Sub32, -, float32View, dest);
                    X(Mul32, *, float32View, dest);
                    X(Div32, /, float32View, dest);
                    X(Equals32, ==, float32View, boolResult);
                    X(NotEquals32, !=, float32View, boolResult);
                    X(GreaterThan32, >, float32View, boolResult);
                    X(LessThan32, <, float32View, boolResult);
                    X(GreaterThanOrEqual32, >=, float32View, boolResult);
                    X(LessThanOrEqual32, <=, float32View, boolResult);
                    X(Add64, +, float64View, dest);
                    X(Sub64, -, float64View, dest);
                    X(Mul64, *, float64View, dest);
                    X(Div64, /, float64View, dest);
                    X(Equals64, ==, float64View, boolResult);
                    X(NotEquals64, !=, float64View, boolResult);
                    X(GreaterThan64, >, float64View, boolResult);
                    X(LessThan64, <, float64View, boolResult);
                    X(GreaterThanOrEqual64, >=, float64View, boolResult);
                    X(LessThanOrEqual64, <=, float64View, boolResult);
#undef X
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
        constexpr FPUOperations getOpcode() const noexcept {
            return static_cast<FPUOperations>(_words[1]);
        }
    private:
        union {
            uint16_t _words[16];
            struct {
                uint16_t enable;
                uint16_t op;
                uint32_t boolResult;
                float src1;
                float src2;
                float dest;
                float rest[3];
            } float32View;
            struct {
                uint16_t enable;
                uint16_t op;
                uint32_t boolResult;
                double src1;
                double src2;
                double dest;
            } float64View;
        };
};
#endif // end ! defined CHIPSET_FPU_CELL_H__

