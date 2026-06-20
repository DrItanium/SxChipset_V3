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

#include "EBI.h"
#include "Pinout.h"
#include "FlexIO.h"

#define OnInvalidFlexIOResultPrint(fp, msg) \
    if (!validFlexIOResult(fp)) { \
        Serial.println(msg); \
        return false; \
    }

bool
EBIWrapperInterface::begin() noexcept {
    
    for (auto a : {
            Pin::EBI_A0,
            Pin::EBI_A1,
            Pin::EBI_A2,
            Pin::EBI_EN,
            Pin::EBI_D0,
            Pin::EBI_D1,
            Pin::EBI_D2,
            Pin::EBI_D3,
            Pin::EBI_D4,
            Pin::EBI_D5,
            Pin::EBI_D6,
            Pin::EBI_D7,
            Pin::EBI_D8,
            Pin::EBI_D9,
            Pin::EBI_D10,
            Pin::EBI_D11,
            Pin::EBI_D12,
            Pin::EBI_D13,
            Pin::EBI_D14,
            Pin::EBI_D15,
            }) {
        pinMode(a, OUTPUT);
        digitalWriteFast(a, LOW);
    }
    // force EBI_A4 and A5 to low  since we will never be accessing that
    setAddress(0);
    digitalWriteFast(Pin::EBI_EN, HIGH);
    setDataLines(0);
    return true;
}

