// SxChipset_v3
// Copyright (c) 2025-2026, Joshua Scoggins
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#ifndef MANAGEMENT_ENGINE_PROTOCOL_H__
#define MANAGEMENT_ENGINE_PROTOCOL_H__
#include <stdint.h>

enum class ManagementEngineReceiveOpcode : uint8_t {
    SetMode,
    PutInReset,
    PullOutOfReset,
    HoldBus,
    ReleaseBus,
};
enum class ManagementEngineRequestOpcode : uint8_t {
    CPUClockConfiguration,
    BusIsHeld,
    BusIsLocked,
    RevID,
    DevID,
    SerialNumber,
    RandomSeed,
    ChipIsReady,
    CpuRunning,
};
constexpr bool valid(ManagementEngineReceiveOpcode code) noexcept {
    switch (code) {
        case ManagementEngineReceiveOpcode::SetMode:
        case ManagementEngineReceiveOpcode::PutInReset:
        case ManagementEngineReceiveOpcode::PullOutOfReset:
        case ManagementEngineReceiveOpcode::HoldBus:
        case ManagementEngineReceiveOpcode::ReleaseBus:
            return true;
        default:
            return false;
    }
}
constexpr bool valid(ManagementEngineRequestOpcode code) noexcept {
    switch (code) {
        case ManagementEngineRequestOpcode::CPUClockConfiguration:
        case ManagementEngineRequestOpcode::BusIsHeld:
        case ManagementEngineRequestOpcode::BusIsLocked:
        case ManagementEngineRequestOpcode::RevID:
        case ManagementEngineRequestOpcode::DevID:
        case ManagementEngineRequestOpcode::SerialNumber:
        case ManagementEngineRequestOpcode::RandomSeed:
        case ManagementEngineRequestOpcode::ChipIsReady:
        case ManagementEngineRequestOpcode::CpuRunning:
            return true;
        default:
            return false;
    }
}

#endif // !defined MANAGEMENT_ENGINE_PROTOCOL_H__
