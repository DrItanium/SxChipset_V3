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
#ifndef I960_COMMON_INTERFACE_H__ 
#define I960_COMMON_INTERFACE_H__ 
#include "InterfaceCommonConcepts.h"
namespace InterfaceEngine::i960 {
    // these are all routines that are common between the chipset and
    // management engine. So we want to provide declarations that the
    // implementations must provide for
    void holdBus() noexcept;
    void releaseBus() noexcept;
    bool isBusHeld() noexcept;
    bool isBusLocked() noexcept;
    bool cpuRunning() noexcept;
    void putCPUInReset() noexcept;
    void pullCPUOutOfReset() noexcept;
}

namespace InterfaceEngine {
    void installI960Commands(struct ush_object* object) noexcept;
    void installI960Devices(struct ush_object* object) noexcept;
}
#endif
