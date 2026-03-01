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

#include "i960CommonInterface.h"

namespace InterfaceEngine {
    static const ush_file_descriptor PROGMEM_MAPPED i960Commands[] {
        {
            .name = "lscpu",
            .description = "print information about the CPU",
            .help = nullptr,
            .exec = [](ush_object* self, ush_file_descriptor const* file, int argc, char* argv[]) { 
                ush_printf(self, "CPU Execution Statistics\r\nRunning: %s\r\nBus Held: %s\r\nBus Locked: %s\r\n", 
                        i960::cpuRunning() ? "yes" : "no",
                        i960::isBusHeld() ? "yes" : "no",
                        i960::isBusLocked() ? "yes" : "no");
            }
        },
    };
    static ush_node_object i960cmds;
    void 
    installI960Commands(struct ush_object* object) noexcept {
        ush_commands_add(object, &i960cmds, i960Commands, sizeof(i960Commands) / sizeof(i960Commands[0]));
    }

    void 
    installI960Devices(struct ush_object* object) noexcept {

    }
}
