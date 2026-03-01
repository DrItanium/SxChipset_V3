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
#include <Arduino.h>

namespace InterfaceEngine {
    static const ush_file_descriptor PROGMEM_MAPPED i960Commands[] {
        {
            .name = "cpustat",
            .description = nullptr,
            .help = nullptr,
            .exec = [](ush_object* self, ush_file_descriptor const* file, int argc, char* argv[]) { 
                ush_printf(self, "CPU Execution Statistics\r\nRunning: %s\r\nBus Held: %s\r\nBus Locked: %s\r\n", 
                        i960::cpuRunning() ? "yes" : "no",
                        i960::isBusHeld() ? "yes" : "no",
                        i960::isBusLocked() ? "yes" : "no");
            }
        },
        {
            .name = "cpu_reset",
            .description = "Put the i960 CPU into the reset state",
            .help = nullptr,
            .exec = [](ush_object* self, ush_file_descriptor const*, int argc, char* argv[]) {
                if (i960::cpuRunning()) {
                    i960::putCPUInReset();
                    ush_printf(self, "cpu now in reset...\r\n");
                } else {
                    ush_printf(self, "cpu already in reset...\r\n");
                }
            },
        },
        {
            .name = "cpu_boot",
            .description = "Release the i960 CPU from the reset state",
            .help = nullptr,
            .exec = [](ush_object* self, ush_file_descriptor const*, int argc, char* argv[]) {
                if (!i960::cpuRunning()) {
                    i960::pullCPUOutOfReset();
                    ush_printf(self, "booting cpu...\r\n");
                } else {
                    ush_printf(self, "cpu already booted...\r\n");
                }
            },
        },
        {
            .name = "hold_bus",
            .description = "Request the control of the bus from the i960 (not instant)",
            .help = nullptr,
            .exec = [](ush_object* self, ush_file_descriptor const*, int argc, char* argv[]) {
                i960::holdBus();
                if (i960::isBusHeld()) {
                    ush_printf(self, "bus is already held...\r\n");
                } else {
                    ush_printf(self, "requesting bus hold...\r\n");
                }
            },
        },
        {
            .name = "release_bus",
            .description = "Relinquish control of the i960 Bus",
            .help = nullptr,
            .exec = [](ush_object* self, ush_file_descriptor const*, int argc, char* argv[]) {
                if (i960::isBusHeld()) {
                    i960::releaseBus();
                    ush_printf(self, "released control of the bus...\r\n");
                } else {
                    ush_printf(self, "bus isn't currently being held...\r\n");
                }
            },
        },
    };
    static ush_node_object i960cmds;
    void 
    installI960Commands(struct ush_object* object) noexcept {
        ush_commands_add(object, &i960cmds, i960Commands, ComputeFileSize(i960Commands));
    }
    static constexpr uint8_t PROGMEM_MAPPED TrueBuffer[4] { '1', '\r', '\n', 0, };
    static constexpr uint8_t PROGMEM_MAPPED FalseBuffer[4] { '0', '\r', '\n', 0, };
    static const ush_file_descriptor PROGMEM_MAPPED i960Files[] {
        {
            .name = "running",
            .description = nullptr,
            .help = nullptr,
            .exec = nullptr,
            .get_data = [](ush_object* self, ush_file_descriptor const* file, uint8_t** data) {
                *data = (i960::cpuRunning()) ? TrueBuffer : FalseBuffer;
                return 3;
            }
        },
        {
            .name = "lock",
            .description = nullptr,
            .help = nullptr,
            .exec = nullptr,
            .get_data = [](ush_object* self, ush_file_descriptor const* file, uint8_t** data) {
                *data = (i960::isBusLocked()) ? TrueBuffer : FalseBuffer;
                return 3;
            }
        },
        {
            .name = "reset",
            .description = nullptr,
            .help = nullptr,
            .exec = nullptr,
            .get_data = [](ush_object* self, ush_file_descriptor const* file, uint8_t** data) {
                *data = (i960::cpuRunning()) ? TrueBuffer : FalseBuffer;
                return 3;
            },
            .set_data = [](ush_object* self, ush_file_descriptor const* file, uint8_t* data, size_t size) {
                long value = 0;
                if (sscanf((const char*)data, "%lu", &value) == EOF) {
                    ush_print_status(self, USH_STATUS_ERROR_COMMAND_SYNTAX_ERROR);
                    return;
                }
                if (value != 0) {
                    i960::pullCPUOutOfReset();
                } else {
                    i960::putCPUInReset();
                }
            },
        },
        {
            .name = "hlda",
            .description = nullptr,
            .help = nullptr,
            .exec = nullptr,
            .get_data = [](ush_object* self, ush_file_descriptor const* file, uint8_t** data) {
                *data = (i960::isBusHeld()) ? TrueBuffer : FalseBuffer;
                return 3;
            },
            .set_data = [](ush_object* self, ush_file_descriptor const* file, uint8_t* data, size_t size) {
                long value = 0;
                if (sscanf((const char*)data, "%lu", &value) == EOF) {
                    ush_print_status(self, USH_STATUS_ERROR_COMMAND_SYNTAX_ERROR);
                    return;
                }
                if (value != 0) {
                    i960::holdBus();
                } else {
                    i960::releaseBus();
                }
            },
        },
        {
            .name = "hold",
            .description = nullptr,
            .help = nullptr,
            .exec = nullptr,
            .get_data = nullptr,
            .set_data = [](ush_object* self, ush_file_descriptor const* file, uint8_t* data, size_t size) {
                long value = 0;
                if (sscanf((const char*)data, "%lu", &value) == EOF) {
                    ush_print_status(self, USH_STATUS_ERROR_COMMAND_SYNTAX_ERROR);
                    return;
                }
                if (value != 0) {
                    i960::holdBus();
                } else {
                    i960::releaseBus();
                }
            },
        },

    };
    static ush_node_object i960Dir;

    void 
    installI960Devices(struct ush_object* object) noexcept {
        ush_node_mount(object, "/dev/i960", &i960Dir, i960Files, ComputeFileSize(i960Files));
    }
}
