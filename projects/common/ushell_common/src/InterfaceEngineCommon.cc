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

#include "InterfaceEngineCommon.h"
#include <Arduino.h>
#include <EEPROM.h>

namespace InterfaceEngine {

    const ush_file_descriptor microsDevice = {
        .name = "micros",
        .description = "microseconds since boot (can easily overflow!)",
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](ush_object* self, ush_file_descriptor const* file, uint8_t** data) {
            static char buffer[16];
            snprintf(buffer, sizeof(buffer), "%ld\r\n", micros());
            buffer[sizeof(buffer) - 1] = 0;
            *data = (uint8_t*)buffer;
            return strlen((char*)(*data));
        }
    };
    const ush_file_descriptor millisDevice = {
        .name = "millis",
        .description = "milliseconds since boot",
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](ush_object* self, ush_file_descriptor const* file, uint8_t** data) {
            static char buffer[16];
            snprintf(buffer, sizeof(buffer), "%ld\r\n", millis());
            buffer[sizeof(buffer) - 1] = 0;
            *data = (uint8_t*)buffer;
            return strlen((char*)(*data));
        }
    };

    const ush_file_descriptor urandomDevice = {
        .name = "urandom",
        .description = "arduino random function as a device",
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](ush_object* self, ush_file_descriptor const* file, uint8_t** data) {
            static char buffer[16];
            snprintf(buffer, sizeof(buffer), "%ld\r\n", random());
            buffer[sizeof(buffer) - 1] = 0;
            *data = (uint8_t*)buffer;
            return strlen((char*)(*data));
        }
    };
    const ush_file_descriptor clockSpeedDevice = {
        .name = "cpufreq",
        .description = "print the clock speed of this device in Hz",
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](ush_object* self, ush_file_descriptor const* file, uint8_t** data) {
            static char buffer[16];
            snprintf(buffer, sizeof(buffer), "%ld\r\n", F_CPU);
            buffer[sizeof(buffer) - 1] = 0;
            *data = (uint8_t*)buffer;
            return strlen((char*)(*data));
        }
    };

    static const ush_file_descriptor commonCommands[] {
        {
            .name = "micros",
            .description = "microseconds since boot (can easily overflow!)",
            .help = nullptr,
            .exec = [](ush_object* self, ush_file_descriptor const* file, int argc, char* argv[]) { ush_printf(self, "%ld\r\n", micros()); }
        },
        {
            .name = "millis",
            .description = "milliseconds since boot",
            .help = nullptr,
            .exec = [](ush_object* self, ush_file_descriptor const* file, int argc, char* argv[]) { ush_printf(self, "%ld\r\n", millis()); }
        },
        {
            .name = "rand",
            .description = "get a random number",
            .help = nullptr,
            .exec = [](ush_object* self, ush_file_descriptor const* file, int argc, char* argv[]) { ush_printf(self, "%ld\r\n", random()); }
        },
        {
            .name = "cpufreq",
            .description = "print the clock speed of the current device in hz",
            .help = nullptr,
            .exec = [](ush_object* self, ush_file_descriptor const* file, int argc, char* argv[]) { ush_printf(self, "%ld\r\n", F_CPU); }
        },
    };
    ush_node_object cmds;

    static const ush_file_descriptor eepromFiles[] {
        {
            .name = "available",
            .description = "do we have access to EEPROM?",
            .help = nullptr,
            .exec = nullptr,
            .get_data = [](ush_object* self, ush_file_descriptor const* file, uint8_t** data) {
                static char buffer[16];
                snprintf(buffer, sizeof(buffer), "%ld\r\n", static_cast<long>(EEPROM.length() > 0 ? 1 : 0));
                buffer[sizeof(buffer) - 1] = 0;
                *data = (uint8_t*)buffer;
                return strlen((char*)buffer);
            }
        },
        {
            .name = "size",
            .description = "how much EEPROM do we have access to?",
            .help = nullptr,
            .exec = nullptr,
            .get_data = [](ush_object* self, ush_file_descriptor const* file, uint8_t** data) {
                static char buffer[16];
                long size = EEPROM.length();
                snprintf(buffer, sizeof(buffer), "%ld\r\n", size);
                buffer[sizeof(buffer) - 1] = 0;
                *data = (uint8_t*)buffer;
                return strlen((char*)(*data));
            }
        }
        /// @todo add the eeprom device itself
    };
    ush_node_object eeprom;
    void 
    installCommonCommands(struct ush_object* object) noexcept {
        ush_commands_add(object, &cmds, commonCommands, sizeof(commonCommands) / sizeof(commonCommands[0]));
    }
    
    void 
    installEepromDeviceDirectory(struct ush_object* object) noexcept {
        ush_node_mount(object, "/dev/eeprom", &eeprom, eepromFiles, sizeof(eepromFiles)/sizeof(eepromFiles[0]));
    }
}
