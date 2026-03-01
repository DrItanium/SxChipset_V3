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
#ifndef INTERFACE_ENGINE_COMMON_H__
#define INTERFACE_ENGINE_COMMON_H__
#include <microshell.h>
#ifndef PROGMEM_MAPPED
#define PROGMEM_MAPPED
#endif
namespace InterfaceEngine {
    /**
     * @brief Install common microshell commands like micros, millis, and more into the given microshell instance
     */
    void installCommonCommands(struct ush_object* object) noexcept;

    /**
     * @brief A millis device that is meant to be cat'd; Add these to your dev directory
     */
    extern const struct ush_file_descriptor PROGMEM_MAPPED millisDevice;
    /**
     * @brief A millis device that is meant to be cat'd; Add these to your dev directory
     */
    extern const struct ush_file_descriptor PROGMEM_MAPPED microsDevice;

    /**
     * @brief Expose the arduino random function as a device
     */
    extern const struct ush_file_descriptor PROGMEM_MAPPED urandomDevice;

    /**
     * @brief Expose the clock speed of this device!
     */
    extern const struct ush_file_descriptor PROGMEM_MAPPED clockSpeedDevice;


    /**
     * @brief Install an interface to the EEPROM library to /dev/eeprom
     */
    void installEepromDeviceDirectory(struct ush_object* object) noexcept;

} // end namespace InterfaceEngine

#define INTERFACE_ENGINE_COMMON_DEVICES \
    InterfaceEngine::millisDevice, \
    InterfaceEngine::microsDevice, \
    InterfaceEngine::urandomDevice, \
    InterfaceEngine::clockSpeedDevice

#endif
