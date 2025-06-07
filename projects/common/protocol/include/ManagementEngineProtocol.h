#ifndef MANAGEMENT_ENGINE_PROTOCOL_H__
#define MANAGEMENT_ENGINE_PROTOCOL_H__
#include <stdint.h>

enum class ManagementEngineReceiveOpcode : uint8_t {
    SetMode,
};
enum class ManagementEngineRequestOpcode : uint8_t {
    CPUClockConfiguration,
};
constexpr bool valid(ManagementEngineReceiveOpcode code) noexcept {
    switch (code) {
        case ManagementEngineReceiveOpcode::SetMode:
            return true;
        default:
            return false;
    }
}
constexpr bool valid(ManagementEngineRequestOpcode code) noexcept {
    switch (code) {
        case ManagementEngineRequestOpcode::CPUClockConfiguration:
            return true;
        default:
            return false;
    }
}

#endif // !defined MANAGEMENT_ENGINE_PROTOCOL_H__
