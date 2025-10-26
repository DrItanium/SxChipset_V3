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
            return true;
        default:
            return false;
    }
}

#endif // !defined MANAGEMENT_ENGINE_PROTOCOL_H__
