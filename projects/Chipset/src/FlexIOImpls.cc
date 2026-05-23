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

#include <cstdint>
#include <functional>
#include "FlexIO.h"

uint32_t 
computeStateMachineBuffer(uint8_t outputs, std::function<uint8_t(bool, bool, bool)> fn) noexcept {
    uint32_t result = (static_cast<uint32_t>(outputs) << 24);
    for (int i = 0; i < 8; ++i) {
        result |= static_cast<uint32_t>(fn(0b001 & i, 0b010 & i, 0b100 & i)) << (i * 3);
    }
    return result;
}
#define OnInvalidFlexIOResultPrint(fp, msg) \
    if (!validFlexIOResult(fp)) { \
        Serial.println(msg); \
        return false; \
    }

bool
FlexIOTransactionDetector::begin() {
    // The algorithm that we use to start processing memory transactions is as
    // follows in a loop
    // 1) Wait for ADS to go low
    // 2) Wait for ADS to go high and DEN to go low
    // 3) Wait for DEN to go high
    //
    //
    // This design requires 3 state machines that are pretty easy to configure.
    // This state machine is running _constantly_
    // all of these should be unique
    //
    // I cannot drop it down to two states because the i960 will sometimes
    // manipulate the DEN pin outside a transaction... very obnoxious...
    //
    // 
    if (!uniqueValues(_ads, _den, _transactionPin)) {
        Serial.println("One or more pins are the same input or output!");
        return false;
    }
    _ioDevice = FlexIOHandler::mapIOPinToFlexIOHandler(_ads, _adsFlexPin);
    if (!_ioDevice) {
        Serial.println("Could not acquire a FlexIO device given the ADS pin!");
        return false;
    }
    OnInvalidFlexIOResultPrint(_adsFlexPin, "Could not map the ads pin");
    _denFlexPin = _ioDevice->mapIOPinToFlexPin(_den);
    OnInvalidFlexIOResultPrint(_denFlexPin, "Could not map the den pin");
    _transactionFlexPin = _ioDevice->mapIOPinToFlexPin(_transactionPin);
    OnInvalidFlexIOResultPrint(_transactionFlexPin, "Could not map the transaction pin");
    if ((_denFlexPin - _adsFlexPin) != 1) {
        Serial.println("den flex and ads pins are not close enough together");
        return false;
    }
    // not enough sanity checking yet but for testing purposes this is fine
    auto* p = &_ioDevice->port();
    _stateMachineTimer = _ioDevice->requestTimers(1);
    OnInvalidFlexIOResultPrint(_stateMachineTimer, "Timer request failed");
    _state0 = _ioDevice->requestShifter();
    OnInvalidFlexIOResultPrint(_state0, "Could not allocate shifter for state 0");
    _state1 = _ioDevice->requestShifter();
    OnInvalidFlexIOResultPrint(_state1, "Could not allocate shifter for state 1");
    _state2 = _ioDevice->requestShifter();
    OnInvalidFlexIOResultPrint(_state2, "Could not allocate shifter for state 2");
    if constexpr (FlexIODebugging) {
        Serial.printf("States: [ %d, %d, %d ]\n", _state0, _state1, _state2);
    }
    uint32_t outputConfiguration;
    // when set, the different components disable the corresponding output
    // drive
    switch (_transactionFlexPin) {
        case 0: // pin 0
            outputConfiguration = FLEXIO_SHIFTCFG_PWIDTH(0b1111) | FLEXIO_SHIFTCFG_SSTOP(0b11)  | FLEXIO_SHIFTCFG_SSTART(0b10);
            break;
        case 1:
            outputConfiguration = FLEXIO_SHIFTCFG_PWIDTH(0b1111) | FLEXIO_SHIFTCFG_SSTOP(0b11)  | FLEXIO_SHIFTCFG_SSTART(0b01);
            break;
        case 2:
            outputConfiguration = FLEXIO_SHIFTCFG_PWIDTH(0b1111) | FLEXIO_SHIFTCFG_SSTOP(0b10)  | FLEXIO_SHIFTCFG_SSTART(0b11);
            break;
        case 3:
            outputConfiguration = FLEXIO_SHIFTCFG_PWIDTH(0b1111) | FLEXIO_SHIFTCFG_SSTOP(0b01)  | FLEXIO_SHIFTCFG_SSTART(0b11);
            break;
        case 4:
            outputConfiguration = FLEXIO_SHIFTCFG_PWIDTH(0b1110) | FLEXIO_SHIFTCFG_SSTOP(0b11)  | FLEXIO_SHIFTCFG_SSTART(0b11);
            break;
        case 5:
            outputConfiguration = FLEXIO_SHIFTCFG_PWIDTH(0b1101) | FLEXIO_SHIFTCFG_SSTOP(0b11)  | FLEXIO_SHIFTCFG_SSTART(0b11);
            break;
        case 6:
            outputConfiguration = FLEXIO_SHIFTCFG_PWIDTH(0b1011) | FLEXIO_SHIFTCFG_SSTOP(0b11)  | FLEXIO_SHIFTCFG_SSTART(0b11);
            break;
        case 7:
            outputConfiguration = FLEXIO_SHIFTCFG_PWIDTH(0b0111) | FLEXIO_SHIFTCFG_SSTOP(0b11)  | FLEXIO_SHIFTCFG_SSTART(0b11);
            break;
        default:
            Serial.printf("Bad Transaction Pin of %d\n", _transactionFlexPin);
            return false;
    }
    p->SHIFTSTATE = 0;
    // we only have one supported output configuration
    p->SHIFTCFG[_state0] = outputConfiguration;
    p->SHIFTCFG[_state1] = outputConfiguration;
    p->SHIFTCFG[_state2] = outputConfiguration;
    // so we need to configure State0 transitions
    // state0 -> in transaction is high
    //  0bxx1 -> state0
    //  0bxx0 -> state1
    p->SHIFTBUF[_state0] = computeStateMachineBuffer(0xFF, [this](bool ads, bool, bool) -> uint8_t { return ads ? _state0 : _state1; });
    if constexpr (FlexIODebugging) {
        Serial.printf("SHIFTBUF[%d] = %x\n", _state0, p->SHIFTBUF[_state0]);
    }
    // state1 -> in transaction is high
    //  0bxx1 -> state2
    //  0bxx0 -> state1
    p->SHIFTBUF[_state1] = computeStateMachineBuffer(0xFF, [this](bool ads, bool den, bool) -> uint8_t { return (ads && !den) ? _state2 : _state1; });
    if constexpr (FlexIODebugging) {
        Serial.printf("SHIFTBUF[%d] = %x\n", _state1, p->SHIFTBUF[_state1]);
    }
    // state2 -> in transaction is low
    //  0bx0x => goto state 2
    //  0bx1x => goto state 0
    p->SHIFTBUF[_state2] = computeStateMachineBuffer(0x00, [this](bool ads, bool den, bool) -> uint8_t { return den ? _state0 : _state2; });
    if constexpr (FlexIODebugging) {
        Serial.printf("SHIFTBUF[%d] = %x\n", _state2, p->SHIFTBUF[_state2]);
    }
    // example value is 0x0080_0206 
    // 0x06 => SMOD is in state mode
    // 0x02 => FXIO_Di (base pin)
    // 0x80 => Shift on negedge of shift clock
    // 0x00 => timer 0
    uint32_t shiftConfiguration = FLEXIO_SHIFTCTL_MODE_STATE |
        FLEXIO_SHIFTCTL_PINSEL(_adsFlexPin) |
        FLEXIO_SHIFTCTL_SHIFT_ON_RISING_EDGE |
        FLEXIO_SHIFTCTL_PINMODE_OUTPUT |
        FLEXIO_SHIFTCTL_TIMSEL(_stateMachineTimer);
    p->SHIFTCTL[_state0] = shiftConfiguration;
    p->SHIFTCTL[_state1] = shiftConfiguration;
    p->SHIFTCTL[_state2] = shiftConfiguration;
    _ioDevice->setClock(24'000'000 * 16);
    if constexpr (FlexIODebugging) {
        auto clockSpeed = _ioDevice->computeClockRate();
        Serial.printf("FlexIO Clock Speed: %d\n", clockSpeed);
    }
    p->TIMCMP[_stateMachineTimer] = 0; // fastest baud rate available
    p->TIMCFG[_stateMachineTimer] = 0; // always enabled
    // 0x0000'0003
    // 0x0F03'0303
    p->TIMCTL[_stateMachineTimer] = FLEXIO_TIMCTL_MODE_16BIT;

    p->CTRL = FLEXIO_CTRL_FLEXEN | FLEXIO_CTRL_FASTACC;
    if (!_ioDevice->setIOPinToFlexMode(_ads)) {
        Serial.printf("Could not set ADS pin (%d) to flex mode!\n", _ads);
        return false;
    }
    if (!_ioDevice->setIOPinToFlexMode(_den)) {
        Serial.printf("Could not set DEN pin (%d) to flex mode!\n", _den);
        return false;
    }
    if (!_ioDevice->setIOPinToFlexMode(_transactionPin)) {
        Serial.printf("Could not set IN TRANSACTION pin (%d) to flex mode!\n", _transactionPin);
        return false;
    }
    if constexpr (FlexIODebugging) {
        Serial.printf("ADS: %d, DEN: %d, TRANSACTION: %d\n", _ads, _den, _transactionPin);
    }
    // we just walk through this over and over
    *(portControlRegister(_transactionPin)) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(2);
    *(portControlRegister(_den)) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(2) | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3);
    *(portControlRegister(_ads)) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(2) | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3);
    return true;
}


bool
FlexIOReadyPulseToLevelConverter::begin() {
    // We want to convert the ready pulse into a level signal using FlexIO here
    // is the loop design. In order to properly detect the change we need to
    // toggle the level each time the pulse is detected
    //
    // 1) wait for READY to go LOW (while this is happening READY level is high)
    // 2) wait for READY to go HIGH (while this is happening READY level is low)
    // 3) wait for READY to go LOW (while this is happening READY level is low)
    // 4) wait for READY to go HIGH (while this is happening READY level is high)
    // 
    // The idea behind this is that the state machine watches the READY pulse
    // generated by the AVR and toggles the LEVEL output as soon as the falling
    // edge is finished. 
    //
    
    // all of these should be unique
    if (_in == _out) {
        Serial.println("Input and output pin are the same!");
        return false;
    }
    _ioDevice = FlexIOHandler::mapIOPinToFlexIOHandler(_in, _inFlexPin);
    if (!_ioDevice) {
        Serial.println("Could not acquire the FlexIO device!");
        return false;
    }
    OnInvalidFlexIOResultPrint(_inFlexPin, "Could not map the ready pulse (input) pin");
    _outFlexPin = _ioDevice->mapIOPinToFlexPin(_out);
    OnInvalidFlexIOResultPrint(_outFlexPin, "Could not map the ready level (output) pin");
    // not enough sanity checking yet but for testing purposes this is fine
    auto* p = &_ioDevice->port();
    _stateMachineTimer = _ioDevice->requestTimers(1);
    OnInvalidFlexIOResultPrint(_stateMachineTimer, "Timer request failed!");
    _state0 = _ioDevice->requestShifter();
    OnInvalidFlexIOResultPrint(_state0, "Could not allocate shifter for state 0");
    _state1 = _ioDevice->requestShifter();
    OnInvalidFlexIOResultPrint(_state1, "Could not allocate shifter for state 1");
    _state2 = _ioDevice->requestShifter();
    OnInvalidFlexIOResultPrint(_state2, "Could not allocate shifter for state 2");
    _state3 = _ioDevice->requestShifter();
    OnInvalidFlexIOResultPrint(_state3, "Could not allocate shifter for state 3");
    if constexpr (FlexIODebugging) {
        Serial.printf("States: [ %d, %d, %d, %d ]\n", _state0, _state1, _state2, _state3);
    }
    uint32_t outputConfiguration;
    // when set, the different components disable the corresponding output
    // drive
    switch (_outFlexPin) {
        case 0: // pin 0
            outputConfiguration = FLEXIO_SHIFTCFG_PWIDTH(0b1111) | FLEXIO_SHIFTCFG_SSTOP(0b11)  | FLEXIO_SHIFTCFG_SSTART(0b10);
            break;
        case 1:
            outputConfiguration = FLEXIO_SHIFTCFG_PWIDTH(0b1111) | FLEXIO_SHIFTCFG_SSTOP(0b11)  | FLEXIO_SHIFTCFG_SSTART(0b01);
            break;
        case 2:
            outputConfiguration = FLEXIO_SHIFTCFG_PWIDTH(0b1111) | FLEXIO_SHIFTCFG_SSTOP(0b10)  | FLEXIO_SHIFTCFG_SSTART(0b11);
            break;
        case 3:
            outputConfiguration = FLEXIO_SHIFTCFG_PWIDTH(0b1111) | FLEXIO_SHIFTCFG_SSTOP(0b01)  | FLEXIO_SHIFTCFG_SSTART(0b11);
            break;
        case 4:
            outputConfiguration = FLEXIO_SHIFTCFG_PWIDTH(0b1110) | FLEXIO_SHIFTCFG_SSTOP(0b11)  | FLEXIO_SHIFTCFG_SSTART(0b11);
            break;
        case 5:
            outputConfiguration = FLEXIO_SHIFTCFG_PWIDTH(0b1101) | FLEXIO_SHIFTCFG_SSTOP(0b11)  | FLEXIO_SHIFTCFG_SSTART(0b11);
            break;
        case 6:
            outputConfiguration = FLEXIO_SHIFTCFG_PWIDTH(0b1011) | FLEXIO_SHIFTCFG_SSTOP(0b11)  | FLEXIO_SHIFTCFG_SSTART(0b11);
            break;
        case 7:
            outputConfiguration = FLEXIO_SHIFTCFG_PWIDTH(0b0111) | FLEXIO_SHIFTCFG_SSTOP(0b11)  | FLEXIO_SHIFTCFG_SSTART(0b11);
            break;
        default:
            Serial.printf("Bad index for output pin (%d)\n", _outFlexPin);
            return false;
    }
    p->SHIFTSTATE = 0;
    // we only have one supported output configuration
    p->SHIFTCFG[_state0] = outputConfiguration;
    p->SHIFTCFG[_state1] = outputConfiguration;
    p->SHIFTCFG[_state2] = outputConfiguration;
    p->SHIFTCFG[_state3] = outputConfiguration;
    // so we need to configure State0 transitions
    // state0 -> ready level is high
    //  0bxx1 -> state0
    //  0bxx0 -> state1
    p->SHIFTBUF[_state0] = computeStateMachineBuffer(0xFF, [this](bool ready, bool, bool) -> uint8_t { return ready ? _state0 : _state1; });
    if constexpr (FlexIODebugging) {
        Serial.printf("SHIFTBUF[%d] = %x\n", _state0, p->SHIFTBUF[_state0]);
    }
    // state1 -> ready level is low
    //  0bxx1 -> state2
    //  0bxx0 -> state1
    p->SHIFTBUF[_state1] = computeStateMachineBuffer(0x00, [this](bool ready, bool, bool) -> uint8_t { return ready ? _state2 : _state1; });
    if constexpr (FlexIODebugging) {
        Serial.printf("SHIFTBUF[%d] = %x\n", _state1, p->SHIFTBUF[_state1]);
    }
    // state2 -> ready level is low
    //  0bxx1 => goto state 2
    //  0bxx0 => goto state 3
    p->SHIFTBUF[_state2] = computeStateMachineBuffer(0x00, [this](bool ready, bool, bool) -> uint8_t { return ready ? _state2 : _state3; });
    if constexpr (FlexIODebugging) {
        Serial.printf("SHIFTBUF[%d] = %x\n", _state2, p->SHIFTBUF[_state2]);
    }
    // state3 -> ready level is high
    //  0bxx0 => goto state 3
    //  0bxx1 => goto state 0
    p->SHIFTBUF[_state3] = computeStateMachineBuffer(0xFF, [this](bool ready, bool, bool) -> uint8_t { return ready ? _state0 : _state3; });
    if constexpr (FlexIODebugging) {
        Serial.printf("SHIFTBUF[%d] = %x\n", _state3, p->SHIFTBUF[_state3]);
    }
    // example value is 0x0080_0206 
    // 0x06 => SMOD is in state mode
    // 0x02 => FXIO_Di (base pin)
    // 0x80 => Shift on negedge of shift clock
    // 0x00 => timer 0
    uint32_t shiftConfiguration = FLEXIO_SHIFTCTL_MODE_STATE |
        FLEXIO_SHIFTCTL_PINSEL(_inFlexPin) |
        FLEXIO_SHIFTCTL_SHIFT_ON_RISING_EDGE |
        FLEXIO_SHIFTCTL_PINMODE_OUTPUT |
        FLEXIO_SHIFTCTL_TIMSEL(_stateMachineTimer);
    p->SHIFTCTL[_state0] = shiftConfiguration;
    p->SHIFTCTL[_state1] = shiftConfiguration;
    p->SHIFTCTL[_state2] = shiftConfiguration;
    p->SHIFTCTL[_state3] = shiftConfiguration;
    _ioDevice->setClock(24'000'000 * 16);
    if constexpr (FlexIODebugging) {
        auto clockSpeed = _ioDevice->computeClockRate();
        Serial.printf("FlexIO Clock Speed: %d\n", clockSpeed);
    }
    p->TIMCMP[_stateMachineTimer] = 0; // fastest baud rate available
    p->TIMCFG[_stateMachineTimer] = 0; // always enabled
    // 0x0000'0003
    // 0x0F03'0303
    p->TIMCTL[_stateMachineTimer] = FLEXIO_TIMCTL_MODE_16BIT;

    p->CTRL = FLEXIO_CTRL_FLEXEN | FLEXIO_CTRL_FASTACC;
    if (!_ioDevice->setIOPinToFlexMode(_in)) {
        Serial.printf("Could not set IN pin (%d) to flex mode!\n", _in);
        return false;
    }
    if (!_ioDevice->setIOPinToFlexMode(_out)) {
        Serial.printf("Could not set OUT pin (%d) to flex mode!\n", _out);
        return false;
    }
    if constexpr (FlexIODebugging) {
        Serial.printf("Input: %d [Flex: %d], Output: %d [Flex: %d]\n", _in, _inFlexPin, _out, _outFlexPin);
    }
    // we just walk through this over and over
    *(portControlRegister(_out)) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(2);
    *(portControlRegister(_in)) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(2) | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3);
    return true;
}


