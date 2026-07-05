/*
SxChipset_v3
Copyright (c) 2025, Joshua Scoggins
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
#include <Arduino.h>
#include <type_traits>
#include <functional>
#include <memory>
// map should be useful for keeping track of file handles
#include <map>
#include <optional>

#include <Wire.h>
#include <SD.h>
#include <NativeEthernet.h>
#include <Entropy.h>
#include <EEPROM.h>
#include <FastCRC.h>
#include <LittleFS.h>
#include <Metro.h>
#include <RTClib.h>
#include <IntervalTimer.h>
#include <USBHost_t36.h>
// gamepad qt
#include <Adafruit_seesaw.h>
#include <Adafruit_I2CDevice.h>
// interface to virtual TFT display
//#include <Adafruit_ILI9341.h>
#include <ILI9341_t3.h>


#include "Pinout.h"
#include "MemoryCell.h"
#include "Core.h"
#include "ManagementEngineProtocol.h"
#include "FlexIO.h"
#include <fileuids.h>
#include "EBI.h"

constexpr uint32_t OnboardSRAMCacheSize = 0x10000;
constexpr uint32_t OnboardSRAM1CacheSize = OnboardSRAMCacheSize - 0x1000;
constexpr uint32_t OnboardSRAM2CacheSize = 0x10000;
constexpr auto MemoryPoolSizeInBytes = (16 * 1024 * 1024);  // 16 megabyte psram pool
volatile bool systemCounterEnabled = false;
bool cpuIsRunning = false;
enum class TimerTrackingTargets {
    None,
    GetAddress,
    DoNothingTransaction,
    ReadLo8,
    ReadHi8,
    WriteDataLines,
    DetermineActionKind,
    DoWriteAction,
    WriteActionCycle,
    DoMemoryCellReadTransaction,
    DoMemoryCellWriteTransaction,
    DoMemoryCellTransaction,
    DoMemoryTransaction,
    TransmitConstantMemoryCell,
    ReadDataLines,
};
constexpr TimerTrackingTargets FunctionToTrackTimeOn = TimerTrackingTargets::None;
constexpr bool shouldEnableTimeTracking(TimerTrackingTargets target) noexcept {
    return FunctionToTrackTimeOn == target;
}
// tracking targets
#define X(func) \
    constexpr auto Track ## func = shouldEnableTimeTracking(TimerTrackingTargets:: func ) 
X(GetAddress);
X(DoNothingTransaction);
X(ReadLo8);
X(ReadHi8);
X(WriteDataLines);
X(DetermineActionKind);
X(DoWriteAction);
X(WriteActionCycle);
X(DoMemoryCellReadTransaction);
X(DoMemoryCellWriteTransaction);
X(DoMemoryCellTransaction);
X(DoMemoryTransaction);
X(TransmitConstantMemoryCell);
X(ReadDataLines);
#undef X
RTC_DS3231 rtc;
IntervalTimer systemTimer;
Adafruit_I2CDevice managementEngine{0x08, &Wire2};
// state machines
// right now, transaction detection is working right! Woo!
FlexIOTransactionDetector inTransactionDetector{Pin::STATE_MACHINE__IN_TRANSACTION_ADS, Pin::STATE_MACHINE__IN_TRANSACTION_DEN };
// ready pulse to level converter needs to be fixed up
FlexIOReadyPulseToLevelConverter rdyFeedback{ Pin::STATE_MACHINE__READY_LEVEL_PULSE, Pin::STATE_MACHINE__READY_LEVEL_OUT };

ILI9341_t3 tft(static_cast<int>(Pin::DISPLAY_CS), static_cast<int>(Pin::DISPLAY_DC));

static_assert(sizeof(MemoryCellBlock) == 16, "MemoryCellBlock needs to be 16 bytes in size");

template<typename From>
uint32_t extractBitPattern(From value) noexcept {
    Converter<float, uint32_t> converter;
    converter.from = value;
    return converter.to;
}

// ----- hard memory space definitions begin
EXTMEM MemoryCellBlock memory960[MemoryPoolSizeInBytes / sizeof(MemoryCellBlock)];
template<uint32_t size>
using DataBlock = MemoryCellBlock[size / sizeof(MemoryCellBlock)];
// this is the backing storage of the first 64k of io space
DataBlock<OnboardSRAMCacheSize> ioSpaceCache;
DMAMEM DataBlock<OnboardSRAM2CacheSize> sramCache2;

// with the 16-bit data bus connection, things have changed somewhat
// 0b000 -> Data Lines Transmit Port (implicit write)
// 0b001 -> Data Lines Receive Port (implicit read)
// 0b010 -> Address Lines Lower Half (implicit read)
// 0b011 -> Address Lines Upper Half (implicit read)
// 0b100 -> Data Lines Transmit Port CFG (implicit write)
// 0b101 -> Data Lines Receive Port CFG (implicit write)
// 0b110 -> Address Lines Lower Port CFG (implicit write)
// 0b111 -> Address Lines Upper Port CFG (implicit write)
//
// This is a major change from the previous 8-bit data port design. Two CH351s
// are still used but both are active at the same time whenever you select an
// address. This change greatly simplifies the design and should produce
// increased overall throughput. However, this class has to change as we are
// describing concepts instead of interfacing with a single CH351. 
//
// So the data lines have the base offset of 0b000
// The address lines have the base offset of 0b010

struct CH351 final {
  constexpr explicit CH351(uint8_t baseAddress) noexcept : _baseAddress(baseAddress & 0b010), _dataPortBaseAddress(baseAddress & 0b010), _cfgPortBaseAddress((baseAddress & 0b010) | 0b100) { }
  [[nodiscard]] constexpr auto getBaseAddress() const noexcept { return _baseAddress; }
  [[nodiscard]] constexpr auto getDataPortBaseAddress() const noexcept { return _dataPortBaseAddress; }
  [[nodiscard]] constexpr auto getDataPortReadAddressBase() const noexcept { return _dataPortBaseAddress + 1; }
  [[nodiscard]] constexpr auto getDataPortWriteAddressBase() const noexcept { return _dataPortBaseAddress; }
  [[nodiscard]] constexpr auto getConfigPortBaseAddress() const noexcept { return _cfgPortBaseAddress; }
private:
  uint8_t _baseAddress;
  uint8_t _dataPortBaseAddress;
  uint8_t _cfgPortBaseAddress;
};
constexpr CH351 addressLines{ 0b010 }, dataLines{ 0b000 };
static_assert(addressLines.getConfigPortBaseAddress() == 0b110, "Address Lines configuration directory is wrong!");


// i960 common interface begin
// we want to make it as easy as possible to do display updates without having
// to override everything, only the parts you need to do

struct InterfaceTimingDescription {
    uint32_t addressWait;
    uint32_t setupTime;
    uint32_t holdTime;
    uint32_t afterTime;
    constexpr InterfaceTimingDescription(uint32_t a, uint32_t b, uint32_t c, uint32_t d) : addressWait(a), setupTime(b), holdTime(c), afterTime(d) { }
};
template<uint32_t value>
inline void fixedDelayNanoseconds() noexcept {
    if constexpr (value > 0) {
        delayNanoseconds(value);
    }
}
constexpr InterfaceTimingDescription defaultWrite8{ 50, 30, 100, 150 }; // 330ns worth of delay
constexpr InterfaceTimingDescription defaultRead8 { 100, 80, 20, 50 }; // 250ns worth of delay
constexpr InterfaceTimingDescription customWrite16 { 10, 0, 20, 0 }; // 30ns total
constexpr InterfaceTimingDescription customRead16 { 10, 0, 20, 0 }; // 30ns total
constexpr auto WriteConfiguration = customWrite16;
constexpr auto ReadConfiguration = customRead16;
struct EBIOperationDescription final {
    constexpr EBIOperationDescription(bool configureDataLinesDirection, bool triggerEnable, bool setAddress, bool setDataLines) : _configureDataLinesDirection(configureDataLinesDirection), _triggerEnable(triggerEnable), _setAddress(setAddress), _setDataLines(setDataLines) { }
    [[nodiscard]] constexpr auto shouldConfigureDataLinesDirection() const noexcept { return _configureDataLinesDirection; }
    [[nodiscard]] constexpr auto shouldControlTriggerEnable() const noexcept { return _triggerEnable; }
    [[nodiscard]] constexpr auto shouldConfigureAddressLines() const noexcept { return _setAddress; }
    [[nodiscard]] constexpr auto shouldConfigureDataLines() const noexcept { return _setDataLines; }
    bool _configureDataLinesDirection;
    bool _triggerEnable;
    bool _setAddress;
    bool _setDataLines; 
};
constexpr EBIOperationDescription defaultConfiguration (true, true, true, true);
constexpr EBIOperationDescription dataLinesDirectionAlreadyConfigured(false, true, true, true);
constexpr EBIOperationDescription getAddressConfiguration(false, false, true, true);
constexpr EBIOperationDescription setDataLinesConfiguration(false, true, false, false);
constexpr EBIOperationDescription getDataLinesConfiguration(false, false, false, true);
struct i960Interface final {
  i960Interface() = delete;
  ~i960Interface() = delete;
  i960Interface(const i960Interface&) = delete;
  i960Interface(i960Interface&&) = delete;
  i960Interface& operator=(const i960Interface&) = delete;
  i960Interface& operator=(i960Interface&&) = delete;
  // signal information from CH351DS3.pdf
  // TWW : Valid Write Strobe Pulse Width : 25ns (40ns) minimum
  // TRW : Valid Read Strobe Pulse Width : 25ns (40ns) minimum
  // TWS : Read or Write strobe pulse interval width : 25ns (40ns) minimum
  // TAS : Address input setup time (RD or WR) [ before ] : 2 ns minimum
  // TAH : Address hold time (RD or WR ) [ after ] : 3 ns minimum
  // TIS : Data input setup time before write strobe : 5 ns minimum
  // TIH : Data input hold time after write strobe : 5 ns minimum
  // TON : Read strobe assert to data out valid time : 
  //        typical time: 15ns (22ns)
  //        maximum time: 22ns (33ns)
  // TOF : Read strobe deassert to data out invalid time :
  //        maximum time: 18 (25ns)
  //
  // The parens contain the 3.3v times so for the teensy, this is what we care
  // about:
  //
  // TWW : Valid Write Strobe Pulse Width : 40ns minimum
  // TRW : Valid Read Strobe Pulse Width : 40ns minimum
  // TWS : Read or Write strobe pulse interval width : 40ns minimum
  // TAS : Address input setup time (RD or WR) [ before ] : 2 ns minimum
  // TAH : Address hold time (RD or WR ) [ after ] : 3 ns minimum
  // TIS : Data input setup time before write strobe : 5 ns minimum
  // TIH : Data input hold time after write strobe : 5 ns minimum
  // TON : Read strobe assert to data out valid time : 
  //        typical time: 22ns
  //        maximum time: 33ns
  // TOF : Read strobe deassert to data out invalid time :
  //        maximum time: 25ns
  template<EBIOperationDescription description = defaultConfiguration>
  static inline void 
  write16(uint8_t address, uint16_t value) noexcept {
      /// @todo once new board comes in we can implement the fixed direction
      /// design
      if constexpr (description.shouldConfigureDataLinesDirection()) {
        EBIInterface::setDataLinesDirection<OUTPUT>();
      }
      if constexpr (description.shouldConfigureAddressLines()) {
          EBIInterface::setAddress(address);
          fixedDelayNanoseconds<WriteConfiguration.addressWait>();
      }
      if constexpr (description.shouldConfigureDataLines()) {
          EBIInterface::setDataLines(value);
      }

      fixedDelayNanoseconds<WriteConfiguration.setupTime>(); // setup time (tDS), normally 30
      if constexpr (description.shouldControlTriggerEnable()) {
          digitalToggleFast(Pin::EBI_EN);
      }
      fixedDelayNanoseconds<WriteConfiguration.holdTime>(); // tWL hold for at least 80ns
      if constexpr (description.shouldControlTriggerEnable()) {
          digitalToggleFast(Pin::EBI_EN);
      }
      // update the address
      fixedDelayNanoseconds<WriteConfiguration.afterTime>(); // data hold after WR + tWH + breathe (50ns)
  }
  template<EBIOperationDescription description>
  static inline void
  write16(uint8_t address) noexcept {
      static_assert(!description.shouldConfigureDataLines(), "This function should only be used when the data lines are configured ahead of time");
      // it doesn't matter what we actually "write" since it won't be used
      write16<description>(address, 0);
  }
  template<EBIOperationDescription description>
  static inline void
  write16() noexcept {
      static_assert(!description.shouldConfigureAddressLines(), "This function should only be used when the target address lines are configured ahead of time!");
      // it doesn't matter what we actually set the address to since it won't be used
      write16<description>(0);
  }
  template<EBIOperationDescription description = defaultConfiguration>
  static inline uint16_t
  read16(uint8_t address) noexcept {
      // the CH351 has some very strict requirements
      // This function will take at least 230 ns to complete
      if constexpr (description.shouldConfigureDataLinesDirection()) {
        EBIInterface::setDataLinesDirection<INPUT>();
      }
      if constexpr (description.shouldConfigureAddressLines()) {
          EBIInterface::setAddress(address);
          fixedDelayNanoseconds<ReadConfiguration.addressWait>();
      }
      if constexpr (description.shouldControlTriggerEnable()) {
          digitalToggleFast(Pin::EBI_EN);
      }
      fixedDelayNanoseconds<ReadConfiguration.setupTime>(); // wait for things to get selected properly
      fixedDelayNanoseconds<ReadConfiguration.holdTime>();
      uint16_t output = EBIInterface::readDataLines();
      if constexpr (description.shouldControlTriggerEnable()) {
          digitalToggleFast(Pin::EBI_EN);
      }
      fixedDelayNanoseconds<ReadConfiguration.afterTime>();
      return output;
  }
  template<EBIOperationDescription description>
  static inline uint16_t
  read16() noexcept {
      static_assert(!description.shouldConfigureAddressLines(), "This function should only be used when the target address lines are configured ahead of time!");
      // doesn't matter what address we "set"
      return read16<description>(0);
  }



  FLASHMEM static void
  begin() noexcept {
      // configure the address lower lines for input
      write16(addressLines.getConfigPortBaseAddress(), 0);
      // configure the address upper lines for input 
      write16(addressLines.getConfigPortBaseAddress()+1, 0);
      // configure data transmit port for output
      write16(dataLines.getConfigPortBaseAddress(), 0xFFFF);
      // then output 0 on the data lines
      write16(dataLines.getDataPortWriteAddressBase(), 0);
      // configure the receieve port for inpu
      write16(dataLines.getConfigPortBaseAddress()+1, 0);
      // Make sure that the data lines are set to zero
      EBIInterface::setDataLines(0);

      // configure the initial capacity information
  }
public:
  static void
  waitForReadySignal() noexcept {
      rdyFeedback.wait();
  }

  static inline bool
  isReadOperation() noexcept {
    return digitalReadFast(Pin::WR) == LOW;
  }

  static inline bool
  isBurstLast() noexcept {
    return digitalReadFast(Pin::BLAST) == LOW;
  }
private:
  struct MemoryBlockSink final {
        void clear() { }
        [[nodiscard]] uint16_t getWord(uint8_t) const noexcept { return 0; }
        void setWord(uint8_t, uint16_t, ActionKind) noexcept { }
  };
  static inline MemoryBlockSink nullSink;
public:

  template<MemoryCell MC>
  static void
  doMemoryCellReadTransaction(const MC& target, uint8_t offset) noexcept {
      TimeTracker<TrackDoMemoryCellReadTransaction> tracker(__PRETTY_FUNCTION__);
      // start overlaying operations ahead of time
      EBIInterface::setDataLines(target.getWord((offset >> 1)));
      for (auto wordOffset = (offset >> 1); ;) {
          // now update the IO expander itself
          write16<setDataLinesConfiguration>();
          if (isBurstLast()) {
              digitalToggleFast(Pin::READY);
              break;
          } else {
              digitalToggleFast(Pin::READY);
              // load the next word and set the data lines ahead of time
              ++wordOffset;
              EBIInterface::setDataLines(target.getWord(wordOffset));
              waitForReadySignal();
          }
      }
      waitForReadySignal();
  }
  static constexpr uint32_t ActionKind_EnableMask = 0b110000000;
  static constexpr uint32_t ActionKind_Low8Mask   = 0b100000000;
  static constexpr uint32_t ActionKind_Hi8Mask    = 0b110000000;
  static inline ActionKind determineActionKind() noexcept {
      TimeTracker<TrackDetermineActionKind> tracker(__PRETTY_FUNCTION__);
      // the i960Sx exposes two byte enable signals, BE0 and BE1
      // Each signal denotes if we should write that byte value. 
      //
      // Previously, we were always reading BE0 and BE1 separately
      //
      // Now, using an external GAL chip, we have two different signals:
      // -- FULL16_ENABLE
      // -- BE0
      // If Full16_ENABLE is low then stop there and read both halves and
      // write it out
      // If it is false then check BE0 and see if it is low. If it is then
      // only read the lower half and update only that byte.
      // Finally, if we get to the third clause then it means that BE1 is
      // low and thus we need to read the upper byte line and write only
      // that out. This seems to improve performance quite a bit
      //
      // Basically we only pay for reading at most two signals and only if
      // the first signal comes back high.
      //
      // This is possible because it is impossible for both enable signals
      // to be high during a legal transaction. The only time that BE0 and
      // BE1 are high is outside a given transaction or when FAIL is setup.
      // So this is a safe optimization.

      // I remapped the pinout for BE0 and FULL16 to be accessible from a
      // single port read operation
      switch (GPIO9_PSR & ActionKind_EnableMask) {
          case ActionKind_Low8Mask:
             return ActionKind::Low8;
          case ActionKind_Hi8Mask:
             return ActionKind::Hi8;
          default:
             return ActionKind::Full16;
      }
  }
  template<MemoryCell MC>
  static void
  doMemoryCellWriteTransaction(MC& target, uint8_t offset) noexcept {
      TimeTracker<TrackDoMemoryCellWriteTransaction> tracker(__PRETTY_FUNCTION__);
      for (uint8_t wordOffset = (offset >> 1); ;) {
          target.setWord(wordOffset, read16<getDataLinesConfiguration>(), determineActionKind());
          if (isBurstLast()) {
              digitalToggleFast(Pin::READY);
              break;
          } else {
              digitalToggleFast(Pin::READY);
              // load the next word and set the data lines ahead of time
              ++wordOffset;
              waitForReadySignal();
          }
      }
      waitForReadySignal();
  }
  template<bool isReadTransaction, MemoryCell MC>
  static inline void
  doMemoryCellTransaction(MC& target, uint8_t offset) noexcept {
      TimeTracker<TrackDoMemoryCellTransaction> tracker(__PRETTY_FUNCTION__);
      if constexpr (isReadTransaction) {
          doMemoryCellReadTransaction(target, offset);
      } else {
          doMemoryCellWriteTransaction(target, offset);
      }
  }
private:
  static void onBuiltinDeviceRead(uint16_t offset, MemoryCellBlock& cacheLine) noexcept {
      //auto& cacheLine = ioSpaceCache[(offset >> 4)&0xFFF];
      // unlike the memory blocks, the address used here is byte related so we
      // have to compensate for it
      //
      // Some of these operations are wasteful but it is consistent which
      // should eliminate some of the headaches if you write to the underlying
      // cells.
      //
      // However, only the initial address is taken into account here. So doing
      // a burst read or burst write will result in only the lowest register
      // being updated. This is a fine design as this is how we operate anyway!
      switch (offset) {
          case 0x00'00:
              cacheLine.setWord32(0, CLK1Value);
              break;
          case 0x00'04:
              cacheLine.setWord32(1, CLK2Value);
              break;
          case 0x00'08:
              cacheLine.setWord32(2, Serial.read());
              break;
          case 0x00'10:
              cacheLine.setWord32(0, millis());
              break;
          case 0x00'14:
              cacheLine.setWord32(1, micros());
              break;
          case 0x00'18:
              cacheLine.setWord32(2, getCurrentCycleCount());
              break;
          case 0x00'1c:
              cacheLine.setWord32(3, rtc_get());
              break;
          case 0x00'20:
              cacheLine.setWord32(0, rtc.now().unixtime());
              break;
          case 0x00'24:
              cacheLine.setWord32(1, rtc.now().secondstime());
              break;
          case 0x00'28:
              cacheLine.setWord32(2, extractBitPattern(rtc.getTemperature()));
              break;
          case 0x00'30:
              cacheLine.setWord32(0, random());
              break;
          case 0x00'38:
              cacheLine.setWord32(2, Entropy.random());
              break;
          case 0x00'44: // sram cache size (~61k)
              cacheLine.setWord32(1, OnboardSRAM1CacheSize);
              break;
          case 0x00'48:
              cacheLine.setWord32(2, OnboardSRAM2CacheSize); 
              break;
          default:
              break;
      }
  }
  static void onBuiltinDeviceWrite(uint16_t offset, const MemoryCellBlock& cacheLine) noexcept {
      //auto& cacheLine = ioSpaceCache[(offset >> 4)&0xFFF];
      // operations will be invoked on the lowest address since a 32-bit
      // operation is actually comprised of 2 parts. Thus, it makes more sense
      // to just see what the base address is. 
      //
      // This design works well when you are operating on one register at a
      // time. You can burst commit but it won't actually trigger anything
      // properly. Only the lowest mapped device.
      //
      switch (offset) {
          case 0x00'08:
              // okay, so we actually only care about the lower word anyways
              Serial.write(static_cast<uint8_t>(cacheLine.getWord(4)));
              break;
          case 0x00'0c:
              Serial.flush();
              break;
          case 0x00'2e: 
              if (cacheLine.getWord32(3) != 0) {
                  rtc.enable32K();
              } else {
                  rtc.disable32K();
              }
              break;
          case 0x00'34: // system counter enable
              systemCounterEnabled = cacheLine.getWord(2) != 0;
              break;
          default:
              break;
      }
  }
  
public:
  template<bool isReadTransaction>
  static void
  doIOTransaction(SplitWord32 address) noexcept {
      auto lineOffset = address.value & 0xF;
      auto sramIndex = (address.value >> 4) & 0xFFF;
      switch (static_cast<uint8_t>(address.value >> 16)) {
          // first 4k of io space is just a block of memory that read/write
          // operations act upon
          case 0x00:
              [lineOffset, offset = static_cast<uint16_t>(address.value), &cacheLine = ioSpaceCache[sramIndex]]() {
                  //auto& cacheLine = ioSpaceCache[sramIndex];
                  if constexpr (isReadTransaction) {
                      onBuiltinDeviceRead(offset, cacheLine);
                  } 
                  doMemoryCellTransaction<isReadTransaction>(cacheLine, lineOffset);
                  if constexpr (!isReadTransaction) {
                      onBuiltinDeviceWrite(offset, cacheLine);
                  }
              }();
              break;
          case 0x01: // SRAM2
              doMemoryCellTransaction<isReadTransaction>(sramCache2[sramIndex], lineOffset);
              break;
          default:
              doMemoryCellTransaction<isReadTransaction>(nullSink, lineOffset);
              break;
      }
  }
  static SplitWord32
  getAddress() noexcept {
      TimeTracker<TrackGetAddress> tracker(__PRETTY_FUNCTION__);
      EBIInterface::setDataLinesDirection<INPUT>();
      digitalToggleFast(Pin::EBI_EN);
      SplitWord32 value;
      for (uint8_t i = 0, addr = addressLines.getBaseAddress(); i < 2; ++i, ++addr) {
          value.shorts[i] = read16<getAddressConfiguration>(addr);
      }
      digitalToggleFast(Pin::EBI_EN);
      return value;
  }
  template<bool isReadTransaction>
  static void 
  doPSRAM0Transaction(SplitWord32 address) noexcept {
      doMemoryCellTransaction<isReadTransaction>(memory960[address.components.targetCellBlock], address.components.offset);
  }
  template<bool isReadTransaction>
  static void
  doNothingTransaction(SplitWord32 address) noexcept {
      doMemoryCellTransaction<isReadTransaction>(nullSink, address.components.offset);
  }
  template<bool isReadTransaction>
  static inline void
  doMemoryTransaction() noexcept {
      TimeTracker<TrackDoMemoryTransaction> tracker(__PRETTY_FUNCTION__);
      auto address = getAddress();
      if constexpr (isReadTransaction) {
          // this will stay this way for the rest of the transaction
          EBIInterface::setDataLinesDirection<OUTPUT>();
          EBIInterface::setAddress<dataLines.getDataPortWriteAddressBase()>();
      } else {
          EBIInterface::setAddress<dataLines.getDataPortReadAddressBase()>();
          digitalToggleFast(Pin::EBI_EN);
      }
      // dispatch on the upper most 8-bits of the address
      switch (static_cast<uint8_t>(address.value >> 24)) {
          case 0x00: // PSRAM
              doPSRAM0Transaction<isReadTransaction>(address);
              break;
          case 0xFE: // IO Space
              doIOTransaction<isReadTransaction>(address);
              break;
          default:
              doNothingTransaction<isReadTransaction>(address);
              break;
      }

      if constexpr (!isReadTransaction) {
          digitalToggleFast(Pin::EBI_EN);
      }
  }
  static void 
  setClockFrequency(uint32_t clk2, uint32_t clk1) noexcept {
      CLK2Value = clk2;
      CLK1Value = clk1;
  }
private:
  static inline uint32_t CLK2Value = 0;
  static inline uint32_t CLK1Value = 0;
};

namespace i960 {
namespace {
    template<ManagementEngineReceiveOpcode code>
    constexpr uint8_t GenericSingleByteSequence[] { static_cast<uint8_t>(code) };
    template<ManagementEngineReceiveOpcode code>
    void doFixedWriteOperation() noexcept {
        managementEngine.write(GenericSingleByteSequence<code>, sizeof (GenericSingleByteSequence<code>));
    }
    template<ManagementEngineRequestOpcode code>
    int requestByte() noexcept {
        static constexpr uint8_t InstructionSequence[] {
            static_cast<uint8_t>(ManagementEngineReceiveOpcode::SetMode),
            static_cast<uint8_t>(code),
        };
        uint8_t result = 0;
        if (managementEngine.write_then_read(InstructionSequence, sizeof(InstructionSequence), 
                                             &result, sizeof(uint8_t))) {
            return result;
        } else {
            return -1;
        }
    }
}
// until this function is used more than once we should keep it in flashmem
FLASHMEM void
putCPUInReset() noexcept {
    doFixedWriteOperation<ManagementEngineReceiveOpcode::PutInReset>();
    cpuIsRunning = false;
}

FLASHMEM void
pullCPUOutOfReset() noexcept {
    doFixedWriteOperation<ManagementEngineReceiveOpcode::PullOutOfReset>();
    cpuIsRunning = true;
}
FLASHMEM bool
cpuRunning() noexcept {
    return cpuIsRunning;
}
FLASHMEM void
holdBus() noexcept {
    doFixedWriteOperation<ManagementEngineReceiveOpcode::HoldBus>();
}
FLASHMEM void
releaseBus() noexcept {
    doFixedWriteOperation<ManagementEngineReceiveOpcode::ReleaseBus>();
}

FLASHMEM bool
isBusLocked() noexcept {
    return (requestByte<ManagementEngineRequestOpcode::BusIsLocked>() > 0);
}

FLASHMEM bool
isBusHeld() noexcept {
    return (requestByte<ManagementEngineRequestOpcode::BusIsHeld>() > 0);
}

}

FLASHMEM void 
setupSDCard() noexcept {
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("No SDCARD found!");
        return;
    } 
    //sdcardTracker.begin();
    Serial.println("SDCARD Found");
    if (!SD.exists("prog.bin")) {
        Serial.println("prog.bin not found! No boot image will be installed");
        return;
    } 
    auto file = SD.open("prog.bin");
    if (file.size() > MemoryPoolSizeInBytes) {
        Serial.println("Boot image is too large!");
    } else {
        Serial.print("Installing prog.bin...");
        file.read(memory960, file.size());
        Serial.println("done");
    }
    file.close();
}


FLASHMEM void 
setupRandomSeed() noexcept {
    // allow the entropy source to actually block until we get enough
    // randomness
  uint32_t newSeed = Entropy.random();
#define X(pin) newSeed += analogRead(pin)
  X(A0);
  X(A1);
  X(A2);
  X(A3);
  X(A4);
  X(A5);
  X(A6);
  X(A7);
  X(A8);
  X(A9);
  X(A10);
  X(A11);
  X(A12);
  X(A14);
  X(A15);
  X(A16);
  X(A17);
#undef X
  newSeed += rtc.now().unixtime();
  randomSeed(newSeed);
  //Serial.printf("Random Seed: 0x%x\n", newSeed);
}
FLASHMEM void 
setupMemory() noexcept {
  Serial.println("Clearing PSRAM");
  for (auto& a : memory960) {
      a.clear();
  }
  Serial.println("Clearing SRAM Caches");
  for (auto& cell : ioSpaceCache) {
      cell.clear();
  }
  for (auto& cell : sramCache2) {
      cell.clear();
  }
}
FLASHMEM void
setupRTC() noexcept {
    if (rtc.begin(&Wire2)) {
        if (rtc.lostPower()) {
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        }
        Serial.println("Found RTC!");
        auto now = rtc.now();
        Serial.printf("unixtime: %d\n", now.unixtime());
    }
    // use the onboard rtc to act as uptime counter instead!
    rtc_set(0);
    Serial.printf("onboard rtc value (time since startup): %d\n", rtc_get());
    // @todo take advantage of the rtc built into the teensy
}
void
sinkWire() noexcept {
    while (Wire2.available()) {
        (void)Wire2.read();
    }
}
const uint8_t setCPUClockMode_CLKAll [] { 0, 0 };

FLASHMEM void
displayClockSpeedInformation() noexcept {
    SplitWord64 clk3;
    managementEngine.write_then_read(setCPUClockMode_CLKAll, sizeof(setCPUClockMode_CLKAll),
            clk3.bytes, sizeof(clk3));
    //Serial.printf("CLK2: %u\nCLK1: %u\n", clk3.words[0], clk3.words[1]);
    i960Interface::setClockFrequency(clk3.words[0], clk3.words[1]);
}
FLASHMEM void
waitForAVRToComeUp() noexcept {
    static const uint8_t InstructionSequence[] {
        static_cast<uint8_t>(ManagementEngineReceiveOpcode::SetMode),
        static_cast<uint8_t>(ManagementEngineRequestOpcode::ChipIsReady),
    };
    managementEngine.write(InstructionSequence, sizeof(InstructionSequence));

    bool chipIsUp = false;
    do {
        managementEngine.read(reinterpret_cast<uint8_t*>(&chipIsUp), 1);
    } while (!chipIsUp);


}
namespace i960 {
    void putCPUInReset() noexcept;
    void pullCPUOutOfReset() noexcept;
}
using i960::putCPUInReset;
using i960::pullCPUOutOfReset;
void 
triggerSystemTimer() noexcept {
    if (systemCounterEnabled) {
        digitalToggleFast(Pin::INT960_0); 
    }
}
template<FlexIODevice TD, ReadyPulseHandlerEngine RD>
FLASHMEM bool configureFlexIO(TD&, RD&) noexcept;
FLASHMEM void setupDisplayConnection() noexcept;
FLASHMEM void 
setup() {
    cpuIsRunning = false;
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    delay(1000);
    Serial.println("UP!");
    Serial.print("Wire2 starting up...");
    Wire2.begin();
    Serial.println("done");
    Serial.print("start management engine...");
    managementEngine.begin();
    Serial.println("done");
    setupRTC();
    Serial.println("Waiting for AVR to come up!");
    waitForAVRToComeUp();
    Serial.println("AVR UP!");
    Serial.print("Putting i960 into reset...");
    putCPUInReset();
    Serial.println("done");
    Serial.print("Configuring pins...");
    inputPin(Pin::ADS);
    outputPin(Pin::INT960_0, HIGH);
    outputPin(Pin::INT960_1, LOW);
    outputPin(Pin::INT960_2, LOW);
    outputPin(Pin::INT960_3, HIGH);
    inputPin(Pin::BE0);
    //inputPin(Pin::BE1);
    inputPin(Pin::FULL16_ENABLE);
    inputPin(Pin::WR);
    outputPin(Pin::READY, HIGH);
    inputPin(Pin::STATE_MACHINE__IN_TRANSACTION_ADS);
    inputPin(Pin::STATE_MACHINE__IN_TRANSACTION_DEN);
    //inputPin(Pin::STATE_MACHINE__READY_LEVEL_PULSE);
    inputPin(Pin::BLAST);
    inputPin(Pin::READY_SYNC);
    Serial.println("done");
#ifdef USB_TRIPLE_SERIAL
    Serial.print("Setting up other USB serial connections...");
    SerialUSB1.begin(115200); // chipset_realtime interface
    SerialUSB2.begin(115200); // propagation of management shell interface
    Serial.println("done");
#endif

    if (!configureFlexIO(inTransactionDetector, rdyFeedback)) {
        Serial.println("Halting...");
        while (true) {
            delay(1000);
        }
    }
    Entropy.Initialize();
    EEPROM.begin();

    // put your setup code here, to run once:
    if (EBIInterface::begin()) {
        Serial.println("EBI Interface Up!");
    } else {
        Serial.println("EBI Interface Could Not Be Started!");
        while (true) {
            delay(1000);
        }
    }
    i960Interface::begin();
    setupMemory();
    setupSDCard();
    setupRandomSeed();
    Entropy.Initialize();
    // there is an RP2040 that is using PicoDVI firmware as an HDMI output port
    setupDisplayConnection();
    systemTimer.begin(triggerSystemTimer, 100'000);
    displayClockSpeedInformation();
    Serial.println("-------");
    pullCPUOutOfReset();
}
inline bool shouldServiceTransaction() noexcept {
    return inTransactionDetector.inTransaction();
}
inline void 
tryDoTransaction() noexcept {
    if (shouldServiceTransaction()) {
        if (i960Interface::isReadOperation()) {
            i960Interface::doMemoryTransaction<true>();
        } else {
            i960Interface::doMemoryTransaction<false>();
        }
    } 
}
void 
loop() {
    do {
        tryDoTransaction();
    } while (true);
}

template<FlexIODevice TD, ReadyPulseHandlerEngine RD>
bool
configureFlexIO(TD& inTransactionDetector, RD& rdyFeedback) noexcept {
    // we want to use FlexIO1 to migrate off of the RP2040 and also allow for
    // faster detection than the RP2040 as well. We use FlexIO1 in state
    // machine mode
    //
    // This FlexIO state machine is used to detect the start of memory
    // transactions and feed that into the In Transaction pin. Eventually, I am
    // hoping I can actually read the output pin directly but that is not
    // currently possible!
    if (inTransactionDetector.begin()) {
        Serial.println("In Transaction Detector Successfully Started!");
    } else {
        Serial.println("In Transaction Detector Failed to start!");
        return false;
    }
    // the second FlexIO device is used to convert the ready signal pulse into
    // a level. It runs at 480MHz like the other state machine does. The layout
    // is actually the same as well.
    if (rdyFeedback.begin()) {
        Serial.println("Ready Pulse -> Level Device Successfully Started!");
    } else {
        Serial.println("Ready Pulse -> Level Device Failed to start!");
        return false;
    }
    return true;
}
// graphics interface routines
FLASHMEM unsigned long testFillScreen() {
  unsigned long start = micros();
  tft.fillScreen(ILI9341_BLACK);
  yield();
  tft.fillScreen(ILI9341_RED);
  yield();
  tft.fillScreen(ILI9341_GREEN);
  yield();
  tft.fillScreen(ILI9341_BLUE);
  yield();
  tft.fillScreen(ILI9341_BLACK);
  yield();
  return micros() - start;
}

FLASHMEM unsigned long testText() {
  tft.fillScreen(ILI9341_BLACK);
  unsigned long start = micros();
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2);
  tft.println(1234.56);
  tft.setTextColor(ILI9341_RED);    tft.setTextSize(3);
  tft.println(0xDEADBEEF, HEX);
  tft.println();
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(5);
  tft.println("Groop");
  tft.setTextSize(2);
  tft.println("I implore thee,");
  tft.setTextSize(1);
  tft.println("my foonting turlingdromes.");
  tft.println("And hooptiously drangle me");
  tft.println("with crinkly bindlewurdles,");
  tft.println("Or I will rend thee");
  tft.println("in the gobberwarts");
  tft.println("with my blurglecruncheon,");
  tft.println("see if I don't!");
  return micros() - start;
}

FLASHMEM unsigned long testLines(uint16_t color) {
  unsigned long start, t;
  int           x1, y1, x2, y2,
                w = tft.width(),
                h = tft.height();

  tft.fillScreen(ILI9341_BLACK);
  yield();
  
  x1 = y1 = 0;
  y2    = h - 1;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t     = micros() - start; // fillScreen doesn't count against timing

  yield();
  tft.fillScreen(ILI9341_BLACK);
  yield();

  x1    = w - 1;
  y1    = 0;
  y2    = h - 1;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  yield();
  tft.fillScreen(ILI9341_BLACK);
  yield();

  x1    = 0;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  yield();
  tft.fillScreen(ILI9341_BLACK);
  yield();

  x1    = w - 1;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);

  yield();
  return micros() - start;
}

FLASHMEM unsigned long testFastLines(uint16_t color1, uint16_t color2) {
  unsigned long start;
  int           x, y, w = tft.width(), h = tft.height();

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for(y=0; y<h; y+=5) tft.drawFastHLine(0, y, w, color1);
  for(x=0; x<w; x+=5) tft.drawFastVLine(x, 0, h, color2);

  return micros() - start;
}

FLASHMEM unsigned long testRects(uint16_t color) {
  unsigned long start;
  int           n, i, i2,
                cx = tft.width()  / 2,
                cy = tft.height() / 2;

  tft.fillScreen(ILI9341_BLACK);
  n     = min(tft.width(), tft.height());
  start = micros();
  for(i=2; i<n; i+=6) {
    i2 = i / 2;
    tft.drawRect(cx-i2, cy-i2, i, i, color);
  }

  return micros() - start;
}

FLASHMEM unsigned long testFilledRects(uint16_t color1, uint16_t color2) {
  unsigned long start, t = 0;
  int           n, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  n = min(tft.width(), tft.height());
  for(i=n; i>0; i-=6) {
    i2    = i / 2;
    start = micros();
    tft.fillRect(cx-i2, cy-i2, i, i, color1);
    t    += micros() - start;
    // Outlines are not included in timing results
    tft.drawRect(cx-i2, cy-i2, i, i, color2);
    yield();
  }

  return t;
}

FLASHMEM unsigned long testFilledCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int x, y, w = tft.width(), h = tft.height(), r2 = radius * 2;

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for(x=radius; x<w; x+=r2) {
    for(y=radius; y<h; y+=r2) {
      tft.fillCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

FLASHMEM unsigned long testCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int           x, y, r2 = radius * 2,
                w = tft.width()  + radius,
                h = tft.height() + radius;

  // Screen is not cleared for this one -- this is
  // intentional and does not affect the reported time.
  start = micros();
  for(x=0; x<w; x+=r2) {
    for(y=0; y<h; y+=r2) {
      tft.drawCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

FLASHMEM unsigned long testTriangles() {
  unsigned long start;
  int           n, i, cx = tft.width()  / 2 - 1,
                      cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  n     = min(cx, cy);
  start = micros();
  for(i=0; i<n; i+=5) {
    tft.drawTriangle(
      cx    , cy - i, // peak
      cx - i, cy + i, // bottom left
      cx + i, cy + i, // bottom right
      tft.color565(i, i, i));
  }

  return micros() - start;
}

FLASHMEM unsigned long testFilledTriangles() {
  unsigned long start, t = 0;
  int           i, cx = tft.width()  / 2 - 1,
                   cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for(i=min(cx,cy); i>10; i-=5) {
    start = micros();
    tft.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      tft.color565(0, i*10, i*10));
    t += micros() - start;
    tft.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      tft.color565(i*10, i*10, 0));
    yield();
  }

  return t;
}

FLASHMEM unsigned long testRoundRects() {
  unsigned long start;
  int           w, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  w     = min(tft.width(), tft.height());
  start = micros();
  for(i=0; i<w; i+=6) {
    i2 = i / 2;
    tft.drawRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(i, 0, 0));
  }

  return micros() - start;
}

FLASHMEM unsigned long testFilledRoundRects() {
  unsigned long start;
  int           i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for(i=min(tft.width(), tft.height()); i>20; i-=6) {
    i2 = i / 2;
    tft.fillRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(0, i, 0));
    yield();
  }

  return micros() - start;
}
FLASHMEM void
setupDisplayConnection() noexcept {
    // we are sending commands to an ILI9341 display which is actually an
    // RP2040 Feather DVI that exposes that exposes an HDMI connection
    // by this point it should be safe to just talk to the ILI9341
    Serial.println("Configuring ILI9341 generic display interface");
    tft.setClock(16'000'000);
    tft.begin();
    tft.fillScreen(ILI9341_BLACK);
    Serial.println("Running graphics tests before boot!");
    // taken from the graphicstest.ino example
    uint8_t x = tft.readcommand8(ILI9341_RDMODE);
    Serial.print(F("Display Power Mode: 0x")); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDMADCTL);
    Serial.print(F("MADCTL Mode: 0x")); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDPIXFMT);
    Serial.print(F("Pixel Format: 0x")); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDIMGFMT);
    Serial.print(F("Image Format: 0x")); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDSELFDIAG);
    Serial.print(F("Self Diagnostic: 0x")); Serial.println(x, HEX); 
    Serial.print(F("Display Width: ")); Serial.println(tft.width());
    Serial.print(F("Display Height: ")); Serial.println(tft.height());
    // try do do the system clear setup
    Serial.println(F("Benchmark                Time (microseconds)"));
    delay(10);
    Serial.print(F("Screen fill              "));
    Serial.println(testFillScreen());
    delay(500);

    Serial.print(F("Text                     "));
    Serial.println(testText());
    delay(3000);

    Serial.print(F("Lines                    "));
    Serial.println(testLines(ILI9341_CYAN));
    delay(500);

    Serial.print(F("Horiz/Vert Lines         "));
    Serial.println(testFastLines(ILI9341_RED, ILI9341_BLUE));
    delay(500);

    Serial.print(F("Rectangles (outline)     "));
    Serial.println(testRects(ILI9341_GREEN));
    delay(500);

    Serial.print(F("Rectangles (filled)      "));
    Serial.println(testFilledRects(ILI9341_YELLOW, ILI9341_MAGENTA));
    delay(500);

    Serial.print(F("Circles (filled)         "));
    Serial.println(testFilledCircles(10, ILI9341_MAGENTA));

    Serial.print(F("Circles (outline)        "));
    Serial.println(testCircles(10, ILI9341_WHITE));
    delay(500);

    Serial.print(F("Triangles (outline)      "));
    Serial.println(testTriangles());
    delay(500);

    Serial.print(F("Triangles (filled)       "));
    Serial.println(testFilledTriangles());
    delay(500);

    Serial.print(F("Rounded rects (outline)  "));
    Serial.println(testRoundRects());
    delay(500);

    Serial.print(F("Rounded rects (filled)   "));
    Serial.println(testFilledRoundRects());
    delay(500);

    Serial.println(F("Done!"));

    Serial.println("Display Connection setup complete!");
}
// ------ Filesystem components begin ------

#if 0
// for the i960 interface side, we pass an i960 memory address in and must
// translate it to a teensy address
//
// The spaces we accept i960 addresses from are:
// 1) PSRAM
// 2) OnboardSRAM2
constexpr bool alignedTo64ByteBoundaries(uint32_t address) noexcept {
    // the lowest 6 bits must be all zeros
    return (address & 0b111111) == 0;
}
constexpr bool isPSRAMAddress(uint32_t address) noexcept {
    return address < 0x0100'0000;
}
constexpr bool isSRAM2Address(uint32_t address) noexcept {
    return (address & 0xFFFF'0000) == 0xFE01'0000;
}
constexpr bool inValidMemorySpace(uint32_t address) noexcept {
    return isPSRAMAddress(address) || isSRAM2Address(address);
}
constexpr uint32_t computePSRAMOffset(uint32_t base) noexcept {
    return reinterpret_cast<uint32_t>(memory960) + (base & 0x00FF'FFFF);
}
constexpr uint32_t computeSRAM2Offset(uint32_t base) noexcept {
    return reinterpret_cast<uint32_t>(sramCache2) + (base & 0x0000'FFFF);
}
constexpr uint32_t computeChipsetMemoryAddress(uint32_t address) noexcept {
    if (isPSRAMAddress(address)) {
        return computePSRAMOffset(address);
    } else if (isSRAM2Address(address)) {
        return computeSRAM2Offset(address);
    } else {
        // don't allow the i960 to mess with teensy internals
        return 0xFFFF'FFFF;
    }
}
// The old fileystem interface design from the old days provided a fixed number
// of File "slots" that the old chipset iterated over to find the first open
// slot and use that to make a request. This works but also was implemented
// where we exposed each byte of the path directly in MMIO space! Very goofy
// and error prone. 
//
// This new design actually operates on pointers that the i960 provides which
// are offsets into PSRAM itself! Then we can just read that data as we desire.
// In fact, we can even hold the bus until action is complete!
//
class FileTracker {
    using ErrorCodes = FileRequestErrorCodes;
    public:
        using OptionalFile = std::optional<std::reference_wrapper<File>>;
        FileTracker() = default;
        void begin(uint32_t startState = Entropy.random()) noexcept {
            _openFiles.clear();
            _lfsrStartState = startState;
            _lfsrState = startState;
        }
        void end() {
            _openFiles.clear();
            _lfsrStartState = 0;
            _lfsrState = 0;
        }
        bool enabled() const noexcept { return _lfsrStartState != 0; }
        
        uint64_t open(const char* path, uint32_t flags, uint32_t i960HandleId) noexcept {
            FileUID result;
            result.i960 = i960HandleId;
            /// @todo do we need to translate the flags to an SdFat compatbile design?
            auto handle = SD.open(path, flags);
            if (handle) {
                // okay, so we were able to open the file so now we need to
                // make sure that it is possible to actually insert it with the
                // specified index
                //
                // We don't want the system to lockup though so introduce a
                // timeout value where we can ask the i960 to generate a
                // different unique index! Interestingly enough, this means
                // that the i960 could also use the same xorshift32 lfsr as a
                // way to break this deadlock. We should try at least 8 times
                // to see if we have tracks of ids that are no longer available
                for (int i = 0; i < 8; ++i) {
                    // try eight times to get a unique handle index
                    auto chipsetHandleId = getNewValue();
                    result.chipset = chipsetHandleId;
                    if (!_openFiles.contains(result.raw)) {
                        _openFiles.emplace(result.raw, handle);
                        return result.raw;
                    }
                }
                // close the file since were unable to use it, the i960 needs
                // to help us by changing its 32-bit component
                return static_cast<uint32_t>(ErrorCodes::CouldNotFindASpotForFileGivenI960UniqueId);
            } else {
                return static_cast<uint32_t>(ErrorCodes::CouldNotOpenFile);
            }
            return result.raw;
        }
        bool close(uint64_t uid) noexcept {
            // just erase the file and that should cause the destructor to be
            // called!
            return _openFiles.erase(uid) == 1;
        }
        OptionalFile find(uint64_t uid) noexcept {
            if (auto result = _openFiles.find(uid); result != _openFiles.end()) {
                return result->second;
            } else {
                return std::nullopt;
            }
        }
    private:
        void doReadOperation(FilesystemOperation& operation) noexcept;
        void doWriteOperation(FilesystemOperation& operation) noexcept;
        void doCloseOperation(FilesystemOperation& operation) {
            if (!close(operation.getUid())) {
                operation.setErrorCode(ErrorCodes::CouldNotCloseFile);
            }
        }
        void doFlushOperation(FilesystemOperation& operation) noexcept {
            auto potentialFile = find(operation.getUid());
            if (potentialFile) {
                potentialFile->get().flush();
            } else {
                operation.setErrorCode(ErrorCodes::NotAnOpenFile);
            }
        }
        void doPeekOperation(FilesystemOperation& operation) noexcept {
            auto potentialFile = find(operation.getUid());
            if (potentialFile) {
                operation.returnComponents[0] = potentialFile->get().peek();
            } else {
                operation.setErrorCode(ErrorCodes::NotAnOpenFile);
            }
        }
        void doSizeOperation(FilesystemOperation& operation) noexcept {
            auto potentialFile = find(operation.getUid());
            if (potentialFile) {
                operation.returnComponents[0] = potentialFile->get().size();
            } else {
                operation.setErrorCode(ErrorCodes::NotAnOpenFile);
            }
        }
        void doPositionOperation(FilesystemOperation& operation) noexcept {
            auto potentialFile = find(operation.getUid());
            if (potentialFile) {
                operation.returnComponents[0] = potentialFile->get().position();
            } else {
                operation.setErrorCode(ErrorCodes::NotAnOpenFile);
            }
        }
        void doSeekOperation(FilesystemOperation& operation, int mode) noexcept {
            auto potentialFile = find(operation.getUid());
            if (potentialFile) {
                potentialFile->get().seek(operation.getPosition(), mode);
            } else {
                operation.setErrorCode(ErrorCodes::NotAnOpenFile);
            }
        }
        void doSetAbsolutePosition(FilesystemOperation& operation) noexcept {
            doSeekOperation(operation, SeekSet);
        }
        void doSetPositionRelativeToCurrentPosition(FilesystemOperation& operation) noexcept {
            doSeekOperation(operation, SeekCur);
        }
        void doSetPositionRelativeToEnd(FilesystemOperation& operation) noexcept {
            doSeekOperation(operation, SeekEnd);
        }
        void doValidOperation(FilesystemOperation& operation) noexcept {
            operation.returnComponents[0] = _openFiles.find(operation.getUid()) != _openFiles.end();
        }
        void doIsOpenOperation(FilesystemOperation& operation) noexcept {
            auto potentialFile = find(operation.getUid());
            if (potentialFile) {
                operation.returnComponents[0] = potentialFile->get().operator bool();
            } else {
                operation.returnComponents[0] = 0;
            }

        }
        void doIsDirectoryOperation(FilesystemOperation& operation) noexcept {
            auto potentialFile = find(operation.getUid());
            if (potentialFile) {
                operation.returnComponents[0] = potentialFile->get().isDirectory();
            } else {
                operation.setErrorCode(ErrorCodes::NotAnOpenFile);
            }
        }
        void doAvailableOperation(FilesystemOperation& operation) noexcept {
            // TODO: be more ambiguous instead of generating error codes?
            auto potentialFile = find(operation.getUid());
            if (potentialFile) {
                operation.returnComponents[0] = potentialFile->get().available();
            } else {
                operation.setErrorCode(ErrorCodes::NotAnOpenFile);
            }
        }
        void doOpenOperation(FilesystemOperation& operation) noexcept;

    public:
        void processRequest(FilesystemOperation& operation) noexcept {
            using FSOpcode = FilesystemOperation::Opcode;
            operation.setErrorCode(ErrorCodes::None);
            // clear these out to prevent potential state leakage
            operation.returnComponents[0] = 0;
            operation.returnComponents[1] = 0;
            if (!FilesystemOperation::valid(operation.getOpcode())) {
                operation.setErrorCode(ErrorCodes::InvalidOperation);
                return;
            }
            switch (operation.getOpcode()) {
                case FSOpcode::Close:
                    doCloseOperation(operation);
                    break;
                case FSOpcode::Read:
                    doReadOperation(operation);
                    break;
                case FSOpcode::Write:
                    doWriteOperation(operation);
                    break;
                case FSOpcode::Flush:
                    doFlushOperation(operation);
                    break;
                case FSOpcode::Peek:
                    doPeekOperation(operation);
                    break;
                case FSOpcode::GetSize:
                    doSizeOperation(operation);
                    break;
                case FSOpcode::GetPosition:
                    doPositionOperation(operation);
                    break;
                case FSOpcode::SetPosition_Absolute:
                    doSetAbsolutePosition(operation);
                    break;
                case FSOpcode::SetPosition_RelativeToCurr:
                    doSetPositionRelativeToCurrentPosition(operation);
                    break;
                case FSOpcode::SetPosition_RelativeToEnd:
                    doSetPositionRelativeToEnd(operation);
                    break;
                case FSOpcode::Valid:
                    doValidOperation(operation);
                    break;
                case FSOpcode::IsOpen:
                    doIsOpenOperation(operation);
                    break;
                case FSOpcode::IsDirectory:
                    doIsDirectoryOperation(operation);
                    break;
                default:
                    operation.setErrorCode(ErrorCodes::UnimplementedOperation);
                    break;
            }
        }
    private:
        uint32_t getNewValue() noexcept {
            // taken from https://en.wikipedia.org/wiki/Xorshift
            auto x = _lfsrState;
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            _lfsrState = x;
            return x;
        }
    private:
        // use an LFSR to get a unique 32-bit id that is combined with the i960
        // provided id field to create a unique 64-bit ID. The idea is that
        // for each i960 provided component, we can have up to 2^32 files. When
        // we hit a collision 8 times in a row then we ask the i960 for a new
        // component. Since we can never use zero, it means we have a builtin
        // error space! 
        uint32_t _lfsrStartState = 0;
        uint32_t _lfsrState = 0;
        std::map<uint64_t, File> _openFiles;
};
//FileTracker sdcardTracker;
constexpr bool validFilesystemOperationAddress(uint32_t address) noexcept {
    return inValidMemorySpace(address) && alignedTo64ByteBoundaries(address);
}
void 
FileTracker::doOpenOperation(FilesystemOperation& operation) noexcept {
    if (!validFilesystemOperationAddress(operation.getOpen_Path())) {
        operation.setErrorCode(ErrorCodes::InvalidBufferAddress);
        return;
    } else {
        auto convertedAddress = computeChipsetMemoryAddress(operation.getOpen_Path());
        FileUID resultant;
        resultant.raw = open(reinterpret_cast<const char*>(convertedAddress), operation.getOpen_Flags(), operation.getOpen_i960Orgid());
        if (resultant.chipset == 0) {
            // an error happened
            operation.setErrorCode(resultant.i960);
            operation.target.raw = 0;
        } else {
            operation.target = resultant;
        }
    }
    auto potentialFile = find(operation.getUid());
    if (potentialFile) {
        operation.returnComponents[0] = potentialFile->get().available();
    } else {
        operation.setErrorCode(ErrorCodes::NotAnOpenFile);
    }
}
void 
FileTracker::doReadOperation(FilesystemOperation& operation) noexcept {
    auto outcome = find(operation.getUid());
    if (outcome) {
        auto addr960 = operation.args.onRead.bufferAddress;
        auto len = operation.args.onRead.size;
        if (!inValidMemorySpace(addr960)) {
            operation.setErrorCode(ErrorCodes::InvalidBufferAddress);
        } else if (!inValidMemorySpace(addr960 + len)) {
            // we would walk into illegal memory so do not allow it to go
            // through
            operation.setErrorCode(ErrorCodes::RequestedLengthTooLong);
        } else {
            auto addrChipset = computeChipsetMemoryAddress(addr960);
            File& f = outcome->get();
            // return the number of bytes read
            operation.returnComponents[0] = f.read(reinterpret_cast<uint8_t*>(addrChipset), len);
        }
    } else {
        operation.setErrorCode(ErrorCodes::NotAnOpenFile);
    }
}

void 
FileTracker::doWriteOperation(FilesystemOperation& operation) noexcept {
    auto outcome = find(operation.getUid());
    if (outcome) {
        auto addr960 = operation.args.onWrite.bufferAddress;
        auto len = operation.args.onWrite.size;
        if (!inValidMemorySpace(addr960)) {
            operation.setErrorCode(ErrorCodes::InvalidBufferAddress);
        } else if (!inValidMemorySpace(addr960 + len)) {
            // we would walk into illegal memory so do not allow it to go
            // through
            operation.setErrorCode(ErrorCodes::RequestedLengthTooLong);
        } else {
            auto addrChipset = computeChipsetMemoryAddress(addr960);
            File& f = outcome->get();
            // return the number of bytes read
            operation.returnComponents[0] = f.write(reinterpret_cast<uint8_t*>(addrChipset), len);
        }
    } else {
        operation.setErrorCode(ErrorCodes::NotAnOpenFile);
    }
}
struct RawFilesystemInterface {
    using ErrorCodes = FilesystemInterfaceErrorCodes;
    void clear() noexcept { 
        _targetAddress.value = 0;
        _errorCode.value = 0;
        _tryCarryOutOperation = false;
    }
    void update() noexcept {
    }
    uint16_t getWord(uint8_t offset) const noexcept {
        switch (offset & 0b111) {
            case 0:
                return _targetAddress.shorts[0];
            case 1:
                return _targetAddress.shorts[1];
            case 2:
            case 3:
                return 0;
            case 4:
                return _errorCode.shorts[0];
            case 5:
                return _errorCode.shorts[1];
            default:
                return 0;
        }
    }
    void setWord(uint8_t offset, uint16_t value, bool updateLo, bool updateHi) noexcept {
        switch (offset & 0b111) {
            case 0:
                _targetAddress.setWord(0, value, updateLo, updateHi);
                break;
            case 1:
                _targetAddress.setWord(1, value, updateLo, updateHi);
                break;
            case 2:
            case 3:
                _tryCarryOutOperation = true;
                break;
            default:
                // don't allow writing to the error code register
                break;

        }
    }
    void setWord(uint8_t offset, uint16_t value) noexcept { 
        switch (offset & 0b111) {
            case 0:
                _targetAddress.setWord(0, value);
                break;
            case 1:
                _targetAddress.setWord(1, value);
                break;
            case 2:
            case 3:
                _tryCarryOutOperation = true;
                break;
            default:
                // don't allow writing to the error code register
                break;
        }
    }
    void onFinish() noexcept { 
        if (_tryCarryOutOperation) {
            _errorCode.clear();
            _tryCarryOutOperation = false;
            if (sdcardTracker.enabled()) {
                auto addr = _targetAddress.value;
                if (!inValidMemorySpace(addr)) {
                    _errorCode.value = static_cast<uint32_t>(ErrorCodes::IllegalAddressProvided);
                } else if (!alignedTo64ByteBoundaries(addr)) {
                    _errorCode.value = static_cast<uint32_t>(ErrorCodes::UnalignedAddressProvided);
                } else {
                    auto convertedAddress = computeChipsetMemoryAddress(addr);
                    // make sure that we don't do anything goofy
                    // TODO: can we eliminate the volatile keyword?
                    FilesystemOperation& fsop = *reinterpret_cast<FilesystemOperation*>(convertedAddress);
                    sdcardTracker.processRequest(fsop);
                }
            } else {
                // begin was never called, probably because no sdcard could be
                // found!
                _errorCode.value = static_cast<uint32_t>(ErrorCodes::NotEnabled);
            }
        }
    }
private:
    bool _tryCarryOutOperation = false;
    SplitWord32 _targetAddress { 0 };
    SplitWord32 _errorCode { 0 };
    // layout for this 16-byte page is:
    //
};
RawFilesystemInterface sdcardInterface;
#endif
