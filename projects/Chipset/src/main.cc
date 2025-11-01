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

#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <Ethernet.h>
#include <Entropy.h>
#include <EEPROM.h>
#include <FastCRC.h>
#include <LittleFS.h>
#include <Metro.h>
#include <RTClib.h>
#include <FlexIO_t4.h>
#include <Adafruit_IS31FL3741.h>
#include <IntervalTimer.h>
// gamepad qt
#include <Adafruit_seesaw.h>

#include "Pinout.h"
#include "MemoryCell.h"
#include "Core.h"
#include "ManagementEngineProtocol.h"

// the dsb instruction makes sure that all instructions in the pipeline
// complete before this instruction finishes. Very necessary to prevent reads
// and writes to peripherals to complete when we want them to. Using this
// instruction also seems to increase performance in some cases. 
//
// Use sparingly because in some cases it will reduce performance if used
// too often on the Cortex-M7.
#define SynchronizeData asm volatile ("dsb")
// Thanks to an interactive session with copilot I am realizing that while the
// CH351 has some real limitations when it comes to write operations. There are
// minimum hold times in between writes. Which is around 50 ns
//
constexpr auto OLEDScreenWidth = 128;
constexpr auto OLEDScreenHeight = 128;
constexpr uint32_t OnboardSRAMCacheSize = 2048;
constexpr auto MemoryPoolSizeInBytes = (16 * 1024 * 1024);  // 16 megabyte psram pool
constexpr auto UseDirectPortManipulation = true;
volatile bool adsTriggered = false;
volatile bool readyTriggered = false;
volatile bool systemCounterEnabled = false;


RTC_DS3231 rtc;
Adafruit_IS31FL3741_QT ledmatrix;
IntervalTimer systemTimer;

namespace PCJoystick {
    Adafruit_seesaw device{&Wire2};
    bool available = false;
    constexpr auto Button1 = 3;
    constexpr auto Button2 = 13;
    constexpr auto Button3 = 2;
    constexpr auto Button4 = 14;
    constexpr uint32_t ButtonMask = (1UL << Button1) | (1UL << Button2) | 
                                    (1UL << Button3) | (1UL << Button4);
    constexpr auto Joy1X = 1;
    constexpr auto Joy1Y = 15;
    constexpr auto Joy2X = 0;
    constexpr auto Joy2Y = 16;

    void begin() noexcept {
        available = device.begin(0x49);
        if (!available) {
            Serial.println("PC Joystick port not found");
        } else {
            Serial.println("PC Joystick port found");
            device.pinModeBulk(ButtonMask, INPUT_PULLUP);
        }
    }
    uint32_t readButtons() noexcept {
        return device.digitalReadBulk(ButtonMask);
    }
    int readJoy1X() noexcept {
        return device.analogRead(Joy1X);
    }
    int readJoy1Y() noexcept {
        return device.analogRead(Joy1Y);
    }
    int readJoy2X() noexcept {
        return device.analogRead(Joy2X);
    }
    int readJoy2Y() noexcept {
        return device.analogRead(Joy2Y);
    }

} // end namespace PCJoystick 

namespace GamepadQT {
    Adafruit_seesaw device{&Wire2};
    bool available = false;
    constexpr auto ButtonX = 6;
    constexpr auto ButtonY = 2;
    constexpr auto ButtonA = 5;
    constexpr auto ButtonB = 1;
    constexpr auto ButtonSelect = 0;
    constexpr auto ButtonStart = 16;
    constexpr uint32_t ButtonMask = (1UL << ButtonX) | (1UL << ButtonY) | 
        (1UL << ButtonA) | (1UL << ButtonB) | 
        (1UL << ButtonSelect) | (1UL << ButtonStart);

    constexpr auto JoyX = 14;
    constexpr auto JoyY = 15;
    void begin() {
        available = device.begin(0x50);
        if (!available) {
            Serial.println("ERROR! seesaw not found (GamepadQT)");
        } else {
            Serial.println("GamepadQT found!");
            device.pinModeBulk(ButtonMask, INPUT_PULLUP);
        }
    }
    uint32_t readButtons() noexcept {
        return device.digitalReadBulk(ButtonMask);
    }
    int readX() noexcept {
        return 1023 - device.analogRead(JoyX);
    }
    int readY() noexcept {
        return 1023 - device.analogRead(JoyY);
    }
} // end namespace GamepadQT


struct EEPROMWrapper {
    constexpr EEPROMWrapper(uint16_t baseAddress) : _baseOffset(baseAddress) { }
    void updateBaseAddress(uint16_t base) noexcept {
        _baseOffset = base & 0x0FF0;
    }
    void update() noexcept { }
    uint16_t getWord(uint8_t offset) const noexcept {
        uint16_t value = 0;
        return EEPROM.get<uint16_t>(_baseOffset + ((offset << 1) & 0b1110), value);
    }
    void setWord(uint8_t offset, uint16_t value, bool updateLo = true, bool updateHi = true) noexcept {
        auto computedOffset = ((offset << 1)) & 0b1110;
        if (updateLo) {
            EEPROM.put(_baseOffset + computedOffset, static_cast<uint8_t>(value));
        }
        if (updateHi) {
            EEPROM.put(_baseOffset + computedOffset + 1, static_cast<uint8_t>(value >> 8));
        }
    }
    void onFinish() noexcept { }

    private:
        uint16_t _baseOffset = 0;
};
static_assert(sizeof(MemoryCellBlock) == 16, "MemoryCellBlock needs to be 16 bytes in size");
struct USBSerialBlock {
    void update() noexcept { }
    void onFinish() noexcept { }
    uint16_t getWord(uint8_t offset) const noexcept {
        switch (offset & 0b11) {
            case 0:
            case 4:
                return Serial.read();
            default:
                return 0;
        }
    }
    void setWord(uint8_t offset, uint16_t value) noexcept {
        setWord(offset, value, true, true);
    }
    void setWord(uint8_t offset, uint16_t value, bool hi, bool lo) noexcept {
        // ignore the hi and lo operations
        // 16-bit value offset so
        // 0 -> Serial.write
        // 1 -> nil
        // 2 -> flush
        // 3 -> nil
        // 4 -> Serial.write
        // 5 -> nil
        // 6 -> flush
        // 7 -> nil
        switch (offset) {
            case 0:
            case 4:
                if (lo) {
                    Serial.write(static_cast<uint8_t>(value));
                }
                break;
            case 2:
            case 6:
                Serial.flush();
                break;
            default:
                break;
        }
    }
};
struct TimingRelatedThings {
    void update() noexcept {
        _currentMillis = millis();
        _currentMicros = micros();
    }
    uint16_t getWord(uint8_t offset) const noexcept {
        switch (offset) {
            case 0: // millis
                return static_cast<uint16_t>(_currentMillis);
            case 1: // millis upper
                return static_cast<uint16_t>(_currentMillis >> 16);
            case 2: // micros
                return static_cast<uint16_t>(_currentMicros);
            case 3:
                return static_cast<uint16_t>(_currentMicros >> 16);
            case 4: // eeprom
                return static_cast<uint16_t>(4096);
            case 5:
                return 0;
            case 6:
                return static_cast<uint16_t>(OnboardSRAMCacheSize);
            case 7:
                return static_cast<uint16_t>(OnboardSRAMCacheSize >> 16);
            default:
                return 0;
        }
    }
    void setWord(uint8_t, uint16_t) noexcept { }
    void setWord(uint8_t, uint16_t, bool, bool) noexcept { }
    void onFinish() noexcept { }

private:
    // for temporary snapshot purposes
    uint32_t _currentMicros = 0;
    uint32_t _currentMillis = 0;
};
struct RandomSourceRelatedThings {
    void update() noexcept {
        _currentRandomValue = random();
    }

    uint16_t getWord(uint8_t offset) const noexcept {
        switch (offset) {
            case 0: // arduino random
                return static_cast<uint16_t>(_currentRandomValue);
            case 1: // arduino random upper
                return static_cast<uint16_t>(_currentRandomValue >> 16);
            case 2: // system counter enable 
                return systemCounterEnabled ? 0xFFFF : 0x0000;
            default:
                return 0;
        }
    }
    void setWord(uint8_t offset, uint16_t value, bool enableLo = true, bool enableHi = true) noexcept { 
        switch (offset) {
            case 2: // system counter enable
                systemCounterEnabled = (value != 0);
                break;
            default:
                break;
        }
    }
    void onFinish() noexcept { }
private:
    uint32_t _currentRandomValue = 0;
};
struct RTCMemoryBlock {
    public:
        void update() noexcept {
            _now = rtc.now();
            _inputTemperature = rtc.getTemperature();
            _32kOutEn = rtc.isEnabled32K(); 
        }
        void onFinish() noexcept { }
        uint16_t getWord(uint8_t offset) const noexcept {
            switch (offset) {
                case 0: // unixtime lower
                    return static_cast<uint16_t>(_now.unixtime());
                case 1: // unixtime upper
                    return static_cast<uint16_t>(_now.unixtime() >> 16);
                case 2: // secondstime lower
                    return static_cast<uint16_t>(_now.secondstime());
                case 3: // secondstime upper
                    return static_cast<uint16_t>(_now.secondstime() >> 16);
                case 4: // temperature
                    return static_cast<uint16_t>(_outputTemperature);
                case 5: // temperature upper
                    return static_cast<uint16_t>(_outputTemperature >> 16);
                case 6: 
                case 7:
                    return _32kOutEn ? 0xFFFF : 0x0000;
                default:
                    return 0;
            }
        }
        void setWord(uint8_t offset, uint16_t value, bool enableLo = true, bool enableHi = true) noexcept {
            switch (offset) {
                case 6:
                    if (enableLo || enableHi) {
                        if (value != 0) {
                            rtc.enable32K();
                        } else {
                            rtc.disable32K();
                        }
                    }
                    break;
                default:
                    break;
            }
        }
    private:
        DateTime _now;
        union {
            float _inputTemperature;
            uint32_t _outputTemperature;
        };
        bool _32kOutEn = false;
};

USBSerialBlock usbSerial;
TimingRelatedThings timingInfo;
RandomSourceRelatedThings randomSource;
EXTMEM MemoryCellBlock memory960[MemoryPoolSizeInBytes / sizeof(MemoryCellBlock)];
EEPROMWrapper eeprom{0};
RTCMemoryBlock rtcInterface;
struct CH351 {
  constexpr CH351(uint8_t baseAddress) noexcept
    : _baseAddress(baseAddress & 0b1111'1000), _dataPortBaseAddress(baseAddress & 0b1111'1000), _cfgPortBaseAddress((baseAddress & 0b1111'1000) | 0b0000'0100) {
  }
  constexpr auto getBaseAddress() const noexcept {
    return _baseAddress;
  }
  constexpr auto getDataPortBaseAddress() const noexcept {
    return _dataPortBaseAddress;
  }
  constexpr auto getConfigPortBaseAddress() const noexcept {
    return _cfgPortBaseAddress;
  }
private:
  uint8_t _baseAddress;
  uint8_t _dataPortBaseAddress;
  uint8_t _cfgPortBaseAddress;
};
constexpr uint32_t makeAddress(uint8_t value) noexcept {
    return static_cast<uint32_t>(((value & 0b100000) >> 5) |
        ((value & 0b000001) << 5) |
        ((value & 0b010000) >> 3) |
        ((value & 0b000010) << 3) |
        ((value & 0b001000) >> 1) |
        ((value & 0b000100) << 1)) << 16;
}
static_assert(makeAddress(0b00'01'00'11) == 0b00'11'00'10'0000'0000'0000'0000);
static_assert(makeAddress(0b00'00'00'01) == 0b00'10'00'00'0000'0000'0000'0000);
constexpr uint32_t EBIAddressTable[256] {
#define X(value) makeAddress(value), 
#include "Entry255.def"
#undef X
};

constexpr uint32_t EBIOutputTransformation[256] {
#define X(value) ((static_cast<uint32_t>(value) << 24) & 0xFF00'0000),
#include "Entry255.def"
#undef X
};

struct EBIWrapperInterface {
public:
  EBIWrapperInterface() = delete;
  ~EBIWrapperInterface() = delete;
  EBIWrapperInterface(const EBIWrapperInterface&) = delete;
  EBIWrapperInterface(EBIWrapperInterface&&) = delete;
  EBIWrapperInterface& operator=(const EBIWrapperInterface&) = delete;
  EBIWrapperInterface& operator=(EBIWrapperInterface&&) = delete;
  static void
  begin() noexcept {
    _currentDirection = OUTPUT;
    for (auto a : {
           Pin::EBI_A5,
           Pin::EBI_A4,
           Pin::EBI_A3,
           Pin::EBI_A2,
           Pin::EBI_A1,
           Pin::EBI_A0,
           Pin::EBI_RD,
           Pin::EBI_WR,
           Pin::EBI_D0,
           Pin::EBI_D1,
           Pin::EBI_D2,
           Pin::EBI_D3,
           Pin::EBI_D4,
           Pin::EBI_D5,
           Pin::EBI_D6,
           Pin::EBI_D7,
         }) {
      pinMode(a, OUTPUT);
      digitalWrite(a, LOW);
    }
    // force EBI_A4 and A5 to low  since we will never be accessing that
    setAddress(0);
    digitalWriteFast(Pin::EBI_RD, HIGH);
    digitalWriteFast(Pin::EBI_WR, HIGH);
    setDataLines<true>(0);
  }
  template<bool directPortManipulation = UseDirectPortManipulation>
  static void
  setAddress(uint8_t address) noexcept {
      if constexpr (directPortManipulation) {
          // the address table lookup is necessary because the address bits are
          // backwards compared to the GPIO index
          // 0: A5
          // 1: A4
          // 2: A3
          // 3: A2
          // 4: A1
          // 5: A0
          //
          // This layout is taken from the Raspberry pi 0-4's SMI alternate
          // mode. It allows me to leverage a PCB I made with the two CH351s
          // meant for a raspberry pi 4.
          GPIO6_DR_CLEAR = EBIAddressTable[0xFF];
          GPIO6_DR_SET = EBIAddressTable[address];
      } else {
          digitalWriteFast(Pin::EBI_A0, address & 0b000001);
          digitalWriteFast(Pin::EBI_A1, address & 0b000010);
          digitalWriteFast(Pin::EBI_A2, address & 0b000100);
          digitalWriteFast(Pin::EBI_A3, address & 0b001000);
          digitalWriteFast(Pin::EBI_A4, address & 0b010000);
          digitalWriteFast(Pin::EBI_A5, address & 0b100000);
      }
  }
  template<bool checkD0 = true, bool useDirectPortRead = UseDirectPortManipulation>
  static uint8_t
  readDataLines() noexcept {
      if constexpr (useDirectPortRead) {
          return static_cast<uint8_t>((GPIO6_PSR) >> 24);
      } else {
          uint8_t value = 0;
#define X(p, t) if ((digitalReadFast(p) != LOW)) value |= t
          //@todo accelerate using direct GPIO port reads
          if constexpr (checkD0) {
              X(Pin::EBI_D0, 0b00000001);
          }
          X(Pin::EBI_D1, 0b00000010);
          X(Pin::EBI_D2, 0b00000100);
          X(Pin::EBI_D3, 0b00001000);
          X(Pin::EBI_D4, 0b00010000);
          X(Pin::EBI_D5, 0b00100000);
          X(Pin::EBI_D6, 0b01000000);
          X(Pin::EBI_D7, 0b10000000);
#undef X
          return value;
      }
  }
  template<bool force = false, bool directPortManipulation = UseDirectPortManipulation>
  static void
  setDataLines(uint8_t value) noexcept {
      // clear then set the corresponding bits
      if constexpr (!force) {
          if (_currentOutputDataLines == value) {
              return;
          }
      }
      if constexpr (directPortManipulation) {
          GPIO6_DR_CLEAR = EBIOutputTransformation[0xFF];
          GPIO6_DR_SET = EBIOutputTransformation[value];
      } else {

          digitalWriteFast(Pin::EBI_D0, (value & 0b00000001));
          digitalWriteFast(Pin::EBI_D1, (value & 0b00000010));
          digitalWriteFast(Pin::EBI_D2, (value & 0b00000100));
          digitalWriteFast(Pin::EBI_D3, (value & 0b00001000));
          digitalWriteFast(Pin::EBI_D4, (value & 0b00010000));
          digitalWriteFast(Pin::EBI_D5, (value & 0b00100000));
          digitalWriteFast(Pin::EBI_D6, (value & 0b01000000));
          digitalWriteFast(Pin::EBI_D7, (value & 0b10000000));
      }
      _currentOutputDataLines = value;
  }

  template<PinDirection direction, bool directPortManipulation = UseDirectPortManipulation>
  static void
  setDataLinesDirection() noexcept {
      if (_currentDirection != direction) {
          if constexpr (directPortManipulation) {
              // I get a warning from the compiler if I do &= and |= directly
              // on GPIO6_GDIR. It warning states that doing that with a
              // volatile variable is deprecated. This form, however, is
              // supported.
              auto value = GPIO6_GDIR & ~EBIOutputTransformation[0xff];
              if constexpr (direction == OUTPUT) {
                  GPIO6_GDIR = (value | EBIOutputTransformation[0xff]);
              } else {
                  GPIO6_GDIR = value;
              }
              SynchronizeData; // Make sure that the direction update is
                               // happening

          } else {
#define X(p) pinMode(p, direction)
              X(Pin::EBI_D0);
              X(Pin::EBI_D1);
              X(Pin::EBI_D2);
              X(Pin::EBI_D3);
              X(Pin::EBI_D4);
              X(Pin::EBI_D5);
              X(Pin::EBI_D6);
              X(Pin::EBI_D7);
#undef X
          }
          _currentDirection = direction; 
      }

  }

private:
  static inline PinDirection _currentDirection = OUTPUT;
  static inline uint8_t _currentOutputDataLines = 0;
};
constexpr CH351 addressLines{ 0 }, dataLines{ 0b0000'1000 };
using EBIInterface = EBIWrapperInterface;
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
static constexpr InterfaceTimingDescription defaultWrite8{
    50, 30, 100, 150
}; // 330ns worth of delay
static constexpr InterfaceTimingDescription customWrite8 {
    10, // address wait
    0,  // setup time
    30, // hold time
    0   // after time / resting time
}; // 40ns worth of delay
static constexpr InterfaceTimingDescription defaultRead8 {
    100, 80, 20, 50
}; // 250ns worth of delay
static constexpr InterfaceTimingDescription customRead8 {
    10, 50, 0, 0 
}; // 60ns worth of delay

struct i960Interface {
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

  template<bool CompareWithPrevious = true, InterfaceTimingDescription decl = customWrite8>
  static inline void write8(uint8_t address, uint8_t value) noexcept {
      if constexpr (CompareWithPrevious) {
          if (EBIOutputStorage[address] == value) {
              return;
          }
      }
      EBIInterface::setDataLinesDirection<OUTPUT>();
      EBIInterface::setAddress(address);
      EBIInterface::setDataLines(value);

      fixedDelayNanoseconds<decl.addressWait>();
      fixedDelayNanoseconds<decl.setupTime>(); // setup time (tDS), normally 30
      digitalWriteFast(Pin::EBI_WR, LOW);
      fixedDelayNanoseconds<decl.holdTime>(); // tWL hold for at least 80ns
      digitalWriteFast(Pin::EBI_WR, HIGH);
      // update the address
      EBIOutputStorage[address] = value;
      fixedDelayNanoseconds<decl.afterTime>(); // data hold after WR + tWH + breathe (50ns)
  }

  template<InterfaceTimingDescription decl = customRead8>
  static inline uint8_t
  read8(uint8_t address) noexcept {
      // the CH351 has some very strict requirements
      // This function will take at least 230 ns to complete
      EBIInterface::setDataLinesDirection<INPUT>();
      EBIInterface::setAddress(address);
      fixedDelayNanoseconds<decl.addressWait>();
      digitalWriteFast(Pin::EBI_RD, LOW);
      fixedDelayNanoseconds<decl.setupTime>(); // wait for things to get selected properly
      uint8_t output = EBIInterface::readDataLines();
      fixedDelayNanoseconds<decl.holdTime>();
      digitalWriteFast(Pin::EBI_RD, HIGH);
      fixedDelayNanoseconds<decl.afterTime>();
      return output;
  }

  static void
  begin() noexcept {
      write8<false>(addressLines.getConfigPortBaseAddress(), 0);
      write8<false>(addressLines.getConfigPortBaseAddress() + 1, 0);
      write8<false>(addressLines.getConfigPortBaseAddress() + 2, 0);
      write8<false>(addressLines.getConfigPortBaseAddress() + 3, 0);
      configureDataLinesForRead<false>();
      EBIInterface::setDataLines(0);
  }
  template<uint16_t value, bool compareWithPrevious = true>
  static inline void
  configureDataLinesDirection() noexcept {
      write8<compareWithPrevious>(dataLines.getConfigPortBaseAddress(), static_cast<uint8_t>(value)); // 235ns delay
      write8<compareWithPrevious>(dataLines.getConfigPortBaseAddress()+1, static_cast<uint8_t>(value >> 8)); // 235ns delay
  }
  template<bool compareWithPrevious = true>
  static inline void
  configureDataLinesForRead() noexcept {
    configureDataLinesDirection<0xFFFF, compareWithPrevious>();
  }
  template<bool compareWithPrevious = true>
  static inline void
  configureDataLinesForWrite() noexcept {
    configureDataLinesDirection<0, compareWithPrevious>();
  }
  template<bool wait = true, uint32_t readyDelayTimer = 0>
  static void
  signalReady() noexcept {
      readyTriggered = false;
      // run and block until we get the completion pulse
      digitalToggleFast(Pin::READY);
      if constexpr (wait) {
          {
              while (!readyTriggered);
          }
          fixedDelayNanoseconds<readyDelayTimer>(); // wait some amount of time
      }
  }
  static inline bool
  isReadOperation() noexcept {
    return digitalReadFast(Pin::WR) == LOW;
  }
  static inline bool
  isWriteOperation() noexcept {
    return digitalReadFast(Pin::WR) == HIGH;
  }
  template<bool enableAddressAcceleration = true>
  static uint32_t 
  getAddress() noexcept {
      uint32_t value = 0;
      value |= static_cast<uint32_t>(read8(addressLines.getBaseAddress()));
      value |= static_cast<uint32_t>(read8(addressLines.getBaseAddress()+1)) << 8;
      value |= static_cast<uint32_t>(read8(addressLines.getBaseAddress()+2)) << 16;
      if constexpr (enableAddressAcceleration) {
          if (digitalReadFast(Pin::IO_SPACE) == LOW) {
              value |= 0xFE00'0000;
          } else if (digitalReadFast(Pin::MEM_SPACE) != LOW) {
              value |= static_cast<uint32_t>(read8(addressLines.getBaseAddress()+3)) << 24;
          }
      } else {
        value |= static_cast<uint32_t>(read8(addressLines.getBaseAddress()+3)) << 24;
      }
      SynchronizeData;
      return value;
  }
  static inline bool
  isBurstLast() noexcept {
    return digitalReadFast(Pin::BLAST) == LOW;
  }
  static inline bool
  byteEnableLow() noexcept {
    return digitalReadFast(Pin::BE0) == LOW;
  }
  static inline bool
  byteEnableHigh() noexcept {
    return digitalReadFast(Pin::BE1) == LOW;
  }
  static inline void
  writeDataLines(uint16_t value) noexcept {
      auto baseAddress = dataLines.getDataPortBaseAddress();
      write8(baseAddress, static_cast<uint8_t>(value)); 
      write8(baseAddress+1, static_cast<uint8_t>(value >> 8));
  }
  static inline uint16_t
  readDataLines() noexcept {
      auto baseAddress = dataLines.getDataPortBaseAddress();
      // this will take at least 390ns to complete
      uint16_t lo = 0, 
               hi = 0;
      lo = read8(baseAddress);
      hi = read8(baseAddress+1);
      SynchronizeData;
      return lo | (hi << 8);
  }

  template<bool isReadTransaction>
  static inline void
  doNothingTransaction() noexcept {
    if constexpr (isReadTransaction) {
      writeDataLines(0);
    }
    while (true) {
      if (isBurstLast()) {
        break;
      }
      signalReady();
    }
    signalReady();
  }
  template<MemoryCell MC>
  static void
  doMemoryCellReadTransaction(const MC& target, uint8_t offset) noexcept {
      for (uint8_t wordOffset = offset >> 1; ; ++wordOffset) {
          writeDataLines(target.getWord(wordOffset));
          if (isBurstLast()) {
              break;
          } 
          signalReady();
      }
      signalReady();
  }
  template<MemoryCell MC>
  static void
  doMemoryCellWriteTransaction(MC& target, uint8_t offset) noexcept {
      for (uint8_t wordOffset = offset >> 1; ; ++wordOffset ) {
          target.setWord(wordOffset, readDataLines(), byteEnableLow(), byteEnableHigh());
          if (isBurstLast()) {
              break;
          } 
          signalReady();
      }
      signalReady();
  }
  template<bool isReadTransaction, MemoryCell MC>
  static inline void
  doMemoryCellTransaction(MC& target, uint8_t offset) noexcept {
      target.update();
      if constexpr (isReadTransaction) {
          doMemoryCellReadTransaction(target, offset);
      } else {
          doMemoryCellWriteTransaction(target, offset);
      }
      target.onFinish();
  }
  template<bool isReadTransaction, MemoryCell MC>
  static inline void 
  transmitConstantMemoryCell(const MC& cell, uint8_t offset) noexcept {
      if constexpr (isReadTransaction) {
          doMemoryCellReadTransaction(cell, offset);
      } else {
          doNothingTransaction<isReadTransaction>();
      }
  }
  template<bool isReadTransaction>
  static inline void
  handleBuiltinDevices(uint8_t offset) noexcept {
      auto lineOffset = offset & 0xF;
      switch (offset) {
          case 0x00 ... 0x07:
              transmitConstantMemoryCell<isReadTransaction>(CLKValues, lineOffset);
              break;
          case 0x08 ... 0x0F:
              doMemoryCellTransaction<isReadTransaction>(usbSerial, lineOffset);
              break;
          case 0x10 ... 0x1F:
              doMemoryCellTransaction<isReadTransaction>(timingInfo, lineOffset);
              break;
          case 0x20 ... 0x2F:
              doMemoryCellTransaction<isReadTransaction>(rtcInterface, lineOffset);
              break;
          case 0x30 ... 0x3F:
              // new entropy related stuff
              doMemoryCellTransaction<isReadTransaction>(randomSource, lineOffset);
              break;
          default:
              doNothingTransaction<isReadTransaction>();
              break;
      }
  }
  template<bool isReadTransaction>
  static inline void
  doIOTransaction(uint32_t address) noexcept {
      switch (address & 0xFF'FFFF) {
          case 0x00'0000 ... 0x00'00FF:
              handleBuiltinDevices<isReadTransaction>(address & 0xFF);
              break;
          //case 0x00'0100 ... 0x00'01FF:
          //    doMemoryCellTransaction<isReadTransaction>(oledDisplay, address & 0xFF);
          //    break;
          case 0x00'0800 ... 0x00'0FFF: 
              doMemoryCellTransaction<isReadTransaction>(sramCache[(address >> 4) & 0x7F], address & 0xF);
              break;
          case 0x00'1000 ... 0x00'1FFF: // EEPROM
              eeprom.updateBaseAddress(address);
              doMemoryCellTransaction<isReadTransaction>(eeprom, address & 0xF);
              break;
          default:
              doNothingTransaction<isReadTransaction>();
              break;
      }
  }

  template<bool isReadTransaction>
  static inline void
  doMemoryTransaction(uint32_t address) noexcept {
      if constexpr (isReadTransaction) {
          configureDataLinesForRead();
      } else {
          configureDataLinesForWrite();
      }
      switch (static_cast<uint8_t>(address >> 24)) {
          case 0x00: // PSRAM
              doMemoryCellTransaction<isReadTransaction>(memory960[(address >> 4) & 0x000F'FFFF], address & 0xF);
              break;
          case 0xFE: // IO Space
              doIOTransaction<isReadTransaction>(address);
              break;
          default:
              doNothingTransaction<isReadTransaction>();
              break;
      }
  }
  static void 
  setClockFrequency(uint32_t clk2, uint32_t clk1) noexcept {
      CLKValues.setWord32(0, clk1);
      CLKValues.setWord32(1, clk2);
  }
private:
  // allocate a 2k memory cache like the avr did
  static inline MemoryCellBlock sramCache[OnboardSRAMCacheSize / sizeof(MemoryCellBlock)];
  static inline MemoryCellBlock CLKValues{12 * 1000 * 1000, 6 * 1000 * 1000 };
  // use this to keep track of the output values that were sent to the EBI
  // useful when you don't want to waste time if the given output address
  // already contains the value in question. This does not apply to reads
  static inline uint8_t EBIOutputStorage[256] = { 0 }; 
};

void 
setupSDCard() noexcept {
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("No SDCARD found!");
        return;
    } 
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



void setupRandomSeed() noexcept {
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
  Serial.printf("Random Seed: 0x%x\n", newSeed);
}
void setupMemory() noexcept {
  Serial.println("Clearing PSRAM");
  for (auto& a : memory960) {
      a.clear();
  }
}
void
setupRTC() noexcept {
    if (rtc.begin(&Wire2)) {
        if (rtc.lostPower()) {
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        }
        Serial.println("Found RTC!");
        auto now = rtc.now();
        Serial.printf("unixtime: %d\n", now.unixtime());
    }
}
void
sinkWire() noexcept {
    while (Wire2.available()) {
        (void)Wire2.read();
    }
}
const uint8_t setCPUClockMode_CLKAll [] { 0, 0 };

void
displayClockSpeedInformation() noexcept {
    SplitWord64 clk3;
    Wire2.beginTransmission(0x08);
    Wire2.write(setCPUClockMode_CLKAll, sizeof(setCPUClockMode_CLKAll));
    Wire2.endTransmission();
    auto count = Wire2.requestFrom(0x08, sizeof(clk3));

    for (int i = 0; i < count;) {
        if (Wire2.available()) {
            clk3.bytes[i] = Wire2.read();
            ++i;
        }
    }
    Serial.printf("CLK2: %u\nCLK1: %u\n", clk3.words[0], clk3.words[1]);
    i960Interface::setClockFrequency(clk3.words[0], clk3.words[1]);
}
void
waitForAVRToComeUp() noexcept {
    Wire2.beginTransmission(0x08);
    Wire2.write(static_cast<uint8_t>(ManagementEngineReceiveOpcode::SetMode));
    Wire2.write(static_cast<uint8_t>(ManagementEngineRequestOpcode::ChipIsReady));
    Wire2.endTransmission();

    bool chipIsUp = false;
    do {
        auto count = Wire2.requestFrom(0x08, 1);
        for (int i = 0; i < count; ) {
            if (Wire2.available()) {
                chipIsUp = Wire2.read() != 0;
                ++i;
            }
        }
    } while (!chipIsUp);


}
void
putCPUInReset() noexcept {
    Wire2.beginTransmission(0x08);
    Wire2.write(static_cast<uint8_t>(ManagementEngineReceiveOpcode::PutInReset));
    Wire2.endTransmission();
}

void
pullCPUOutOfReset() noexcept {
    Wire2.beginTransmission(0x08);
    Wire2.write(static_cast<uint8_t>(ManagementEngineReceiveOpcode::PullOutOfReset));
    Wire2.endTransmission();
}
void
triggerADS() noexcept {
    adsTriggered = true;
    //SynchronizeData;
}
void
triggerReadySync() noexcept {
    readyTriggered = true;
    //SynchronizeData;
}
void
setupLEDMatrix() noexcept {
    if (!ledmatrix.begin(IS3741_ADDR_DEFAULT, &Wire2)) {
        Serial.println("IS41 not found!");
    } else {
        ledmatrix.setLEDscaling(0xff);
        ledmatrix.setGlobalCurrent(0xff);
        Serial.print("Global LED Current set to: ");
        Serial.println(ledmatrix.getGlobalCurrent());
        ledmatrix.fill(0);
        ledmatrix.enable(true);
        ledmatrix.setRotation(0);
        ledmatrix.setTextWrap(false);
        ledmatrix.print("i960");
        ledmatrix.setGlobalCurrent(5);
    }
}
void 
triggerSystemTimer() noexcept {
    if (systemCounterEnabled) {
        digitalToggleFast(Pin::INT960_0); 
    }
}
void 
setup() {
    Wire2.begin();
    delay(1000);
    waitForAVRToComeUp();
    putCPUInReset();
    inputPin(Pin::ADS);
    outputPin(Pin::INT960_0, HIGH);
    inputPin(Pin::BE0);
    inputPin(Pin::BE1);
    inputPin(Pin::WR);
    outputPin(Pin::READY, HIGH);
    inputPin(Pin::BLAST);
    inputPin(Pin::READY_SYNC);
    inputPin(Pin::IO_SPACE);
    inputPin(Pin::MEM_SPACE);


    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    Entropy.Initialize();
    EEPROM.begin();
    // put your setup code here, to run once:
    EBIInterface::begin();
    i960Interface::begin();
    setupMemory();
    setupRTC();
    setupSDCard();
    SPI.begin();
    setupRandomSeed();
    setupLEDMatrix();
    Entropy.Initialize();
    PCJoystick::begin();
    GamepadQT::begin();
    systemTimer.begin(triggerSystemTimer, 100'000);
    attachInterrupt(Pin::ADS, triggerADS, FALLING);
    attachInterrupt(Pin::READY_SYNC, triggerReadySync, FALLING);
    displayClockSpeedInformation();
    pullCPUOutOfReset();
    // so attaching the interrupt seems to not be functioning fully
}

void 
loop() {
    if (adsTriggered) {
        adsTriggered = false;
        if (auto targetAddress = i960Interface::getAddress(); i960Interface::isReadOperation()) {
            i960Interface::doMemoryTransaction<true>(targetAddress);
        } else {
            i960Interface::doMemoryTransaction<false>(targetAddress);
        }
    } 
}


