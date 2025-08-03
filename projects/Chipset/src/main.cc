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
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
#include <Adafruit_EEPROM_I2C.h>
#include <arduino_freertos.h>
#include <semphr.h>

#include "Pinout.h"
#include "MemoryCell.h"
#include "Core.h"
#include "ManagementEngineProtocol.h"
// Thanks to an interactive session with copilot I am realizing that while the
// CH351 has some real limitations when it comes to write operations. There are
// minimum hold times in between writes. Which is around 50 ns
//
constexpr auto OLEDScreenWidth = 128;
constexpr auto OLEDScreenHeight = 128;
constexpr uint32_t OnboardSRAMCacheSize = 2048;
constexpr auto MemoryPoolSizeInBytes = (16 * 1024 * 1024);  // 16 megabyte psram pool
constexpr auto EEPROM_I2C_Address = 0x50; // default address
volatile bool adsTriggered = false;
volatile bool readyTriggered = false;
volatile bool systemCounterEnabled = false;
Adafruit_EEPROM_I2C eeprom2;
RTC_DS3231 rtc;
Adafruit_SSD1351 tft(OLEDScreenWidth, 
        OLEDScreenHeight, 
        &SPI,
        pinIndexConvert(Pin::EYESPI_TCS),
        pinIndexConvert(Pin::EYESPI_DC),
        pinIndexConvert(Pin::EYESPI_RST));
Metro systemCounterWatcher{25}; // every 25ms, do a toggle
constexpr uint16_t color565(uint8_t r, uint8_t g, uint8_t b) noexcept {
    // taken from the color565 routine in Adafruit GFX
    return (static_cast<uint16_t>(r & 0xF8) << 8) |
           (static_cast<uint16_t>(g & 0xFC) << 3) |
           (static_cast<uint16_t>(b >> 3));
}
constexpr uint16_t Color_Black = color565(0, 0, 0);
constexpr uint16_t Color_Blue = color565(0, 0, 255);
constexpr uint16_t Color_Red = color565(255, 0, 0);
constexpr uint16_t Color_Green = color565(0, 255, 0);
constexpr uint16_t Color_Purple = color565(255, 0, 255);
constexpr uint16_t Color_White = color565(255, 255, 255);
void
setupEEPROM2() noexcept {
    if (eeprom2.begin(EEPROM_I2C_Address, &Wire2)) {
        Serial.println("Found I2C EEPROM!");
    } else {
        Serial.println("No I2C EEPROM Found!");
    }
}

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
        //Serial.printf("offset: 0x%x: 0x%x\n", offset, value);
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
  template<bool directPortManipulation = true>
  static void
  setAddress(uint8_t address) noexcept {
      if constexpr (directPortManipulation) {
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
  template<bool checkD0 = true, bool useDirectPortRead = true>
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
  template<bool force = false, bool directPortManipulation = true>
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
  static void
  setDataLinesDirection(PinDirection direction) noexcept {
    if (_currentDirection != direction) {
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
      _currentDirection = direction;
    }
  }
  template<PinDirection direction>
  static void
  setDataLinesDirection() noexcept {
    if (_currentDirection != direction) {
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
struct OLEDInterface final {
    enum class ExecCodes : uint16_t {
        Nothing,
        DrawPixel,
        FillScreen,
        DrawLine,
        DrawFastHLine,
        DrawFastVLine,
        DrawRect,
        FillRect,
        DrawCircle,
        FillCircle,
        DrawTriangle,
        FillTriangle,
        DrawRoundRect,
        FillRoundRect,
    };
    enum class Fields : uint8_t {
        Execute,
        Opcode,
        Argument0,
        Argument1,
        Argument2,
        Argument3,
        Argument4,
        Argument5,
        Argument6,
        Argument7,
        Argument8,
        Argument9,
        Argument10,
        Argument11,
        Argument12,
        Argument13,
        // then we start with the other fields
        Rotation,
        Brightness,
        ScreenWidth,
        ScreenHeight,

        CursorX,
        CursorY,

        TextSize,

    };
    // this "MemoryBlock" reserves a 256 byte section of the 32-bit IO space
    void update() noexcept {
        // the lower 16 bits are the "enable" bits
        // each time we access this information we make sure that it can't
        // accidentally execute
        clearExecStatus();
    }
    uint16_t getWord(uint8_t offset) const noexcept {
        switch (static_cast<Fields>(offset)) {
            case Fields::Execute:
            case Fields::Opcode:
            case Fields::Argument0:
            case Fields::Argument1:
            case Fields::Argument2:
            case Fields::Argument3:
            case Fields::Argument4:
            case Fields::Argument5:
            case Fields::Argument6:
            case Fields::Argument7:
            case Fields::Argument8:
            case Fields::Argument9:
            case Fields::Argument10:
            case Fields::Argument11:
            case Fields::Argument12:
            case Fields::Argument13:
                return _argumentStorage[offset];
            case Fields::Rotation:
                return tft.getRotation();
            case Fields::ScreenWidth:
                return tft.width();
            case Fields::ScreenHeight:
                return tft.height();
            case Fields::CursorX:
                return tft.getCursorX();
            case Fields::CursorY:
                return tft.getCursorY();
            default:
                return 0;
        }
    }
    void setWord(uint8_t offset, uint16_t value, bool enableLo = true, bool enableHi = true) noexcept { 
        switch (static_cast<Fields>(offset)) {
            case Fields::Execute:
            case Fields::Opcode:
            case Fields::Argument0:
            case Fields::Argument1:
            case Fields::Argument2:
            case Fields::Argument3:
            case Fields::Argument4:
            case Fields::Argument5:
            case Fields::Argument6:
            case Fields::Argument7:
            case Fields::Argument8:
            case Fields::Argument9:
            case Fields::Argument10:
            case Fields::Argument11:
            case Fields::Argument12:
            case Fields::Argument13:
                _argumentStorage[offset] = value; 
                break;
            case Fields::Rotation:
                tft.setRotation(value);
                break;
            case Fields::CursorX:
                tft.setCursor(value, tft.getCursorY());
                break;
            case Fields::CursorY:
                tft.setCursor(tft.getCursorX(), value);
                break;
            case Fields::TextSize: 
                tft.setTextSize(
                        static_cast<uint8_t>(value), 
                        static_cast<uint8_t>(value >> 8));
                break;
            default:
                break;
        }
    }
    void onFinish() noexcept { 
        if (shouldExecute()) {
            switch (getExecCode()) {
                case ExecCodes::DrawPixel:
                    tft.drawPixel(
                            _argumentStorage[2],
                            _argumentStorage[3],
                            _argumentStorage[4]);
                    break;
                case ExecCodes::FillScreen:
                    tft.fillScreen(_argumentStorage[2]);
                    break;
                case ExecCodes::DrawLine:
                    tft.drawLine(
                            _argumentStorage[2],
                            _argumentStorage[3],
                            _argumentStorage[4],
                            _argumentStorage[5],
                            _argumentStorage[6]);
                    break;
                case ExecCodes::DrawFastHLine:
                    tft.drawFastHLine(
                            _argumentStorage[2],
                            _argumentStorage[3],
                            _argumentStorage[4],
                            _argumentStorage[5]);
                    break;
                case ExecCodes::DrawFastVLine:
                    tft.drawFastVLine(
                            _argumentStorage[2],
                            _argumentStorage[3],
                            _argumentStorage[4],
                            _argumentStorage[5]);
                    break;
                case ExecCodes::DrawRect:
                    tft.drawRect(
                            _argumentStorage[2],
                            _argumentStorage[3],
                            _argumentStorage[4],
                            _argumentStorage[5],
                            _argumentStorage[6]);
                    break;
                case ExecCodes::FillRect:
                    tft.fillRect(
                            _argumentStorage[2],
                            _argumentStorage[3],
                            _argumentStorage[4],
                            _argumentStorage[5],
                            _argumentStorage[6]);
                    break;
                case ExecCodes::DrawCircle:
                    tft.drawCircle(
                            _argumentStorage[2],
                            _argumentStorage[3],
                            _argumentStorage[4],
                            _argumentStorage[5]);
                    break;
                case ExecCodes::FillCircle:
                    tft.fillCircle(
                            _argumentStorage[2],
                            _argumentStorage[3],
                            _argumentStorage[4],
                            _argumentStorage[5]);
                    break;
                case ExecCodes::DrawTriangle:
                    tft.drawTriangle(
                            _argumentStorage[2],
                            _argumentStorage[3],

                            _argumentStorage[4],
                            _argumentStorage[5],

                            _argumentStorage[6],
                            _argumentStorage[7],

                            _argumentStorage[8]
                            
                            );
                    break;
                case ExecCodes::FillTriangle:
                    tft.fillTriangle(
                            _argumentStorage[2],
                            _argumentStorage[3],

                            _argumentStorage[4],
                            _argumentStorage[5],

                            _argumentStorage[6],
                            _argumentStorage[7],

                            _argumentStorage[8]
                            
                            );
                    break;
                case ExecCodes::DrawRoundRect:
                    tft.drawRoundRect(
                            _argumentStorage[2],
                            _argumentStorage[3],

                            _argumentStorage[4],
                            _argumentStorage[5],

                            _argumentStorage[6],
                            _argumentStorage[7]
                            );
                    break;
                case ExecCodes::FillRoundRect:
                    tft.fillRoundRect(
                            _argumentStorage[2],
                            _argumentStorage[3],

                            _argumentStorage[4],
                            _argumentStorage[5],

                            _argumentStorage[6],
                            _argumentStorage[7]
                            );
                    break;
                default:
                    break;
            }
        }
    }
    private:
        constexpr ExecCodes getExecCode() const noexcept {
            return static_cast<ExecCodes>(_argumentStorage[1]);
        }
        constexpr bool shouldExecute() const noexcept {
            return _argumentStorage[0] != 0;
        }
        void clearExecStatus() noexcept {
            _argumentStorage[0] = 0;
        }
    private:
        uint16_t _argumentStorage[16];
};

OLEDInterface oledDisplay;

struct i960Interface {
  i960Interface() = delete;
  ~i960Interface() = delete;
  i960Interface(const i960Interface&) = delete;
  i960Interface(i960Interface&&) = delete;
  i960Interface& operator=(const i960Interface&) = delete;
  i960Interface& operator=(i960Interface&&) = delete;

  static inline void write8(uint8_t address, uint8_t value) noexcept {
    EBIInterface::setDataLinesDirection<OUTPUT>();
    EBIInterface::setAddress(address);
    delayNanoseconds(50);
    EBIInterface::setDataLines(value);
    delayNanoseconds(30); // setup time (tDS), normally 30
    digitalWriteFast(Pin::EBI_WR, LOW);
    delayNanoseconds(100); // tWL hold for at least 80ns
    digitalWriteFast(Pin::EBI_WR, HIGH);
    delayNanoseconds(100); // data hold after WR + tWH
  }
  static void
  begin() noexcept {
      write8(addressLines.getConfigPortBaseAddress(), 0);
      delayNanoseconds(50); // breathe for 50ns
      write8(addressLines.getConfigPortBaseAddress() + 1, 0);
      delayNanoseconds(50); // breathe for 50ns
      write8(addressLines.getConfigPortBaseAddress() + 2, 0);
      delayNanoseconds(50); // breathe for 50ns
      write8(addressLines.getConfigPortBaseAddress() + 3, 0);
      delayNanoseconds(50); // breathe for 50ns
      configureDataLinesForRead<false>();
      EBIInterface::setDataLines(0);
  }
  template<uint16_t value, bool compareWithPrevious = true>
  static inline void
  configureDataLinesDirection() noexcept {
      if constexpr (compareWithPrevious) {
          if (_dataLinesDirection == value) {
              return;
          }
      }
      write8(dataLines.getConfigPortBaseAddress(), static_cast<uint8_t>(value));
      delayNanoseconds(50); // breathe
      write8(dataLines.getConfigPortBaseAddress()+1, static_cast<uint8_t>(value >> 8));
      delayNanoseconds(50); // breathe
      _dataLinesDirection = value;
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
  inline static void
  waitForReadyTrigger() noexcept {
        while (!readyTriggered) {
            yield();
        }
  }
  inline static void 
  triggerReady() noexcept {
      digitalWriteFast(Pin::READY, LOW);
  }
  inline static void
  finishReadyTrigger() noexcept {
      readyTriggered = false;
      digitalWriteFast(Pin::READY, HIGH);
      delayNanoseconds(200);  // wait some amount of time
  }
  static inline void
  signalReady() noexcept {
      // run and block until we get the completion pulse
      triggerReady();
      waitForReadyTrigger();
      finishReadyTrigger();
  }
  static inline bool
  isReadOperation() noexcept {
    return digitalReadFast(Pin::WR) == LOW;
  }
  static inline bool
  isWriteOperation() noexcept {
    return digitalReadFast(Pin::WR) == HIGH;
  }
  static inline uint8_t
  read8(uint8_t address) noexcept {
      // the CH351 has some very strict requirements
      EBIInterface::setDataLinesDirection<INPUT>();
      EBIInterface::setAddress(address);
      delayNanoseconds(100);
      digitalWriteFast(Pin::EBI_RD, LOW);
      delayNanoseconds(80); // wait for things to get selected properly
      uint8_t output = EBIInterface::readDataLines();
      delayNanoseconds(20);
      digitalWriteFast(Pin::EBI_RD, HIGH);
      delayNanoseconds(50);
      return output;
  }
  static inline uint32_t
  getAddress() noexcept {
      uint32_t a = read8(addressLines.getDataPortBaseAddress());
      delayNanoseconds(50);
      uint32_t b = read8(addressLines.getDataPortBaseAddress() + 1);
      delayNanoseconds(50);
      uint32_t c = read8(addressLines.getDataPortBaseAddress() + 2);
      delayNanoseconds(50);
      uint32_t d = read8(addressLines.getDataPortBaseAddress() + 3);
      delayNanoseconds(50);
      return a |  (b << 8) | (c << 16) | (d << 24);
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
      write8(dataLines.getDataPortBaseAddress(), static_cast<uint8_t>(value));
      delayNanoseconds(50); // breathe
      write8(dataLines.getDataPortBaseAddress()+1, static_cast<uint8_t>(value >> 8));
      delayNanoseconds(50); // breathe
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
  static inline void
  doMemoryCellReadTransaction(const MC& target, uint8_t offset) noexcept {
      for (uint8_t wordOffset = offset >> 1; ; ++wordOffset) {
          Serial.printf("\t%d: %x\r\n", wordOffset, target.getWord(wordOffset));
          writeDataLines(target.getWord(wordOffset));
          if (isBurstLast()) {
              signalReady();
              break;
          } else {
              signalReady();
          }
      }
      // nothing to do on target end
  }
  static inline uint16_t
  readDataLines() noexcept {
      uint16_t a = read8(dataLines.getDataPortBaseAddress());
      uint16_t b = read8(dataLines.getDataPortBaseAddress()+1);
      return a| (b<< 8);
  }
  template<MemoryCell MC>
  static inline void
  doMemoryCellWriteTransaction(MC& target, uint8_t offset) noexcept {
      for (uint8_t wordOffset = offset >> 1; ; ++wordOffset ) {
          target.setWord(wordOffset, readDataLines(), byteEnableLow(), byteEnableHigh());
          if (isBurstLast()) {
              signalReady();
              break;
          } else {
              signalReady();
          }
      }
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
          case 0x00'0100 ... 0x00'01FF:
              doMemoryCellTransaction<isReadTransaction>(oledDisplay, address & 0xFF);
              break;
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
      switch (address) {
          case 0x0000'0000 ... 0x00FF'FFFF: // PSRAM
              doMemoryCellTransaction<isReadTransaction>(memory960[(address >> 4) & 0x000F'FFFF], address & 0xF);
              break;
          case 0xFE00'0000 ... 0xFEFF'FFFF: // IO Space
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
  static inline uint16_t _dataLinesDirection = 0xFFFF;
  // allocate a 2k memory cache like the avr did
  static inline MemoryCellBlock sramCache[OnboardSRAMCacheSize / sizeof(MemoryCellBlock)];
  static inline MemoryCellBlock CLKValues{12 * 1000 * 1000, 6 * 1000 * 1000 };
};

void
setupTFTDisplay() noexcept {
    tft.begin();
    tft.setFont();
    tft.fillScreen(Color_Black);
    tft.setTextColor(Color_White, Color_Black);
    Serial.printf("TFT Display (WxH): %d pixels by %d pixels\n", tft.width(), tft.height());
}
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
//SemaphoreHandle_t adsTriggeredSemaphore;
TaskHandle_t memoryTask = nullptr;
void
triggerADS() noexcept {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveIndexedFromISR(memoryTask, 0, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}
void
triggerReadySync() noexcept {
    readyTriggered = true;
}
void
handleMemoryTransaction(void*) noexcept {
    Entropy.Initialize();
    EEPROM.begin();
    // put your setup code here, to run once:
    EBIInterface::begin();
    i960Interface::begin();
    setupMemory();
    setupRTC();
    setupSDCard();
    setupTFTDisplay();
    setupRandomSeed();
    setupEEPROM2();
    Entropy.Initialize();
    taskDISABLE_INTERRUPTS();
    attachInterrupt(Pin::ADS, triggerADS, RISING);
    attachInterrupt(Pin::READY_SYNC, triggerReadySync, RISING);
    taskENABLE_INTERRUPTS();
    taskENTER_CRITICAL();
    displayClockSpeedInformation();
    pullCPUOutOfReset();
    taskEXIT_CRITICAL();
    //vTaskDelay(pdMS_TO_TICKS(1000));
    // so attaching the interrupt seems to not be functioning fully
    Serial.println("Booted i960");
    while (true) {
        (void)ulTaskNotifyTakeIndexed(0, pdTRUE, portMAX_DELAY);
        readyTriggered = false;
        while (digitalReadFast(Pin::DEN) == HIGH) ;
        auto targetAddress = i960Interface::getAddress();
        Serial.printf("Target Address: %x\n", targetAddress);
        if (i960Interface::isReadOperation()) {
            i960Interface::doMemoryTransaction<true>(targetAddress);
        } else {
            i960Interface::doMemoryTransaction<false>(targetAddress);
        }
    }
    vTaskDelete(nullptr);
}
void
handleSystemCounterWatcher(void*) noexcept {
    while (true) {
        if (systemCounterWatcher.check()) {
            if (systemCounterEnabled) {
                digitalToggleFast(Pin::INT960_0);
            }
        }
    }
    vTaskDelete(nullptr);
}
void
lifeTest(void*) noexcept {
    while (true) {
        SerialUSB1.println("TICK");
        vTaskDelay(pdMS_TO_TICKS(1000));
        SerialUSB1.println("TOCK");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(nullptr);
}
void 
setup() {
    Wire2.begin();
    inputPin(Pin::AVR_UP);
    while (digitalReadFast(Pin::AVR_UP) != HIGH) {
        delay(10);
    }
    putCPUInReset();
    inputPin(Pin::ADS);
    inputPin(Pin::DEN);
    //outputPin(Pin::HOLD, LOW);
    outputPin(Pin::INT960_0, HIGH);
    outputPin(Pin::INT960_1, LOW);
    outputPin(Pin::INT960_2, LOW);
    outputPin(Pin::INT960_3, HIGH);
    //inputPin(Pin::LOCK);
    inputPin(Pin::BE0);
    inputPin(Pin::BE1);
    inputPin(Pin::WR);
    outputPin(Pin::READY, HIGH);
    inputPin(Pin::BLAST);
    inputPin(Pin::READY_SYNC);

    Serial.begin(9600);
    while (!Serial) {
        delay(10);
    }
    SerialUSB1.begin(115200);
    //adsTriggeredSemaphore = xSemaphoreCreateBinary();
    // okay so we want to handle the initial boot process
    xTaskCreate(handleMemoryTransaction, "memory", 32768, nullptr, 3, &memoryTask);
    xTaskCreate(handleSystemCounterWatcher, "syscount", 256, nullptr, 2, nullptr);
    xTaskCreate(lifeTest, "life", 128, nullptr, 2, nullptr);

    Serial.println(F("Starting scheduler ..."));
    Serial.flush();
    vTaskStartScheduler();
}
void 
loop() {
}

