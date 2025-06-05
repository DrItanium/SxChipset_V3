#include <Arduino.h>
#include <type_traits>
#include <concepts>
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
// Thanks to an interactive session with copilot I am realizing that while the
// CH351 has some real limitations when it comes to write operations. There are
// minimum hold times in between writes. Which is around 50 ns
//
//
constexpr auto DefaultWaitAmount = 100; // ns
constexpr uint32_t OnboardSRAMCacheSize = 2048;
constexpr auto MemoryPoolSizeInBytes = (16 * 1024 * 1024);  // 16 megabyte psram pool
template<typename T>
struct TreatAs final {
    using underlying_type = T;
};
// A high speed interface that we can abstract contents of memory 
template<typename T>
concept MemoryCell = requires(T a) {
    { a.update() };
    // only operate on 16-bit words
    { a.getWord(0) } -> uint16_t;
    { a.setWord(0, 0) };
    { a.setWord(0, 0, true, true) };
};

union MemoryCellBlock {
  constexpr MemoryCellBlock(uint32_t a = 0, uint32_t b = 0, uint32_t c = 0, uint32_t d = 0) : words{a, b, c, d} { }
private:
  uint8_t bytes[16];
  uint16_t shorts[8];
  uint32_t words[4];
public:
  void update() noexcept {

  }
  void clear() noexcept {
      for (auto& a : words) {
          a = 0;
      }
  }
  [[nodiscard]] inline constexpr uint16_t getWord(uint8_t offset) const noexcept { return shorts[offset & 0b111]; }
  inline void setWord(uint8_t offset, uint16_t value) noexcept { shorts[offset & 0b111] = value; }
  inline void setWord(uint8_t offset, uint16_t value, bool updateLo, bool updateHi) noexcept {
    // convert to an 8-bit setup so we can do conversions as needed
    uint8_t baseOffset = (offset << 1) & 0b1110;
    if (updateLo) {
        bytes[baseOffset] = static_cast<uint8_t>(value);
    }
    if (updateHi) {
        bytes[baseOffset+1] = static_cast<uint8_t>(value >> 8);
    }
  }
};
static_assert(sizeof(MemoryCellBlock) == 16, "MemoryCellBlock needs to be 16 bytes in size");
struct USBSerialBlock {
    void update() noexcept {

    }
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
            default:
                return 0;
        }
    }
    void setWord(uint8_t, uint16_t) noexcept { }
    void setWord(uint8_t, uint16_t, bool, bool) noexcept { }
private:
    uint32_t _currentRandomValue = 0;
};
USBSerialBlock usbSerial;
TimingRelatedThings timingInfo;
RandomSourceRelatedThings randomSource;
EXTMEM MemoryCellBlock memory960[MemoryPoolSizeInBytes / sizeof(MemoryCellBlock)];
enum class Pin : uint8_t {
  RPI_D0 = 19,
  RPI_D1 = 18,
  RPI_D2 = 14,
  RPI_D3 = 15,
  RPI_D4 = 40,
  RPI_D5 = 41,
  RPI_D6 = 17,
  RPI_D7 = 16,
  RPI_D8 = 22,
  RPI_D9 = 23,
  RPI_D10 = 20,
  RPI_D11 = 21,
  RPI_D12 = 38,
  RPI_D13 = 39,
  RPI_D14 = 26,
  RPI_D15 = 27,
  RPI_D16 = 8,
  RPI_D17 = 7,
  RPI_D18 = 36,
  RPI_D19 = 37,
  RPI_D20 = 32,
  RPI_D21 = 9,
  RPI_D22 = 6,
  RPI_D23 = 2,
  RPI_D24 = 3,
  RPI_D25 = 4,
  RPI_D26 = 33,
  RPI_D27 = 5,
  RPI_D28 = 28,
  RPI_D29 = 29,
  RPI_D30 = 30,
  RPI_D31 = 31,
  RPI_D32 = 34,
  RPI_D33 = 35,
  EBI_A5 = RPI_D0, // AD_B0_03
  EBI_A4 = RPI_D1, // AD_B0_02
  EBI_A3 = RPI_D2, // EMC_04
  EBI_A2 = RPI_D3, // EMC_05
  EBI_A1 = RPI_D4, // EMC_06
  EBI_A0 = RPI_D5, // EMC_08
  EBI_RD = RPI_D6, // B0_10
  EBI_WR = RPI_D7, // B1_01
  EBI_D0 = RPI_D8, // B1_00
  EBI_D1 = RPI_D9, // B0_11
  EBI_D2 = RPI_D10, // B0_00
  EBI_D3 = RPI_D11, // B0_02
  EBI_D4 = RPI_D12, // B0_01
  EBI_D5 = RPI_D13, // B0_03
  EBI_D6 = RPI_D14, // AD_B1_02
  EBI_D7 = RPI_D15, // AD_B1_03
  ADS = RPI_D16,
  BLAST = RPI_D17,
  DEN = RPI_D18,
  WR = RPI_D19,
  BE0 = RPI_D20,
  BE1 = RPI_D21,
  LOCK = RPI_D22,
  HOLD = RPI_D23,
  INT960_3 = RPI_D24,
  INT960_2 = RPI_D25,
  INT960_1 = RPI_D26,
  INT960_0 = RPI_D27,  
  RESET = RPI_D28,
  HLDA = RPI_D29,
  READY = RPI_D30,
  READY_SYNC = RPI_D31,
  FAIL = RPI_D32,
};

constexpr std::underlying_type_t<Pin> pinIndexConvert(Pin value) noexcept {
  return static_cast<std::underlying_type_t<Pin>>(value);
}
using PinDirection = decltype(OUTPUT);
using PinValue = decltype(HIGH);
inline void pinMode(Pin pin, PinDirection direction) noexcept {
  ::pinMode(pinIndexConvert(pin), direction);
}

inline void digitalWriteFast(Pin pin, PinValue value) noexcept {
  ::digitalWriteFast(pinIndexConvert(pin), value);
}
inline void digitalWrite(Pin pin, PinValue value) noexcept {
  ::digitalWrite(pinIndexConvert(pin), value);
}
inline decltype(auto) digitalReadFast(Pin pin) noexcept {
  return ::digitalReadFast(pinIndexConvert(pin));
}
inline decltype(auto) digitalRead(Pin pin) noexcept {
  return ::digitalRead(pinIndexConvert(pin));
}
inline void attachInterrupt(Pin pin, void (*function)(void), decltype(RISING) mode) noexcept {
  ::attachInterrupt(digitalPinToInterrupt(pinIndexConvert(pin)), function, mode);
}

inline void digitalToggle(Pin pin) noexcept {
  ::digitalToggle(pinIndexConvert(pin));
}
template<Pin p>
inline void digitalToggle() noexcept {
  digitalToggle(p);
}

inline void digitalToggleFast(Pin pin) noexcept {
  ::digitalToggleFast(pinIndexConvert(pin));
}
template<Pin p>
inline void digitalToggleFast() noexcept {
  digitalToggleFast(p);
}
inline void outputPin(Pin p, PinValue initialValue) noexcept {
  pinMode(p, OUTPUT);
  digitalWrite(p, initialValue);
}
inline void inputPin(Pin p) noexcept {
  pinMode(p, INPUT);
}
volatile bool adsTriggered = false;
volatile bool readyTriggered = false;
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
    setAddress<0>();
    digitalWriteFast(Pin::EBI_RD, HIGH);
    digitalWriteFast(Pin::EBI_WR, HIGH);
    setDataLines<true>(0);
  }
  template<bool directPortManipulation = false>
  static inline void
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
  template<uint8_t address, bool directPortManipulation = false>
  static inline void
  setAddress() noexcept {
    if constexpr (directPortManipulation) {
        GPIO6_DR_CLEAR = EBIAddressTable[0xFF];
        GPIO6_DR_SET = EBIAddressTable[address];
    } else {
#define X(pin, mask) \
      if constexpr ((address & mask) != 0) { \
          digitalWriteFast(pin, HIGH); \
      } else { \
          digitalWriteFast(pin, LOW); \
      }
      X(Pin::EBI_A0, 0b000001);
      X(Pin::EBI_A1, 0b000010);
      X(Pin::EBI_A2, 0b000100);
      X(Pin::EBI_A3, 0b001000);
      X(Pin::EBI_A4, 0b010000);
      X(Pin::EBI_A5, 0b100000);
#undef X
    }
  }
  template<bool checkD0 = true, bool useDirectPortRead = true>
  static inline uint8_t
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
  template<bool force = false, bool directPortManipulation = false>
  static inline void
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
  template<uint8_t value, bool force = false, bool directPortManipulation = false>
  static inline void
  setDataLines() noexcept {
      if constexpr (!force) {
          if (_currentOutputDataLines == value) {
              return;
          }
      }
      if constexpr (directPortManipulation) {
          GPIO6_DR_CLEAR = EBIOutputTransformation[0xFF];
          GPIO6_DR_SET = EBIOutputTransformation[value];
      } else {
#define X(pin, mask) \
      if constexpr ((value & mask) != 0) { \
          digitalWriteFast(pin, HIGH); \
      } else { \
          digitalWriteFast(pin, LOW); \
      }
      X(Pin::EBI_D0, 0b0000'0001);
      X(Pin::EBI_D1, 0b0000'0010);
      X(Pin::EBI_D2, 0b0000'0100);
      X(Pin::EBI_D3, 0b0000'1000);
      X(Pin::EBI_D4, 0b0001'0000);
      X(Pin::EBI_D5, 0b0010'0000);
      X(Pin::EBI_D6, 0b0100'0000);
      X(Pin::EBI_D7, 0b1000'0000);
#undef X
      }
      _currentOutputDataLines = value;
  }
  static inline void
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
  static inline void
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
  template<uint32_t delay = DefaultWaitAmount>
  static inline void
  write8(uint8_t address, uint8_t value) noexcept {
    setDataLinesDirection<OUTPUT>();
    setAddress(address);
    setDataLines(value);
    digitalWriteFast(Pin::EBI_WR, LOW);
    delayNanoseconds(delay);
    digitalWriteFast(Pin::EBI_WR, HIGH);
    delayNanoseconds(50); // to satisfy minimum writes to the CH351
  }
  template<uint32_t delay = DefaultWaitAmount>
  static inline void
  write16(uint8_t baseAddress, uint16_t value) noexcept {
      if ((baseAddress & 0b1) == 0) {
          // it is aligned so we don't need to worry
          setDataLinesDirection<OUTPUT>();
          setAddress(baseAddress);
          setDataLines(static_cast<uint8_t>(value));
          digitalWriteFast(Pin::EBI_WR, LOW);
          delayNanoseconds(delay);
          digitalWriteFast(Pin::EBI_WR, HIGH);
          digitalToggleFast(Pin::EBI_A0); // since it is aligned to 16-bit
                                          // boundaries we can safely bump it
                                          // forward by one
          setDataLines(static_cast<uint8_t>(value >> 8));
          delayNanoseconds(50); // to satisfy minimum writes to the CH351
          digitalWriteFast(Pin::EBI_WR, LOW);
          delayNanoseconds(delay);
          digitalWriteFast(Pin::EBI_WR, HIGH);
      } else {
          write8<delay>(baseAddress, static_cast<uint8_t>(value));
          write8<delay>(baseAddress + 1, static_cast<uint8_t>(value >> 8));

      } 

  }

  template<uint16_t value, uint32_t delay = DefaultWaitAmount>
  static inline void
  write16(uint8_t baseAddress) noexcept {
      if ((baseAddress & 0b1) == 0) {
          // it is aligned so we don't need to worry
          setDataLinesDirection<OUTPUT>();
          setAddress(baseAddress);
          setDataLines<static_cast<uint8_t>(value)>();
          digitalWriteFast(Pin::EBI_WR, LOW);
          delayNanoseconds(delay);
          digitalWriteFast(Pin::EBI_WR, HIGH);
          digitalToggleFast(Pin::EBI_A0); // since it is aligned to 16-bit
                                          // boundaries we can safely bump it
                                          // forward by one
          if constexpr (static_cast<uint8_t>(value >> 8) != static_cast<uint8_t>(value)) {
              // if the upper and lower halves are already equal then don't
              // waste any time
              setDataLines<static_cast<uint8_t>(value >> 8)>();
          }
          delayNanoseconds(50); // to satisfy minimum writes to the CH351
          digitalWriteFast(Pin::EBI_WR, LOW);
          delayNanoseconds(delay);
          digitalWriteFast(Pin::EBI_WR, HIGH);
      } else {
          write8<delay>(baseAddress, static_cast<uint8_t>(value));
          write8<delay>(baseAddress + 1, static_cast<uint8_t>(value >> 8));

      } 

  }
  template<uint32_t delay = DefaultWaitAmount>
  static inline void
  write32(uint8_t baseAddress, uint32_t value) noexcept {
      if ((baseAddress & 0b11) == 0) {
          // it is aligned so we don't need to worry
          setDataLinesDirection<OUTPUT>();
          setAddress(baseAddress);
          setDataLines(static_cast<uint8_t>(value));
          digitalWriteFast(Pin::EBI_WR, LOW);
          delayNanoseconds(delay);
          digitalWriteFast(Pin::EBI_WR, HIGH);
          digitalToggleFast(Pin::EBI_A0); // since it is aligned to 16-bit
                                          // boundaries we can safely bump it
                                          // forward by one
          setDataLines(static_cast<uint8_t>(value >> 8));
          delayNanoseconds(50); // to satisfy minimum writes to the CH351
          digitalWriteFast(Pin::EBI_WR, LOW);
          delayNanoseconds(delay);
          digitalWriteFast(Pin::EBI_WR, HIGH);

          digitalWriteFast(Pin::EBI_A0, LOW);
          digitalWriteFast(Pin::EBI_A1, HIGH);
          setDataLines(static_cast<uint8_t>(value >> 16));
          delayNanoseconds(50); // to satisfy minimum writes to the CH351
          digitalWriteFast(Pin::EBI_WR, LOW);
          delayNanoseconds(delay);
          digitalWriteFast(Pin::EBI_WR, HIGH);

          digitalWriteFast(Pin::EBI_A0, HIGH);
          setDataLines(static_cast<uint8_t>(value >> 24));
          delayNanoseconds(50); // to satisfy minimum writes to the CH351
          digitalWriteFast(Pin::EBI_WR, LOW);
          delayNanoseconds(delay);
          digitalWriteFast(Pin::EBI_WR, HIGH);

      } else {
        write16<delay>(baseAddress, static_cast<uint16_t>(value));
        write16<delay>(baseAddress + 2, static_cast<uint16_t>(value >> 16));
      }
  }
  template<uint32_t delay = DefaultWaitAmount>
  static inline uint8_t
  read8(uint8_t address) noexcept {
    setDataLinesDirection<INPUT>();
    setAddress(address);
    digitalWriteFast(Pin::EBI_RD, LOW);
    delayNanoseconds(delay);
    uint8_t result = readDataLines();
    digitalWriteFast(Pin::EBI_RD, HIGH);
    return result;
  }
  static inline uint16_t
  read16(uint8_t baseAddress) noexcept {
    union {
      uint8_t bytes[2];
      uint16_t result;
    } storage;
    storage.bytes[0] = read8(baseAddress);
    storage.bytes[1] = read8(baseAddress + 1);
    return storage.result;
  }
  static inline uint32_t
  read32(uint8_t baseAddress) noexcept {
      union {
          uint8_t bytes[4];
          uint32_t result;
      } storage;
      storage.bytes[0] = read8(baseAddress);
      storage.bytes[1] = read8(baseAddress + 1);
      storage.bytes[2] = read8(baseAddress + 2);
      storage.bytes[3] = read8(baseAddress + 3);
      return storage.result;
  }
  

private:
  static inline PinDirection _currentDirection = OUTPUT;
  static inline uint8_t _currentOutputDataLines = 0;
};
constexpr CH351 addressLines{ 0 }, dataLines{ 0b0000'1000 };
using EBIInterface = EBIWrapperInterface;

struct i960Interface {
  i960Interface() = delete;
  ~i960Interface() = delete;
  i960Interface(const i960Interface&) = delete;
  i960Interface(i960Interface&&) = delete;
  i960Interface& operator=(const i960Interface&) = delete;
  i960Interface& operator=(i960Interface&&) = delete;
  static void
  begin() noexcept {
    EBIInterface::write32(addressLines.getConfigPortBaseAddress(), 0);
    configureDataLinesForRead<false>();
    EBIInterface::setDataLines(0);
  }
  template<uint16_t pattern, bool compareWithPrevious = true>
  static inline void
  configureDataLinesDirection() noexcept {
      if constexpr (compareWithPrevious) {
          if (_dataLinesDirection == pattern) {
              return;
          }
      }
      EBIInterface::write16<pattern>(dataLines.getConfigPortBaseAddress());
      _dataLinesDirection = pattern;
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
  static constexpr uint32_t DefaultReadyTriggerWaitAmount = DefaultWaitAmount;
  template<uint32_t delayAmount = DefaultReadyTriggerWaitAmount>
  inline static void
  finishReadyTrigger() noexcept {
      readyTriggered = false;
      digitalWriteFast(Pin::READY, HIGH);
      delayNanoseconds(delayAmount);  // wait some amount of time
  }
  template<uint32_t delayAmount = DefaultReadyTriggerWaitAmount>
  static inline void
  signalReady() noexcept {
      // run and block until we get the completion pulse
      triggerReady();
      waitForReadyTrigger();
      finishReadyTrigger<delayAmount>();
  }
  static inline bool
  isReadOperation() noexcept {
    return digitalReadFast(Pin::WR) == LOW;
  }
  static inline bool
  isWriteOperation() noexcept {
    return digitalReadFast(Pin::WR) == HIGH;
  }
  
  template<uint32_t delayAmount = DefaultWaitAmount>
  static inline uint32_t
  getAddress() noexcept {
      // the CH351 has some very strict requirements
      EBIInterface::setDataLinesDirection<INPUT>();
      EBIInterface::setAddress<addressLines.getDataPortBaseAddress()>();

      digitalWriteFast(Pin::EBI_RD, LOW);
      delayNanoseconds(100); // wait for things to get selected properly
      uint32_t a = EBIInterface::readDataLines<true>(); // A0 is always 0
      digitalWriteFast(Pin::EBI_RD, HIGH);

      digitalWriteFast(Pin::EBI_A0, HIGH);
      delayNanoseconds(50); 
                            
      digitalWriteFast(Pin::EBI_RD, LOW);
      delayNanoseconds(100);
      uint32_t b = EBIInterface::readDataLines();
      digitalWriteFast(Pin::EBI_RD, HIGH);

      digitalWriteFast(Pin::EBI_A1, HIGH);
      digitalWriteFast(Pin::EBI_A0, LOW);
      delayNanoseconds(50);


      digitalWriteFast(Pin::EBI_RD, LOW);
      delayNanoseconds(100);
      uint32_t c = EBIInterface::readDataLines();
      digitalWriteFast(Pin::EBI_RD, HIGH);

      digitalWriteFast(Pin::EBI_A0, HIGH);
      delayNanoseconds(50);
      digitalWriteFast(Pin::EBI_RD, LOW);
      delayNanoseconds(100);
      uint32_t d = EBIInterface::readDataLines();
      digitalWriteFast(Pin::EBI_RD, HIGH);
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
  template<bool skipSetup = false, uint32_t delayAmount = DefaultWaitAmount >
  static inline void
  writeDataLines(uint16_t value) noexcept {
      if constexpr (skipSetup) {
          EBIInterface::setDataLinesDirection<OUTPUT>();
          EBIInterface::setAddress<dataLines.getDataPortBaseAddress()>();
          EBIInterface::setDataLines(static_cast<uint8_t>(value));
      }
      digitalWriteFast(Pin::EBI_WR, LOW);
      delayNanoseconds(delayAmount);
      digitalWriteFast(Pin::EBI_WR, HIGH);
      digitalWriteFast(Pin::EBI_A0, HIGH);
      EBIInterface::setDataLines(static_cast<uint8_t>(value >> 8));
      delayNanoseconds(50); // to satisfy minimum writes to the CH351
      digitalWriteFast(Pin::EBI_WR, LOW);
      delayNanoseconds(delayAmount);
      digitalWriteFast(Pin::EBI_WR, HIGH);
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
  static inline void
  doMemoryCellReadTransaction(const MemoryCell& target, uint8_t offset) noexcept {
      // start the word ahead of time
      auto currentWord = target.getWord(offset >> 1);
      EBIInterface::setDataLinesDirection<OUTPUT>();
      EBIInterface::setAddress<dataLines.getDataPortBaseAddress()>();
      EBIInterface::setDataLines(static_cast<uint8_t>(currentWord));
      for (uint8_t wordOffset = offset >> 1; wordOffset < 8; ++wordOffset) {
          writeDataLines<true>(currentWord);
          if (isBurstLast()) {
              triggerReady();
              // reset for the start of the next transaction
              EBIInterface::setDataLinesDirection<INPUT>();
              waitForReadyTrigger();
              finishReadyTrigger();
              break;
          } else {
              triggerReady();
              // use the delay wait time to actually grab the next value
              currentWord = target.getWord(wordOffset + 1);
              // reset the address lines to the base address
              EBIInterface::setAddress<dataLines.getDataPortBaseAddress()>();
              // prepare for the next cycle
              EBIInterface::setDataLines(static_cast<uint8_t>(currentWord));
              waitForReadyTrigger();
              finishReadyTrigger();
          }
      }
  }
  template<bool skipSetup = false, uint32_t delayAmount = DefaultWaitAmount>
  static inline uint16_t
  readDataLines() noexcept {
      if constexpr (!skipSetup) {
          EBIInterface::setDataLinesDirection<INPUT>();
          EBIInterface::setAddress<dataLines.getDataPortBaseAddress()>();
      }
      digitalWriteFast(Pin::EBI_RD, LOW);
      delayNanoseconds(delayAmount);
      uint16_t lo = EBIInterface::readDataLines();
      digitalWriteFast(Pin::EBI_RD, HIGH);
      digitalWriteFast(Pin::EBI_A0, HIGH);
      digitalWriteFast(Pin::EBI_RD, LOW);
      delayNanoseconds(delayAmount);
      uint16_t hi = EBIInterface::readDataLines();
      digitalWriteFast(Pin::EBI_RD, HIGH);
      return lo | (hi << 8);
  }
  static inline void
  doMemoryCellWriteTransaction(MemoryCell& target, uint8_t offset) noexcept {
      EBIInterface::setDataLinesDirection<INPUT>();
      EBIInterface::setAddress<dataLines.getDataPortBaseAddress()>();
      for (uint8_t wordOffset = offset >> 1; wordOffset < 8; ++wordOffset ) {
          target.setWord(wordOffset, readDataLines<true>(), byteEnableLow(), byteEnableHigh());
          if (isBurstLast()) {
              triggerReady();
              // reset for the start of the next transaction
              EBIInterface::setDataLinesDirection<INPUT>();
              waitForReadyTrigger();
              finishReadyTrigger();
              break;
          } else {
              triggerReady();
              EBIInterface::setAddress<dataLines.getDataPortBaseAddress()>();
              waitForReadyTrigger();
              finishReadyTrigger();
          }
      }
  }
  template<bool isReadTransaction>
  static inline void
  doMemoryCellTransaction(MemoryCell& target, uint8_t offset) noexcept {
      target.update();
      if constexpr (isReadTransaction) {
          doMemoryCellReadTransaction(target, offset);
      } else {
          doMemoryCellWriteTransaction(target, offset);
      }
  }
  template<bool isReadTransaction>
  static inline void 
  transmitConstantMemoryCell(const MemoryCell& cell, uint8_t offset) noexcept {
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
          // case 0x20 ... 0x2f: // more timing related sources
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
          case 0x00'0800 ... 0x00'0FFF: 
              doMemoryCellTransaction<isReadTransaction>(sramCache[(address >> 4) & 0x7F], address & 0xF);
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
private:
  static inline uint16_t _dataLinesDirection = 0xFFFF;
  // allocate a 2k memory cache like the avr did
  static inline MemoryCellBlock sramCache[OnboardSRAMCacheSize / sizeof(MemoryCellBlock)];
  static inline MemoryCellBlock CLKValues{12 * 1000 * 1000, 6 * 1000 * 1000 };
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
  randomSeed(newSeed);
  Serial.print("Random Seed: ");
  Serial.println(newSeed);
}
void setupMemory() noexcept {
  Serial.println("Clearing PSRAM");
  for (auto& a : memory960) {
      a.clear();
  }
}
void
triggerADS() noexcept {
  adsTriggered = true;
}
void
triggerReadySync() noexcept {
    readyTriggered = true;
    //digitalWriteFast(Pin::READY, HIGH);
}
volatile uint32_t failCount = 0;
void
triggerFAIL() noexcept {
    ++failCount;
}
void 
setup() {
    inputPin(Pin::ADS);
    outputPin(Pin::RESET, LOW);
    inputPin(Pin::DEN);
    outputPin(Pin::HOLD, LOW);
    inputPin(Pin::HLDA);
    Serial.begin(9600);
    while (!Serial) {
        delay(10);
    }
    Entropy.Initialize();
    setupRandomSeed();
    // put your setup code here, to run once:
    EBIInterface::begin();
    i960Interface::begin();
    setupMemory();
    Wire.begin();
    setupSDCard();
    outputPin(Pin::INT960_0, HIGH);
    outputPin(Pin::INT960_1, LOW);
    outputPin(Pin::INT960_2, LOW);
    outputPin(Pin::INT960_3, HIGH);
    inputPin(Pin::LOCK);
    inputPin(Pin::FAIL);
    inputPin(Pin::BE0);
    inputPin(Pin::BE1);
    inputPin(Pin::WR);
    outputPin(Pin::READY, HIGH);
    inputPin(Pin::BLAST);
    inputPin(Pin::READY_SYNC);
    Entropy.Initialize();
    attachInterrupt( Pin::ADS, triggerADS, RISING);
    attachInterrupt( Pin::READY_SYNC, triggerReadySync, RISING);
    attachInterrupt(Pin::FAIL, triggerFAIL, FALLING);
    delay(1000);  // make sure that the i960 has enough time to setup
    digitalWriteFast(Pin::RESET, HIGH);
    // so attaching the interrupt seems to not be functioning fully
    delayNanoseconds(100);
    Serial.println("Booted i960");
    // okay so we want to handle the initial boot process
}
void 
loop() {
    if (adsTriggered) {
        readyTriggered = false;
        adsTriggered = false;
        while (digitalReadFast(Pin::DEN) == HIGH) ;
        uint32_t targetAddress = i960Interface::getAddress<100>();
        //Serial.printf("Target Address: 0x%x\n", targetAddress);
        if (i960Interface::isReadOperation()) {
            i960Interface::doMemoryTransaction<true>(targetAddress);
        } else {
            i960Interface::doMemoryTransaction<false>(targetAddress);
        }
    } 
}

