#include <Arduino.h>
#include <type_traits>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <Ethernet.h>

constexpr auto MemoryPoolSizeInBytes = (16 * 1024 * 1024);  // 16 megabyte psram pool
union MemoryCell {
  uint8_t bytes[16];
  uint16_t shorts[8];
  uint32_t words[4];
  uint64_t longs[2];
  void clear() noexcept {
    for (auto& a : words) {
      a = 0;
    }
  }
};
static_assert(sizeof(MemoryCell) == 16, "MemoryCell needs to be 16 bytes in size");
EXTMEM MemoryCell memory960[MemoryPoolSizeInBytes / sizeof(MemoryCell)];
enum class Pin : uint8_t {
  RPI_D0 = 0,
  RPI_D1,
  RPI_D2,
  RPI_D3,
  RPI_D4,
  RPI_D5,
  RPI_D6,
  RPI_D7,
  RPI_D8,
  RPI_D9,
  RPI_D10,
  RPI_D11,
  RPI_D12,
  RPI_D13,
  RPI_D14,
  RPI_D15,
  RPI_D16,
  RPI_D17,
  RPI_D18,
  RPI_D19,
  RPI_D20,
  RPI_D21,
  RPI_D22,
  RPI_D23,
  RPI_D24 = 36,
  RPI_D25 = 37,
  RPI_D26 = 40,
  RPI_D27 = 41,
  EBI_A5 = RPI_D0,
  EBI_A4 = RPI_D1,
  EBI_A3 = RPI_D2,
  EBI_A2 = RPI_D3,
  EBI_A1 = RPI_D4,
  EBI_A0 = RPI_D5,
  EBI_RD = RPI_D6,
  EBI_WR = RPI_D7,
  EBI_D0 = RPI_D8,
  EBI_D1 = RPI_D9,
  EBI_D2 = RPI_D10,
  EBI_D3 = RPI_D11,
  EBI_D4 = RPI_D12,
  EBI_D5 = RPI_D13,
  EBI_D6 = RPI_D14,
  EBI_D7 = RPI_D15,
  BLAST = RPI_D17,
  ADS = RPI_D16,
  WR = RPI_D18,
  DEN = RPI_D19,
  BE0 = RPI_D20,
  BE1 = RPI_D21,
  HOLD = RPI_D22,
  HLDA = RPI_D23,
  RESET = RPI_D24,
  LOCK = RPI_D25,
  READY = RPI_D26,
  READY_SYNC = RPI_D27,
  FAIL = 31,
  INT960_0 = 32,
  INT960_1 = 35,
  INT960_2 = 34,
  INT960_3 = 33,  
  

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



struct EBIInterface {
public:
  EBIInterface() = delete;
  ~EBIInterface() = delete;
  EBIInterface(const EBIInterface&) = delete;
  EBIInterface(EBIInterface&&) = delete;
  EBIInterface& operator=(const EBIInterface&) = delete;
  EBIInterface& operator=(EBIInterface&&) = delete;
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
    setAddress(0);
    digitalWriteFast(Pin::EBI_RD, HIGH);
    digitalWriteFast(Pin::EBI_WR, HIGH);
    setDataLines(0);
  }
  static void
  setAddress(uint8_t address) noexcept {
    if ((address & 0b0011'1111) != _currentAddress) {
      digitalWriteFast(Pin::EBI_A0, (address & 0b000001) != 0 ? HIGH : LOW);
      digitalWriteFast(Pin::EBI_A1, (address & 0b000010) != 0 ? HIGH : LOW);
      digitalWriteFast(Pin::EBI_A2, (address & 0b000100) != 0 ? HIGH : LOW);
      digitalWriteFast(Pin::EBI_A3, (address & 0b001000) != 0 ? HIGH : LOW);
      digitalWriteFast(Pin::EBI_A4, (address & 0b010000) != 0 ? HIGH : LOW);
      digitalWriteFast(Pin::EBI_A5, (address & 0b100000) != 0 ? HIGH : LOW);
      _currentAddress = address & 0b0011'1111;
    }
  }
  static uint8_t
  readDataLines() noexcept {
    uint8_t value = 0;
#define X(p, t) value |= ((digitalReadFast(p) != LOW) ? t : 0)
    X(Pin::EBI_D0, 0b0000'0001);
    X(Pin::EBI_D1, 0b0000'0010);
    X(Pin::EBI_D2, 0b0000'0100);
    X(Pin::EBI_D3, 0b0000'1000);
    X(Pin::EBI_D4, 0b0001'0000);
    X(Pin::EBI_D5, 0b0010'0000);
    X(Pin::EBI_D6, 0b0100'0000);
    X(Pin::EBI_D7, 0b1000'0000);
#undef X
    return value;
  }
  static void
  setDataLines(uint8_t value) noexcept {
    digitalWriteFast(Pin::EBI_D0, (value & 0b00000001) != 0 ? HIGH : LOW);
    digitalWriteFast(Pin::EBI_D1, (value & 0b00000010) != 0 ? HIGH : LOW);
    digitalWriteFast(Pin::EBI_D2, (value & 0b00000100) != 0 ? HIGH : LOW);
    digitalWriteFast(Pin::EBI_D3, (value & 0b00001000) != 0 ? HIGH : LOW);
    digitalWriteFast(Pin::EBI_D4, (value & 0b00010000) != 0 ? HIGH : LOW);
    digitalWriteFast(Pin::EBI_D5, (value & 0b00100000) != 0 ? HIGH : LOW);
    digitalWriteFast(Pin::EBI_D6, (value & 0b01000000) != 0 ? HIGH : LOW);
    digitalWriteFast(Pin::EBI_D7, (value & 0b10000000) != 0 ? HIGH : LOW);
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
  static void
  write8(uint8_t address, uint8_t value) noexcept {
    setDataLinesDirection(OUTPUT);
    setAddress(address);
    setDataLines(value);
    digitalWriteFast(Pin::EBI_WR, LOW);
    delayNanoseconds(150);
    digitalWriteFast(Pin::EBI_WR, HIGH);
  }
  static void
  write16(uint8_t baseAddress, uint16_t value) noexcept {
    write8(baseAddress, static_cast<uint8_t>(value));
    write8(baseAddress + 1, static_cast<uint8_t>(value >> 8));
  }
  static void
  write32(uint8_t baseAddress, uint32_t value) noexcept {
    write16(baseAddress, static_cast<uint16_t>(value));
    write16(baseAddress + 2, static_cast<uint16_t>(value >> 16));
  }
  static uint8_t
  read8(uint8_t address) noexcept {
    setDataLinesDirection(INPUT);
    setAddress(address);
    digitalWriteFast(Pin::EBI_RD, LOW);
    delayNanoseconds(150);
    uint8_t result = readDataLines();
    digitalWriteFast(Pin::EBI_RD, HIGH);
    return result;
  }
  static uint16_t
  read16(uint8_t baseAddress) noexcept {
    union {
      uint8_t bytes[2];
      uint16_t result;
    } storage;
    storage.bytes[0] = read8(baseAddress);
    storage.bytes[1] = read8(baseAddress + 1);
    return storage.result;
  }
  static uint32_t
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
  static inline uint8_t _currentAddress = 0xFF;  // start with an illegal value to make sure
};
constexpr CH351 addressLines{ 0 }, dataLines{ 0b0000'1000 };

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
    configureDataLinesForRead();
    EBIInterface::setDataLines(0);
  }
  static void
  configureDataLinesDirection(uint16_t pattern) noexcept {
    EBIInterface::write16(dataLines.getConfigPortBaseAddress(), pattern);
  }
  static void
  configureDataLinesForRead() noexcept {
    configureDataLinesDirection(0xFFFF);
  }
  static void
  configureDataLinesForWrite() noexcept {
    configureDataLinesDirection(0);
  }
  static void
  signalReady() noexcept {
      Serial.println("signalReady");
      // run and block until we get the completion pulse
      digitalWriteFast(Pin::READY, LOW);

      while (!readyTriggered) {
          // wait
      }
      readyTriggered = false;
      digitalWriteFast(Pin::READY, HIGH);
      delayNanoseconds(250);  // wait 200 ns to make sure that all other signals have "caught up"
  }
  static bool
  isReadOperation() noexcept {
    return digitalReadFast(Pin::WR) == LOW;
  }
  static bool
  isWriteOperation() noexcept {
    return digitalReadFast(Pin::WR) == HIGH;
  }
  static void
  writeDataLines(uint16_t value) noexcept {
    EBIInterface::write16(dataLines.getDataPortBaseAddress(), value);
  }
  static void
  writeDataLines(uint8_t lo, uint8_t hi) noexcept {
    EBIInterface::write8(dataLines.getDataPortBaseAddress(), lo);
    EBIInterface::write8(dataLines.getDataPortBaseAddress() + 1, hi);
  }
  static uint16_t
  readDataLines() noexcept {
    return EBIInterface::read16(dataLines.getDataPortBaseAddress());
  }
  
  static uint32_t
  getAddress() noexcept {
    return EBIInterface::read32(addressLines.getDataPortBaseAddress());
  }
  static bool
  isBurstLast() noexcept {
    return digitalReadFast(Pin::BLAST) == LOW;
  }
  static bool
  byteEnableLow() noexcept {
    return digitalReadFast(Pin::BE0) == LOW;
  }
  static bool
  byteEnableHigh() noexcept {
    return digitalReadFast(Pin::BE1) == LOW;
  }
  template<bool isReadTransaction>
  static void
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

  template<bool isReadTransaction>
  static void
  doPSRAMTransaction(MemoryCell& target, uint8_t offset) noexcept {
    if (isReadTransaction) {
        Serial.print("Base Address: 0x");
        Serial.println(reinterpret_cast<size_t>(&target), HEX);
        Serial.print("Offset: 0x");
        Serial.println(offset, HEX);
      for (uint8_t wordOffset = offset >> 1; wordOffset < 8; ++wordOffset) {
        Serial.println(wordOffset);
        if (isBurstLast()) {
            writeDataLines(target.shorts[wordOffset]);
            signalReady();
            break;
        } else {
            writeDataLines(target.shorts[wordOffset]);
            signalReady();
        }
      }

    } else {
      for (uint8_t wordOffset = offset; wordOffset < 16; wordOffset += 2) {
        uint16_t data = readDataLines();
        if (byteEnableLow()) {
          target.bytes[wordOffset] = static_cast<uint8_t>(data);
        }
        if (byteEnableHigh()) {
          target.bytes[wordOffset + 1] = static_cast<uint8_t>(data >> 8);
        }
        if (isBurstLast()) {
            break;
        } else {
            signalReady();
        }
      }
      signalReady();
    }
  }


  template<bool isReadTransaction>
  static void
  doMemoryTransaction(uint32_t address) noexcept {
      if constexpr (isReadTransaction) {
          Serial.println("Configuring Data Lines For Read");
          configureDataLinesForRead();
      } else {
          Serial.println("Configuring Data Lines For Write");
          configureDataLinesForWrite();
      }
      switch (address) {
          case 0x0000'0000 ... 0x00FF'FFFF:  // PSRAM
              Serial.println("PSRAM Transaction");
              doPSRAMTransaction<isReadTransaction>(memory960[(address >> 4) & 0x000F'FFFF], address & 0xF);
              break;
          default:
              Serial.println("Do Nothing Transaction");
              doNothingTransaction<isReadTransaction>();
              break;
      }
  }
  template<bool waitForADS>
  static void
  singleTransaction() noexcept {
    if constexpr (waitForADS) {
        while (!adsTriggered);
    }
    {
        readyTriggered = false;
        adsTriggered = false;
    }
    uint32_t targetAddress = getAddress();
    Serial.print("Target Address: 0x");
    Serial.println(targetAddress, HEX);
    while (digitalReadFast(Pin::DEN) == HIGH) ;
    if (isReadOperation()) {
        doMemoryTransaction<true>(targetAddress);
    } else {
        doMemoryTransaction<false>(targetAddress);
    }
  }
};






void setupRandomSeed() noexcept {
  uint32_t newSeed = 0;
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
}
void setupMemory() noexcept {
  memset(memory960, 0, sizeof(memory960));
}
void
triggerADS() noexcept {
  adsTriggered = true;
}
void
triggerReadySync() noexcept {
    readyTriggered = true;
}
void setup() {
  Serial.begin(9600);
  while (!Serial) {
      delay(10);
  }
  //delay(500);
  setupRandomSeed();
  // put your setup code here, to run once:
  EBIInterface::begin();
  i960Interface::begin();
  setupMemory();
  pinMode(Pin::ADS, INPUT_PULLUP);
  outputPin(Pin::RESET, LOW);
  inputPin(Pin::DEN);
  outputPin(Pin::HOLD, LOW);
  inputPin(Pin::HLDA);
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

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("No SDCARD found!");
  } else {
    Serial.println("SDCARD Found");
  }
  attachInterrupt( Pin::ADS, triggerADS, RISING);
  attachInterrupt( Pin::READY_SYNC, triggerReadySync, RISING);
  delay(1000);  // make sure that the i960 has enough time to setup
  digitalWriteFast(Pin::RESET, HIGH);
  // so attaching the interrupt seems to not be functioning fully
  delayNanoseconds(100);
}
bool waiting = false;
void loop() {
  if (adsTriggered) {
    i960Interface::singleTransaction<false>();
    waiting = false;
  } else {
    if (!waiting) {
      Serial.println("Waiting for transaction...");
    }
    waiting = true;
  }
  
}

