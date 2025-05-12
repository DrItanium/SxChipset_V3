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
// core arduino stuff
#include <Arduino.h>
// std lib :)
#include <tuple>
#include <optional>
#include <type_traits>
// builtin device libraries 
#include <Adafruit_NeoPixel.h> 
#include <Adafruit_ZeroTimer.h>
#include <SdFat_Adafruit_Fork.h>
#include <Adafruit_SPIFlash.h>
#include <SPI.h>
#include <Wire.h>
// over i2c
#include <RTClib.h>
// local headers
#include "Pinout.h"

Adafruit_FlashTransport_QSPI flashTransport; // for the 8MB of onboard FLASH
Adafruit_SPIFlash onboardFlash(&flashTransport);
SdFat onboardSdCard;
Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL);
volatile bool hasRTC = false;
volatile bool hasSDCard = false;
RTC_DS3231 rtc;
// Memory interface operations
// The way that the SAMD51P20A on the GCM4 is laid out means that there is
// actually no contiguous 16-bit block of bits starting from position 0 and
// walking upward. Working around this means that we actually use PC00-PC07 for
// the lower half of the data lines and PC10-PC17 for the upper half of the
// data lines. What it requires is that we shift the upper half right two places
// and combine when handling a write operation. On a read operation, we take the upper
// half of the data value and shift it left by 2 places. This makes it possible
// to fairly quickly update the data ports. 
// 
//
union DataLines {
    uint32_t receive;
    uint16_t send;
    struct {
        uint32_t lo : 8;
        uint32_t free2 : 2;
        uint32_t hi : 8;
    } from960;
    struct {
        uint32_t lo : 8;
        uint32_t hi : 8;
    } to960;
};
constexpr uint32_t DataMask = 0x0003FCFF;
[[gnu::used]]
[[gnu::always_inline]]
inline void 
setDataLines(uint16_t value) noexcept {
    // properly updating the output port without causing problems requires two
    // port writes
    //
    // First, we need to set everything to low to create a normal state
    // Second, we then need to update only the bits that make up the 16-bit
    // data bus lines via OUTSET. So the value specified (and then transformed
    // internally) describes which bits should be HIGH with everything else
    // being kept low
    PORT->Group[PORTC].OUTCLR.reg = DataMask;
    DataLines d;
    d.send = value; // load the 16-bit value directly
    d.from960.hi = d.to960.hi; // then have the compiler shift the upper half
                               // left by two places
    // However, this creates a potentially damaging state since the two bits
    // (8, 9) will be preserved. At the end we clear those bits via an and
    // operation.
    PORT->Group[PORTC].OUTSET.reg = DataMask & d.receive;
}
[[gnu::used]]
[[gnu::always_inline]]
inline uint16_t 
getDataLines() noexcept {
    DataLines d;
    d.receive = PORT->Group[PORTC].IN.reg; // Read the input port fully
    d.to960.hi = d.from960.hi; // make the compiler actually fix the off by two
                               // piece. In the process of doing this, the
                               // lower 16-bits are now correct.
    return d.send; // just return the lower half of the 32-bit allocation
}

volatile bool systemBooted = false;
volatile bool addressTransactionFound = false;
volatile bool readySignalFeedbackAccepted = false;
// this is an interrupt which is fired when ADS goes from low to high
// The ADS pin is meant to denote when a new transaction starts. It acts as a
// synchronization point and is also stable throughout the lifetime of the
// processor. I used to use the DEN pin exclusively but the problem with using
// it is that it can wildly cavitate as well. Same thing with the BLAST pin as
// well!
//
// 
void
EIC_4_Handler() {
    // make the handler as simple as possible
    addressTransactionFound = true;
}
void
EIC_5_Handler() {
    // make the handler as simple as possible
    readySignalFeedbackAccepted = true;
}

bool
signalReady() noexcept {
    // check the current status of the BLAST pin before we do anything else!
    // If it is currently enabled then we will terminate after being finished
    bool isBurstLast = digitalRead<Pin::BLAST>() == LOW;
    {
        // this scope is meant to ensure that the check for BLAST is done ahead
        // of entering this scope
        readySignalFeedbackAccepted = false;
        // we signal ready and then wait
        digitalWrite<Pin::READY, LOW>();
        // wait until we get the feedback of the ready signal being done
        while (!readySignalFeedbackAccepted);
        readySignalFeedbackAccepted = false;
        digitalWrite<Pin::READY, HIGH>(); // now move the READY signal back up
    }
    return isBurstLast;
}
uint32_t
readAddress() noexcept {
    union {
        uint32_t result;
        uint8_t bytes[4];
    } q;
    SPI.beginTransaction(SPISettings{5'000'000, MSBFIRST, SPI_MODE0});
    // access data elements through SPI for now using four device in series
    digitalWrite<Pin::AddressCapture_SPI_EN, LOW>();
    q.bytes[3] = SPI.transfer(0);
    q.bytes[2] = SPI.transfer(0);
    q.bytes[1] = SPI.transfer(0);
    q.bytes[0] = SPI.transfer(0);
    digitalWrite<Pin::AddressCapture_SPI_EN, HIGH>();
    SPI.endTransaction();
    return q.result;
}
union DataCell {
    uint8_t bytes[16];
    uint16_t halves[8];
    uint32_t words[4];
    uint64_t longWords[2];
    void clear() noexcept {
        longWords[0] = 0;
        longWords[1] = 0;
    }
};
DataCell localData[0x10000 / sizeof(DataCell)];
static_assert(sizeof(DataCell) == 16, "DataCell needs to be 16-bytes in size");
static_assert(sizeof(localData) == 0x10000, "localData not the correct size");
template<bool isReadOperation>
void
handleDataCellTransaction(uint8_t offset, DataCell& cell) noexcept {
    if constexpr (isReadOperation) {
        for (uint8_t start = offset >> 1; start < 8; ++start) {
            setDataLines(cell.halves[start]);
            if (signalReady()) {
                break;
            }
        }
    } else {
        for (uint8_t start = offset; start < 16; start +=2) {
            uint16_t value = getDataLines();
            if (digitalRead<Pin::BE0>() == LOW) {
                cell.bytes[start] = static_cast<uint8_t>(value);
            }
            if (digitalRead<Pin::BE1>() == LOW) {
                cell.bytes[start+1] = static_cast<uint8_t>(value >> 8);
            }
            if (signalReady()) {
                break;
            }
        }
    }
}
template<bool isReadOperation>
void
localDataOperation(uint32_t address) noexcept {
    uint16_t targetCellIndex = static_cast<uint16_t>(address) & 0xFFF0;
    // least significant bit is always zero
    uint8_t offset = static_cast<uint8_t>(address & 0x000F);
    auto& targetCell = localData[targetCellIndex];
    handleDataCellTransaction<isReadOperation>(offset, targetCell);

}
template<bool isReadOperation>
void
doNothingOperation() noexcept {
    if constexpr (isReadOperation) {
        setDataLines(0);
    }
    while (!signalReady());
}
template<bool isReadOperation> void handleTransaction(uint32_t address) noexcept;

void 
executionLoop() noexcept {
    // just sit and spin in a loop
    while (!addressTransactionFound) {
        yield(); // yield time to other things
        // do nothing
    }
    addressTransactionFound = false;
    if (auto address = readAddress(); digitalRead<Pin::WR>() == LOW) {
        handleTransaction<true>(address);
    } else {
        handleTransaction<false>(address);
    }
}
// tracking information
// SERCOM0 -> Serial1 (D0/D1 pair) [optional]
// --- PAD[0]: PB24 (D1)
// --- PAD[1]: PB25 (D0)
// --- PAD[2]: Unused
// --- PAD[3]: Unused
// SERCOM1 -> Serial3 (D16/D17 pair) [optional]
// --- PAD[0]: PC22 (D16)
// --- PAD[1]: PC23 (D17)
// --- PAD[2]: Unused
// --- PAD[3]: Unused
// SERCOM2 -> SD (Reserved) [required]
// --- PAD[0]: PB26 (MOSI)
// --- PAD[1]: PB27 (SCK)
// --- PAD[2]: PB28 (CS)
// --- PAD[3]: PB29 (MISO)
// SERCOM3 -> Wire (D20/D21 pair) [required]
// --- PAD[0]: PB20 (SDA)
// --- PAD[1]: PB21 (SCL)
// --- PAD[2]: Unused
// --- PAD[3]: Unused
// SERCOM4 -> Serial2 (D18/D19 pair) [optional]
// --- PAD[0]: PB12 (D18)
// --- PAD[1]: PB13 (D19)
// --- PAD[2]: Unused
// --- PAD[3]: Unused
// SERCOM5 -> Serial4 (D14/D15 pair) [optional]
// --- PAD[0]: PB16 (D14)
// --- PAD[1]: PB17 (D15)
// --- PAD[2]: Unused
// --- PAD[3]: Unused
// SERCOM6 -> Free for other use since Wire1 is being used for data lines
// -- Pads free for use --
// --- PAD[0]: Unavailable
// --- PAD[1]: Unavailable
// --- PAD[2]: PC18 (D2)
// --- PAD[3]: PC19 (D3)
// SERCOM7 -> SPI (D50,D51,D52 triple + ISP)
// --- PAD[0]: PD09 (SCK)
// --- PAD[1]: PD08 (MOSI)
// --- PAD[2]: PD10 (~CS)
// --- PAD[3]: PD11 (MISO)
// system init functions
void
setupRTC() noexcept {
    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC");
    } else {
        Serial.println("Found RTC!");
        hasRTC = true;
        if (rtc.lostPower()) {
            Serial.println("RTC lost power, setting time!");
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        }
    }
}
void
setNeopixelColor(uint32_t value) noexcept {
    pixel.clear();
    pixel.setPixelColor(0, value);
    pixel.show();
}
void
setNeopixelColor(uint8_t r, uint8_t g, uint8_t b) noexcept {
    setNeopixelColor(Adafruit_NeoPixel::Color(r, g, b));
}
void
setupNeopixel() noexcept {
    // enable the onboard neopixel
    pixel.begin();
    pixel.setBrightness(1);
}
[[nodiscard]]
bool
sdcardInstalled() noexcept {
    return digitalRead<Pin::SD_Detect>() == HIGH;
}
Adafruit_ZeroTimer neopixelTimer(3);
void
TC3_Handler() {
    Adafruit_ZeroTimer::timerHandler(3);
}

void
doNeopixel() noexcept {
    if (!systemBooted) 
        return;
    setNeopixelColor(random(), random(), random());
}
std::tuple<uint16_t, uint16_t, tc_clock_prescaler>
constexpr computeFrequency(float freq) noexcept {
    uint16_t divider = 1;
    uint16_t compare = 0;
    tc_clock_prescaler prescaler = TC_CLOCK_PRESCALER_DIV1;
    if ((freq < 24000000) && (freq > 800)) {
        divider = 1;
        prescaler = TC_CLOCK_PRESCALER_DIV1;
        compare = 48000000/freq;
    } else if (freq > 400) {
        divider = 2;
        prescaler = TC_CLOCK_PRESCALER_DIV2;
        compare = (48000000/2)/freq;
    } else if (freq > 200) {
        divider = 4;
        prescaler = TC_CLOCK_PRESCALER_DIV4;
        compare = (48000000/4)/freq;
    } else if (freq > 100) {
        divider = 8;
        prescaler = TC_CLOCK_PRESCALER_DIV8;
        compare = (48000000/8)/freq;
    } else if (freq > 50) {
        divider = 16;
        prescaler = TC_CLOCK_PRESCALER_DIV16;
        compare = (48000000/16)/freq;
    } else if (freq > 12) {
        divider = 64;
        prescaler = TC_CLOCK_PRESCALER_DIV64;
        compare = (48000000/64)/freq;
    } else if (freq > 3) {
        divider = 256;
        prescaler = TC_CLOCK_PRESCALER_DIV256;
        compare = (48000000/256)/freq;
    } else if (freq >= 0.75) {
        divider = 1024;
        prescaler = TC_CLOCK_PRESCALER_DIV1024;
        compare = (48000000/1024)/freq;
    } 
    return std::make_tuple(divider, compare, prescaler);
}
void
setupTimers() noexcept {
    auto [div, cmp, scale] = computeFrequency(2.0f); // two times per second
    neopixelTimer.enable(false);
    neopixelTimer.configure(scale, 
            TC_COUNTER_SIZE_16BIT,
            TC_WAVE_GENERATION_MATCH_PWM
            );
    neopixelTimer.setCompare(0, cmp);
    neopixelTimer.setCallback(true, TC_CALLBACK_CC_CHANNEL0, doNeopixel);
    neopixelTimer.enable(true);
}
void
configureOnboardFlash() noexcept {
    Serial.print("Starting up onboard QSPI Flash...");
    onboardFlash.begin();
    Serial.println("Done");
    Serial.println("Onboard Flash information");
    Serial.print("JEDEC ID: 0x");
    Serial.println(onboardFlash.getJEDECID(), HEX);
    Serial.print("Flash size: ");
    Serial.print(onboardFlash.size() / 1024);
    Serial.println(" KB");
}
void
configureSDCard() noexcept {
    Serial.print("Starting up SD Card...");
    if (!onboardSdCard.begin(SDCARD_SS_PIN)) {
        Serial.println("No card found (is one inserted?)");
    } else {
        Serial.println("Card found!");
        hasSDCard = true;
    }
}
void
configureAddressCaptureComponent() noexcept {
    outputPin<Pin::AddressCapture_SPI_EN, HIGH>();
}
void 
configurePSRAMEnables() noexcept {
    outputPin<Pin::PSRAM_EN0, HIGH>();
    outputPin<Pin::PSRAM_EN1, HIGH>();
    outputPin<Pin::PSRAM_EN2, HIGH>();
    outputPin<Pin::PSRAM_EN3, HIGH>();
}
void
configurePins() noexcept {
    pinMode<Pin::SD_Detect, INPUT>();
    outputPin<Pin::READY, HIGH>();
    // trigger on the rising edge (EIC_4_Handler)
    pinMode<Pin::ADS, INPUT>();
    // trigger on the rising edge (EIC_5_Handler)
    pinMode<Pin::READY_SYNC, INPUT>();
    pinMode<Pin::BE0, INPUT>();
    pinMode<Pin::BE1, INPUT>();
    pinMode<Pin::WR, INPUT>();
    outputPin<Pin::Data0, LOW>();
    outputPin<Pin::Data1, LOW>();
    outputPin<Pin::Data2, LOW>();
    outputPin<Pin::Data3, LOW>();
    outputPin<Pin::Data4, LOW>();
    outputPin<Pin::Data5, LOW>();
    outputPin<Pin::Data6, LOW>();
    outputPin<Pin::Data7, LOW>();
    outputPin<Pin::Data8, LOW>();
    outputPin<Pin::Data9, LOW>();
    outputPin<Pin::Data10, LOW>();
    outputPin<Pin::Data11, LOW>();
    outputPin<Pin::Data12, LOW>();
    outputPin<Pin::Data13, LOW>();
    outputPin<Pin::Data14, LOW>();
    outputPin<Pin::Data15, LOW>();
    configureAddressCaptureComponent();
    configurePSRAMEnables();
}
void
setupSerialConsole() noexcept {
    Serial.begin(9600);
    // now we wait until we get an actual serial link up
    uint8_t index = 0;
    while (!Serial) {
        // just configure the neopixel to inform me that we are waiting for a
        // serial connection
        //
        // this pattern will actually get really bright and then falloff and
        // repeat
        pixel.setBrightness(map(index < 128 ? index : static_cast<uint8_t>(~index), 0, 127, 0, 64));
        setNeopixelColor(255, 0, 255);
        ++index;
        delay(10);
    }
    pixel.setBrightness(10);
    setNeopixelColor(0, 0, 0);
}
void
seedRandom() noexcept {
    long newSeed = 0l;
    for (auto a : {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15}) {
        newSeed += analogRead(a);
    }
    randomSeed(newSeed);
}
void configurePSRAM() noexcept;
void configureCache() noexcept;
void
setup() {
    seedRandom();
    setupNeopixel();
    setupSerialConsole();
    configurePins();
    SPI.begin();
    configureOnboardFlash();
    configureSDCard();
    configurePSRAM();
    configureCache();
    setupRTC();
    setupTimers();
    systemBooted = true;
}
void
loop() {
    executionLoop();
}

// while 33mhz is the max speed, some of the expansion boards are requiring
// that I lower the clock speed when interfacing with the PSRAM
//
// It is that the mega prototype boards and shields are introducing some very
// strange capacitance (?) problems...
const SPISettings defaultPSRAMSettings{10'000'000, MSBFIRST, SPI_MODE0};
template<Pin pin>
void
activatePSRAM() noexcept {
    Serial.print("Activating PSRAM Chip enabled by Pin #");
    Serial.print(static_cast<int>(pin));
    Serial.print("...");
    SPI.beginTransaction(defaultPSRAMSettings);
    digitalWrite<pin, LOW>();
    SPI.transfer(0x66);
    digitalWrite<pin, HIGH>();
    delay(1);
    digitalWrite<pin, LOW>();
    SPI.transfer(0x99);
    digitalWrite<pin, HIGH>();
    SPI.endTransaction();
    Serial.println("Done");
}

[[gnu::always_inline]]
inline void
psramTransferAddress(uint32_t address) noexcept {
    // the MSB doesn't actually matter
    SPI.transfer(static_cast<uint8_t>(address >> 16));
    SPI.transfer(static_cast<uint8_t>(address >> 8));
    SPI.transfer(static_cast<uint8_t>(address));
}
template<Pin pin, bool isReadOperation>
size_t
psramRW(uint32_t address, uint8_t* contents, size_t count) noexcept {
    SPI.beginTransaction(defaultPSRAMSettings);
    digitalWrite<pin, LOW>();
    if constexpr (isReadOperation) {
        SPI.transfer(0x03);
    } else {
        SPI.transfer(0x02);
    }
    psramTransferAddress(address);
    SPI.transfer(contents, count);
    digitalWrite<pin, HIGH>();
    return count;
}
template<Pin pin>
uint32_t
psramRead32(uint32_t address) noexcept {
    uint32_t result = 0;
    (void)psramRW<pin, true>(address, reinterpret_cast<uint8_t*>(&result), sizeof(result));
    return result;
}
template<Pin pin>
void
psramWrite32(uint32_t address, uint32_t value) noexcept {
    (void)psramRW<pin, false>(address, reinterpret_cast<uint8_t*>(&value), sizeof(value));
}
template<Pin pin, uint32_t checkSize = 4096>
bool
psramSanityCheck() noexcept {
    Serial.print("Checking PSRAM Chip enabled by Pin #");
    Serial.print(static_cast<int>(pin));
    Serial.print("...");
    for (uint32_t i = 0; i < checkSize; ++i) {
        uint32_t value = ~i;
        psramWrite32<pin>(i, value);
        auto result = psramRead32<pin>(i);
        if (result != value) {
            Serial.println("FAILED");
            Serial.printf("\t@0x%x: Expected: 0x%x, Got: 0x%x\n", i, value, result);
            return false;
        }
    }
    Serial.println("PASSED");
    return true;
}
void
configurePSRAM() noexcept {
    delay(300);
    // give a system delay to make sure 
    activatePSRAM<Pin::PSRAM_EN0>();
    activatePSRAM<Pin::PSRAM_EN1>();
    activatePSRAM<Pin::PSRAM_EN2>();
    activatePSRAM<Pin::PSRAM_EN3>();
    // okay, now we need to do some sanity checking
    (void)psramSanityCheck<Pin::PSRAM_EN0>();
    (void)psramSanityCheck<Pin::PSRAM_EN1>();
    (void)psramSanityCheck<Pin::PSRAM_EN2>();
    (void)psramSanityCheck<Pin::PSRAM_EN3>();
}
template<bool isReadOperation>
void 
commitData(uint32_t alignedAddress, DataCell& contents) noexcept {
    switch (alignedAddress) {
        case 0x0000'0000 ... 0x007F'FFFF: // psram0
            (void)psramRW<Pin::PSRAM_EN0, isReadOperation>(alignedAddress, contents.bytes, sizeof(DataCell));
            break;
        case 0x0080'0000 ... 0x00FF'FFFF: // psram1
            (void)psramRW<Pin::PSRAM_EN1, isReadOperation>(alignedAddress, contents.bytes, sizeof(DataCell));
            break;
        case 0x0100'0000 ... 0x017F'FFFF: // psram2
            (void)psramRW<Pin::PSRAM_EN2, isReadOperation>(alignedAddress, contents.bytes, sizeof(DataCell));
            break;
        case 0x0180'0000 ... 0x01FF'FFFF: // psram3
            (void)psramRW<Pin::PSRAM_EN3, isReadOperation>(alignedAddress, contents.bytes, sizeof(DataCell));
            break;
        /// @todo add more backing targets if it makes sense
        default: 
            if constexpr (isReadOperation) {
                // clear the cell contents if it is a read operation
                contents.clear();
            }
            break;
    }
}
class CacheLine {
public:
    CacheLine() = default;
    void clear() noexcept {
        _valid = false;
        _dirty = false;
        _targetAddress = 0;
        _contents.clear();
    }
    static constexpr uint32_t alignAddress(uint32_t newAddress) noexcept {
        return newAddress & 0xFFFF'FFF0;
    }
    [[nodiscard]] constexpr bool matches(uint32_t newAddress) const noexcept {
        return _valid && (_targetAddress == alignAddress(newAddress));
    }
    [[nodiscard]] constexpr bool valid() const noexcept { return _valid; }
    [[nodiscard]] constexpr auto getTargetAddress() const noexcept { return _targetAddress; }
    [[nodiscard]] constexpr auto dirty() const noexcept { return _dirty; }
    void markDirty() noexcept { _dirty = true; }
    void synchronize(uint32_t newAddress) noexcept {
        if (!matches(newAddress)) {
            replace(newAddress);
        }
    }
    void replace(uint32_t newAddress) noexcept {
        if (_valid) {
            if (_dirty) {
                commitData<false>(_targetAddress, _contents);
            }
        } 
        _targetAddress = alignAddress(newAddress);
        _valid = true;
        _dirty = false;
        commitData<true>(_targetAddress, _contents);
    }
    DataCell& getContents() noexcept {
        return _contents;
    }
private:
    uint32_t _targetAddress = 0;
    bool _valid = false;
    bool _dirty = false;
    DataCell _contents;
};
// A very simple 64k direct mapped data cache
class Cache {
public:
    CacheLine& find(uint32_t address) noexcept {
        auto& cacheLine = _store[static_cast<uint16_t>(address >> 4) & 0xFFF];
        if (!cacheLine.matches(address)) {
            cacheLine.replace(address);
        }
        return cacheLine;
    }
    void clear() noexcept {
        for(auto& a : _store) {
            a.clear();
        }
    }
private:
    CacheLine _store[4096];
};

Cache dataCache;
void 
configureCache() noexcept {
    dataCache.clear();
}

template<bool isReadOperation>
void cacheOperation(uint32_t address) noexcept {
    auto& line = dataCache.find(address);
    handleDataCellTransaction<isReadOperation>(static_cast<uint8_t>(address & 0xF), line.getContents());
}

template<bool isReadOperation>
void 
handleTransaction(uint32_t address) noexcept {
    if constexpr (isReadOperation) {
        // read operation (output)
        PORT->Group[PORTC].DIRSET.reg = DataMask;
    } else {
        // write operation (input)
        PORT->Group[PORTC].DIRCLR.reg = DataMask;
    }
    switch (address) {
        // lower two gigabytes are connected to the backing store cache
        case 0x0000'0000 ... 0x7FFF'FFFF:
            cacheOperation<isReadOperation>(address);
            break;
        case 0xFE01'0000 ... 0xFE01'FFFF:
            localDataOperation<isReadOperation>(address);
            break;
        default:
            doNothingOperation<isReadOperation>();
            break;
    }
}
