#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <RTClib.h>
#include <type_traits>
#include <Adafruit_ZeroTimer.h>
#include <tuple>
#include <optional>
Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL);
volatile bool hasRTC = false;
volatile bool hasSDCard = false;
RTC_DS3231 rtc;
using PinIndex = decltype(PIN_PA02);
enum class Pin : PinIndex {
    PB25,
    PB24,
    PC18,
    PC19,
    PC20,
    PC21,
    PD20,
    PD21,
    PB18,
    PB02,
    PB22,
    PB23,
    PB00,
    PB01,
    PB16,
    PB17,
    PC22,
    PC23,
    PB12,
    PB13,
    PB20,
    PB21,
    PD12,
    PA15,
    PC17,
    PC16,
    PA12,
    PA13,
    PA14,
    PB19,
    PA23,
    PA22,
    PA21,
    PA20,
    PA19,
    PA18,
    PA17,
    PA16,
    PB15,
    PB14,
    PC13,
    PC12,
    PC15,
    PC14,
    PC11,
    PC10,
    PC06,
    PC07,
    PC04,
    PC05,
    PD11,
    PD08,
    PD09,
    PD10,
    PB05,
    PB06,
    PB07,
    PB08,
    PB09,
    PA04,
    PA06,
    PA07,
    // there is a hole in ids here
    PA02 = 67,
    PA05,
    PB03,
    PC00,
    PC01,
    PC02,
    PC03,
    PB04,
    Neopixel = PIN_NEOPIXEL,
    SD_Detect = 95,
    // custom signals (with comments about the digital index as well, mostly
    // sorted based on the digital connector block but not fully)
    READY = PD12, // 22
    BLAST = PA15, // 23
    ADRMUX_SEL0 = PA12, // 26
    ADRMUX_SEL1 = PA13, // 27
    ADRMUX_EN = PA14, // 28
    WR = PB19, // 29
    ADRMUX0 = PA16, // 37
    ADRMUX1 = PA17, // 36
    ADRMUX2 = PA18, // 35
    ADRMUX3 = PA19, // 34
    ADRMUX4 = PA20, // 33
    ADRMUX5 = PA21, // 32
    ADRMUX6 = PA22, // 31
    ADRMUX7 = PA23, // 30
    BE1 = PB15, // 38
    BE0 = PB14, // 39
    Data0 = PC00, // 70/A3
    Data1 = PC01, // 71/A4
    Data2 = PC02, // 72/A5
    Data3 = PC03, // 73/A6
    Data4 = PC04, // 48
    Data5 = PC05, // 49
    Data6 = PC06, // 46
    Data7 = PC07, // 47
    Data8 = PC10, // 45
    Data9 = PC11, // 44
    Data10 = PC12, // 41
    Data11 = PC13, // 40
    Data12 = PC14, // 43
    Data13 = PC15, // 42
    Data15 = PC17, // 24
    Data14 = PC16, // 25

};


[[gnu::always_inline]]
inline void 
pinMode(Pin targetPin, decltype(OUTPUT) direction) noexcept {
    ::pinMode(static_cast<std::underlying_type_t<Pin>>(targetPin), direction);
}

template<Pin p, decltype(OUTPUT) direction> 
[[gnu::always_inline]]
inline void 
pinMode() noexcept {
    pinMode(p, direction);
}

[[gnu::always_inline]]
inline void
digitalWrite(Pin targetPin, decltype(LOW) value) noexcept {
    ::digitalWrite(static_cast<std::underlying_type_t<Pin>>(targetPin), value);
}
template<Pin p>
[[gnu::always_inline]]
inline void
digitalWrite(decltype(LOW) value) noexcept {
    digitalWrite(p, value);
}
template<Pin p, decltype(LOW) value>
[[gnu::always_inline]]
inline void
digitalWrite() noexcept {
    digitalWrite(p, value);
}

[[gnu::always_inline]]
inline auto
digitalRead(Pin targetPin) noexcept {
    return ::digitalRead(static_cast<std::underlying_type_t<Pin>>(targetPin));
}
template<Pin p>
[[gnu::always_inline]]
inline decltype(auto)
digitalRead() noexcept {
    return digitalRead(p);
}
[[gnu::always_inline]]
inline void
outputPin(Pin targetPin, decltype(LOW) initialValue = HIGH) noexcept {
    pinMode(targetPin, OUTPUT);
    digitalWrite(targetPin, initialValue);
}
template<Pin p, decltype(LOW) initialValue = HIGH>
[[gnu::always_inline]]
inline void 
outputPin() noexcept {
    pinMode<p, OUTPUT>();
    digitalWrite<p, initialValue>();
}
template<Pin p>
decltype(auto) 
getCorrespondingPort() noexcept {
    return digitalPinToPort(static_cast<std::underlying_type_t<Pin>>(p));
}
// Memory interface operations
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
[[gnu::always_inline]]
inline void 
setDataLines(uint16_t value) noexcept {
    DataLines d;
    d.receive = 0;
    d.send = value;
    d.from960.hi = d.to960.hi;
    d.from960.free2 = 0;
    getCorrespondingPort<Pin::Data0>()->OUTCLR.reg = DataMask;
    getCorrespondingPort<Pin::Data0>()->OUTSET.reg = DataMask & d.receive;
}
[[gnu::always_inline]]
inline uint16_t 
getDataLines() noexcept {
    DataLines d;
    d.receive = getCorrespondingPort<Pin::Data0>()->IN.reg;
    d.to960.hi = d.from960.hi;
    return d.send;
}

volatile bool systemBooted = false;
volatile bool addressTransactionFound = false;
void 
leftAddressState() noexcept {
    addressTransactionFound = true;
}

void 
executionLoop() noexcept {
    // just sit and spin in a loop
    while (!addressTransactionFound) {
        yield(); // yield time to other things
        // do nothing
    }
    addressTransactionFound = false;
    if (digitalRead<Pin::WR>() == LOW) {
        // read operation (output)
        getCorrespondingPort<Pin::Data0>()->DIRSET.reg = DataMask;
    } else {
        // write operation (input)
        getCorrespondingPort<Pin::Data0>()->DIRCLR.reg = DataMask;
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
    pixel.clear();
    pixel.setPixelColor(0, pixel.Color(random(), random(), random()));
    pixel.show();
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
setup() {
    Serial.begin(9600);
    pinMode<Pin::SD_Detect, INPUT>();
    SD.begin();
    SPI.begin();
    Wire.begin();
    // @todo figure out why the pinout is locking up the upload port
    outputPin<Pin::READY, HIGH>();
    outputPin<Pin::ADRMUX_SEL0, LOW>();
    outputPin<Pin::ADRMUX_SEL1, LOW>();
    outputPin<Pin::ADRMUX_EN, HIGH>();
    pinMode<Pin::ADRMUX0, INPUT>();
    pinMode<Pin::ADRMUX1, INPUT>();
    pinMode<Pin::ADRMUX2, INPUT>();
    pinMode<Pin::ADRMUX3, INPUT>();
    pinMode<Pin::ADRMUX4, INPUT>();
    pinMode<Pin::ADRMUX5, INPUT>();
    pinMode<Pin::ADRMUX6, INPUT>();
    pinMode<Pin::ADRMUX7, INPUT>();
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
    setupRTC();
    setupNeopixel();
    setupTimers();
    systemBooted = true;
}
void
loop() {
    delay(500);
}
