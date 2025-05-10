#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <RTClib.h>
#include <type_traits>
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

    DAC0 = PA02,
    DAC1 = PA05,
    READY = PD12,
    ADRMUX_DEN1 = PA12,
    ADRMUX_DEN2 = PA13,
    ADRMUX_CLK = PA14,
    // Alternate view for the pinout
    ADRMUX_SEL0 = ADRMUX_DEN1,
    ADRMUX_SEL1 = ADRMUX_DEN2,
    ADRMUX_EN = ADRMUX_CLK,
    BLAST = PA15,
    ADRMUX0 = PA16,
    ADRMUX1 = PA17,
    ADRMUX2 = PA18,
    ADRMUX3 = PA19,
    ADRMUX4 = PA20,
    ADRMUX5 = PA21,
    ADRMUX6 = PA22,
    ADRMUX7 = PA23,
    BE0 = PB14,
    BE1 = PB15,
    WR = PB19,
    Data0 = PC00,
    Data1 = PC01,
    Data2 = PC02,
    Data3 = PC03,
    Data4 = PC04,
    Data5 = PC05,
    Data6 = PC06,
    Data7 = PC07,
    Data8 = PC10,
    Data9 = PC11,
    Data10 = PC12,
    Data11 = PC13,
    Data12 = PC14,
    Data13 = PC15,
    Data14 = PC16,
    Data15 = PC17,
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
}
void
loop() {
    pixel.clear();
    pixel.setPixelColor(0, pixel.Color(random(), random(), random()));
    pixel.show();
    delay(500);
}
