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
    DAC0 = PIN_PA02,
    DAC1 = PIN_PA05,
    READY = PIN_PD12,
    ADRMUX_DEN1 = PIN_PA12,
    ADRMUX_DEN2 = PIN_PA13,
    ADRMUX_CLK = PIN_PA14,
    // Alternate view for the pinout
    ADRMUX_SEL0 = ADRMUX_DEN1,
    ADRMUX_SEL1 = ADRMUX_DEN2,
    ADRMUX_EN = ADRMUX_CLK,
    BLAST = PIN_PA15,
    ADRMUX0 = PIN_PA16,
    ADRMUX1 = PIN_PA17,
    ADRMUX2 = PIN_PA18,
    ADRMUX3 = PIN_PA19,
    ADRMUX4 = PIN_PA20,
    ADRMUX5 = PIN_PA21,
    ADRMUX6 = PIN_PA22,
    ADRMUX7 = PIN_PA23,
    BE0 = PIN_PB14,
    BE1 = PIN_PB15,
    WR = PIN_PB19,
    Data0 = PIN_PC00,
    Data1 = PIN_PC01,
    Data2 = PIN_PC02,
    Data3 = PIN_PC03,
    Data4 = PIN_PC04,
    Data5 = PIN_PC05,
    Data6 = PIN_PC06,
    Data7 = PIN_PC07,
    Data8 = PIN_PC10,
    Data9 = PIN_PC11,
    Data10 = PIN_PC12,
    Data11 = PIN_PC13,
    Data12 = PIN_PC14,
    Data13 = PIN_PC15,
    Data14 = PIN_PC16,
    Data15 = PIN_PC17,
};

[[gnu::always_inline]]
inline void 
pinMode(Pin targetPin, decltype(OUTPUT) direction) noexcept {
    ::pinMode(static_cast<std::underlying_type_t<Pin>>(targetPin), direction);
}

[[gnu::always_inline]]
inline void
digitalWrite(Pin targetPin, decltype(LOW) value) noexcept {
    ::digitalWrite(static_cast<std::underlying_type_t<Pin>>(targetPin), value);
}

[[gnu::always_inline]]
inline auto
digitalRead(Pin targetPin) noexcept {
    return ::digitalRead(static_cast<std::underlying_type_t<Pin>>(targetPin));
}
[[gnu::always_inline]]
inline void
outputPin(Pin targetPin, decltype(LOW) initialValue = HIGH) noexcept {
    pinMode(targetPin, OUTPUT);
    digitalWrite(targetPin, initialValue);
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
void
setup() {
    Serial.begin(9600);
    SD.begin();
    SPI.begin();
    Wire.begin();
    outputPin(Pin::READY, HIGH);
    outputPin(Pin::ADRMUX_SEL0, LOW);
    outputPin(Pin::ADRMUX_SEL1, LOW);
    outputPin(Pin::ADRMUX_EN, HIGH);
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
