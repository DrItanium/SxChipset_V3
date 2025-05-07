#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <RTClib.h>
Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL);
volatile bool hasRTC = false;
volatile bool hasSDCard = false;
RTC_DS3231 rtc;
const char daysOfTheWeek[7][12] {
    "Sunday",
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday",
};
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
setup() {
    Serial.begin(9600);
    SD.begin();
    SPI.begin();
    pixel.begin();
    pixel.setBrightness(1);
}
void
displayDateTime() noexcept {
    if (hasRTC) {
        auto now = rtc.now();
        Serial.print(now.year(), DEC);
        Serial.print('/');
        Serial.print(now.month(), DEC);
        Serial.print('/');
        Serial.print(now.day(), DEC);
        Serial.print(" (");
        Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
        Serial.print(") ");
        Serial.print(now.hour(), DEC);
        Serial.print(':');
        Serial.print(now.minute(), DEC);
        Serial.print(':');
        Serial.print(now.second(), DEC);
        Serial.println();

        Serial.print(" since midnight 1/1/1970 = ");
        Serial.print(now.unixtime());
        Serial.print("s = ");
        Serial.print(now.unixtime() / 86400L);
        Serial.println("d");
    }
}
void
loop() {
    //displayDateTime();
    pixel.clear();
    pixel.setPixelColor(0, pixel.Color(random(), random(), random()));
    pixel.show();
    delay(500);
}
