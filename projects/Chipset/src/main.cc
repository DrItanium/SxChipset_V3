#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL);
void
setup() {
    Serial.begin(9600);
    SD.begin();
    SPI.begin();
    Wire.begin();
    pixel.begin();
    pixel.setBrightness(1);
}

void
loop() {
    pixel.clear();
    pixel.setPixelColor(0, pixel.Color(random(), random(), random()));
    pixel.show();
    delay(500);
}
