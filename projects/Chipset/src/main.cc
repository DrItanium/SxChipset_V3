#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

void
setup() {
    SD.begin();
    SPI.begin();
    Wire.begin();
}

void
loop() {

}
