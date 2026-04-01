#include <Arduino.h>

extern "C" {
#include <apio.h>
}

constexpr auto ADS = 9;
constexpr auto DEN = 10;
constexpr auto TransactionStart = 11;
void 
setup() {
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
}

void 
loop() {
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    delay(1000);
}
