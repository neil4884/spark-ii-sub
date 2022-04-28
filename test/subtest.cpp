#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    Serial.println("hello moon");
}

void loop() {
    Serial.println("hello");
    delay(1000);
}