#include <Arduino.h>
#include "SdrIron.h"
#include "HotAir.h"


SdrIron irn;
HotAit air;


void setup() {
    Serial.begin(9600);
    irn.begin();
    air.begin();
}

unsigned long last = 0;

void loop() {
    unsigned long now = millis();

    if (Serial.available())
        terminal = Serial.readStringUntil('=');

    irn.listen(terminal);
    air.listen(terminal);

    if (now > last) {
        last = now + 250;
        air.manage();
        irn.manage();
        terminal = "";
    }
}