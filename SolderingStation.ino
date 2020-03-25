#include <Arduino.h>
#include "SdrIrn.h"
#include "HotAir.h"


SolderingStation sos;
SdrIrn irn;
HotAit air;


void setup() {
    Serial.begin(9600);
    irn.begin();
    air.begin();
    sos.begin();
}


void loop() {
    now = millis();

    if (Serial.available()) terminal = Serial.readStringUntil('=');

    sos.listen();
    irn.listen(terminal);
    air.listen(terminal);

    if (now > last) {
        last = now + refreshing;

        sos.draw();

        air.manage();
        irn.manage();
        terminal = "";
    }
}