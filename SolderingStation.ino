#include <Arduino.h>
#include "SdrIrn.h"
#include "HotAir.h"


SolderingStation sos;
SdrIrn irn;
HotAir air;


void setup() {
    now = millis();
    Serial.begin(9600);
    irn.begin();
    air.begin();
    sos.begin();
    digitalWrite(pinBuzzer, HIGH);
    delay(100);
    digitalWrite(pinBuzzer, LOW);
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
        terminal = "";
    }

        air.manage(sos);
        irn.manage(sos);

}