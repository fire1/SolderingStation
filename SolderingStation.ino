#include <Arduino.h>
#include "SdrIrn.h"
#include "HotAir.h"


SolderingStation sos;
SdrIrn irn;
HotAir air;


void setup() {

    Serial.begin(9600);
    // pin D9/D10 PWM
    TCCR1B = TCCR1B & B11111000 | B00000001;

    irn.begin();
    air.begin();
    sos.begin();

    digitalWrite(pinBuzzer,HIGH);
    delay(100);
    digitalWrite(pinBuzzer,LOW);
}



void loop_(){

    Serial.println(analogRead(pinBtnHot));
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

        air.manage(sos);
        irn.manage();
        terminal = "";
    }
}