//
// Created by Admin on 3/20/2020.
//

#ifndef SdrIron_h
#define SdrIron_h

#include "SolderingStation.h"


/*PID variables. Fine tune this values. Start with P=1 and D and I = 0. Start increasing till you get good results*/
float sdrKp = 7.98, sdrKi = 0.055, sdrKd = 0.86, sdrHz = 10;          /*My values: Kp=7.98, Ki=0.055, Kd=0.86, Hz=10;*/
FastPID sdrPID(sdrKp, sdrKi, sdrKd, sdrHz, 8, false);


class SdrIrn {

private:

    static int8_t toggleSwc;
    uint8_t outputPwm;
    int16_t targetRawTmp = 0;
    int16_t activeRawTmp = 0;
    uint16_t drawTmp = 0;
    double averageRawTmp = 0;

    void readTemp() {
        analogWrite(pinIronPwm, 0);
        delay(15);
        unsigned int avr = 0;
        for (index = 0; index < adcSamples; ++index) {
            avr += analogRead(pinIronTmp);
            delayMicroseconds(50);
        }
        activeRawTmp = avr / index;
        averageRawTmp += (activeRawTmp - averageRawTmp) * 0.05;
        drawTmp = (int16_t) 1.758 * averageRawTmp - 0.8; // 	Y = 1.059*X - 39.39
        if (drawTmp < 0) {
            drawTmp = 0;
        }

    }


    void terminal(String where) {
        if (Serial.available()) {

            if (where == F("ip")) {
                outputPwm = Serial.readStringUntil('\n').toInt();
                analogWrite(pinIronPwm, outputPwm);
                Serial.println();
                Serial.print(F("IRON PWM: "));
                Serial.print(outputPwm);
                Serial.println();
            }

            if (where == F("it")) {
                targetRawTmp = Serial.readStringUntil('\n').toInt();
                Serial.println();
                Serial.print(F("IRON TARGET: "));
                Serial.print(targetRawTmp);
                Serial.println();
            }

        }
    }

    void debug() {
        Serial.println();

        Serial.print(F(" IRON:  "));
        Serial.print(drawTmp);

        Serial.print(F(" ITar: "));
        Serial.print(targetRawTmp);

        Serial.print(F(" ITmp: "));
        Serial.print(activeRawTmp);

        Serial.print(F(" IPwm: "));
        Serial.print(outputPwm);

    }

public:
    SdrIrn() {

    }


    static void toggleSdrIron() {
        SdrIrn::toggleSwc++;
    }

    void begin() {
//        enableInterrupt(pinIronSwc, SdrIrn::toggleSdrIron, CHANGE);
        pinMode(pinIronPwm, OUTPUT);
        pinMode(pinIronSwc, INPUT_PULLUP);
        pinMode(pinIronTmp, INPUT);
    }

    void listen(String where) {
        terminal(where);
    }


    void manage() {
        readTemp();
        if (targetRawTmp > 0)
            outputPwm = (uint8_t) sdrPID.step(targetRawTmp, activeRawTmp);
        analogWrite(pinIronPwm, outputPwm);
        debug();
    }


};

static int8_t SdrIrn::toggleSwc = 0;


#endif //SOLDERINGSTATION_SOLDERINGSTATION_H
