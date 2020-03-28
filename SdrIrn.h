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
    boolean isOnToggle = false;
    uint8_t outputPwm;
    uint8_t standbyState = 0;
    int16_t targetTmp = 0;
    int16_t activeTmp = 0;

    double averageTmp = 0;
    unsigned long standbyBegin = 0;

    void readTemp() {
        analogWrite(pinIronPwm, 0);
        delay(15);
        unsigned int avr = 0;
        for (index = 0; index < adcSamples; ++index) {
            avr += analogRead(pinIronTmp);
            delayMicroseconds(50);
        }
        activeTmp = getTemp(avr / index);
        averageTmp += (activeTmp - averageTmp) * 0.05;
    }

    uint16_t getTemp(uint16_t raw) {
        return (int16_t) 1.758 * raw - 0.8; // 	Y = 1.059*X - 39.39
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
                targetTmp = Serial.readStringUntil('\n').toInt();
                Serial.println();
                Serial.print(F("IRON TARGET: "));
                Serial.print(targetTmp);
                Serial.println();
            }

        }
    }

    void debug() {
        Serial.print(F("  "));

        Serial.print(F(" IRON:  "));
        Serial.print((uint16_t) averageTmp);

        Serial.print(F(" ITar: "));
        Serial.print(targetTmp);

        Serial.print(F(" ITmp: "));
        Serial.print(activeTmp);

        Serial.print(F(" IPwm: "));
        Serial.print(outputPwm);

        Serial.print(F(" ISwc:"));
        Serial.print(standbyBegin);

    }

    void standbys() {


        uint8_t state = digitalRead(pinIronSwc);
        if (standbyState != state) {
            delay(10);
            if (digitalRead(pinIronSwc) == state) {
                standbyState = state;
                standbyBegin = now;
                isIrnStandby = false;
            }
        }

        if (now - standbyBegin > standby && standbyBegin > 0 && !isIrnStandby) {
            isIrnStandby = true;
            standbyBegin = now;
        }

        if (now - standbyBegin > shutdown && standbyBegin > 0 && isIrnStandby) {
            standbyBegin = 0;
            isIrnOn = false;
            isIrnStandby = false;
        }
    }

public:
    SdrIrn() {

    }


    void begin() {
        pinMode(pinIronPwm, OUTPUT);
        pinMode(pinIronSwc, INPUT_PULLUP);
        pinMode(pinIronTmp, INPUT);
        standbyBegin = now;
    }

    void listen(String where) {
        terminal(where);
        standbys();
    }


    void manage(SolderingStation sos) {

        if (isOnToggle != isIrnOn) {
            isOnToggle = isIrnOn;
            standbyBegin = now;
        }

        readTemp();
        if (isIrnOn && !isIrnStandby) {
            targetTmp = map(setIrn, 0, 1024, 200, 450) + 15;
        }

        if (isIrnOn && isIrnStandby) {
            targetTmp = 200;
        }

        outputPwm = sdrPID.step(targetTmp, activeTmp);

        if (!isIrnOn) {
            outputPwm = 0;
            targetTmp = 0;
        }
        analogWrite(pinIronPwm, outputPwm);
        debug();
    }


};


#endif //SOLDERINGSTATION_SOLDERINGSTATION_H
