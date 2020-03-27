//
// Created by Admin on 3/20/2020.
//

#ifndef HotAir_h
#define HotAir_h

#include "SolderingStation.h"
#include <dimmable_light.h>

#ifndef DIMMABLE_LIGHT_H

#include "../libraries/Dimmable_Light_for_Arduino/src/dimmable_light.h"

#endif

DimmableLight hot(pinHotPwm);


/*PID variables. Fine tune this values. Start with P=1 and D and I = 0. Start increasing till you get good results*/
float airKp = 7.98, airKi = 0.055, airKd = 0.86, airHz = 10;          /*My values: Kp=7.98, Ki=0.055, Kd=0.86, Hz=10;*/
FastPID airPID(airKp, airKi, airKd, airHz, 8, false);

uint8_t hotPwm = 0;
uint8_t hertz = 75;

//
// 75 For 60Hz =>65
void isrAC() {
    unsigned int dimTime = (/*75*/ hertz * hotPwm);
    delayMicroseconds(dimTime);    // Off cycle
    digitalWrite(pinHotPwm, HIGH);   // triac firing
    delayMicroseconds(10);         // triac On propagation delay (for 60Hz use 8.33)
//    delayMicroseconds((dimTime - 10000) / 1000);         // triac On propagation delay (for 60Hz use 8.33)
    digitalWrite(pinHotPwm, LOW);    // triac Off
}

class HotAir {

    boolean isTermina = false;
    uint8_t outputHotPwm, outputAirPwm;
    int16_t activeRawTmp, activeTemp;
    uint16_t targetTmp = 0;
    double averageTmp;

private:

    void terminal(String where) {
        if (Serial.available()) {

            if (where == F("ap")) {
                isTermina = true;
                outputHotPwm = Serial.readStringUntil('\n').toInt();
                Serial.println();
                Serial.print(F("HOT PWM: "));
                Serial.print(outputHotPwm);
                Serial.println();
            }

            if (where == F("at")) {
                isTermina = true;
                targetTmp = Serial.readStringUntil('\n').toInt();
                Serial.println();
                Serial.print(F("HOT TARGET: "));
                Serial.print(targetTmp);
                Serial.println();
                airPID.clear();
            }


            if (where == F("fp")) {
                isTermina = true;
                outputAirPwm = Serial.readStringUntil('\n').toInt();
                Serial.println();
                Serial.print(F("AIR SPEED: "));
                Serial.print(outputAirPwm);
                Serial.println();
            }

        }
    }

    void debug() {
        Serial.print(F("\t//\t"));

        Serial.print(F(" AIR:  "));
        Serial.print((uint16_t) averageTmp);
        Serial.print(F(" / "));
        Serial.print(activeTemp);

        Serial.print(F(" ATar: "));
        Serial.print(targetTmp);

        Serial.print(F(" ATmp: "));
        Serial.print(activeRawTmp);

        Serial.print(F(" APwm: "));
        Serial.print(outputHotPwm);

        Serial.print(F(" FPwm: "));
        Serial.print(outputAirPwm);
    }


    void readTemp() {
        unsigned int avr = 0;
        for (index = 0; index < adcSamples; ++index) {
            avr += analogRead(pinAirTmp);
            delayMicroseconds(50);
        }
        activeRawTmp = avr / index;
        activeTemp = map(activeRawTmp, 200, 450, 220, 500) + 10;
        averageTmp += (activeTemp - averageTmp) * 0.05;


    }


public:

    HotAir() {}


    void begin() {
        pinMode(pinHotPwm, OUTPUT);
        pinMode(pinAirPwm, OUTPUT);
        pinMode(pinSyncAc, INPUT_PULLUP);
        pinMode(pinAirSwc, INPUT_PULLUP);
        pinMode(pinAirTmp, INPUT);
        DimmableLight::setSyncPin(pinSyncAc);
        DimmableLight::begin();
        analogWrite(pinAirPwm, 255);
    }


    void listen(String where) {
        terminal(where);
    }


    void manage(SolderingStation sos) {
        readTemp();
        if (sos.isHot() && !isTermina) {
            uint16_t input;
            input = sos.getHotSetPoint();
            targetTmp = (uint16_t) map(input, 0, 1024, 200, 450);
            input = sos.getAirSetPoint();
            outputAirPwm = (uint16_t) map(input, 0, 1024, 1, 50);
        } else if (!sos.isHot() && !isTermina) {
            targetTmp = 0;
            outputAirPwm = 0;
        }

        if (targetTmp > 0)
            outputHotPwm = (uint8_t) airPID.step(targetTmp, activeTemp);


        if (targetTmp == 0 && activeTemp > 80 || activeTemp > 500) {
            analogWrite(pinAirPwm, 255);
            hot.setBrightness(0);
        } else {
            hot.setBrightness(outputHotPwm);
            analogWrite(pinAirPwm, outputAirPwm);
        }

        debug();
    }


};


#endif //SOLDERINGSTATION_AIR_H
