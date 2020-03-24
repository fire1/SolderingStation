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

class HotAit {


    uint8_t outputHotPwm, outputAirPwm;
    int16_t drawTmp;
    int16_t activeRawTmp;
    uint16_t targetRawTmp = 0;
    uint32_t averageRawTmp;

private:

    void setHotPwm(uint8_t pwm) {
        hotPwm = pwm;
    }

    void terminal(String where) {
        if (Serial.available()) {

            if (where == F("ap")) {
                outputHotPwm = Serial.readStringUntil('\n').toInt();
                Serial.println();
                Serial.print(F("HOT PWM: "));
                Serial.print(outputHotPwm);
                Serial.println();
            }

            if (where == F("at")) {
                targetRawTmp = Serial.readStringUntil('\n').toInt();
                Serial.println();
                Serial.print(F("HOT TARGET: "));
                Serial.print(targetRawTmp);
                Serial.println();
            }


            if (where == F("fp")) {
                outputAirPwm = Serial.readStringUntil('\n').toInt();
                Serial.println();
                Serial.print(F("AIR SPEED: "));
                Serial.print(outputAirPwm);
                Serial.println();
            }

        }
    }

    void debug() {
        Serial.print(F("  //   "));

        Serial.print(F(" AIR:  "));
        Serial.print(drawTmp);

        Serial.print(F(" ATar: "));
        Serial.print(targetRawTmp);

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
//        activeRawTmp = (uint16_t) 1.758 * activeRawTmp - 0.8;
        averageRawTmp += (activeRawTmp - averageRawTmp) * 0.05;

        drawTmp = (int16_t) 1.43 * averageRawTmp - 8.793; // 	Y = 1.143*X - 8.793
        if (drawTmp < 0) {
            drawTmp = 0;
        }


    }


public:

    HotAit() {}


    void begin() {
        pinMode(pinHotPwm, OUTPUT);
        pinMode(pinAirPwm, OUTPUT);
        pinMode(pinSyncAc, INPUT_PULLUP);
        pinMode(pinAirSwc, INPUT_PULLUP);
        pinMode(pinAirTmp, INPUT);
        DimmableLight::setSyncPin(pinSyncAc);
        DimmableLight::begin();
    }


    void listen(String where) {
        terminal(where);
    }


    void manage() {
        readTemp();
        if (targetRawTmp > 0)
            outputHotPwm = (uint8_t) airPID.step(targetRawTmp, activeRawTmp);
        hot.setBrightness(outputHotPwm);

        if (outputAirPwm == 0 && activeRawTmp > 200) {
            analogWrite(pinAirPwm, 255);
        } else
            analogWrite(pinAirPwm, outputAirPwm);

        debug();
    }


};


#endif //SOLDERINGSTATION_AIR_H
