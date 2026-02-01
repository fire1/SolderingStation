//
// Created by Admin on 3/20/2020.
//

#ifndef HotAir_h
#define HotAir_h

#include "SolderingStation.h"
#include <dimmable_light.h>



DimmableLight hot(pinHotPwm);


/*PID variables. Fine tune this values. Start with P=1 and D and I = 0. Start increasing till you get good results*/
float airKp = 7.98, airKi = 0.055, airKd = 0.86, airHz = 30; /*My values: Kp=7.98, Ki=0.055, Kd=0.86, Hz=10;*/
FastPID airPID(airKp, airKi, airKd, airHz, 8, false);

uint8_t hotPwm = 0;
uint8_t hertz = 75;


class HotAir {


    boolean isTerminal = false;
    uint8_t lastState = 0;
    uint8_t outputHotPwm, outputAirPwm;
    int16_t activeRawTmp, activeTemp;
    uint16_t targetTmp = 0;
    uint16_t readIndTmp = 0;
    unsigned long readRawTmp = 0;
    unsigned long standBegin = 0;
    double averageTmp;

private:

    void terminal(String where) {
        if (Serial.available()) {

            if (where == F("ap")) {
                isTerminal = true;
                outputHotPwm = Serial.readStringUntil('\n').toInt();
                Serial.println();
                Serial.print(F("HOT PWM: "));
                Serial.print(outputHotPwm);
                Serial.println();
            }

            if (where == F("at")) {
                isTerminal = true;
                targetTmp = Serial.readStringUntil('\n').toInt();
                Serial.println();
                Serial.print(F("HOT TARGET: "));
                Serial.print(targetTmp);
                Serial.println();
                airPID.clear();
            }


            if (where == F("fp")) {
                isTerminal = true;
                outputAirPwm = Serial.readStringUntil('\n').toInt();
                Serial.println();
                Serial.print(F("AIR SPEED: "));
                Serial.print(outputAirPwm);
                Serial.println();
            }

        }
    }

    void debug() {
        Serial.print(F(" "));

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


    void calTemp() {
        /*unsigned int avr = 0;
        for (index = 0; index < adcSamples; ++index) {
            avr += analogRead(pinAirTmp);
            delayMicroseconds(50);
        }
        activeTmp = avr / index;
        */

        activeRawTmp = readRawTmp / readIndTmp;
        readRawTmp = 0;
        readIndTmp = 0;

        activeTemp = map(activeRawTmp, 200, 450, 220, 500) -30;
        averageTmp += (activeTemp - averageTmp) * 0.05;

    }


    void standbys() {


        int state = digitalRead(pinAirSwc);
        if (state == LOW && state != lastState && isAirOn) {
            delay(10);
            if (digitalRead(pinAirSwc) == LOW) {
                lastState = LOW;
                isAirStandby = true;
                standBegin = now;
                tick();
            }
        } else if (state == HIGH && state != lastState && isAirOn) {
            delay(10);
            if (digitalRead(pinAirSwc) == HIGH) {
                lastState = HIGH;
                isAirStandby = false;
                standBegin = 0;
            }

        }

        //
        // turn off
        if (now - standBegin > standby && standBegin > 0) {
            isAirOn = false;
            isAirStandby = false;
            standBegin = 0;
        }
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
        standbys();
        readRawTmp += analogRead(pinAirTmp);
        readIndTmp++;
    }


    void manage(SolderingStation sos) {

        calTemp();

        if (!isAirOn && !isTerminal) {
            targetTmp = 0;
            outputAirPwm = 0;
        } else if (isAirOn && isAirStandby) {
            targetTmp = 200;
            outputAirPwm = 100;
        } else if (isAirOn && !isAirStandby) {
            targetTmp = (uint16_t) map(setAir, 1020, 0, 200, 500);
            outputAirPwm = (uint8_t) map(setFan, 1020, 0, 70, 220);
        }

        outputHotPwm = (uint8_t) airPID.step(targetTmp + 7, activeTemp);
        tarAir = targetTmp, actAir = (uint16_t) activeTemp;

        if (targetTmp == 0 && activeTemp > 75 || activeTemp > 560) {
            outputHotPwm = 0;
            outputAirPwm = 255;
        }

        if (tarAir - 30 < actAir && tarAir - 25 > actAir)
            alarm();

        hot.setBrightness(outputHotPwm);
        analogWrite(pinAirPwm, outputAirPwm);


        debug();
    }


};


#endif //SOLDERINGSTATION_AIR_H
