//
// Created by Admin on 3/21/2020.
//

#ifndef SolderingStation_h
#define SolderingStation_h

#include <Arduino.h>
#include <FastPID.h>
#include <LiquidCrystal_I2C.h>

#ifndef FastPID_H

#include "../libraries/FastPID/src/FastPID.h"

#endif

#ifndef FDB_LIQUID_CRYSTAL_I2C_H

#include "../libraries/LiquidCrystal_I2C/LiquidCrystal_I2C.h"

#endif

LiquidCrystal_I2C lcd(0x27, 16, 2);

unsigned long now = 0, last = 0, bounce = 0;
String terminal;


boolean isAirOn = false, isIrnOn = false, isAirStandby = false, isIrnStandby = false;
uint16_t setIrn, setAir, setFan = 1, tarIrn, actIrn, tarAir, actAir;
//
// Solder iron
volatile uint8_t index = 0; // Index for "For" :D
const uint8_t adcSamples = 8;
const uint8_t pinIronPwm = 10;
const uint8_t pinIronTmp = A7;
const uint8_t pinIronSwc = 4;

//
// Hot air
const uint8_t pinSyncAc = 2;
const uint8_t pinHotPwm = 9;
const uint8_t pinAirPwm = 11;
const uint8_t pinAirSwc = 5;
const uint8_t pinAirTmp = A6;

//
// Inputs
const uint8_t pinPotAir = A0;
const uint8_t pinPotHot = A1;
const uint8_t pinPotIrn = A2;
const uint8_t pinBtnHot = 12;
const uint8_t pinBtnIrn = A3;

const uint8_t pinBuzzer = 8;
const uint8_t pinBlinks = 13;

const int16_t refreshing = 300;
const unsigned long standby = 120000;
const unsigned long shutdown = 240000;
//
// Blinks + Sounds
boolean blinker = 0;

void blink() {
    blinker = !blinker;
    digitalWrite(pinBlinks, blinker);
}

unsigned long startAlarmed = 0;
int16_t soundLength = 0;

void alarm() {
    if (startAlarmed == 0) {
        digitalWrite(pinBuzzer, HIGH);
        startAlarmed = millis();
        soundLength = 150;
    }
}

void tick() {
    if (startAlarmed == 0) {
        digitalWrite(pinBuzzer, HIGH);
        startAlarmed = millis();
        soundLength = 50;
    }
}

class SolderingStation {


    char num[4];


    boolean compare(int read, int last) {
        int8_t gap = abs(last - read);
        return gap > 5;
    }


    void inputs() {
        int read;

        read = analogRead(pinPotAir);
        if (compare(read, setFan))
            setFan = read;

        read = analogRead(pinPotHot);
        if (compare(read, setAir))
            setAir = read;


        read = analogRead(pinPotIrn);
        if (compare(read, setIrn))
            setIrn = read;


        if (digitalRead(pinBtnHot) == LOW && now > bounce) {
            digitalWrite(pinBtnHot, HIGH);
            delay(30);
            if (digitalRead(pinBtnHot) == LOW) {
                bounce = now + 450;
                isAirOn = !isAirOn;
                tick();
            }
        }

        if (digitalRead(pinBtnIrn) == LOW && now > bounce) {
            digitalWrite(pinBtnIrn, HIGH);
            delay(30);
            if (digitalRead(pinBtnIrn) == LOW) {
                bounce = now + 450;
                isIrnOn = !isIrnOn;
                tick();
            }
        }
    }


    void soundBtn() {
        for (index = 0; index < 4; ++index) {
            delay(5);
            digitalWrite(pinBuzzer, HIGH);
            delay(50);
            digitalWrite(pinBuzzer, LOW);
        }
        digitalWrite(pinBuzzer, LOW);
    }


    void play() {
        if (millis() > startAlarmed + soundLength) {
            digitalWrite(pinBuzzer, LOW);
            startAlarmed = 0;
        }
    }


    void debug() {
        Serial.println();

        Serial.print(F(" IO: "));
        Serial.print(isIrnOn);

        Serial.print(F(" AO: "));
        Serial.print(isAirOn);

        Serial.print(F(" IS: "));
        Serial.print(isIrnStandby);

        Serial.print(F(" AS: "));
        Serial.print(isAirStandby);
    }

public:
    SolderingStation() {}


    void begin() {
        pinMode(pinPotAir, INPUT_PULLUP);
        pinMode(pinPotHot, INPUT_PULLUP);
        pinMode(pinPotIrn, INPUT_PULLUP);
        pinMode(pinBtnHot, INPUT_PULLUP);
        pinMode(pinBtnIrn, INPUT_PULLUP);
        pinMode(pinBuzzer, OUTPUT);

        digitalWrite(pinBtnHot, HIGH);
        digitalWrite(pinBtnIrn, HIGH);


        lcd.begin();

    }

    void listen() {
        play();
        inputs();
    }


    void draw() {

        digitalWrite(pinBlinks, LOW);
        if (actAir > 600) actAir = 0;
        if (actIrn > 600) actIrn = 0;


        lcd.setCursor(0, 0);
        lcd.print(F("AIR: "));
        sprintf(num, "%03d", tarAir);
        (isAirOn) ? lcd.print(num) : lcd.print(F("OFF"));

        lcd.setCursor(0, 1);
        lcd.print(F("SDR: "));
        sprintf(num, "%03d", tarIrn);
        (isIrnOn) ? lcd.print(num) : lcd.print(F("OFF"));

        lcd.setCursor(9, 0);
        sprintf(num, "%03d", actAir);
        (isAirOn) ? lcd.print(num) : lcd.print(F("   "));
        (isAirOn && isAirStandby) ? lcd.print(F("^")) : lcd.print(F(" "));


        lcd.setCursor(9, 1);
        sprintf(num, "%03d", actIrn);
        (isIrnOn) ? lcd.print(num) : lcd.print(F("   "));
        (isIrnOn && isIrnStandby) ? lcd.print(F("^")) : lcd.print(F(" "));

        lcd.setCursor(14, 0);
        sprintf(num, "%02d", map(setFan, 1020, 0, 1, 99));
        (isAirOn) ? lcd.print(num) : lcd.print(F("  "));


        debug();
    }


};


#endif //SOLDERINGSTATION_SOLDERINGSTATION_H_H
