//
// Created by Admin on 3/21/2020.
//

#ifndef SolderingStation_h
#define SolderingStation_h

#include <Arduino.h>
#include <FastPID.h>
//#include <EnableInterrupt.h>

#ifndef FastPID_H

#include "../libraries/FastPID/src/FastPID.h"

#endif

#ifndef EnableInterrupt_h

//#include "../libraries/EnableInterrupt/EnableInterrupt.h"

#endif

unsigned long now = 0, last = 0;
String terminal;


boolean isAirOn = false, isIrnOn = false, isAirStandby = false, isIrnStandby;
uint16_t  setIrn, setAir, setFan;
//
// Solder iron
volatile uint8_t index = 0; // Index for "For" :D
const uint8_t adcSamples = 5;
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
const int16_t refreshing = 250;
const unsigned long standby = 160000;
const unsigned long shutdown = 240000;
//
// Sounds
typedef uint8_t sound[3];
// Format {<HIGH time>,<LOW time>,<offset>}
sound toneOn = {50, 10, 1}; // on
sound toneOff = {25, 10, 2}; // off


class SolderingStation {


    sound tones = {0, 0, 0};
    uint8_t playIndex = 0;
    unsigned long playSound = 0, playMute = 0;


    boolean compare(int read, int last) {
        return read != last;
        return read != last && read > last + 5 && read < last - 5;
    }

    void play(sound value) {
        tones[1] = value[1];
        tones[2] = value[2];
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


        if (digitalRead(pinBtnHot) == LOW) {
            delay(10);
            if (digitalRead(pinBtnHot) == LOW) {
                isAirOn = !isAirOn;
                isAirOn ? play(toneOn) : play(toneOff);
                delay(250);
            }
        }

        if (digitalRead(pinBtnIrn) == LOW) {
            delay(10);
            if (digitalRead(pinBtnIrn) == LOW) {
                isIrnOn = !isIrnOn;
                isIrnOn ? play(toneOn) : play(toneOff);
                delay(250);
            }
        }
    }


    void player() {
        if (tones[3] == 0) return;


        if (tones[3] < playIndex) {
            if (playSound == 0 && playMute < now) {
                playSound = now + tones[1];
                playIndex++;
                digitalWrite(pinBuzzer, HIGH);
            } else if (playSound > now) {
                playSound = 0;
                playMute = now + tones[2];
                digitalWrite(pinBuzzer, LOW);
            }

        } else {
            tones[1] = 0;
            tones[2] = 0;
            tones[3] = 0;
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

    }

    void listen() {
        inputs();
        player();

    }


    void draw() {

        debug();
    }


};


#endif //SOLDERINGSTATION_SOLDERINGSTATION_H_H
