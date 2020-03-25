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


//
// Solder iron
volatile uint8_t index = 0; // Index for "For" :D
const uint8_t adcSamples = 5;
const uint8_t pinIronPwm = 10;
const uint8_t pinIronTmp = A7;
const uint8_t pinIronSwc = 2;

//
// Hot air
const uint8_t pinSyncAc = 2;
const uint8_t pinHotPwm = 9;
const uint8_t pinAirPwm = 11;
const uint8_t pinAirSwc = 5;
const uint8_t pinAirTmp = A6;

//
// Inputs
const uint8_t pinPotAir = A2;
const uint8_t pinPotHot = A1;
const uint8_t pinPotIrn = A0;
const uint8_t pinBtnHot = A3;
const uint8_t pinBtnIrn = 12;

const uint8_t pinBuzzer = 8;
const int16_t refreshing = 250;

//
// Sounds
typedef uint8_t sound[3];
// Format {<HIGH time>,<LOW time>,<offset>}
sound toneOn = {50, 10, 1}; // on
sound toneOff = {25, 10, 2}; // off


class SolderingStation {

    boolean stateIrn = false, stateHot = false;
    sound tones = {0, 0, 0};
    int valAir, valHot, valIrn, lastAir, lastHot, lastIrn;

    uint8_t playIndex = 0;
    unsigned long playSound = 0, playMute = 0;


    boolean compare(int read, int last) {
        return read != last && read > last + 5 && read < last - 5;
    }


    void terminal(String where) {

        if (where == F("it")) {
            valIrn = Serial.readStringUntil('\n').toInt();
            Serial.println();
            Serial.print(F("IRON TARGET: "));
            Serial.print(valIrn);
            Serial.println();
        }
    }

    void play(sound value) {
        tones[1] = value[1];
        tones[2] = value[2];
    }


    void inputs() {
        int read;

        read = analogRead(pinPotAir);
        if (compare(read, valAir))
            valAir = read;

        read = analogRead(pinPotHot);
        if (compare(read, valHot))
            valHot = read;


        read = analogRead(pinPotHot);
        if (compare(read, valIrn))
            valIrn = read;


        if (digitalRead(pinBtnHot) == LOW) {
            delay(10);
            if (digitalRead(pinBtnHot) == LOW) {
                stateHot = !stateHot;
                stateHot ? play(toneOn) : play(toneOff);
            }
        }

        if (digitalRead(pinBtnIrn) == LOW) {
            delay(10);
            if (digitalRead(pinBtnIrn) == LOW) {
                stateIrn = !stateIrn;
                stateIrn ? play(toneOn) : play(toneOff);
            }
        }
    }


    void player() {
        if (tones[3] == 0) return;

        unsigned long time = millis();
        if (tones[3] < playIndex) {
            if (playSound == 0 && playMute < time) {
                playSound = time + tones[1];
                playIndex++;
                digitalWrite(pinBuzzer, HIGH);
            } else if (playSound > time) {
                playSound = 0;
                playMute = time + tones[2];
                digitalWrite(pinBuzzer, LOW);
            }

        } else {
            tones[1] = 0;
            tones[2] = 0;
            tones[3] = 0;
        }
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

        // pin D9/D10 PWM
        TCCR1B = TCCR1B & B11111000 | B00000101; // for PWM frequency of 30.64 Hz

        // TCCR1B = TCCR1B & B11111000 | B00000011; // for PWM frequency of 490.20 Hz (The DEFAULT)
        // TCCR1B = TCCR1B & B11111000 | B00000100; // for PWM frequency of 122.55 Hz

    }

    void listen() {
        inputs();
        player();
    }


    void draw() {

    }

    inline boolean isHot() {
        return stateHot;
    }

    inline boolean isIrn() {
        return stateIrn;
    }


    inline int getAirSetpoint() {
        return valAir;
    }

    inline int getHotSetpoint() {
        return valHot;
    }

    inline int getIrnSetpoint() {
        return valIrn;
    }


};


#endif //SOLDERINGSTATION_SOLDERINGSTATION_H_H
