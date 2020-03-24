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


#endif //SOLDERINGSTATION_SOLDERINGSTATION_H_H
