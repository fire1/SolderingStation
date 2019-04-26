/**
 * Soldering station Air&Iron
 *
 *      FIRMWARE
 *      @author <fire1@abv.bg>
 *      @version 1.0
 *
 */

#include <Arduino.h>
#include <TimerOne.h>    // Avaiable from http://www.arduino.cc/playground/Code/Timer1
#include <PID_v1.h>

//
// Only for IDE
#ifndef TimerOne_h_

#include "../libraries/TimerOne/TimerOne.h"

#endif

#ifndef PID_v1_h

#include "../libraries/PID/PID_v1.h"

#endif

//
// Screen or Serial print
//#define DEBUG

//
// TODO: Handle of 65hz:
// #define FREQ 65    // 50Hz power in these parts
#define FREQ 8    // // Way more stable to be lower value

#define SDR_MIN  350
#define AIR_MIN  300
#define RPM_MIN  440

#define SDR_MAX  420
#define AIR_MAX  480
#define RPM_MAX  1021

//
// Sleep mode start
#define SLEEP_BEGIN_AIR 6000
#define SLEEP_BEGIN_SDR 60000

#define SLEEP_TARGET_AIR 200
#define SLEEP_TARGET_SDR 200

//
// Shutdown mode start [SLEEP_BEGIN * OFF_MULTIPLIER]
#define OFF_MULTIPLIER 3


#ifdef DEBUG
#define LOOP_PRINT 1500
#else
#define LOOP_PRINT 300
#endif

//
// LCD
#ifndef DEBUG

#include <LiquidCrystal.h>

#ifndef LiquidCrystal_h

#include <LiquidCrystal/src/LiquidCrystal.h>

#endif
LiquidCrystal lcd(5, 4, 3, 2, 1, 0);

byte char_cel[] = {
        B01000,
        B10100,
        B01000,
        B00011,
        B00100,
        B00100,
        B00011,
        B00000
};
byte char_std[] = {
        B00000,
        B00000,
        B00000,
        B00000,
        B00100,
        B00100,
        B01010,
        B00100
};

#endif

/**
 * Methods definition
 */
static void buttonSolder();

static void buttonHotAir();

static void standSolder();

static void standHotAir();

static void inputSerial();

static void printSerial(unsigned long airNow, unsigned long sdrNow, uint16_t showAirTemp, uint16_t showSdrTemp);

static void printScreen(uint16_t showSdrTemp, uint16_t showAirTemp, uint16_t showAirRpm);

boolean fenStart = false;
boolean isSleepAirOn = false;
boolean isSleepSdrOn = false;
boolean isSolderUse = true;
boolean isSolderOn = false, isHotAirOn = false, isHotAirUse = true;

const uint8_t SDR_INPUT = A5; // A5
const uint8_t AIR_INPUT = A4;

const uint8_t SDR_OUTPUT = 11; // Solder heater
const uint8_t FEN_OUTPUT = 9; // fen output
const uint8_t HOT_OUTPUT = 10; // fen output

//
// Hardware interface
const uint8_t SET_SOLDER_TARGET = A1;
const uint8_t SET_AIRRPM_TARGET = A2;
const uint8_t SET_HOTAIR_TARGET = A3;

const uint8_t SET_BUTTON_SOLDER = 7;
const uint8_t SET_BUTTON_HOTAIR = 8;

//
// Stands
const uint8_t SET_HOTAIR_STANDS = A0;
const uint8_t SET_SOLDER_STANDS = 6;


uint16_t rpmSetPoint = 1024;


unsigned long standStartHotAir = 0;
unsigned long standStartSolder = 0;



/*
    Kp: Determines how aggressively the PID reacts to the current amount of error (Proportional) (double >=0)
    Ki: Determines how aggressively the PID reacts to error over time (Integral) (double>=0)
    Kd: Determines how aggressively the PID reacts to the change in error (Derivative) (double>=0)
    POn: Either P_ON_E (Default) or P_ON_M. Allows Proportional on Measurement to be specified.

    SetTunings(Kp, Ki, Kd, POn)
 */

//
// AIR
const double aggKpAir = 8, aggKiAir = 1, aggKdAir = 1;
const double consKpAir = 2, consKiAir = 0.05, consKdAir = 0.25;
double airSetPoint, airInput, airOutput;
PID airPID(&airInput, &airOutput, &airSetPoint, consKpAir, consKiAir, consKdAir, DIRECT);

//
// Solder
const double aggKpSdr = 1, aggKiSdr = 0.5, aggKdSdr = 0.15;
const double consKpSdr = 2, consKiSdr = 1, consKdSdr = 1;
double sdrSetPoint, sdrInput, sdrOutput;
PID sdrPID(&sdrInput, &sdrOutput, &sdrSetPoint, consKpSdr, consKiSdr, consKdSdr, DIRECT);


unsigned long debounceSolder, debounceHotAir;


void setup() {

    // 1000000 / (2 * FREQ);//The Timerone PWM period, 50Hz = 10000 uS
    Timer1.initialize(FREQ); // Way more stable variant

    pinMode(SDR_INPUT, INPUT);
    pinMode(SDR_OUTPUT, OUTPUT);
    pinMode(FEN_OUTPUT, OUTPUT);
    pinMode(HOT_OUTPUT, OUTPUT);

    pinMode(SET_SOLDER_TARGET, INPUT);
    pinMode(SET_AIRRPM_TARGET, INPUT);
    pinMode(SET_HOTAIR_TARGET, INPUT);
    pinMode(SET_HOTAIR_STANDS, INPUT_PULLUP);
    pinMode(SET_BUTTON_SOLDER, INPUT_PULLUP);
    pinMode(SET_BUTTON_HOTAIR, INPUT_PULLUP);
    pinMode(SET_SOLDER_STANDS, INPUT_PULLUP);

    digitalWrite(SET_SOLDER_STANDS, HIGH);
    digitalWrite(8, HIGH);
//    analogWrite(SDR_INPUT, 0);
    digitalWrite(8, HIGH);
    analogWrite(SDR_OUTPUT, 0);
    analogWrite(FEN_OUTPUT, 0);
    analogWrite(HOT_OUTPUT, 0);
    analogWrite(FEN_OUTPUT, 255);
#ifdef DEBUG
    Serial.begin(115200);
#else
    lcd.begin(16, 2);
    lcd.createChar(1, char_cel);
    lcd.createChar(2, char_std);
    lcd.setCursor(0, 0);
    lcd.print(F(" SOLDER STATION "));
    lcd.setCursor(0, 1);
    lcd.print(F(" ver.1.1.0 "));
    delay(1500);
    lcd.clear();
#endif


    airSetPoint = 0;
    sdrSetPoint = 0;
    airPID.SetOutputLimits(0, 1024);
    sdrPID.SetOutputLimits(0, 255);

    airInput = analogRead(AIR_INPUT);
    sdrInput = analogRead(SDR_INPUT);

    airPID.SetMode(AUTOMATIC);
    sdrPID.SetMode(AUTOMATIC);

}


#define READ_SENSOR 10
boolean dbg = false;
unsigned long loopPrint = 0;
uint64_t airTemp, sdrTemp = 0;
uint16_t loopIndex = 0;
volatile uint8_t ms;
volatile unsigned long airNow, sdrNow;

void loop() {


//#ifndef DEBUG
    //
    // My inputs are inverted ...
    sdrSetPoint = (isSolderOn) ? (uint16_t) map(analogRead(SET_SOLDER_TARGET), 1021, 0, SDR_MIN, SDR_MAX) : 0;
    airSetPoint = (isHotAirOn) ? (uint16_t) map(analogRead(SET_HOTAIR_TARGET), 1021, 0, AIR_MIN, AIR_MAX) : 0;
    rpmSetPoint = (isHotAirOn) ? (uint16_t) map(analogRead(SET_AIRRPM_TARGET), 1021, 0, RPM_MIN, RPM_MAX) : 0;


    /*
     * Temperature resolver
     */
    for (ms = 0; ms < READ_SENSOR; ++ms) {
        airNow += ((analogRead(AIR_INPUT) * 9 + 32 * 5 + 2) / 5);
        sdrNow += ((analogRead(SDR_INPUT) * 9 + 32 * 5 + 2) / 5) * 0.1;
        delayMicroseconds(10);
    }

    airNow = airNow / READ_SENSOR;
    sdrNow = sdrNow / READ_SENSOR;


    //
    // Resolve solder temperature
    uint16_t currentSolderTemp;
    if (sdrNow < 60) {
        currentSolderTemp = (uint16_t) map(sdrNow, 10, 60, 15, 200);
    } else if (sdrNow < 115) {
        currentSolderTemp = (uint16_t) map(sdrNow, 60, 115, 200, 325);
    } else {
        currentSolderTemp = (uint16_t) map(sdrNow, 115, 145, 325, 420);
    }
    //
    // Resolve hot air temperature
    uint16_t currentHotAirTemp;
    if (airNow < 280) {
        currentHotAirTemp = (uint16_t) map(airNow, 100, 280, 120, 390);
    } else {
        currentHotAirTemp = (uint16_t) map(airNow, 280, 315, 390, 430);
    }


    airInput = currentHotAirTemp;
    sdrInput = currentSolderTemp;

    sdrTemp += sdrInput;
    airTemp += airInput;

    loopIndex++;

    uint8_t showAirRpm = (uint8_t) map(analogRead(SET_AIRRPM_TARGET), 1023, 0, 0, 99);

    uint16_t showAirTemp = (airTemp / loopIndex);
    uint16_t showSdrTemp = (sdrTemp / loopIndex);


    //
    // Smoothing temperatures to show
    if (loopIndex > 6) {
        airTemp = (airTemp / loopIndex);
        sdrTemp = (sdrTemp / loopIndex);
        loopIndex = 1;
    }

    buttonSolder();
    buttonHotAir();
    standSolder();
    standHotAir();
    inputSerial();

    /*
     * Soldering iron
     */
    uint16_t constrainsSolder = (uint16_t) sdrOutput;

    sdrPID.Compute();

    if (sdrSetPoint == 0) {
        constrainsSolder = 0;
        sdrOutput = 0;
    }


    if (isSolderOn) {
        analogWrite(SDR_OUTPUT, constrainsSolder);
    } else {
        analogWrite(SDR_OUTPUT, 0);
    }

    double gapSdr = abs(airSetPoint - airInput); //distance away from setpoint
    if (gapSdr < 10) {
        //we're close to set point, use conservative tuning parameters
        sdrPID.SetTunings(consKpSdr, consKiSdr, consKdSdr);
    } else {
        //we're far from set point, use aggressive tuning parameters
        sdrPID.SetTunings(aggKpSdr, aggKiSdr, aggKdSdr);
    }


    double gapAir = abs(airSetPoint - airInput); //distance away from setpoint
    if (gapAir < 10) {
        //we're close to set point, use conservative tuning parameters
        airPID.SetTunings(consKpAir, consKiAir, consKdAir);
    } else {
        //we're far from set point, use aggressive tuning parameters
        airPID.SetTunings(aggKpAir, aggKiAir, aggKdAir);
    }


    //
    // Protection from temperature of hot air when boot
    if (millis() > 5000 && !fenStart && airInput < 200) {
        rpmSetPoint = 0;
        fenStart = true;
    }

    airPID.Compute();

    uint16_t constrainsHotAir;
    if (airSetPoint == 0) {
        constrainsHotAir = 0;
        airOutput = 0;
    } else {
        constrainsHotAir = (uint16_t) airOutput;
    }
    //
    // HOT AIR
    if (airInput > 550 || airSetPoint > 600 /*|| airInput + 200 > airSetPoint*/) {
        constrainsHotAir = 0;
        airOutput = 0;
    }
    if (airSetPoint == 0) {
        constrainsHotAir = 0;
    }

    if (isHotAirOn) {
        Timer1.pwm(FEN_OUTPUT, rpmSetPoint);
        Timer1.pwm(HOT_OUTPUT, constrainsHotAir);
    } else {
        Timer1.pwm(HOT_OUTPUT, 0);
        if (airInput < 80) {
            Timer1.pwm(FEN_OUTPUT, 0);
        } else {
            Timer1.pwm(FEN_OUTPUT, 1024);
        }
    }


    unsigned long time = millis();
    if (time - loopPrint > LOOP_PRINT) {
        loopPrint = time;

#ifdef DEBUG

        //
        // DEBUG display
        //
        printSerial(airNow, sdrNow, showAirTemp, showSdrTemp);


#else


        //
        // LCD display
        //
        printScreen(showSdrTemp, showAirTemp, showAirRpm);
#endif
    }


}
/////////////////////////////////////////////////////////////////////////
////// Methods begin
/////////////////////////////////////////////////////////////////////////
/**
 * Detect button solder
 */
void buttonSolder() {
    if (digitalRead(SET_BUTTON_SOLDER) == LOW && debounceSolder + 350 < millis()) {
        delay(10);
        if (digitalRead(SET_BUTTON_SOLDER) == LOW) {
            if (!isSolderOn) {
#ifdef DEBUG
                Serial.println(F("SOLDER air ON "));
#endif
                isSolderUse = true;
                isSolderOn = true;
                isSleepSdrOn = false;
            } else {
#ifdef DEBUG
                Serial.println(F("SOLDER air OFF "));
#endif
                isSolderOn = false;
            }
            debounceSolder = millis();
        }
    }
}

/**
 * Detect hot air button
 */
void buttonHotAir() {
    if (digitalRead(SET_BUTTON_HOTAIR) == LOW && debounceHotAir + 350 < millis()) {
        delay(10);
        if (digitalRead(SET_BUTTON_HOTAIR) == LOW) {
            if (!isHotAirOn) {
#ifdef DEBUG
                Serial.println(F("Hot air ON "));
#endif
                isHotAirUse = true;
                isHotAirOn = true;
                isSleepAirOn = false;
            } else {
#ifdef DEBUG
                Serial.println(F("Hot air OFF "));
#endif
                isHotAirOn = false;

            }
            debounceHotAir = millis();
        }
    }
}


void standSolder() {
    //
    // Listen for sleep mode
    if (isSolderOn && digitalRead(SET_SOLDER_STANDS) == LOW) {

        if (isSolderUse) {
            standStartSolder = millis();
            isSolderUse = false;
        }

        if (!isSolderUse && millis() - standStartSolder > SLEEP_BEGIN_SDR) {
            isSleepSdrOn = true;
            isSolderUse = false;
            sdrSetPoint = SLEEP_TARGET_SDR;
        }
        if (millis() - standStartSolder > SLEEP_BEGIN_SDR * OFF_MULTIPLIER) {
            isSolderOn = false;
        }
    } else {
        isSolderUse = true;
        isSleepSdrOn = false;
    }
}

void standHotAir() {
    //
    // Listen for sleep mode
    if (isHotAirOn && digitalRead(SET_HOTAIR_STANDS) == LOW) {

        if (isHotAirUse) {
            standStartHotAir = millis();
            isHotAirUse = false;
        }

        if (!isHotAirUse && millis() - standStartHotAir > SLEEP_BEGIN_AIR) {
            isSleepAirOn = true;
            isHotAirUse = false;
            airSetPoint = SLEEP_TARGET_AIR;
        }
        if (millis() - standStartHotAir > SLEEP_BEGIN_AIR * OFF_MULTIPLIER) {
            isHotAirOn = false;
        }
    } else {
        isHotAirUse = true;
        isSleepAirOn = false;
    }
}

void inputSerial() {
#ifdef DEBUG
    if (Serial.available()) {
        String where = Serial.readStringUntil('=');
        if (where == F("dbg")) {
            dbg = (boolean) Serial.readStringUntil('\n').toInt();
        }

        if (where == F("sdr")) {
            sdrSetPoint = (uint16_t) Serial.readStringUntil('\n').toInt();
        }

        if (where == F("air")) {
            rpmSetPoint = (uint16_t) Serial.readStringUntil('\n').toInt();
        }

        if (where == F("hot")) {
            airSetPoint = (uint16_t) Serial.readStringUntil('\n').toInt();
        }
    }
#endif
}

void printScreen(uint16_t showSdrTemp, uint16_t showAirTemp, uint16_t showAirRpm) {
    //
    // Solder
#ifndef DEBUG
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print(F("SDR"));
    if (digitalRead(SET_SOLDER_STANDS) == LOW) {
        lcd.setCursor(3, 1);
        lcd.print("\2");
    }
    lcd.setCursor(4, 1);
    if (isSolderOn) {
        if (isSleepSdrOn) {
            lcd.print(F(" SLP "));
        } else {
            lcd.print((uint16_t) sdrSetPoint);
            lcd.print("\1");
        }
        lcd.setCursor(9, 1);

        lcd.print(showSdrTemp);
        lcd.print("\1");
    } else lcd.print(" OFF ");


    //
    // Hot air
    lcd.setCursor(0, 0);
    lcd.print(F("AIR"));
    if (digitalRead(SET_HOTAIR_STANDS) == LOW) {
        lcd.setCursor(3, 0);
        lcd.print("\2");
    }
    lcd.setCursor(4, 0);
    if (isHotAirOn) {
        if (isSleepAirOn) {
            lcd.print(F(" SLP "));
        } else {
            lcd.print((uint16_t) airSetPoint);
            lcd.print("\1");
        }
        lcd.setCursor(9, 0);
        lcd.print(showAirTemp);
        lcd.print("\1");
        lcd.setCursor(14, 0);
        lcd.print(showAirRpm);
    } else lcd.print(" OFF ");

#endif
}

/**
 *
 * @param airNow
 * @param sdrNow
 * @param showAirTemp
 * @param showSdrTemp
 */
void printSerial(unsigned long airNow, unsigned long sdrNow, uint16_t showAirTemp, uint16_t showSdrTemp) {


    //
    // HOT AIR
    Serial.print(F("AIR: now "));
    Serial.print(airNow);

    Serial.print(F(" out "));
    Serial.print(airOutput);

    Serial.print(F(" / show "));
    Serial.print(showAirTemp);

    Serial.print(F("*C / in "));
    Serial.print(airInput);
    Serial.print(F(" / tar "));
    Serial.print(airSetPoint);
    Serial.print(" | SND ");
    Serial.print(digitalRead(SET_HOTAIR_STANDS));
    if (isSleepAirOn) {
        Serial.print(" SLEEP ");
    }
    Serial.println('\n');

    //
    // IRON
    Serial.print(F("IRON: now "));
    Serial.print(sdrNow);
    Serial.print(F(" / out "));
    Serial.print(sdrOutput);
    Serial.print(F(" / show "));
    Serial.print(showSdrTemp);
    Serial.print(F("*C / in "));
    Serial.print(sdrInput);
    Serial.print(F(" / tar "));
    Serial.print(sdrSetPoint);
    Serial.print(" | SND ");
    Serial.print(digitalRead(SET_SOLDER_STANDS));
    if (isSleepSdrOn) {
        Serial.print(" SLEEP ");

    }
    Serial.println('\n');
}
