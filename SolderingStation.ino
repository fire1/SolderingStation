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
//TODO: Handle of 65hz:
//#define FREQ 50    // 50Hz power in these parts
#define FREQ 65    // 50Hz power in these parts

#define SDR_MIN  250
#define HOT_MIN  200
#define RPM_MIN  500

#define SDR_MAX  450
#define HOT_MAX  480
#define RPM_MAX  1021

//
// Sleep mode start
#define SLEEP_BEGIN 30000
//
// Shutdown mode start
#define CUTOFF_BEGIN 120000


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

#endif


boolean fenStart = false;
boolean isHotAirStand = false;
boolean isSolderStand = false;
boolean isSleepAirOn = false;
boolean isSleepSdrOn = false;

const uint8_t SDR_INPUT = A5; // A5
const uint8_t AIR_INPUT = A4;
const uint8_t SDR_OUTPUT = 11;
const uint8_t FEN_OUTPUT = 9; // fen output
const uint8_t HOT_OUTPUT = 10; // fen output

//
// Hardware interface
const uint8_t SET_SOLDER_TARGET = A1;
const uint8_t SET_AIRRPM_TARGET = A2;
const uint8_t SET_HOTAIR_TARGET = A3;
const uint8_t SET_HOTAIR_STANDS = A0;

const uint8_t SET_BUTTON_HOTAIR = 7;
const uint8_t SET_BUTTON_SOLDER = 8;

uint16_t rpmSetPoint = 1024;


unsigned long int halfSineTime = 1000000 / (2 * FREQ);//The Timerone PWM period, 50Hz = 10000 uS


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
const double aggKp = 8, aggKi = 1, aggKd = 1;
const double consKp = 2, consKi = 0.05, consKd = 0.25;
double airSetPoint, airInput, airOutput;
PID airPID(&airInput, &airOutput, &airSetPoint, consKp, consKi, consKd, DIRECT);

//
// Solder
double sdrSetPoint, sdrInput, sdrOutput;
PID sdrPID(&sdrInput, &sdrOutput, &sdrSetPoint, 2, 1, 1, DIRECT);


unsigned long debounceSolder, debounceHotAir;
boolean isSolderOn = false, isHotAirOn = false, isHotAirUse = false;

/**
 * Detect button solder
 */
void ButtonSolder() {
    if (digitalRead(SET_BUTTON_SOLDER) == LOW && debounceSolder + 350 < millis()) {
        delay(10);
        if (digitalRead(SET_BUTTON_SOLDER) == LOW) {
            if (!isSolderOn) {
#ifdef DEBUG
                Serial.println("SOLDER air ON ");
#endif
                isSolderOn = true;
            } else {
#ifdef DEBUG
                Serial.println("SOLDER air OFF ");
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
void ButtonHotAir() {
    if (digitalRead(SET_BUTTON_HOTAIR) == LOW && debounceHotAir + 350 < millis()) {
        delay(10);
        if (digitalRead(SET_BUTTON_HOTAIR) == LOW) {
            if (!isHotAirOn) {
#ifdef DEBUG
                Serial.println("Hot air ON ");
#endif
                isHotAirOn = true;
            } else {
#ifdef DEBUG
                Serial.println("Hot air OFF ");
#endif
                isHotAirOn = false;
                isHotAirStand = false; // disable sleep mode
            }
            debounceHotAir = millis();
        }
    }
}


void setup() {

    Timer1.initialize(halfSineTime);

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
    lcd.setCursor(0, 0);
    lcd.print(" SOLDER STATION ");
    lcd.setCursor(0, 1);
    lcd.print(" ver.1 ");
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

void loop() {


//#ifndef DEBUG
    //
    // My inputs are inverted ...
    sdrSetPoint = (isSolderOn) ? (uint16_t) map(analogRead(SET_SOLDER_TARGET), 1021, 0, SDR_MIN, SDR_MAX) : 0;
    airSetPoint = (isHotAirOn) ? (uint16_t) map(analogRead(SET_HOTAIR_TARGET), 1021, 0, HOT_MIN, HOT_MAX) : 0;
    rpmSetPoint = (isHotAirOn) ? (uint16_t) map(analogRead(SET_AIRRPM_TARGET), 1021, 0, RPM_MIN, RPM_MAX) : 0;

//
// Extra debugging
//        Serial.print("TARGETS sdr ");
//        Serial.print(sdrSetPoint);
//        Serial.print(" air ");
//        Serial.print(airSetPoint);
//        Serial.print(" rpm ");
//        Serial.print(rpmSetPoint);
//        Serial.println();

//#else
//    isHotAirOn = true;
//    isSolderOn = true;
//#endif



    /*
     * Temperature resolver
     */
    unsigned long airNow = analogRead(AIR_INPUT);
    unsigned long sdrNow = analogRead(SDR_INPUT);
    for (uint8_t ms = 1; ms < READ_SENSOR; ++ms) {
        airNow += analogRead(AIR_INPUT);
        sdrNow += analogRead(SDR_INPUT);
        delayMicroseconds(1);
    }

    airNow = airNow / READ_SENSOR;
    sdrNow = sdrNow / READ_SENSOR;


    //
    // Resolve solder temperature
    uint16_t currentSolderTemp;
    if (airNow < 350) {
        currentSolderTemp = (uint16_t) map(sdrNow, 0, 350, 0, 250);
    } else {
        currentSolderTemp = (uint16_t) map(sdrNow, 350, 700, 250, 480);
    }

    //
    // Resolve hot air temperature
    uint16_t currentHotAirTemp;
    if (airNow < 20) {
        currentHotAirTemp = (uint16_t) map(airNow, 0, 20, 0, 100);
    } else {
        currentHotAirTemp = (uint16_t) map(airNow, 20, 183, 100, 450);
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
    if (loopIndex > 3) {
        airTemp = (airTemp / loopIndex);
        sdrTemp = (sdrTemp / loopIndex);
        loopIndex = 1;
    }


    ButtonSolder();
    ButtonHotAir();

    //
    // Stands
    if (digitalRead(SET_HOTAIR_STANDS) == HIGH && isHotAirOn && !isHotAirUse) {
        isHotAirUse = true;
    } else if (digitalRead(SET_HOTAIR_STANDS) == LOW && isHotAirUse && isHotAirOn) {
        isHotAirOn = false;
        isHotAirUse = false;
        isHotAirStand = true;
        standStartHotAir = millis();
    }

    //
    // Listen for stand
    if (isHotAirStand && digitalRead(SET_HOTAIR_STANDS) == HIGH) {
        isSleepAirOn = false;
        isHotAirOn = true;
        standStartHotAir = millis(); //standStartHotAir = 0;
    }

    //
    // Listen for sleep mode
    if (isHotAirStand && digitalRead(SET_HOTAIR_STANDS) == LOW) {
        if (millis() - standStartHotAir > SLEEP_BEGIN) {
            isSleepAirOn = true;
            isHotAirOn = false;
        }
    }

    //
    // Turn off air by reseting the stand and sleep
    if(isSleepAirOn && millis() - standStartHotAir > CUTOFF_BEGIN){
        isSleepAirOn = false;
        isHotAirOn = false;
        isHotAirStand = false;
    }

#ifdef DEBUG
    if (Serial.available()) {
        String where = Serial.readStringUntil('=');
        if (where == "dbg") {
            dbg = (boolean) Serial.readStringUntil('\n').toInt();
        }

        if (where == "sdr") {
            sdrSetPoint = (uint16_t) Serial.readStringUntil('\n').toInt();
        }

        if (where == "air") {
            rpmSetPoint = (uint16_t) Serial.readStringUntil('\n').toInt();
        }

        if (where == "hot") {
            airSetPoint = (uint16_t) Serial.readStringUntil('\n').toInt();
        }
    }
#endif

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


    double gap = abs(airSetPoint - airInput); //distance away from setpoint
    if (gap < 10) {
        //we're close to set point, use conservative tuning parameters
        airPID.SetTunings(consKp, consKi, consKd);
    } else {
        //we're far from set point, use aggressive tuning parameters
        airPID.SetTunings(aggKp, aggKi, aggKd);
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


        //
        // HOT AIR
        Serial.print("AIR: now ");
        Serial.print(airNow);

        Serial.print(" do ");
        Serial.print(airOutput);

        Serial.print(" / HOT: ");
        Serial.print(showAirTemp);

        Serial.print("*C | in ");
        Serial.print(airInput);
        Serial.print(" | tar ");
        Serial.print(airSetPoint);
        Serial.println('\n');

        //
        // IRON
        Serial.print("IRON: now ");
        Serial.print(sdrNow);
        Serial.print(" / out ");
        Serial.print(sdrOutput);
        Serial.print(" / show ");
        Serial.print(showSdrTemp);
        Serial.print("*C in ");
        Serial.print(sdrInput);
        Serial.print(" / tar ");
        Serial.print(sdrSetPoint);
        Serial.println('\n');



#else


        //
        // LCD display
        //


        //
        // Solder
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print(F("SDR"));
        lcd.setCursor(4, 1);
        if (isSolderOn) {
            lcd.print((uint16_t) sdrSetPoint);
            lcd.print("\1");
            lcd.setCursor(9, 1);
            lcd.print(showSdrTemp);
            lcd.print("\1");
        } else lcd.print(" OFF ");


        //
        // Hot air
        lcd.setCursor(0, 0);
        lcd.print(F("AIR"));
        lcd.setCursor(4, 0);
        if (isHotAirOn) {
            lcd.print((uint16_t) airSetPoint);
            lcd.print("\1");
            lcd.setCursor(9, 0);
            lcd.print(showAirTemp);
            lcd.print("\1");
            lcd.setCursor(14, 0);
            lcd.print(showAirRpm);
        } else {
            (isHotAirStand) ? lcd.print(" SLEEP ") : lcd.print(" OFF ");
        }


#endif
    }


}



