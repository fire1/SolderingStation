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
#define DEBUG

//
//TODO: Handle of 65hz:
//#define FREQ 50    // 50Hz power in these parts
#define FREQ 65    // 50Hz power in these parts

#define SDR_MIN  300
#define HOT_MIN  300
#define RPM_MIN  440

#define SDR_MAX  450
#define HOT_MAX  520
#define RPM_MAX  1021

//
// Sleep mode start
#define SLEEP_BEGIN_AIR 120000
#define SLEEP_BEGIN_SDR 120000

#define SLEEP_SETPOINT_AIR 100
#define SLEEP_SETPOINT_SDR 200

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

/**
 * Methods definition
 */
static void ButtonSolder();

static void ButtonHotAir();

static void StandSolder();

static void StandHotAir();

static void DebugSerial();

boolean fenStart = false;
boolean isSleepAirOn = false;
boolean isSleepSdrOn = false;
boolean isSolderUse = true;
boolean isSolderOn = false, isHotAirOn = false, isHotAirUse = true;

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

const uint8_t SET_BUTTON_HOTAIR = 7;
const uint8_t SET_BUTTON_SOLDER = 8;

//
// Stands
const uint8_t SET_HOTAIR_STANDS = A0;
const uint8_t SET_SOLDER_STANDS = 6;


uint16_t rpmSetPoint = 1024;


//unsigned long int halfSineTime = 1000000 / (2 * FREQ);//The Timerone PWM period, 50Hz = 10000 uS
unsigned long int halfSineTime = 8;//The Timerone PWM period, 50Hz = 10000 uS


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
    if (airNow < 430) {
        currentSolderTemp = (uint16_t) map(sdrNow, 0, 430, 0, 330);
    } else {
        currentSolderTemp = (uint16_t) map(sdrNow, 430, 550, 330, 425);
    }

    //
    // Resolve hot air temperature
    uint16_t currentHotAirTemp;
    if (airNow < 90) {
        currentHotAirTemp = (uint16_t) map(airNow, 0, 90, 0, 320);
    } else {
        currentHotAirTemp = (uint16_t) map(airNow, 90, 160, 320, 430);
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
    StandSolder();
    StandHotAir();
    DebugSerial();

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


        //
        // HOT AIR
        Serial.print(F("AIR: now "));
        Serial.print(airNow);

        Serial.print(F(" do "));
        Serial.print(airOutput);

        Serial.print(F(" / HOT: "));
        Serial.print(showAirTemp);

        Serial.print(F("*C | in "));
        Serial.print(airInput);
        Serial.print(F(" | tar "));
        Serial.print(airSetPoint);
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
        Serial.print(F("*C in "));
        Serial.print(sdrInput);
        Serial.print(F(" / tar "));
        Serial.print(sdrSetPoint);
        Serial.print(" | SND ");
        Serial.print(digitalRead(SET_SOLDER_STANDS));
        if (isSleepSdrOn) {
            Serial.print(" SLEEP ");

        }
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


}
/////////////////////////////////////////////////////////////////////////
////// Methods begin
/////////////////////////////////////////////////////////////////////////
/**
 * Detect button solder
 */
void ButtonSolder() {
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
void ButtonHotAir() {
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


void StandSolder() {
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
            sdrSetPoint = SLEEP_SETPOINT_SDR;
            if (millis() - standStartSolder > SLEEP_BEGIN_SDR * 3) {
                isSolderOn = false;
            }
        }
    }
    //
    // Listen for stand
    if (isSolderOn && digitalRead(SET_SOLDER_STANDS) == HIGH) {
        isSolderUse = true;
        isSleepSdrOn = false;
    }
}

void StandHotAir() {
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
            airSetPoint = SLEEP_SETPOINT_AIR;
            if (millis() - standStartHotAir > SLEEP_BEGIN_AIR * 3) {
                isHotAirOn = false;
            }
        }
    }
    //
    // Listen for stand
    if (isHotAirOn && digitalRead(SET_HOTAIR_STANDS) == HIGH) {
        isHotAirUse = true;
        isSleepAirOn = false;
    }
}


void DebugSerial() {
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
