//
// Created by Admin on 3/21/2020.
//

#ifndef SOLDERINGSTATION_AIR_H_H
#define SOLDERINGSTATION_AIR_H_H

#include <Arduino.h>

/*
 * Hot air gun controller based on atmega328 IC
 * Version 1.33
 * Released Jan 24, 2020
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <CommonControls.h>
#include <EEPROM.h>
#include <SPI.h>

const uint16_t temp_minC 	= 100;
const uint16_t temp_maxC	= 500;
const uint16_t temp_ambC    = 25;
const uint16_t temp_tip[3] = {200, 300, 400};                               // Temperature reference points for calibration
const uint16_t min_working_fan = 100;                                       // Minimal possible fan speed

const uint8_t AC_SYNC_PIN   = 2;                                            // Outlet 220 v synchronization pin. Do not change!
const uint8_t HOT_GUN_PIN   = 7;                                            // Hot gun heater management pin
const uint8_t FAN_GUN_PIN   = 9;                                            // Hot gun fan management pin. Do not change!
const uint8_t TEMP_GUN_PIN	= A0;                                           // Hot gun temperature checking pin

const uint8_t R_MAIN_PIN	= 3;                                            // Rotary encoder main pin. Do not change!
const uint8_t R_SECD_PIN	= 4;                                            // Rotary encoder secondary pin
const uint8_t R_BUTN_PIN	= 5;                                            // Rotary encoder button pin

const uint8_t REED_SW_PIN   = 8;                                            // Reed switch pin
const uint8_t BUZZER_PIN	= 6;                                            // Buzzer pin

//------------------------------------------ Configuration data ------------------------------------------------
/* Config record in the EEPROM has the following format:
 * uint32_t ID                           each time increment by 1
 * struct cfg                            config data, 8 bytes
 * byte CRC                              the checksum
*/
struct cfg {
    uint32_t    calibration;                                                // Packed calibration data by three temperature points
    uint16_t    temp;                                                       // The preset temperature of the IRON in internal units
    uint8_t     fan;                                                        // The preset fan speed 0 - 255
    uint8_t     off_timeout;                                                // Automatic switch-off timeout
};

class CONFIG {
public:
    CONFIG() {
        can_write     = false;
        buffRecords   = 0;
        rAddr = wAddr = 0;
        eLength       = 0;
        nextRecID     = 0;
        uint8_t rs = sizeof(struct cfg) + 5;                             // The total config record size
        // Select appropriate record size; The record size should be power of 2, i.e. 8, 16, 32, 64, ... bytes
        for (record_size = 8; record_size < rs; record_size <<= 1);
    }
    void init();
    bool load(void);
    void getConfig(struct cfg &Cfg);                                    // Copy config structure from this class
    void updateConfig(struct cfg &Cfg);                                 // Copy updated config into this class
    bool save(void);                                                    // Save current config copy to the EEPROM
    bool saveConfig(struct cfg &Cfg);                                   // write updated config into the EEPROM

protected:
    struct   cfg Config;

private:
    bool     readRecord(uint16_t addr, uint32_t &recID);
    bool     can_write;                                                 // The flag indicates that data can be saved
    uint8_t  buffRecords;                                               // Number of the records in the outpt buffer
    uint16_t rAddr;                                                     // Address of thecorrect record in EEPROM to be read
    uint16_t wAddr;                                                     // Address in the EEPROM to start write new record
    uint16_t eLength;                                                   // Length of the EEPROM, depends on arduino model
    uint32_t nextRecID;                                                 // next record ID
    uint8_t  record_size;                                               // The size of one record in bytes
};

// Read the records until the last one, point wAddr (write address) after the last record
void CONFIG::init(void) {
    eLength = EEPROM.length();
    uint32_t recID;
    uint32_t minRecID = 0xffffffff;
    uint16_t minRecAddr = 0;
    uint32_t maxRecID = 0;
    uint16_t maxRecAddr = 0;
    uint8_t  records = 0;

    nextRecID = 0;

    // read all the records in the EEPROM find min and max record ID
    for (uint16_t addr = 0; addr < eLength; addr += record_size) {
        if (readRecord(addr, recID)) {
            ++records;
            if (minRecID > recID) {
                minRecID = recID;
                minRecAddr = addr;
            }
            if (maxRecID < recID) {
                maxRecID = recID;
                maxRecAddr = addr;
            }
        } else {
            break;
        }
    }

    if (records == 0) {
        wAddr = rAddr = 0;
        can_write = true;
        return;
    }

    rAddr = maxRecAddr;
    if (records < (eLength / record_size)) {                                // The EEPROM is not full
        wAddr = rAddr + record_size;
        if (wAddr > eLength) wAddr = 0;
    } else {
        wAddr = minRecAddr;
    }
    can_write = true;
}

void CONFIG::getConfig(struct cfg &Cfg) {
    memcpy(&Cfg, &Config, sizeof(struct cfg));
}

void CONFIG::updateConfig(struct cfg &Cfg) {
    memcpy(&Config, &Cfg, sizeof(struct cfg));
}

bool CONFIG::saveConfig(struct cfg &Cfg) {
    updateConfig(Cfg);
    return save();                                                          // Save new data into the EEPROM
}

bool CONFIG::save(void) {
    if (!can_write) return can_write;
    if (nextRecID == 0) nextRecID = 1;

    uint16_t startWrite = wAddr;
    uint32_t nxt = nextRecID;
    uint8_t summ = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        EEPROM.write(startWrite++, nxt & 0xff);
        summ <<=2; summ += nxt;
        nxt >>= 8;
    }
    uint8_t* p = (byte *)&Config;
    for (uint8_t i = 0; i < sizeof(struct cfg); ++i) {
        summ <<= 2; summ += p[i];
        EEPROM.write(startWrite++, p[i]);
    }
    summ ++;                                                                // To avoid empty records
    EEPROM.write(wAddr+record_size-1, summ);

    rAddr = wAddr;
    wAddr += record_size;
    if (wAddr > EEPROM.length()) wAddr = 0;
    nextRecID ++;                                                           // Get ready to write next record
    return true;
}

bool CONFIG::load(void) {
    bool is_valid = readRecord(rAddr, nextRecID);
    nextRecID ++;
    return is_valid;
}

bool CONFIG::readRecord(uint16_t addr, uint32_t &recID) {
    uint8_t Buff[record_size];

    for (uint8_t i = 0; i < record_size; ++i)
        Buff[i] = EEPROM.read(addr+i);

    uint8_t summ = 0;
    for (byte i = 0; i < sizeof(struct cfg) + 4; ++i) {
        summ <<= 2; summ += Buff[i];
    }
    summ ++;                                                                // To avoid empty fields
    if (summ == Buff[record_size-1]) {                                      // Checksumm is correct
        uint32_t ts = 0;
        for (char i = 3; i >= 0; --i) {
            ts <<= 8;
            ts |= Buff[byte(i)];
        }
        recID = ts;
        memcpy(&Config, &Buff[4], sizeof(struct cfg));
        return true;
    }
    return false;
}

//------------------------------------------ class HOT GUN CONFIG ----------------------------------------------
class HOTGUN_CFG : public CONFIG {
public:
    HOTGUN_CFG()                                                        { }
    void     init(void);
    uint16_t tempPreset(void);                                          // The preset temperature in internal units
    uint8_t	 fanPreset(void);                                           // The preset fan speed 0 - 255
    uint16_t tempInternal(uint16_t temp);                               // Translate the human readable temperature into internal value
    uint16_t tempHuman(uint16_t temp);                                  // Translate temperature from internal units to the Celsius
    void     save(uint16_t temp, uint8_t fanSpeed);                     // Save preset temperature in the internal units and fan speed
    void     applyCalibrationData(uint16_t tip[3]);
    void     getCalibrationData(uint16_t tip[3]);
    void     saveCalibrationData(uint16_t tip[3]);
    void     setDefaults(bool Write);                                   // Set default parameter values if failed to load data from EEPROM
private:
    uint16_t t_tip[3];
    const   uint16_t def_tip[3] = {587, 751, 850};                      // Default values of internal sensor readings at reference temperatures
    const   uint16_t min_temp  = 50;
    const   uint16_t max_temp  = 900;
    const   uint16_t def_temp  = 600;                                   // Default preset temperature
    const   uint8_t  def_fan   = 64;                                  	// Default preset fan speed 0 - 255
    const   uint16_t ambient_temp = 0;
    const   uint16_t ambient_tempC= 25;
};

void HOTGUN_CFG::init(void) {
    CONFIG::init();
    if (!CONFIG::load()) setDefaults(false);                                // If failed to load the data from EEPROM, initialize the config data with the default values
    uint32_t   cd = Config.calibration;
    t_tip[0] = cd & 0x3FF; cd >>= 10;                                       // 10 bits per calibration parameter, because the ADC readings are 10 bits
    t_tip[1] = cd & 0x3FF; cd >>= 10;
    t_tip[2] = cd & 0x3FF;
    // Check the tip calibration is correct
    if ((t_tip[0] >= t_tip[1]) || (t_tip[1] >= t_tip[2])) {
        setDefaults(false);
        for (uint8_t i = 0; i < 3; ++i)
            t_tip[i] = def_tip[i];
    }
    return;
}
uint32_t    calibration;                                                // Packed calibration data by three temperature points
uint16_t    temp;                                                       // The preset temperature of the IRON in internal units
uint8_t     fan;                                                        // The preset fan speed 0 - 255
uint8_t     off_timeout;                                                // Automatic switch-off timeout

uint16_t HOTGUN_CFG::tempPreset(void) {
    return Config.temp;
}

uint8_t HOTGUN_CFG::fanPreset(void) {
    return Config.fan;
}

uint16_t HOTGUN_CFG::tempInternal(uint16_t t) {                             // Translate the human readable temperature into internal value
    t = constrain(t, temp_minC, temp_maxC);
    uint16_t left   = 0;
    uint16_t right  = 1023;                                                 // Maximum temperature value in internal units
    uint16_t temp = map(t, temp_tip[0], temp_tip[2], t_tip[0], t_tip[2]);

    if (temp > (left+right)/ 2) {
        temp -= (right-left) / 4;
    } else {
        temp += (right-left) / 4;
    }

    for (uint8_t i = 0; i < 20; ++i) {
        uint16_t tempH = tempHuman(temp);
        if (tempH == t) {
            return temp;
        }
        uint16_t new_temp;
        if (tempH < t) {
            left = temp;
            new_temp = (left+right)/2;
            if (new_temp == temp)
                new_temp = temp + 1;
        } else {
            right = temp;
            new_temp = (left+right)/2;
            if (new_temp == temp)
                new_temp = temp - 1;
        }
        temp = new_temp;
    }
    return temp;
}

// Thanslate temperature from internal units to the human readable value (Celsius or Fahrenheit)
uint16_t HOTGUN_CFG::tempHuman(uint16_t temp) {
    uint16_t tempH = 0;

    if (temp <= ambient_temp) {
        tempH = ambient_tempC;
    } else if (temp < t_tip[0]) {
        tempH = map(temp, ambient_temp, t_tip[0], ambient_tempC, temp_tip[0]);
    } else if (temp <= t_tip[1]) {
        tempH = map(temp, t_tip[0], t_tip[1], temp_tip[0], temp_tip[1]);
    } else if (temp <= t_tip[2]) {
        tempH = map(temp, t_tip[1], t_tip[2], temp_tip[1], temp_tip[2]);
    } else {
        tempH = map(temp, t_tip[0], t_tip[2], temp_tip[0], temp_tip[2]);
    }
    return tempH;
}

void HOTGUN_CFG::save(uint16_t temp, uint8_t fanSpeed) {
    Config.temp        = constrain(temp, min_temp, max_temp);
    Config.fan         = fanSpeed;
    CONFIG::save();                                                         // Save new data into the EEPROM
}

void HOTGUN_CFG::applyCalibrationData(uint16_t tip[3]) {
    if (tip[0] < ambient_temp) {
        uint16_t t = ambient_temp + tip[1];
        tip[0] = t >> 1;
    }
    t_tip[0] = tip[0];
    t_tip[1] = tip[1];
    if (tip[2] > max_temp) tip[2] = max_temp;
    t_tip[2] = tip[2];
}

void HOTGUN_CFG::getCalibrationData(uint16_t tip[3]) {
    tip[0] = t_tip[0];
    tip[1] = t_tip[1];
    tip[2] = t_tip[2];
}

void HOTGUN_CFG::saveCalibrationData(uint16_t tip[3]) {
    if (tip[2] > max_temp) tip[2] = max_temp;
    uint32_t cd = tip[2] & 0x3FF; cd <<= 10;                                // Pack tip calibration data in one 32-bit word: 10-bits per value
    cd |= tip[1] & 0x3FF; cd <<= 10;
    cd |= tip[0];
    Config.calibration = cd;
    t_tip[0] = tip[0];
    t_tip[1] = tip[1];
    t_tip[2] = tip[2];
}

void HOTGUN_CFG::setDefaults(bool Write) {
    uint32_t c = def_tip[2] & 0x3FF; c <<= 10;
    c |= def_tip[1] & 0x3FF;         c <<= 10;
    c |= def_tip[0] & 0x3FF;
    Config.calibration = c;
    Config.temp        = def_temp;
    Config.fan         = def_fan;
    if (Write) {
        CONFIG::save();
    }
}

//------------------------------------------ class BUZZER ------------------------------------------------------
class BUZZER {
public:
    BUZZER(byte buzzerP)  { buzzer_pin = buzzerP; }
    void init(void);
    void shortBeep(void)  { tone(buzzer_pin, 3520, 160); }
    void lowBeep(void)    { tone(buzzer_pin,  880, 160); }
    void doubleBeep(void) { tone(buzzer_pin, 3520, 160); delay(300); tone(buzzer_pin, 3520, 160); }
    void failedBeep(void) { tone(buzzer_pin, 3520, 160); delay(170);
        tone(buzzer_pin,  880, 250); delay(260);
        tone(buzzer_pin, 3520, 160);
    }
private:
    byte buzzer_pin;
};

void BUZZER::init(void) {
    pinMode(buzzer_pin, OUTPUT);
    noTone(buzzer_pin);
}

//------------------------------------------ class lcd DSPLay for soldering IRON -----------------------------
class DSPL : protected LiquidCrystal_I2C {
public:
    DSPL(void) : LiquidCrystal_I2C(0x27, 16, 2) { }
    void    init(void);
    void    clear(void)                                                 { LiquidCrystal_I2C::clear(); }
    void    tSet(uint16_t t, bool Celsius = true);                      // Show the preset temperature
    void    tCurr(uint16_t t);                                          // Show the current temperature
    void    tInternal(uint16_t t);                                      // Show the current temperature in internal units
    void    tReal(uint16_t t);                                          // Show the real temperature in Celsius in calibrate mode
    void    fanSpeed(uint8_t s);                                        // Show the fan speed
    void	appliedPower(uint8_t p, bool show_zero = true);			    // Show applied power (%)
    void    setupMode(uint8_t mode);
    void    msgON(void);                                                // Show message: "ON"
    void    msgOFF(void);
    void    msgReady(void);
    void    msgCold(void);
    void    msgFail(void);                                              // Show 'Fail' message
    void    msgTune(void);                                              // Show 'Tune' message
private:
    bool 	full_second_line;                                           // Whether the second line is full with the message
    char 	temp_units;
    const   uint8_t custom_symbols[3][8] = {
            { 0b00110,                                        // Degree
                    0b01001,
                    0b01001,
                    0b00110,
                    0b00000,
                    0b00000,
                    0b00000,
                    0b00000
            },
            { 0b00100,                                        // Fan sign
                    0b01100,
                    0b01100,
                    0b00110,
                    0b01011,
                    0b11001,
                    0b10000,
                    0b00000
            },
            { 0b00011,                                        // Power sign
                    0b00110,
                    0b01100,
                    0b11111,
                    0b00110,
                    0b01100,
                    0b01000,
                    0b10000
            }
    };
};

void DSPL::init(void) {
    LiquidCrystal_I2C::begin();
    LiquidCrystal_I2C::clear();
    for (uint8_t i = 0; i < 3; ++i)
        LiquidCrystal_I2C::createChar(i+1, (uint8_t *)custom_symbols[i]);
    full_second_line = false;
    temp_units = 'C';
}

void DSPL::tSet(uint16_t t, bool Celsius) {
    char buff[10];
    if (Celsius) {
        temp_units = 'C';
    } else {
        temp_units = 'F';
    }
    LiquidCrystal_I2C::setCursor(0, 0);
    sprintf(buff, "Set:%3d%c%c", t, (char)1, temp_units);
    LiquidCrystal_I2C::print(buff);
}

void DSPL::tCurr(uint16_t t) {
    char buff[6];
    LiquidCrystal_I2C::setCursor(0, 1);
    if (t < 1000) {
        sprintf(buff, "%3d%c ", t, (char)1);
    } else {
        LiquidCrystal_I2C::print(F("xxx"));
        return;
    }
    LiquidCrystal_I2C::print(buff);
    if (full_second_line) {
        LiquidCrystal_I2C::print(F("           "));
        full_second_line = false;
    }
}

void DSPL::tInternal(uint16_t t) {
    char buff[6];
    LiquidCrystal_I2C::setCursor(0, 1);
    if (t < 1023) {
        sprintf(buff, "%4d ", t);
    } else {
        LiquidCrystal_I2C::print(F("xxxx"));
        return;
    }
    LiquidCrystal_I2C::print(buff);
    if (full_second_line) {
        LiquidCrystal_I2C::print(F("           "));
        full_second_line = false;
    }
}

void DSPL::tReal(uint16_t t) {
    char buff[6];
    LiquidCrystal_I2C::setCursor(11, 1);
    if (t < 1000) {
        sprintf(buff, ">%3d%c", t, (char)1);
    } else {
        LiquidCrystal_I2C::print(F("xxx"));
        return;
    }
    LiquidCrystal_I2C::print(buff);
}

void DSPL::fanSpeed(uint8_t s) {
    char buff[6];
    s = map(s, 0, 255, 0, 99);
    sprintf(buff, " %c%2d%c", (char)2, s, '%');
    LiquidCrystal_I2C::setCursor(11, 1);
    LiquidCrystal_I2C::print(buff);
}

void DSPL::appliedPower(uint8_t p, bool show_zero) {
    char buff[6];
    if (p > 99) p = 99;
    LiquidCrystal_I2C::setCursor(5, 1);
    if (p == 0 && !show_zero) {
        LiquidCrystal_I2C::print(F("     "));
    } else {
        sprintf(buff, " %c%2d%c", (char)3, p, '%');
        LiquidCrystal_I2C::print(buff);
    }
}

void DSPL::setupMode(byte mode) {
    LiquidCrystal_I2C::clear();
    LiquidCrystal_I2C::print(F("setup"));
    LiquidCrystal_I2C::setCursor(1,1);
    switch (mode) {
        case 0:                                                             // tip calibrate
            LiquidCrystal_I2C::print(F("calibrate"));
            break;
        case 1:                                                             // tune
            LiquidCrystal_I2C::print(F("tune"));
            break;
        case 2:                                                             // save
            LiquidCrystal_I2C::print(F("save"));
            break;
        case 3:                                                             // cancel
            LiquidCrystal_I2C::print(F("cancel"));
            break;
        case 4:                                                             // set defaults
            LiquidCrystal_I2C::print(F("reset config"));
            break;
        default:
            break;
    }
}

void DSPL::msgON(void) {
    LiquidCrystal_I2C::setCursor(10, 0);
    LiquidCrystal_I2C::print(F("    ON"));
}

void DSPL::msgOFF(void) {
    LiquidCrystal_I2C::setCursor(10, 0);
    LiquidCrystal_I2C::print(F("   OFF"));
}


void DSPL::msgReady(void) {
    LiquidCrystal_I2C::setCursor(10, 0);
    LiquidCrystal_I2C::print(F(" Ready"));
}

void DSPL::msgCold(void) {
    LiquidCrystal_I2C::setCursor(10, 0);
    LiquidCrystal_I2C::print(F("  Cold"));
}

void DSPL::msgFail(void) {
    LiquidCrystal_I2C::setCursor(0, 1);
    LiquidCrystal_I2C::print(F(" -== Failed ==- "));
}

void DSPL::msgTune(void) {
    LiquidCrystal_I2C::setCursor(0, 0);
    LiquidCrystal_I2C::print(F("Tune"));
}

//------------------------------------------ class HISTORY ----------------------------------------------------
#define H_LENGTH 16
class HISTORY {
public:
    HISTORY(void)                               						{ len = 0; }
    void     init(void)                         						{ len = 0; }
    uint16_t last(void);
    uint16_t top(void)                          						{ return queue[0]; }
    void     put(uint16_t item);                						// Put new entry to the history
    uint16_t average(void);                     						// calculate the average value
    float    dispersion(void);                                          // calculate the math dispersion
private:
    volatile uint16_t queue[H_LENGTH];
    volatile byte len;                          						// The number of elements in the queue
    volatile byte index;                        						// The current element position, use ring buffer
};

void HISTORY::put(uint16_t item) {
    if (len < H_LENGTH) {
        queue[len++] = item;
    } else {
        queue[index ] = item;
        if (++index >= H_LENGTH) index = 0;         						// Use ring buffer
    }
}

uint16_t HISTORY::last(void) {
    if (len == 0) return 0;
    uint8_t i = len - 1;
    if (index)
        i = index - 1;
    return queue[i];
}

uint16_t HISTORY::average(void) {
    uint32_t sum = 0;
    if (len == 0) return 0;
    if (len == 1) return queue[0];
    for (uint8_t i = 0; i < len; ++i) sum += queue[i];
    sum += len >> 1;                              							// round the average
    sum /= len;
    return uint16_t(sum);
}

float HISTORY::dispersion(void) {
    if (len < 3) return 1000;
    uint32_t sum = 0;
    uint32_t avg = average();
    for (uint8_t i = 0; i < len; ++i) {
        long q = queue[i];
        q -= avg;
        q *= q;
        sum += q;
    }
    sum += len << 1;
    float d = (float)sum / (float)len;
    return d;
}

//------------------------------------------ class PID algoritm to keep the temperature -----------------------
/*  The PID algorithm
 *  Un = Kp*(Xs - Xn) + Ki*summ{j=0; j<=n}(Xs - Xj) + Kd(Xn - Xn-1),
 *  Where Xs - is the setup temperature, Xn - the temperature on n-iteration step
 *  In this program the interactive formula is used:
 *    Un = Un-1 + Kp*(Xn-1 - Xn) + Ki*(Xs - Xn) + Kd*(Xn-2 + Xn - 2*Xn-1)
 *  With the first step:
 *  U0 = Kp*(Xs - X0) + Ki*(Xs - X0); Xn-1 = Xn;
 *
 *  PID coefficients history:
 *  10/14/2017  [768, 32, 328]
 *  11/27/2019  [ 2009, 1600, 20]
 */
class PID {
public:
    PID(void) {
        Kp = 2009;
        Ki = 1600;
        Kd =   20;
    }
    void resetPID(int temp = -1);                                       // reset PID algorithm history parameters
    // Calculate the power to be applied
    long reqPower(int temp_set, int temp_curr);
    int  changePID(uint8_t p, int k);                                   // set or get (if parameter < 0) PID parameter
private:
    void  debugPID(int t_set, int t_curr, long kp, long ki, long kd, long delta_p);
    int   temp_h0, temp_h1;                                             // previously measured temperature
    bool  pid_iterate;                                                  // Whether the iterative process is used
    long  i_summ;                                                       // Ki summary multiplied by denominator
    long  power;                                                        // The power iterative multiplied by denominator
    long  Kp, Ki, Kd;                                                   // The PID algorithm coefficients multiplied by denominator
    const byte denominator_p = 11;                                      // The common coefficient denominator power of 2 (11 means divide by 2048)
};

void PID::resetPID(int temp) {
    temp_h0 = 0;
    power  = 0;
    i_summ = 0;
    pid_iterate = false;
    if ((temp > 0) && (temp < 1000))
        temp_h1 = temp;
    else
        temp_h1 = 0;
}

int PID::changePID(uint8_t p, int k) {
    switch(p) {
        case 1:
            if (k >= 0) Kp = k;
            return Kp;
        case 2:
            if (k >= 0) Ki = k;
            return Ki;
        case 3:
            if (k >= 0) Kd = k;
            return Kd;
        default:
            break;
    }
    return 0;
}

long PID::reqPower(int temp_set, int temp_curr) {
    if (temp_h0 == 0) {
        // When the temperature is near the preset one, reset the PID and prepare iterative formula
        if ((temp_set - temp_curr) < 30) {
            if (!pid_iterate) {
                pid_iterate = true;
                power = 0;
                i_summ = 0;
            }
        }
        i_summ += temp_set - temp_curr;                                     // first, use the direct formula, not the iterate process
        power = Kp*(temp_set - temp_curr) + Ki*i_summ;
        // If the temperature is near, prepare the PID iteration process
    } else {
        long kp = Kp * (temp_h1 - temp_curr);
        long ki = Ki * (temp_set - temp_curr);
        long kd = Kd * (temp_h0 + temp_curr - 2*temp_h1);
        long delta_p = kp + ki + kd;
        power += delta_p;                                                   // power kept multiplied by denominator!
    }
    if (pid_iterate) temp_h0 = temp_h1;
    temp_h1 = temp_curr;
    long pwr = power + (1 << (denominator_p-1));                            // prepare the power to delete by denominator, round the result
    pwr >>= denominator_p;                                                  // delete by the denominator
    return pwr;
}

//--------------------- High frequency PWM signal calss on D9 pin ------------------------- ---------------
class FastPWM_D9 {
public:
    FastPWM_D9()                                { }
    void init(void);
    void        duty(uint8_t d)                 { OCR1A = d; }
    uint8_t     fanSpeed(void)                  { return OCR1A; }
};

void FastPWM_D9::init(void) {
    pinMode(9, OUTPUT);
    digitalWrite(9, LOW);
    noInterrupts();
    TCNT1   = 0;
    TCCR1B  = _BV(WGM13);                           // set mode as phase and frequency correct pwm, stop the timer
    TCCR1A  = 0;
    ICR1    = 256;
    TCCR1B  = _BV(WGM13) | _BV(CS10);               // Top value = ICR1, prescale = 1
    TCCR1A |= _BV(COM1A1);                          // XOR D9 on OCR1A, detached from D10
    OCR1A   = 0;                                    // Switch-off the signal on pin 9;
    interrupts();
}

//--------------------- Hot air gun manager using total sine shape to power on the hardware ---------------
class HOTGUN : public PID {
public:
    typedef enum { POWER_OFF, POWER_ON, POWER_FIXED, POWER_COOLING } PowerMode;
    HOTGUN(uint8_t HG_sen_pin, uint8_t HG_pwr_pin);
    void        init(void);
    bool		isOn(void)												{ return (mode == POWER_ON || mode == POWER_FIXED);             }
    void        setTemp(uint16_t temp)                                  { temp_set  = constrain(temp, 0, int_temp_max);                 }
    uint16_t	getTemp(void)											{ return temp_set;                                              }
    uint16_t	getCurrTemp(void)										{ return h_temp.last();                                         }
    uint16_t 	tempAverage(void)                  						{ return h_temp.average();                                      }
    uint8_t     powerAverage(void)                                      { return h_power.average();                                     }
    uint8_t     appliedPower(void)                						{ return actual_power;                                          }
    void		setFanSpeed(uint8_t f)									{ fan_speed = constrain(f, min_working_fan, max_fan_speed);   }
    uint8_t	    getFanSpeed(void)   									{ return fan_speed;                                             }
    uint16_t    tempDispersion(void)                                    { return h_temp.dispersion();                                   }
    bool        isCold(void)                                            { return h_temp.average() < temp_gun_cold;                      }
    bool        areExternalInterrupts(void)                             { return millis() - last_period < period * 15;                  }
    uint8_t     avgPowerPcnt(void);
    void        switchPower(bool On);
    void        fixPower(uint8_t Power);                                // Set the specified power to the the hot gun
    void     	keepTemp(void);
    uint8_t     getMaxFixedPower(void)                                  { return max_fix_power; }
    bool        syncCB(void);											// Return true at the end of the power period
private:
    bool        isGunConnected(void)                                    { return true; }
    void        shutdown(void);
    uint16_t    emulateTemp(void);                                      // To debug the project, simulate the Hot Air Gun heating process
    FastPWM_D9  hg_fan;
    uint16_t	temp_set;												// The preset temperature of the hot air gun (internal units)
    uint8_t		fan_speed;
    uint8_t     sen_pin;
    uint8_t		gun_pin;
    HISTORY  	h_power;                           						// The history queue of power applied values
    HISTORY  	h_temp;                            						// The history queue of the temperature
    volatile    uint8_t     cnt;
    volatile    uint8_t     actual_power;
    volatile    bool        active;
    uint8_t     actual_fan  = 0;                                        // Power applied to the fan (can be turned off)
    uint8_t     fix_power   = 0;                                        // Fixed power value of the Hot Air Gun (or zero if off)
    PowerMode   mode        = POWER_OFF;
    bool        chill;                                                  // To chill the hot gun
    volatile    uint32_t    last_period;                                // The time in ms when the counter reset
    const       uint8_t     period 			= 100;
    const       uint16_t    int_temp_max    = 900;
    const       uint8_t     max_fix_power   = 70;
    const       uint8_t     max_power       = 99;
    const       uint16_t    min_fan_speed   = 30;
    const       uint16_t    max_fan_speed   = 255;
    const       uint16_t    max_cool_fan    = 220;
    const       uint16_t    temp_gun_cold   = 20;
};

HOTGUN::HOTGUN(uint8_t HG_sen_pin, uint8_t HG_pwr_pin) {
    sen_pin = HG_sen_pin;
    gun_pin	= HG_pwr_pin;
}

void HOTGUN::init(void) {
    cnt             = 0;
    fan_speed       = 0;
    actual_power    = 0;
    fix_power       = 0;
    active          = false;
    chill           = false;
    last_period     = 0;
    pinMode(sen_pin, INPUT);
    pinMode(gun_pin, OUTPUT);
    digitalWrite(gun_pin, LOW);
    hg_fan.init();
    h_temp.init();
    resetPID();
}

bool HOTGUN::syncCB(void) {
    if (++cnt >= period) {
        cnt = 0;
        last_period = millis();                                             // Save the current time to check the external interrupts
        if (!active && (actual_power > 0)) {
            digitalWrite(gun_pin, HIGH);
            active = true;
        }
    } else if (cnt >= actual_power) {
        if (active) {
            digitalWrite(gun_pin, LOW);
            active = false;
        }
    }
    return (cnt == 0);														// End of the Power period (period AC voltage shapes)
}

void HOTGUN::switchPower(bool On) {
    switch (mode) {
        case POWER_OFF:
            if (hg_fan.fanSpeed() == 0) {                                   // Not power supplied to the Fan
                if (On)                                                     // !FAN && On
                    mode = POWER_ON;
            } else {
                if (On) {
                    if (isGunConnected()) {                                 // FAN && On && connected
                        mode = POWER_ON;
                    } else {                                                // FAN && On && !connected
                        shutdown();
                    }
                } else {
                    if (isGunConnected()) {                                 // FAN && !On && connected
                        if (isCold()) {                                     // FAN && !On && connected && cold
                            shutdown();
                        } else {                                            // FAN && !On && connected && !cold
                            mode = POWER_COOLING;
                        }
                    }
                }
            }
            break;
        case POWER_ON:
            if (!On) {
                mode = POWER_COOLING;
            }
            break;
        case POWER_FIXED:
            if (hg_fan.fanSpeed()) {
                if (On) {                                                   // FAN && On
                    mode = POWER_ON;
                } else {                                                    // FAN && !On
                    if (isGunConnected()) {                                 // FAN && !On && connected
                        if (isCold()) {                                     // FAN && !On && connected && cold
                            shutdown();
                        } else {                                            // FAN && !On && connected && !cold
                            mode = POWER_COOLING;
                        }
                    }
                }
            } else {                                                        // !FAN
                if (!On) {                                                  // !FAN && !On
                    shutdown();
                }
            }
            break;
        case POWER_COOLING:
            if (hg_fan.fanSpeed()) {
                if (On) {                                                   // FAN && On
                    if (isGunConnected()) {                                 // FAN && On && connected
                        mode = POWER_ON;
                    } else {                                                // FAN && On && !connected
                        shutdown();
                    }
                } else {                                                    // FAN && !On
                    if (isGunConnected()) {
                        if (isCold()) {                                     // FAN && !On && connected && cold
                            shutdown();
                        }
                    } else {                                                // FAN && !On && !connected
                        shutdown();
                    }
                }
            } else {
                if (On) {                                                   // !FAN && On
                    mode = POWER_ON;
                }
            }
    }
    h_power.init();
}

// This routine is used to keep the hot air gun temperature near required value
void HOTGUN::keepTemp(void) {
    uint16_t temp = analogRead(sen_pin);                                    // Check the hot air gun temperature
    //uint16_t temp   = emulateTemp();
    h_temp.put(temp);

    if ((temp >= int_temp_max + 30) || (temp > (temp_set + 100))) {         // Prevent global over heating
        if (mode == POWER_ON) chill = true;                                 // Turn off the power in main working mode only;
    }

    long p = 0;
    switch (mode) {
        case POWER_OFF:
            break;
        case POWER_ON:
            hg_fan.duty(fan_speed);                                         // Turn on the fan immediately
            if (chill) {
                if (temp < (temp_set - 8)) {
                    chill = false;
                    resetPID();
                } else {
                    break;
                }
            }
            p = PID::reqPower(temp_set, temp);
            p = constrain(p, 0, max_power);
            break;
        case POWER_FIXED:
            p  = fix_power;
            hg_fan.duty(fan_speed);
            break;
        case POWER_COOLING:
            if (hg_fan.fanSpeed() < min_fan_speed) {
                shutdown();
            } else {
                if (isGunConnected()) {
                    if (isCold()) {                                         // FAN && connected && cold
                        shutdown();
                    } else {                                                // FAN && connected && !cold
                        uint16_t fan = map(temp, temp_gun_cold, temp_set, max_cool_fan, min_fan_speed);
                        fan = constrain(fan, min_fan_speed, max_fan_speed);
                        hg_fan.duty(fan);
                    }
                } else {                                                    // FAN && !connected
                    shutdown();
                }
            }
            break;
        default:
            break;
    }
    h_power.put(p);
    actual_power = p;
    if (p == 0) {
        digitalWrite(gun_pin, LOW);
    }
}

void HOTGUN::fixPower(uint8_t Power) {
    if (Power == 0) {                                        // To switch off the hot gun, set the Power to 0
        switchPower(false);
        return;
    }

    if (Power > max_power) Power = max_power;
    mode = POWER_FIXED;
    fix_power   = Power;
}

uint8_t HOTGUN::avgPowerPcnt(void) {
    uint8_t pcnt = 0;
    if (mode == POWER_FIXED) {
        pcnt = map(fix_power, 0, max_fix_power, 0, 100);
    } else {
        pcnt = map(h_power.average(), 0, max_power, 0, 100);
    }
    if (pcnt > 100) pcnt = 100;
    return pcnt;
}

void HOTGUN::shutdown(void) {
    digitalWrite(gun_pin, LOW);
    hg_fan.duty(0);
    mode            = POWER_OFF;
    actual_power    = 0;
    active          = false;
}

uint16_t HOTGUN::emulateTemp(void) {
    static int16_t t = 0;
    uint8_t ap = actual_power;
    if (mode == POWER_FIXED)
        ap = fix_power;
    ap = constrain(ap, 0, 100);
    t += map(ap, 0, 100, 0, 30);
    uint8_t fn = hg_fan.fanSpeed();
    t -= fn/40 + t/50 + 1;
    if (t < 0) t = 0;
    return t;
}

//------------------------------------------ class SCREEN ------------------------------------------------------
class SCREEN {
public:
    SCREEN* next;                               						// Pointer to the next screen
    SCREEN() {
        ...

        This file has been truncated, please download it to see its full contents.
#endif //SOLDERINGSTATION_AIR_H_H
