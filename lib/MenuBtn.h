//
// Created by Angel Zaprianov on 2019-04-04.
//

#ifndef SOLDERING_STATION_MENU_H
#define SOLDERING_STATION_MENU_H

#include <Arduino.h>


#define BUTTON_DEBOUNCE 350
const uint8_t SET_BUTTON_SOLDER = 7;
const uint8_t SET_BUTTON_HOTAIR = 8;

class MenuBtn {
private:
    boolean isSleepAirOn = false;
    boolean isSleepSdrOn = false;
    boolean isTogetherOn = false;
    boolean isSolderOn = false, isHotAirOn = false, isHotAirUse = true, isSolderUse = true;
    unsigned long debounceSolder, debounceHotAir, debounceTogether;

    void handleButtonSolder() {
        if (digitalRead(SET_BUTTON_SOLDER) == LOW && debounceSolder + BUTTON_DEBOUNCE < millis()) {
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
    void handleButtonHotAir() {
        if (digitalRead(SET_BUTTON_HOTAIR) == LOW && debounceHotAir + BUTTON_DEBOUNCE < millis()) {
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

    void handleTogetherButtons() {
        if (digitalRead(SET_BUTTON_HOTAIR) == LOW && digitalRead(SET_BUTTON_SOLDER) == LOW &&
            debounceTogether + BUTTON_DEBOUNCE < millis()) {
            delay(5);
            if (digitalRead(SET_BUTTON_HOTAIR) == LOW && digitalRead(SET_BUTTON_SOLDER) == LOW) {
                debounceTogether = millis();
                isTogetherOn = !isTogetherOn;
#ifdef DEBUG
                Serial.print(F("Together state: "));
                Serial.println(isTogetherOn);
#endif
            }
        }
    }


public:

    void listener() {
        handleTogetherButtons();
        handleButtonHotAir();
        handleButtonSolder();
    }


    boolean isSdrOn() {
        return isHotAirOn;
    }

    boolean isAirOn() {
        return isHotAirOn;
    }

    boolean isSdrSleep() {
        return isSleepSdrOn;
    }

    boolean isAirSleep() {
        return isSleepAirOn;
    }

    boolean isMenu() {
        return isTogetherOn;
    }


    void sleepSdr() {
        isSleepSdrOn = true;
    }

    void sleepAir() {
        isSleepAirOn = true;
    }

};

#endif //SOLDERINGSTATION_MENU_H
