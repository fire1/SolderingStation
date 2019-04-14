//
// Created by Angel Zaprianov on 2019-04-04.
//

#ifndef SOLDERING_STATION_EEP_ROM_H
#define SOLDERING_STATION_EEP_ROM_H

#include <avr/io.h>
#include <EEPROM.h>

#ifndef EEP_ROM_INDEXES
#define EEP_ROM_INDEXES 18
#endif

#define ROM_SLEEP_BEGIN_AIR 1
#define ROM_SLEEP_BEGIN_SDR 2
#define ROM_SLEEP_TARGET_AIR 3
#define ROM_SLEEP_TARGET_SDR 4


class EepRom {
private:
    uint8_t index;
    uint16_t data[EEP_ROM_INDEXES] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


public:
    void save() {
        for (index = 0; index < EEP_ROM_INDEXES; ++index) {
            EEPROM.put(index, data[index]);
        }
    }

    void load() {
        for (index = 0; index < EEP_ROM_INDEXES; ++index) {
            EEPROM.get(index, data[index]);
        }
    }

    uint16_t get(uint8_t offset) {
        return data[offset];
    }

    uint16_t *all() {
        return data;
    }

};

#endif //SOLDERINGSTATION_EEPROM_H_H
