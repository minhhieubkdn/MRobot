#ifndef _MEEPROM_h_
#define _MEEPROM_h_

#include <EEPROM.h>

bool InitEEPROM(float *_kp, float *_ki, float *_kd)
{
    uint16_t rawData;
    bool noData = false;
    EEPROM.init();

    if (EEPROM.read(512) == 0)
        return false;

    rawData = EEPROM.read(512);
    *_kp = (float)rawData;

    rawData = EEPROM.read(512 + sizeof(_kp));
    *_ki = (float)rawData;

    rawData = EEPROM.read(512 + 2*sizeof(_kp));
    *_kd = (float)rawData;

    return true;
}

void SaveData(float *kp, float *ki, float *kd)
{
    EEPROM.update(512, *kp);
    EEPROM.update(512 + sizeof(float), *ki);
    EEPROM.update(512 + 2*sizeof(float), *kd);
}


#endif