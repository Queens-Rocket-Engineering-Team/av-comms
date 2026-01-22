#ifndef BATTERY_MANAGEMENT_H
#define BATTERY_MANAGEMENT_H

#include <Arduino.h>
#include "pinouts.h"
#include <MedianFilterLib.h>

// External variables (from main .ino file)
extern const double BATT_VDIV;
extern MedianFilter<uint32_t> vccVoltFilter;
extern MedianFilter<uint32_t> battVoltFilter;
extern MedianFilter<uint32_t> sysCurrentFilter;
extern MedianFilter<uint32_t> ambTempFilter;

// Function prototypes
uint16_t getPSUVoltage();
uint16_t getBatteryVoltage();
uint16_t getSystemCurrent();
int16_t getAmbTemperature();
uint8_t voltToPercent(uint16_t mv);
uint8_t getChargingStatus();
uint16_t esimtateVREF(uint8_t adcPin, uint8_t samples);

#endif // BATTERY_MANAGEMENT_H
