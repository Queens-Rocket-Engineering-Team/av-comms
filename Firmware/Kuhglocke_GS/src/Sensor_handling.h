#ifndef SENSOR_HANDLING_H
#define SENSOR_HANDLING_H

#include <Arduino.h>
#include <NAU7802_2CH.h>
#include <MedianFilterLib.h>
#include "pinouts.h"
#include "libraries.h"

// External variables (from main .ino file)
extern NAU7802 nau;
extern uint32_t lastSensorRead;
extern bool nauChannel;
extern MedianFilter<uint32_t> vccVoltFilter;
extern MedianFilter<uint32_t> battVoltFilter;
extern MedianFilter<uint32_t> sysCurrentFilter;
extern MedianFilter<uint32_t> ambTempFilter;
extern const uint16_t SENSOR_SAMPLE_PERIOD;

// Function prototypes
void initNAU7802();
void handleReadSensors();
String byteArrayToHexString(byte* byteArray, int length);
int16_t rawReadTempSensor();

#endif // SENSOR_HANDLING_H
