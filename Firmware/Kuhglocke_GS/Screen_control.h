#pragma once

#ifndef SCREEN_CONTROL_H
#define SCREEN_CONTROL_H

#include "libraries.h"
#include <Arduino.h>
#include <GxEPD2_BW.h>
#include <TinyGPSPlus.h>

// External variables (from main .ino file)
extern GxEPD2_BW<GxEPD2_213_B74, GxEPD2_213_B74::HEIGHT> display;
extern volatile int32_t rocketGPSLat;
extern volatile int32_t rocketGPSLon;
extern volatile uint8_t rocketGPSSats;
extern volatile int32_t rocketAltitude;
extern volatile uint8_t rocketStatus;
extern volatile double rocketVelocity;
extern volatile uint32_t rfmLastRFReceived;
extern volatile float rfmLastSNR;
extern TinyGPSPlus gps;

// Function prototypes
void updateEPD();
void printTimeToEPD();

// External function prototypes (from other files)
int32_t getDistanceToRocket();
uint16_t getBatteryVoltage();
uint8_t voltToPercent(uint16_t mv);

#endif // SCREEN_CONTROL_H
