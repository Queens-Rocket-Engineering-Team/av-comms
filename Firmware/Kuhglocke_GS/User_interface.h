#pragma once

#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "SD_MMC.h"
#include "pinouts.h"

#include "libraries.h"

// External variables (from main .ino file)
extern Adafruit_NeoPixel rgbLEDs;
extern bool usbDataOnly;
extern File logFile;
extern uint16_t logFileNumber;
extern const uint8_t DEFAULT_LED_BRIGHTNESS;
extern const uint16_t IND_RF_FLASH_TIME;
extern const uint16_t IND_BATTERY_FLASH_GAP;
extern bool indBattFlashState;
extern uint32_t indBattLastToggle;
extern uint32_t indRFFlashStart;
extern volatile uint32_t rfmLastRFReceived;
extern volatile uint8_t rocketStatus;
extern volatile uint8_t rocketGPSSats;

// Function prototypes
void setUSBDataOnlyMode(bool state);
void setLEDBrightness(uint8_t brightness);
void setRGB(byte index, byte r, byte g, byte b);
void setRGB(byte index, byte r, byte g, byte b, bool push);
void setRGB(byte r, byte g, byte b);
bool makeNextSDLog();
bool writeToSDLog(String txt);
void handleLEDs();

// External function prototypes (from other files)
uint16_t getBatteryVoltage();
uint8_t getChargingStatus();
bool isRFMConnected();

#endif // USER_INTERFACE_H
