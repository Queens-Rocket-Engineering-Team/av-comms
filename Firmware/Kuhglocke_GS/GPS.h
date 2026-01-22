#ifndef GPS_H
#define GPS_H

#include "libraries.h"
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// Constants


// External variables (declared in main .ino file)
extern TinyGPSPlus gps;
extern HardwareSerial gpsSerial;
extern uint32_t lastLocalGPSLog;
extern const uint32_t LOCAL_GPS_LOG_RATE;

// Rocket GPS data (received via RF)
extern volatile int32_t rocketGPSLat;
extern volatile int32_t rocketGPSLon;
extern volatile uint8_t rocketGPSSats;

// Function prototypes
void gpsInit();
void gpsDisplayInfoGrand();
void handleGPS();
double degToRad(double degs);
int32_t getDistanceToRocket();
void calculateRocketVelocity();

// External function prototypes (from main .ino file)
bool writeToSDLog(String txt);

#endif // GPS_H