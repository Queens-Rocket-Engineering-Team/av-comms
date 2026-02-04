#pragma once

#ifndef RADIO_CONTROL_H
#define RADIO_CONTROL_H

#include "libraries.h"
#include <Arduino.h>
#include <RadioLib.h>

// External variables (from main .ino file)
//extern Module radio;
extern volatile bool rfmReceivedFlag;
extern volatile uint32_t rfmLastRFReceived;
extern volatile int16_t rfmLastRSSI;
extern volatile float rfmLastSNR;
extern volatile int32_t rfmLastFreqErr;
extern byte rfmLastPacket[];
extern volatile bool rfmLastPacketValid;
extern volatile int32_t rocketGPSLat;
extern volatile int32_t rocketGPSLon;
extern volatile uint8_t rocketGPSSats;
extern volatile int32_t rocketAltitude;
extern volatile uint8_t rocketStatus;
extern volatile double rocketVelocity;
extern volatile char rocketCallsign[];
extern double freqOpts[];
extern uint8_t numFreqOpts;
extern double bandwidthOpts[];
extern uint8_t numBandwidthOpts;
extern int32_t spreadOpts[];
extern uint8_t numSpreadOpts;
extern int32_t codingOpts[];
extern uint8_t numCodingOpts;
extern int32_t freqCorrectionOpts[];
extern uint8_t numFreqCorrectionOpts;
extern uint8_t freqSelected;
extern uint8_t bandwidthSelected;
extern uint8_t spreadSelected;
extern uint8_t codingSelected;
extern uint8_t freqCorrectionSelected;
extern int32_t curFreqOffset;
extern const uint8_t RFM_PACKET_SIZE;
extern const uint16_t RFM_CONNECTED_TIMEOUT;
extern uint32_t indRFFlashStart;

// Function prototypes
void setFlag();
bool isRFMConnected();
void rfmInit();
void indRFDoFlash();
void onRFMReceive();

// External function prototypes (from other files)
bool writeToSDLog(const String& txt);
String byteArrayToHexString(byte* byteArray, int length);
void calculateRocketVelocity();

#endif // RADIO_CONTROL_H
