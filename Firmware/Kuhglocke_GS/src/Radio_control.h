#pragma once

#include <Arduino.h>

void rfmInit();
bool isRFMConnected();
void onRFMReceive();
bool getRfmReceivedFlag();
void clearRfmReceivedFlag();

// Radio option accessors
double getRadioFreq();
double getRadioBandwidth();
int32_t getRadioSF();
int32_t getRadioCR();

// Getter functions for incoming RF status
uint32_t getRfmLastRFReceived();
int16_t getRfmLastRSSI();
float getRfmLastSNR();
int32_t getRfmLastFreqErr();
const uint8_t* getRfmLastPacket();
bool isRfmLastPacketValid();

// Getter/Setter functions for decoded rocket packet data
int32_t getRocketGPSLat();
int32_t getRocketGPSLon();
uint8_t getRocketGPSSats();
int32_t getRocketAltitude();
uint8_t getRocketStatus();
double getRocketVelocity();

void setRocketVelocity(double vel);

int32_t getCurFreqOffset();
void changeFreqOffset(int32_t amount);
void setRadioConfig(const String& name, uint16_t value);
