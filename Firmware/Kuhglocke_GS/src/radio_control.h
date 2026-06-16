#pragma once

#include <Arduino.h>

void rfmInit();
bool isRFMConnected();
void onRFMReceive();
bool getRfmReceivedFlag();
void clearRfmReceivedFlag();

// Radio option accessors
double getRadioFreq();

// NOTE: Kept for future manual tuning / local menu support
double getRadioBandwidth();
int32_t getRadioSF();
int32_t getRadioCR();
// Getter functions for incoming RF status
uint32_t getRfmLastRFReceived();
int16_t getRfmLastRSSI();
float getRfmLastSNR();

void getRfmLastPacket(uint8_t* dest);
bool isRfmLastPacketValid();

// Getter/Setter functions for decoded rocket packet data
int32_t getRocketGPSLat();
int32_t getRocketGPSLon();
uint8_t getRocketGPSSats();
uint8_t getRocketStatus();
double getRocketVelocity();

// Engineering-unit accessors: the single place the aim_catalog wire scaling
// (GPS degrees x10^7, altitude meters x100) is undone. Display/derived code
// must use these, never the raw catalog-unit getters above.
double getRocketLatDeg();
double getRocketLonDeg();
double getRocketAltitudeMeters();

void setRocketVelocity(double vel);

// NOTE: Kept for future manual tuning / local menu support
int32_t getCurFreqOffset();
void changeFreqOffset(int32_t amount);
void setRadioConfig(const String& name, uint16_t value);
