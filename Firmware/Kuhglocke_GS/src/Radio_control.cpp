#include "Radio_control.h"
#include "pinouts.h"
#include "Global.h"
#include "User_interface.h"
#include <SPI.h>
#include <RadioLib.h>

// Static state variables
static double s_freqOpts[] = {902.0, 905.4, 928.0};

static double s_bandwidthOpts[] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500};
static uint8_t s_numBandwidthOpts = 10;

static int32_t s_spreadOpts[] = {7, 8, 9, 10, 11, 12};
static uint8_t s_numSpreadOpts = 6;

static int32_t s_codingOpts[] = {5, 6, 7, 8};
static uint8_t s_numCodingOpts = 4;

static uint8_t s_freqSelected = 1;
static uint8_t s_bandwidthSelected = 6;
static uint8_t s_spreadSelected = 3;
static uint8_t s_codingSelected = 1;

static volatile uint32_t s_rfmLastRFReceived = 0;
static volatile int16_t s_rfmLastRSSI = 0;
static volatile float s_rfmLastSNR = 0;
static volatile int32_t s_rfmLastFreqErr = 0;
static uint8_t s_rfmLastPacket[RFM_PACKET_SIZE] = {0};
static volatile bool s_rfmLastPacketValid = false;

static volatile int32_t s_rocketGPSLat = 0;
static volatile int32_t s_rocketGPSLon = 0;
static volatile uint8_t s_rocketGPSSats = 0;
static volatile int32_t s_rocketAltitude = 0;
static volatile uint8_t s_rocketStatus = 0;
static volatile double s_rocketVelocity = 0;
static volatile char s_rocketCallsign[7] = {'-', '-', '-', '-', '-', '-', '\0'};

static int32_t s_curFreqOffset = 0;
static volatile bool s_rfmReceivedFlag = false;

// SPI + Radio Objects
static SPIClass s_rfmSPI(HSPI);
static SPISettings s_rfmSPISettings(RFM_SPI_CLOCK, MSBFIRST, SPI_MODE0);
static Module s_radioModule(pins::kRfCs, pins::kRfDio0, pins::kRfReset, pins::kRfDio1, s_rfmSPI, s_rfmSPISettings);
static RFM95 s_radio(&s_radioModule);
static String byteArrayToHexString(const byte* byteArray, int length) {
  String hexString;
  hexString.reserve(length * 2);
  static const char hex[] = "0123456789ABCDEF";
  for (int i = 0; i < length; i++) {
    uint8_t b = byteArray[i];
    hexString += hex[(b >> 4) & 0x0F];
    hexString += hex[b & 0x0F];
  }
  return hexString;
}

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  s_rfmReceivedFlag = true;
}

bool isRFMConnected() {
  return (millis() - s_rfmLastRFReceived < RFM_CONNECTED_TIMEOUT);
}

void rfmInit() {
  // Initialize SPI Bus
  s_rfmSPI.begin(pins::kRfSck, pins::kRfMiso, pins::kRfMosi, pins::kRfCs);
  pinMode(s_rfmSPI.pinSS(), OUTPUT);

  setRGB(2, 255, 0, 0, true); // set RF light to RED
  Serial.print(F("RFM95: Initializing ... "));
  s_radio.reset();
  delay(100);
  int state = s_radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    setRGB(2, 255, 0, 0, true); // set RF light to RED
    return; // degraded mode: return instead of hanging
  }

  s_radio.setDio0Action(setFlag, RISING);

  double baseFreq = s_freqOpts[s_freqSelected];
  double freqOffset = (s_curFreqOffset / 1000000.0); // Hz -> MHz
  
  s_radio.setFrequency(baseFreq + freqOffset);
  s_radio.setBandwidth(s_bandwidthOpts[s_bandwidthSelected]);
  s_radio.setSpreadingFactor(s_spreadOpts[s_spreadSelected]);
  s_radio.setCodingRate(s_codingOpts[s_codingSelected]);
  s_radio.setSyncWord(0x12);
  s_radio.setPreambleLength(8);

  state = s_radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Radio Listening Active"));
  } else {
    Serial.print(F("failed starting listening, code "));
    Serial.println(state);
  }
}

void onRFMReceive() {
  byte rfmPayload[RFM_PACKET_SIZE];
  
  int state = s_radio.readData(rfmPayload, RFM_PACKET_SIZE);

  // Restart receiver immediately
  s_radio.startReceive();

  if (state != RADIOLIB_ERR_NONE) {
    if (state != RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.print("Receive failed, code ");
      Serial.println(state);
    }
    return;
  }

  s_rfmLastPacketValid = true;

  // Cast volatile destination array
  memcpy((char*)s_rocketCallsign, rfmPayload, 6);
  
  s_rocketGPSLat = (rfmPayload[9] << 24) + (rfmPayload[8] << 16) + (rfmPayload[7] << 8) + rfmPayload[6];
  s_rocketGPSLon = (rfmPayload[13] << 24) + (rfmPayload[12] << 16) + (rfmPayload[11] << 8) + rfmPayload[10];
  s_rocketGPSSats = rfmPayload[14];
  s_rocketAltitude = (rfmPayload[18] << 24) + (rfmPayload[17] << 16) + (rfmPayload[16] << 8) + rfmPayload[15];
  s_rocketStatus = rfmPayload[19];

  s_rfmLastRFReceived = millis();
  triggerRFFlash();
  s_rfmLastRSSI = s_radio.getRSSI();
  s_rfmLastSNR = s_radio.getSNR();
  s_rfmLastFreqErr = s_radio.getFrequencyError();
  memcpy(s_rfmLastPacket, rfmPayload, RFM_PACKET_SIZE);

  char logBuf[100];
  snprintf(logBuf, sizeof(logBuf), "RocketPacket:%u,%d,%.1f,%d,%s",
           s_rfmLastRFReceived,
           s_rfmLastRSSI,
           s_rfmLastSNR,
           s_rfmLastFreqErr,
           byteArrayToHexString(rfmPayload, RFM_PACKET_SIZE).c_str());
  writeToSDLog(logBuf);

  char logBuf2[100];
  snprintf(logBuf2, sizeof(logBuf2), "RocketData:%u,%.6f,%.6f,%d,%.2f,%u",
           s_rocketGPSSats,
           s_rocketGPSLat / 1000000.0,
           s_rocketGPSLon / 1000000.0,
           s_rocketAltitude,
           s_rocketVelocity,
           s_rocketStatus);
  writeToSDLog(logBuf2);
}

bool getRfmReceivedFlag() {
  return s_rfmReceivedFlag;
}

void clearRfmReceivedFlag() {
  s_rfmReceivedFlag = false;
}

double getRadioFreq() {
  return s_freqOpts[s_freqSelected];
}

double getRadioBandwidth() {
  return s_bandwidthOpts[s_bandwidthSelected];
}

int32_t getRadioSF() {
  return s_spreadOpts[s_spreadSelected];
}

int32_t getRadioCR() {
  return s_codingOpts[s_codingSelected];
}

uint32_t getRfmLastRFReceived() {
  return s_rfmLastRFReceived;
}

int16_t getRfmLastRSSI() {
  return s_rfmLastRSSI;
}

float getRfmLastSNR() {
  return s_rfmLastSNR;
}

int32_t getRfmLastFreqErr() {
  return s_rfmLastFreqErr;
}

const uint8_t* getRfmLastPacket() {
  return s_rfmLastPacket;
}

bool isRfmLastPacketValid() {
  return s_rfmLastPacketValid;
}

int32_t getRocketGPSLat() {
  return s_rocketGPSLat;
}

int32_t getRocketGPSLon() {
  return s_rocketGPSLon;
}

uint8_t getRocketGPSSats() {
  return s_rocketGPSSats;
}

int32_t getRocketAltitude() {
  return s_rocketAltitude;
}

uint8_t getRocketStatus() {
  return s_rocketStatus;
}

double getRocketVelocity() {
  return s_rocketVelocity;
}

const char* getRocketCallsign() {
  return (const char*)s_rocketCallsign;
}

void setRocketVelocity(double vel) {
  s_rocketVelocity = vel;
}

int32_t getCurFreqOffset() {
  return s_curFreqOffset;
}

void changeFreqOffset(int32_t amount) {
  s_curFreqOffset += amount;
}

void setRadioConfig(const String& name, uint16_t value) {
  if (name == "bandwidth") {
    if (value < s_numBandwidthOpts) {
      s_bandwidthSelected = value;
    }
  } else if (name == "spreadingfactor") {
    if (value < s_numSpreadOpts) {
      s_spreadSelected = value;
    }
  } else if (name == "codingrate") {
    if (value < s_numCodingOpts) {
      s_codingSelected = value;
    }
  }
}