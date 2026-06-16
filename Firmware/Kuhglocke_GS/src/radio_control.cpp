#include "radio_control.h"
#include "pinouts.h"
#include "global.h"
#include "user_interface.h"
#include <SPI.h>
#include <RadioLib.h>
#include <aim_catalog.h>

// Static state variables
static double s_freqOpts[] = {902.0, 905.4, 928.0};
static uint8_t s_freqSelected = 1;

static constexpr uint8_t s_numBandwidthOpts = 10;
static double s_bandwidthOpts[s_numBandwidthOpts] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500};
static uint8_t s_bandwidthSelected = 6;

static constexpr uint8_t s_numSpreadOpts = 6;
static int32_t s_spreadOpts[s_numSpreadOpts] = {7, 8, 9, 10, 11, 12};
static uint8_t s_spreadSelected = 3;

static constexpr uint8_t s_numCodingOpts = 4;
static int32_t s_codingOpts[s_numCodingOpts] = {5, 6, 7, 8};
static uint8_t s_codingSelected = 1;

static volatile uint32_t s_rfmLastRFReceived = 0;
static volatile int16_t s_rfmLastRSSI = 0;
static volatile float s_rfmLastSNR = 0;
static volatile int32_t s_rfmLastFreqErr = 0;
static uint8_t s_rfmLastPacket[kRfmPacketSize] = {0};
static volatile bool s_rfmLastPacketValid = false;

static volatile int32_t s_rocketGPSLat = 0;
static volatile int32_t s_rocketGPSLon = 0;
static volatile uint8_t s_rocketGPSSats = 0;
static volatile int32_t s_rocketAltitude = 0;
static volatile uint8_t s_rocketStatus = 0;
static volatile double s_rocketVelocity = 0;

static int32_t s_curFreqOffset = 0;
static volatile bool s_rfmReceivedFlag = false;

// Source liveness tracker for "SRAD OK" status
static uint32_t s_lastHeardGps = 0;
static uint32_t s_lastHeardAltimeter = 0;
static uint32_t s_lastHeardUcmLcm = 0;

// Mutex for cross-core synchronization
static portMUX_TYPE s_rocketMux = portMUX_INITIALIZER_UNLOCKED;

// SPI + Radio Objects
static SPIClass s_rfmSPI(HSPI);
static SPISettings s_rfmSPISettings(kRfmSpiClock, MSBFIRST, SPI_MODE0);
static Module s_radioModule(pins::kRfCs, pins::kRfDio0, pins::kRfReset, pins::kRfDio1, s_rfmSPI, s_rfmSPISettings);
static RFM95 s_radio(&s_radioModule);

static void byteArrayToHexStr(const byte* byteArray, int length, char* outBuf, size_t maxLen) {
  if (static_cast<size_t>(length * 2 + 1) > maxLen) {
    if (maxLen > 0) outBuf[0] = '\0';
    return;
  }
  static const char hex[] = "0123456789ABCDEF";
  for (int i = 0; i < length; i++) {
    uint8_t b = byteArray[i];
    outBuf[i * 2] = hex[(b >> 4) & 0x0F];
    outBuf[i * 2 + 1] = hex[b & 0x0F];
  }
  outBuf[length * 2] = '\0';
}

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  s_rfmReceivedFlag = true;
}

bool isRFMConnected() {
  portENTER_CRITICAL(&s_rocketMux);
  uint32_t lastRF = s_rfmLastRFReceived;
  portEXIT_CRITICAL(&s_rocketMux);
  return (millis() - lastRF < kRfmConnectedTimeout);
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
  byte rfmPayload[kRfmPacketSize];
  
  int state = s_radio.readData(rfmPayload, kRfmPacketSize);

  // Restart receiver immediately
  s_radio.startReceive();

  if (state != RADIOLIB_ERR_NONE) {
    if (state != RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.print("Receive failed, code ");
      Serial.println(state);
    }
    return;
  }

  // Extract AIM fields from the 10-byte framed record
  aim::Class cls = static_cast<aim::Class>(rfmPayload[0] >> 4);
  aim::Source src = static_cast<aim::Source>(rfmPayload[0] & 0x0F);
  uint8_t subject = rfmPayload[1];

  int32_t value;
  memcpy(&value, &rfmPayload[2], 4);

  // Update source liveness
  uint32_t nowMs = millis();
  if (src == aim::Source::Gps) {
    s_lastHeardGps = nowMs;
  } else if (src == aim::Source::Altimeter) {
    s_lastHeardAltimeter = nowMs;
  } else if (src == aim::Source::Ucm || src == aim::Source::Lcm || src == aim::Source::Power) {
    s_lastHeardUcmLcm = nowMs;
  }

  // Derive rocket status (seenGPS=bit0, seenAltimeter=bit1, seenSensors=bit2)
  uint8_t status = 0;
  if (nowMs - s_lastHeardGps < 5000) status |= 1;
  if (nowMs - s_lastHeardAltimeter < 5000) status |= 2;
  if (nowMs - s_lastHeardUcmLcm < 5000) status |= 4;

  // Protect writes to shared variables
  portENTER_CRITICAL(&s_rocketMux);
  s_rfmLastPacketValid = true;
  s_rocketStatus = status;

  // Map fields from subject catalog
  if (cls == aim::Class::Sensor) {
    if (subject == aim::subject::GpsLat) {
      s_rocketGPSLat = value;
    } else if (subject == aim::subject::GpsLon) {
      s_rocketGPSLon = value;
    } else if (subject == aim::subject::GpsNumSats) {
      s_rocketGPSSats = value;
    } else if (subject == aim::subject::Altitude) {
      s_rocketAltitude = value;
    }
  }

  s_rfmLastRFReceived = nowMs;
  s_rfmLastRSSI = s_radio.getRSSI();
  s_rfmLastSNR = s_radio.getSNR();
  s_rfmLastFreqErr = s_radio.getFrequencyError();
  memcpy(s_rfmLastPacket, rfmPayload, kRfmPacketSize);
  portEXIT_CRITICAL(&s_rocketMux);

  triggerRFFlash();

  // Local stack buffer for formatting hex
  char hexBuf[kRfmPacketSize * 2 + 1];
  byteArrayToHexStr(rfmPayload, kRfmPacketSize, hexBuf, sizeof(hexBuf));

  char logBuf[100];
  snprintf(logBuf, sizeof(logBuf), "RocketPacket:%u,%d,%.1f,%d,%s",
           nowMs,
           s_rfmLastRSSI,
           s_rfmLastSNR,
           s_rfmLastFreqErr,
           hexBuf);
  writeToSDLog(logBuf);

  char logBuf2[100];
  snprintf(logBuf2, sizeof(logBuf2), "RocketData:%u,%.6f,%.6f,%.2f,%.2f,%u",
           getRocketGPSSats(),
           getRocketLatDeg(),
           getRocketLonDeg(),
           getRocketAltitudeMeters(),
           getRocketVelocity(),
           getRocketStatus());
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
// NOTE: Kept for future manual tuning / local menu support
double getRadioBandwidth() {
  return s_bandwidthOpts[s_bandwidthSelected];
}

// NOTE: Kept for future manual tuning / local menu support
int32_t getRadioSF() {
  return s_spreadOpts[s_spreadSelected];
}

// NOTE: Kept for future manual tuning / local menu support
int32_t getRadioCR() {
  return s_codingOpts[s_codingSelected];
}
uint32_t getRfmLastRFReceived() {
  portENTER_CRITICAL(&s_rocketMux);
  uint32_t val = s_rfmLastRFReceived;
  portEXIT_CRITICAL(&s_rocketMux);
  return val;
}

int16_t getRfmLastRSSI() {
  portENTER_CRITICAL(&s_rocketMux);
  int16_t val = s_rfmLastRSSI;
  portEXIT_CRITICAL(&s_rocketMux);
  return val;
}

float getRfmLastSNR() {
  portENTER_CRITICAL(&s_rocketMux);
  float val = s_rfmLastSNR;
  portEXIT_CRITICAL(&s_rocketMux);
  return val;
}



void getRfmLastPacket(uint8_t* dest) {
  portENTER_CRITICAL(&s_rocketMux);
  memcpy(dest, s_rfmLastPacket, kRfmPacketSize);
  portEXIT_CRITICAL(&s_rocketMux);
}

bool isRfmLastPacketValid() {
  portENTER_CRITICAL(&s_rocketMux);
  bool val = s_rfmLastPacketValid;
  portEXIT_CRITICAL(&s_rocketMux);
  return val;
}

int32_t getRocketGPSLat() {
  portENTER_CRITICAL(&s_rocketMux);
  int32_t val = s_rocketGPSLat;
  portEXIT_CRITICAL(&s_rocketMux);
  return val;
}

int32_t getRocketGPSLon() {
  portENTER_CRITICAL(&s_rocketMux);
  int32_t val = s_rocketGPSLon;
  portEXIT_CRITICAL(&s_rocketMux);
  return val;
}

uint8_t getRocketGPSSats() {
  portENTER_CRITICAL(&s_rocketMux);
  uint8_t val = s_rocketGPSSats;
  portEXIT_CRITICAL(&s_rocketMux);
  return val;
}

uint8_t getRocketStatus() {
  portENTER_CRITICAL(&s_rocketMux);
  uint8_t val = s_rocketStatus;
  portEXIT_CRITICAL(&s_rocketMux);
  return val;
}

double getRocketVelocity() {
  portENTER_CRITICAL(&s_rocketMux);
  double val = s_rocketVelocity;
  portEXIT_CRITICAL(&s_rocketMux);
  return val;
}

// Catalog wire scaling (aim_catalog.h): GPS degrees x10^7, altitude meters x100.
double getRocketLatDeg() {
  portENTER_CRITICAL(&s_rocketMux);
  int32_t val = s_rocketGPSLat;
  portEXIT_CRITICAL(&s_rocketMux);
  return val / 1.0e7;
}

double getRocketLonDeg() {
  portENTER_CRITICAL(&s_rocketMux);
  int32_t val = s_rocketGPSLon;
  portEXIT_CRITICAL(&s_rocketMux);
  return val / 1.0e7;
}

double getRocketAltitudeMeters() {
  portENTER_CRITICAL(&s_rocketMux);
  int32_t val = s_rocketAltitude;
  portEXIT_CRITICAL(&s_rocketMux);
  return val / 100.0;
}

void setRocketVelocity(double vel) {
  portENTER_CRITICAL(&s_rocketMux);
  s_rocketVelocity = vel;
  portEXIT_CRITICAL(&s_rocketMux);
}
// NOTE: Kept for future manual tuning / local menu support
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
