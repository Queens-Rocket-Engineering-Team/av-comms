#include "Radio_control.h"
#include "libraries.h"

// this function is called when a complete packet
// is received by the RFM95 module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  // we got a packet, set the flag
  rfmReceivedFlag = true;
}//setFlag()

// Returns if we're connected to the rocket
bool isRFMConnected() {
  return (millis() - rfmLastRFReceived < RFM_CONNECTED_TIMEOUT);
}//isRFMConnected()


void rfmInit() {
  // Initialize RFM95 with default settings
  setRGB(2, 255,0,0, true); //set RF light to RED
  Serial.print(F("RFM95: Initializing ... "));
  radio.reset();
  delay(100);
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) {
      Serial.println("Fail");
      delay(1000);
    }//while
  }//if

  //radio.forceLDRO(true);

  radio.setDio0Action(setFlag, RISING);

  double baseFreq = freqOpts[freqSelected];
  double freqOffset = (freqCorrectionOpts[freqCorrectionSelected]/1000000.0); //Hz -> MHz
  freqOffset = (curFreqOffset/1000000.0); //Hz -> MHz (OVERRIDE)
  if (radio.setFrequency(baseFreq+freqOffset) == RADIOLIB_ERR_INVALID_FREQUENCY) {
    Serial.println(F("Frequency is invalid!"));
    while (true);
  }//if
  
  if (radio.setBandwidth(bandwidthOpts[bandwidthSelected]) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
    Serial.println(F("Bandwidth is invalid!"));
    while (true);
  }//if

  // set spreading factor to 10
  if (radio.setSpreadingFactor(spreadOpts[spreadSelected]) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
    Serial.println(F("Spreading factor is invalid!"));
    while (true);
  }//if

  // set coding rate to 6
  if (radio.setCodingRate(codingOpts[codingSelected]) == RADIOLIB_ERR_INVALID_CODING_RATE) {
    Serial.println(F("Coding rate is invalid!"));
    while (true);
  }//if

  // Start listening (async)
  radio.startReceive();

  Serial.println(F("RFM95 configuration success!"));
}//rfmInit()

// Triggers the RF indicator LED to flash (async)
// To be called whenever an RF ping is received
void indRFDoFlash() {
  indRFFlashStart = millis();
}//indRFDoFlash


/*
 * Function called when a packet is received
 * by the RFM95W radio
 * 
 * TODO: Add writeToSDLog(""); logging to this function!!
 */
void onRFMReceive() {
  // Flash ping indicator
  indRFDoFlash();

  // Read data
  byte rfmPayload[RFM_PACKET_SIZE];
  int state = radio.readData(rfmPayload, RFM_PACKET_SIZE);

  // Clone data to global array
  memcpy(rfmLastPacket, rfmPayload, RFM_PACKET_SIZE);

  //byteArrayToHexString(rfmPayload, RFM_PACKET_SIZE)
  //writeToSDLog("")

  // Print debug
  Serial.print("RFM95W Packet: ");
  for (int i=0; i<RFM_PACKET_SIZE; i++) {
    Serial.print(rfmPayload[i],HEX);
  }//for
  Serial.println();
  Serial.print("RFM95W Stats: RSSI=");
  Serial.print(radio.getRSSI());
  Serial.print("dBm, SNR=");
  Serial.print(radio.getSNR());
  Serial.print("dB, FreqErr=");
  Serial.print(radio.getFrequencyError());
  Serial.println("Hz");

  // Ensure packet valid
  if (state != RADIOLIB_ERR_NONE) {
    rfmLastPacketValid = false;
    Serial.print("[WARN] RFM95W Packet error: ");
    if (state == RADIOLIB_ERR_RX_TIMEOUT) {
      Serial.println("RX Timeout");
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.println("CRC Error");
    } else {
      Serial.print("Unknown, code");
      Serial.println(state);
    }//if (failure mode)
    return;
  }//if (invalid packet)

  // Process packet data
  rfmLastPacketValid = true;

  rocketCallsign[0] = rfmPayload[0];
  rocketCallsign[1] = rfmPayload[1];
  rocketCallsign[2] = rfmPayload[2];
  rocketCallsign[3] = rfmPayload[3];
  rocketCallsign[4] = rfmPayload[4];
  rocketCallsign[5] = rfmPayload[5];
  
  rocketGPSLat = (rfmPayload[9]<<24) + (rfmPayload[8]<<16) + (rfmPayload[7]<<8) + rfmPayload[6];
  rocketGPSLon = (rfmPayload[13]<<24) + (rfmPayload[12]<<16) + (rfmPayload[11]<<8) + rfmPayload[10];
  rocketGPSSats = rfmPayload[14];
  rocketAltitude = (rfmPayload[18]<<24) + (rfmPayload[17]<<16) + (rfmPayload[16]<<8) + rfmPayload[15];
  rocketStatus = rfmPayload[19];

  // Record stats
  rfmLastRFReceived = millis();
  rfmLastRSSI = radio.getRSSI();
  rfmLastSNR = radio.getSNR();
  rfmLastFreqErr = radio.getFrequencyError();

  calculateRocketVelocity();

  // Log to SD card
  char logBuf[256];
snprintf(logBuf,sizeof(logBuf),"RocketPacket:%lu,%s,%d,%d,%ld,%s",
  millis() - rfmLastRFReceived,
  byteArrayToHexString(rfmLastPacket, RFM_PACKET_SIZE).c_str(),
  rfmLastRSSI,
  rfmLastSNR,
  rfmLastFreqErr,
  rfmLastPacketValid ? "Yes" : "NO");

writeToSDLog(String(logBuf));

  char logBuf2[256];

snprintf(logBuf2, sizeof(logBuf2),"RocketData:%u,%.6f,%.6f,%ld,%.2f,%u",
         rocketGPSSats,
         rocketGPSLat / 1e6,
         rocketGPSLon / 1e6,
         rocketAltitude,
         rocketVelocity,
         rocketStatus); 
writeToSDLog(String(logBuf2));
 
}//onRFMReceive()