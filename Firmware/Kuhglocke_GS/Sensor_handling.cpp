#include "libraries.h"

extern NAU7802 nau;

// ================================================================================================
/*
 * Initializes the NAU7802 ADC
 */
void initNAU7802() {

  if (!nau.begin(Wire, true)) {
      Serial.println("[ERROR] NAU7802 initialization error");
      while(true);
  }//if
  
  // Ensure correct settings are being used
  nau.setChannel(NAU7802_CHANNEL_1);
  nau.setPGACapEnable(false);
  nau.setBypassPGA(true);
  nau.setLDO(NAU7802_LDO_3V3); //so ref=3.3V
  nau.setSampleRate(NAU7802_SPS_40); //TODO: Does setting this to max speed up measurements? And does it greatly affect accuracy?

  // Calibrate Analog Front End (AFE) after major changes
  nau.calibrateAFE();

  Serial.println("NAU7802 setup & calibration complete");

  //TODO: Flush out readings (10x for loop that reads?)
}//initNAU7802()

/*
 * Handles reading various power sensors,
 * such as battery voltage, system current,
 * charge status, and USB status.
 * To be run in loop.
 * 
 * TODO: Optimize by reading other sensors
 * while NAU7802 isn't available
 */
void handleReadSensors() {
  if (millis() - lastSensorRead > SENSOR_SAMPLE_PERIOD) {
    lastSensorRead = millis();

    // === NAU7802 Reading ===
    if (nau.available()) {

      int32_t nauRaw = nau.getReading();

      // Validate reading
      if (nauRaw >= 0 && nauRaw < 0xFFFFFF) { // Reasonable range check
        if (nauChannel == 0) {
          // Battery voltage reading
          nauChannel = 1;
          nau.setChannel(NAU7802_CHANNEL_2);
          battVoltFilter.AddValue(nauRaw);
        } // if
        else {
          // read Current 
          nauChannel = 0;
          nau.setChannel(NAU7802_CHANNEL_1);
          sysCurrentFilter.AddValue(nauRaw);
        }// else
      }// if 
      else {
           // log bad data
      }// else
    }// if
     else {
         //log not ready
    }// else

    // === Temperature Sensor Reading ===
    int16_t tempReading = rawReadTempSensor();
    if (tempReading != -999 && tempReading > -500 && tempReading < 1500) {
      ambTempFilter.AddValue(tempReading);
    }// if
     else {
      //log bad data
    }// else

    // === VCC Reading ===
    vccVoltFilter.AddValue(3300);

    // === Additional Sensors (Future Implementation) ===
    // readChargingStatus();
    // readUSBStatus();
    // readCoreVoltage();
  }// if
}// handleReadSensors



String byteArrayToHexString(byte* byteArray, int length) {
  String hexString;
  hexString.reserve(length * 2);// reserve memory from heap

  static const char hex[] = "0123456789ABCDEF";// hex look up table

  for (int i = 0; i < length; i++) {
    uint8_t b = byteArray[i];
    hexString += hex[(b >> 4) & 0x0F];//read upper 4 bits
    hexString += hex[b & 0x0F];//read lower 4 bits
  }

  return hexString;
}//byteArrayToHexString()


int16_t rawReadTempSensor() {
  // Reads the raw value from the temperature sensor.
  // For a filtered value, see getAmbTemperature
  // Returns temperature in C (10 = 1.00C)

  Wire.requestFrom(P3T1755_ADDR, 2);
  if (Wire.available() != 2) {return -999;}//Error indicator

  uint8_t b1 = Wire.read();
  uint8_t b2 = Wire.read();
  int16_t rawData = (b1 << 8) | b2;
  //twos compliment
  if (rawData & 0x800) {
        rawData = rawData - 0x1000; // Sign extend
    }

  rawData = rawData >> 4;


  return (rawData * 6.25);

  //1111111 11110000 (very hot from heat gun)
  //100100 11110000 (warm from heat gun)
  //11010 1010000 (basically room temperature)
}//rawReadTempSensor()
