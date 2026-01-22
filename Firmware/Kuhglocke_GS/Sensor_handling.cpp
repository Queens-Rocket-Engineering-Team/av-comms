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

    // Read NAU channel & then switch to other
    // (CH1 & CH2 get read in alternating cycles)
    // (gives time for MUX to stabilize)
    while (!nau.available()) {}
    int32_t nauRaw = nau.getReading();
    if (nauRaw < 0) {nauRaw = 0;}
    if (nauChannel == 0) {
      // Just read battery; switch to Current
      nauChannel = 1;
      nau.setChannel(NAU7802_CHANNEL_2);
      battVoltFilter.AddValue(nauRaw);
    } else {
      // Just read current; switch to battery
      nauChannel = 0;
      nau.setChannel(NAU7802_CHANNEL_1);
      sysCurrentFilter.AddValue(nauRaw);
    }//if

    // Attempt to read VCC
    vccVoltFilter.AddValue(3300); //Assume 3.3V
    /*
    uint16_t estVREF = esimtateVREF(CHRG_STAT_PIN, 5);
    if (estVREF == 0) {
      estVREF = esimtateVREF(MENU_BTNS_PIN, 5);
    }//if
    if (estVREF > 0) {
      vccVoltFilter.AddValue(estVREF);
    } else {
      vccVoltFilter.AddValue(3300); //Assume 3.3V if can't calibrate
    }//if
    */

    ambTempFilter.AddValue(rawReadTempSensor());

    //read core voltage
    //read temp sensor
    //read nau7802
    //parse data & load into global vars
    //parse CHRG status & make into integer?
    //parse USB status & make into integer?
    
  }//if (sample)
}//handleReadSensors



String byteArrayToHexString(byte* byteArray, int length) {
  String hexString = "";
  for (int i = 0; i < length; i++) {
    if (byteArray[i] < 0x10) {
      hexString += "0"; // Add leading zero for single digit hex values
    }
    hexString += String(byteArray[i], HEX);
  }
  hexString.toUpperCase(); // Convert to uppercase if desired
  return hexString;
}//byteArrayToHexString()


int16_t rawReadTempSensor() {
  // Reads the raw value from the temperature sensor.
  // For a filtered value, see getAmbTemperature
  // Returns temperature in dC (100 = 1.00C)
  // TODO: Simple math can only handle positive temps; see datasheet for proper method

  Wire.requestFrom(P3T1755_ADDR, 2);
  uint8_t b1 = Wire.read();
  uint8_t b2 = Wire.read();
  int16_t rawData = (b1 << 8) | b2;
  rawData = rawData >> 4;
  //Serial.print("TempRaw = ");
  //Serial.print(b1, BIN);
  //Serial.print(" ");
  //Serial.println(b2, BIN);
  return (rawData * 6.25);

  //1111111 11110000 (very hot from heat gun)
  //100100 11110000 (warm from heat gun)
  //11010 1010000 (basically room temperature)
}//rawReadTempSensor()

