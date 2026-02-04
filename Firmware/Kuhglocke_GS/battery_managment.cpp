#include "battery_managment.h"
#include "Global.h"

#include "libraries.h"
/*
 * Returns the estimated PSU voltage
 * in mV. NOTE: Not nessesarily
 * accurate!
 */
uint16_t getPSUVoltage() {
  return vccVoltFilter.GetFiltered();
}//getPSUVoltage()

/*
 * Returns the battery voltage in mV
 */
uint16_t getBatteryVoltage() {
  // Acquire raw values
  uint16_t vRef = getPSUVoltage()/2; //full-scale range is 0.5*VREF
  uint32_t battCount = battVoltFilter.GetFiltered();
  // Calculate battery voltage
  return uint16_t(BATT_VDIV * vRef * (battCount / 8388607.0));
}//getBatteryVoltage()

/*
 * Returns the system current consumption
 * in mA
 */
uint16_t getSystemCurrent() {
  // Acquire raw values
  uint16_t vRef = getPSUVoltage()/2; //full-scale range is 0.5*VREF
  uint32_t currCount = sysCurrentFilter.GetFiltered();
  // Calculate system current
  uint16_t currVolt = (vRef * (currCount / 8388607.0));
  return (currVolt-250)*1.25; //Simplified formula, see ACS70331 datasheet
}//getSystemCurrent()

/*
 * Returns the reading from the onboard 
 * temperature sensor (in 0.01C increments)
 */
int16_t getAmbTemperature() {
  return ambTempFilter.GetFiltered();
}//getAmbTemperature()


uint8_t voltToPercent(uint16_t mv) {
  // A rough converstion that converts the OCV of
  // a 1S Li-Ion (INR) battery to a SoC percentage.
  
  if (mv>=4160) { return 100; }
  else if (mv>=4110) { return 96; }
  else if (mv>=4080) { return 92; }
  else if (mv>=4050) { return 88; }
  else if (mv>=4010) { return 84; }
  else if (mv>=3970) { return 80; }
  else if (mv>=3920) { return 76; }
  else if (mv>=3880) { return 72; }
  else if (mv>=3840) { return 68; }
  else if (mv>=3800) { return 64; }
  else if (mv>=3750) { return 60; }
  else if (mv>=3720) { return 56; }
  else if (mv>=3690) { return 52; }
  else if (mv>=3660) { return 48; }
  else if (mv>=3630) { return 44; }
  else if (mv>=3620) { return 40; }
  else if (mv>=3590) { return 36; }
  else if (mv>=3570) { return 32; }
  else if (mv>=3540) { return 28; }
  else if (mv>=3500) { return 24; }
  else if (mv>=3460) { return 20; }
  else if (mv>=3430) { return 16; }
  else if (mv>=3330) { return 12; }
  else if (mv>=3220) { return 8; }
  else if (mv>=3100) { return 4; }
  else { return 0; }

}//voltToPercent

/*
 * Determines the current charge status.
 * 0 = Charging
 * 1 = Fully Charged
 * 2 = Disabled (on battery)
 */
uint8_t getChargingStatus() {
  uint16_t chrgStat = analogRead(CHRG_STAT_PIN);
  if (chrgStat < 200) {
    // 0.00V = Charging
    return 0;
  } else if (chrgStat > 3800) {
    // 3.30V = Disabled
    return 2;
  }//if
  // 1.65V = Fully Charged
  return 1;
}//getChargingStatus


uint16_t esimtateVREF(uint8_t adcPin, uint8_t samples) {
  /*
   * Estimates the VREF voltage with no extra hardware.
   * 
   * Given an ADC pin that has a VOLTAGE BELOW VREF (3.3V)
   * and ABOVE ZERO, attempts to back-calculate the VREF
   * voltage using analogRead, analogReadMilliVolts, and a
   * correction function (accounts for inaccuracies in
   * analogReadMillivolts & the non-linearity of the ADC).
   * Blocking function. Assumes ADC resolution is 12 bits.
   * 
   * Calibration will vary between ESP32 modules.
   * 
   * To calibrate this function, measure various analog voltages
   * using both analogReadMilliVolts() and an external multimeter,
   * recording them in a table. Calculate (dmmVolt/adcVolt) for each
   * entry to get a correction factor. Plotting these factors against 
   * the adcReadMilliVolts values should produce a strong 3rd-order
   * polynomial trend. Get an equation for this curve and evaluate it
   * for N steps over the 12-bit range (e.g. 0,409,819,...,3686,4095).
   * Fill these values into the if-statemtn in calibration stage.
   */
  // Get an average reading from the ADC pin
  uint32_t mvAvg = 0;
  uint32_t countAvg = 0;
  for (uint8_t i=0; i<samples; i++) {
      mvAvg += analogReadMilliVolts(adcPin);
      countAvg += analogRead(adcPin);
  }//for
  mvAvg /= samples;
  countAvg /= samples;
  
  // Ensure ADC pin is 0>x>3.3
  if (countAvg > 4090 or countAvg < 5) {
    return 0;
  }//if (not enough data)
  
  // Back-calculate VREF
  uint16_t backVolt = mvAvg/(countAvg/4095.0);

  // Apply calibrated correction factor
  if (mvAvg < 128) {backVolt *= 0.884;}
  else if (mvAvg < 256) {backVolt *= 0.902;}
  else if (mvAvg < 384) {backVolt *= 0.917;}
  else if (mvAvg < 512) {backVolt *= 0.930;}
  else if (mvAvg < 768) {backVolt *= 0.940;}
  else if (mvAvg < 1024) {backVolt *= 0.956;}
  else if (mvAvg < 1280) {backVolt *= 0.966;}
  else if (mvAvg < 1536) {backVolt *= 0.972;}
  else if (mvAvg < 1792) {backVolt *= 0.976;}
  else if (mvAvg < 2048) {backVolt *= 0.980;}
  else if (mvAvg < 2303) {backVolt *= 0.987;}
  else if (mvAvg < 2559) {backVolt *= 0.999;}
  else if (mvAvg < 2815) {backVolt *= 1.016;}
  else if (mvAvg < 3071) {backVolt *= 1.042;}
  else if (mvAvg < 3327) {backVolt *= 1.078;}
  else if (mvAvg < 3583) {backVolt *= 1.127;}
  else if (mvAvg < 3839) {backVolt *= 1.189;}
  else {backVolt *= 1.269;}

  return backVolt;
}//esimtateVREF()
