#include "User_interface.h"
#include "libraries.h"

void setUSBDataOnlyMode(bool state) {
  usbDataOnly = state;
  digitalWrite(pins::kDisable5v, state);
}//setUSBDataOnlyMode()

/*
 * Sets the global brightness for all LED
 * indicators.
 */
void setLEDBrightness(uint8_t brightness) {
  rgbLEDs.setBrightness(brightness);
  rgbLEDs.show();
}//setLEDBrightness

void setRGB(byte index, byte r, byte g, byte b) {
  setRGB(index, r,g,b, true);
}//setRGB(byte, byte, byte, byte)

void setRGB(byte index, byte r, byte g, byte b, bool push) {
  rgbLEDs.setPixelColor(index, rgbLEDs.Color(r,g,b));
  if (push) {rgbLEDs.show();}
}//setRGB(byte, byte, byte, byte, bool)

void setRGB(byte r, byte g, byte b) {
  for (byte i=0; i<NUM_RGB_LEDS; i++) {
    setRGB(i, r,g,b);
  }//for
}//setRGB(byte, byte, byte)
bool makeNextSDLog() {
  // Ensure SD card is present
  if (SD_MMC.cardType() == CARD_NONE) {
    return false;
  }//if

  // Ensure logs directory exists

  SD_MMC.mkdir("/logs");

  // Find next Log ID
  uint16_t id = 0;
  while (true) {
    if (SD_MMC.exists("/logs/Kuhglocke_Log" + String(id) + ".txt")) {
      id++;
    } else {
      break;
    }//if
  }//while
  logFileNumber = id;
  Serial.println("Selected log file number");

  // Create new log file
  logFile = SD_MMC.open("/logs/Kuhglocke_Log" + String(id) + ".txt", FILE_WRITE);
  if (!logFile) {
    Serial.println("[WARN] Unable to open log file for writing");
    return false;
  }//if

  return true;
}//makeNextSDLog()

/*
 * Given a string, writes it to the
 * ongoing log file. 
 *  
 * If no SD card is present, function
 * will silently & gracefully fail.
 */
bool writeToSDLog(const String& txt) {
  if (!logFile) return false;

  logFile.print('[');
  logFile.print(millis());
  logFile.print("] ");
  bool r = logFile.println(txt);

  static uint32_t lastFlush = 0;
  if (millis() - lastFlush > 2000) {  // flush every 2s
    logFile.flush();
    lastFlush = millis();
  }
  return r;
}

void handleLEDs() {
  // Power LED remains constant

  // Battery/Charging indicator
  //0 = charging, otherwise not
  uint8_t chrgStatus = getChargingStatus();
  uint16_t battVolt = getBatteryVoltage();
  if (battVolt < 3500) {
    if (chrgStatus == 1) {
      //Full & Off
      setRGB(1, 0,255,0, false);
    } else {
      //Low
      setRGB(1, 255,0,0, false);
    }//if (chrgd)
  } else if (battVolt < 3700) {
    setRGB(1, 255,80,0, false);
  } else if (battVolt < 3900) {
    setRGB(1, 255,220,0, false);
  } else {
    setRGB(1, 0,255,0, false);
  }//if (batt volt to colour)

  if (chrgStatus == 0) {
    //Charging; flash LED
    if (indBattFlashState) {
      setRGB(1, 0,0,0, true);
    }//if (turn off)
    if (millis() - indBattLastToggle > IND_BATTERY_FLASH_GAP) {
      indBattLastToggle = millis();
      indBattFlashState = !indBattFlashState;
    }//if (flash time)
  }//if (charging)


  // RF Indicator
    // Default OFF, Green if connected
    // Flash Blue every received ping
  if (isRFMConnected()) {
    if (millis() - indRFFlashStart < IND_RF_FLASH_TIME) {
      setRGB(2, 255,255,255); // Ping flash
    } else {
      setRGB(2, 0,255,0); // Default
    }//if (flash RF led)
  } else {
    setRGB(2, 0,0,0, true);
  }//if


  // SRAD OKAY LIGHT
  if (isRFMConnected()) {
    if (rocketStatus == 0b111) {
      setRGB(3, 255,255,255);
    } else {
      setRGB(3, 0,0,0);
    }//if    
  } else {
      setRGB(3, 0,0,0);
  }//if
  

  // Rocket GPS fix
  if (isRFMConnected()) {
    if (rocketGPSSats > 3) {
      setRGB(4, 0,255,0, true);
    } else {
      setRGB(4, 0,0,0, true);
    }//if    
  } else {
      setRGB(4, 0,0,0, true);
  }//if
  
}//handleLEDs()
