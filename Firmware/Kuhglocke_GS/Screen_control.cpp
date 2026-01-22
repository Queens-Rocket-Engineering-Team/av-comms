#include "libraries.h"
#include "Screen_control.h"

/*
 * Quick method to write data to the EPD using a partial update.
 * 
 * This is only meant to update VALUES, not an entire screen. If
 * you're changing menus, you need to do a full refresh and change
 * the global menu variable.
 * 
 * TAKES ABOUT 500ms (BLOCKING)
 */
void updateEPD() {
  uint16_t x = 0; // X coordinate of the update area
  uint16_t y = 15; // Y coordinate of the update area
  uint16_t w = 250; // Width of the update area
  uint16_t h = 91; // Height of the update area
  display.setPartialWindow(x, y, w, h);
  display.firstPage();
  do
  {
    display.setTextSize(2);
    display.fillScreen(GxEPD_WHITE); // Clear the partial window
    
    
    display.setCursor(x, y);
    display.print("LAT:");
    display.println(rocketGPSLat/1000000.0, 6);
    //display.print(gps.location.lat(), 6);
    //display.setCursor(x, y + 18);
    display.print("LON:");
    display.println(rocketGPSLon/1000000.0, 6);
    //display.print(gps.location.lng(), 6);

    //display.setCursor(x, y + 16*2);
    display.print("ALT:");
    //display.print(gps.altitude.feet(), 0);
    display.print(rocketAltitude * 3.28084f, 0);
    display.println("ft");

    //display.setCursor(x, y + 16*3);
    display.print("AGE:");
    display.print((millis()-rfmLastRFReceived)/1000.0, 1);
    display.println("s");

    display.print("VEL:");
    display.print(rocketVelocity * 3.28084f, 1);
    display.print("ft/s");

    display.setCursor(x+160, y + 16*3);
    //display.print("D=");
    int32_t dist = getDistanceToRocket();
    if (dist > 0) {
      display.print(dist);
    } else {
      display.print("????");
    }//if
    display.print("m");

    display.setCursor(x+160, y + 16*2);
    //display.print("Q=");
    display.print(rfmLastSNR,1);
    display.print("dB");

    //Update display at bottom of screen
    display.setTextSize(1);
    bool localGPS = (gps.location.age() < 3000);
    display.setCursor(x+5,98);
    if (localGPS) {
      display.print("+GPS");
    }
    uint16_t battVolt = getBatteryVoltage();
    display.setCursor(x+42,98);
    display.print(voltToPercent(battVolt));
    display.print("%");
    display.setCursor(x+85,98);
    display.print(rfmLastSNR, 1);
    display.print("dB");
    display.setCursor(x+150,98);
    display.print(rocketStatus, BIN);
    display.setCursor(x+210,98);
    display.print(rocketGPSSats);

  }
  while (display.nextPage());
}//updateEPD()



void printTimeToEPD() {
  // Quick method that uses partial refresh to write millis() to screen. Takes about 500ms (blocking)
  uint16_t x = 0; // X coordinate of the update area
  uint16_t y = 20; // Y coordinate of the update area
  uint16_t w = 100; // Width of the update area
  uint16_t h = 30; // Height of the update area
  display.setPartialWindow(x, y, w, h);
  display.firstPage();
  do
  {
    display.setTextSize(1);
    display.fillScreen(GxEPD_WHITE); // Clear the partial window
    display.setCursor(x, y + 20); // Set cursor position within the partial window
    display.print(millis());
  }
  while (display.nextPage());
}//printTimeToEPD()