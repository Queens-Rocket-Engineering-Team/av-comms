#include "screen_control.h"
#include "pinouts.h"
#include "global.h"
#include "radio_control.h"
#include "power_sensors.h"
#include "gps.h"
#include <SPI.h>
#include <GxEPD2_BW.h>
#include "GxEPD2_display_selection_new_style.h"
#include "GxEPD2_selection_check.h"

static SPIClass s_epdSPI(FSPI);
static GxEPD2_BW<GxEPD2_213_B74, GxEPD2_213_B74::HEIGHT>
s_display(GxEPD2_213_B74(pins::kEinkCs, pins::kEinkDc, pins::kEinkReset, pins::kEinkBusy));

void screenInit() {
  s_epdSPI.begin(pins::kEinkSck, pins::kEinkMiso, pins::kEinkMosi, pins::kEinkCs);
  pinMode(s_epdSPI.pinSS(), OUTPUT);
  s_display.init(kEpdBaud, true, 2, false, s_epdSPI, SPISettings(kEpdSpiClock, MSBFIRST, SPI_MODE0));
  s_display.setRotation(1);
}

void drawLoadingScreen() {
  s_display.setTextColor(GxEPD_BLACK);
  s_display.firstPage();
  do {
    s_display.fillScreen(GxEPD_WHITE);
    s_display.setTextSize(1);
    s_display.println(String("QRET Kuhglocke    FW=") + kFirmwareVersion);
    s_display.setCursor(30,20);
    s_display.setTextSize(7);
    s_display.println("QRET");
    s_display.setTextSize(2);
    s_display.println("     Loading");

    s_display.setTextSize(1);
    s_display.setCursor(0,112);
    s_display.println("POWER  BATT    RADIO   SRADOK   GPS FIX");
  } while (s_display.nextPage());
}

void updateEPD() {
  uint16_t x = 0;  // X coordinate of the update area
  uint16_t y = 15; // Y coordinate of the update area
  uint16_t w = 250; // Width of the update area
  uint16_t h = 91;  // Height of the update area
  
  s_display.setPartialWindow(x, y, w, h);
  s_display.firstPage();
  do {
    s_display.setTextSize(2);
    s_display.fillScreen(GxEPD_WHITE); // Clear the partial window
    
    s_display.setCursor(x, y);
    s_display.print("LAT:");
    s_display.println(getRocketLatDeg(), 6);

    s_display.print("LON:");
    s_display.println(getRocketLonDeg(), 6);

    s_display.print("ALT:");
    s_display.print(getRocketAltitudeMeters() * 3.28084, 0);
    s_display.println("ft");

    s_display.print("AGE:");
    s_display.print((millis() - getRfmLastRFReceived()) / 1000.0, 1);
    s_display.println("s");

    s_display.print("VEL:");
    s_display.print(getRocketVelocity() * 3.28084f, 1);
    s_display.print("ft/s");

    s_display.setCursor(x + 160, y + 16 * 3);
    int32_t dist = getDistanceToRocket();
    if (dist > 0) {
      s_display.print(dist);
    } else {
      s_display.print("????");
    }
    s_display.print("m");

    s_display.setCursor(x + 160, y + 16 * 2);
    s_display.print(getRfmLastSNR(), 1);
    s_display.print("dB");

    s_display.setTextSize(1);
    bool localGPS = (getGPSAge() < 3000);
    s_display.setCursor(x + 5, 98);
    if (localGPS) {
      s_display.print("+GPS");
    }
    uint16_t battVolt = getBatteryVoltage();
    s_display.setCursor(x + 42, 98);
    s_display.print(voltToPercent(battVolt));
    s_display.print("%");
    s_display.setCursor(x + 85, 98);
    s_display.print(getRfmLastSNR(), 1);
    s_display.print("dB");
    s_display.setCursor(x + 150, 98);
    s_display.print(getRocketStatus(), BIN);
    s_display.setCursor(x + 210, 98);
    s_display.print(getRocketGPSSats());
  } while (s_display.nextPage());
}