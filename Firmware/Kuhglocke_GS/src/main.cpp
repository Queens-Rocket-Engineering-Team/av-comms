/*
 * Author: Kennan Bays (Kenneract)
 * Created: Aug.2.2024
 * Updated: Jun.11.2025
 * Purpose: Testing firmware for the Kuhglocke ground station
 * Hardware: QRET Kuhglocke ground station V1.0 (ESP32-S3-N16R2; 16MiB Flash (QSPI), 2MiB PSRAM (QSPI))
 * Environment: Arduino IDE 1.8.10, ESP32 Core V2.0.5, ESPtool.py V4.2.1
 *                                                     NOTE: ESPtool.py only compatible with Arduino IDE 1.x
 * 
 * Suggested Configration:
 * - Board: ESP32S3 Dev Module
 * - Upload Speed: 921600
 * - USB Mode: Hardware CDC and JTAG [IMPORTANT]
 * - USB CDC On Boot: Disabled
 * - USB Firmware MSC On Boot: Disabled
 * - USB DFU On Boot: Disabled
 * - Upload Mode: UART0 / Hardware CDC [IMPORTANT]
 * - CPU Frequency: 240MHz (WiFi)
 * - Flash Mode: QIO 80MHz
 * - Flash Size: 16MB (128Mb) [IMPORTANT]
 * - Partition Scheme: 8M with SPIFFS (3MB APP/1.5MB SPIFFS) [IMPORTANT]
 * - Core Debug Level: None
 * - PSRAM: QSPI PSRAM [IMPORTANT]
 * - Arduino Runs On: Core 1 [IMPORTANT]
 * - Events Run On: Core 1 [IMPORTANT]
 * - Erase All Flash Before Sketch Upload: Disabled
 * 
 * Use the ESP32FS.jar plugin for uploading data to SPIFFS
 * 
 * Onboard Peripherals
 *  - Temperature Sensor
 *  - MicroSD Card
 *  - GPS
 *  - LoRa Radio
 *  - Speaker
 *  - I2C ADC
 *    - Battery Sense
 *    - Current Sense
 *  - RGB LEDs
 *  - Batt voltage
 *  - Current sense
 *  
 */


#include <Arduino.h>
#include "libraries.h"

// Forward declarations
void loopAltCoreHandler(void * pvParameters);
void loopAltCore();
void configWebServer();











void setup() {
  // Configure pinmodes
  pinMode(pins::kDisable5v, OUTPUT);
  pinMode(pins::kDebugLed, OUTPUT);
  pinMode(pins::kGpsReset, OUTPUT);
  digitalWrite(pins::kGpsReset, HIGH); //TODO: NESSESARY??
  pinMode(pins::kMenuBtns, INPUT);
  pinMode(pins::kChrgStat, INPUT);
  analogReadResolution(12);

  // Ensure USB mode is PWR+DATA by default
  setUSBDataOnlyMode(false);

  // Add WS2812B
  rgbLEDs.begin(); // initialize WS2812Bs
  setRGB(0,0,0);
  setLEDBrightness(DEFAULT_LED_BRIGHTNESS);
  // Set power LED to "loading"
  setRGB(0,128,32,0);

  // Start USB Serial
  Serial.begin(USB_BAUD);

  // Prepare SPI busses
  epdSPI.begin(pins::kEinkSck, pins::kEinkMiso, pins::kEinkMosi, pins::kEinkCs);
  rfmSPI.begin(pins::kRfSck, pins::kRfMiso, pins::kRfMosi, pins::kRfCs);
  pinMode(epdSPI.pinSS(), OUTPUT);
  pinMode(rfmSPI.pinSS(), OUTPUT);

  // Initialize E-Paper Display (EPD)
  display.init(EPD_BAUD, true, 2, false, epdSPI, SPISettings(EPD_SPI_CLOCK, MSBFIRST, SPI_MODE0));
  display.setRotation(1);

  display.setTextColor(GxEPD_BLACK);
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextSize(1);
    display.println(String("QRET Kuhglocke    FW=") + FIRMWARE_VERSION);
    display.setCursor(30,20);
    display.setTextSize(7);
    display.println("QRET");
    display.setTextSize(2);
    display.println("     Loading");

    display.setTextSize(1);
    display.setCursor(0,112);
    display.println("POWER  BATT    RADIO   SRADOK   GPS FIX");
    
  } while (display.nextPage());

  // Configure I2C Bus
  Wire.begin(pins::kI2cSda, pins::kI2cScl);
  Wire.setClock(I2C_SPEED);

  // Initialize MicroSD card (1-bit mode)
  SD_MMC.setPins(pins::kSdmmcClk, pins::kSdmmcCmd, pins::kSdmmcD0);
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("[WARN] MicroSD Card Mount Failed");
  } else {
    Serial.println("SD: Card Mount Success");
    makeNextSDLog();
    writeToSDLog("Kuhglocke SD card initialized");
  }//if (SD OK)

  // Initialize NAU7802 ADC
  initNAU7802();

  // Initialize GPS
  gpsInit(); 

  // Initialize RFM95 Radio
  rfmInit();

  // Initialize SPIFFS
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }//if

  // Launch WiFi AP
  WiFi.begin(AP_SSID, AP_PASSWORD);//TODO: Change to names to WIFI_SSID,WIFI_PASSWORD
  WiFi.setTxPower(WIFI_TX_POWER);
  uint32_t wifiConnectStart = millis();
while (WiFi.status() != WL_CONNECTED && millis() - wifiConnectStart < 1000) {  // 1 second timeout
    delay(500);
    Serial.print(".");//conntect status
}// while
 
if (WiFi.status() == WL_CONNECTED) {
    IPAddress IP = WiFi.localIP();
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(IP);
} else {
    Serial.println("\nWiFi connection failed - falling back to AP mode");
    // Fallback to AP mode if station fails
    WiFi.softAP(AP_SSID, AP_PASSWORD, WIFI_CHANNEL);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("Fallback AP IP address: ");
    Serial.println(IP);
}

  // Configure Async web server
  configWebServer();
  webServer.begin();

  
  // Launch background task for Core 0
  xTaskCreatePinnedToCore(
      loopAltCoreHandler,   // Task function
      "Core0Loop",          // Name of the task
      ALT_CORE_STACKS_SIZE, // Stack size
      NULL,                 // Task input parameter
      1,                    // Priority of the task
      &Core0Task,           // Task handle
      0                     // Core number
  );
  

  // Set power LED to "ready"
  setRGB(0,0,128,0); //Turn on power LED
  Serial.println("Kuhglocke Initialization Complete!");
  
}//setup()


/*
 * Code to run repeatedly
 */
void loop() {
  // Note loop start time
  uint32_t loopStart = millis();
  
  
  handleGPS();
  handleReadSensors();
  handleLEDs();
  if (analogRead(pins::kMenuBtns) > 40) {
    setLEDBrightness(255);

  } else {
    setLEDBrightness(DEFAULT_LED_BRIGHTNESS);

  }

  // Check for incoming RFM95 packets
  if (rfmReceivedFlag) {
    // Reset flag
    rfmReceivedFlag = false;
    onRFMReceive();
  }//if (rfm recv)

  // Note loop execution time
  uint16_t core1LoopTime = (millis()-loopStart);
  core1LoopFilter.AddValue(core1LoopTime);
  if (core1LoopTime > maxCore1LoopTime) {maxCore1LoopTime = core1LoopTime;}



}//loop()


/*
 * Run as a task on Core0; runs the alternate loop
 * function indefinitely.
 */
void loopAltCoreHandler(void * pvParameters) {
  Serial.println("Starting background thread on Core0");
  while (true) {
    loopAltCore();
  }//while
}//loopAltCoreHandler()


/*
 * Like loop(), but runs on Core0 of the ESP32
 * 
 * Make sure you use a MUTEX when exchanging
 * lots of info with Core1.
 * 
 * NOTE: STACK SIZE IS LIMITED HERE.
 */
void loopAltCore() {
  // Note loop start time
  uint32_t loopStart = millis();
  
//  Serial.print("AltCore Loop: ");
//  Serial.println(core0FreeStack);
//  delay(100);

  // Check to update EPD
  if (millis() - lastEPDUpdate > EPD_UPDATE_INT) {
    lastEPDUpdate = millis();
    updateEPD();
  }//if

  //for reading sensors or updating the EPD, and maybe playing audio

  // Note free stack space
  core0FreeStack = uxTaskGetStackHighWaterMark(NULL);
  // Note loop execution time
  core0LoopTime = (millis()-loopStart);
  if (core0LoopTime > maxCore0LoopTime) {maxCore0LoopTime = core0LoopTime;}
}//loopAltCore()
