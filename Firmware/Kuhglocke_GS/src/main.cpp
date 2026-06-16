#include <Arduino.h>
#include "Global.h"
#include "pinouts.h"
#include "power_sensors.h"
#include "Radio_control.h"
#include "Screen_control.h"
#include "User_interface.h"
#include "GPS.h"
#include "SD_MMC.h"
#include "SPIFFS.h"
#include "WiFi.h"
#include <Wire.h>
#include <MedianFilterLib.h>


// Forward declarations of Core0 entrypoint
static void loopAltCoreHandler(void* pvParameters);
static void loopAltCore();

// Declared in web_endpoints.cpp
void startWebServer();

// Static Task Handles and stats
static TaskHandle_t s_core0Task = nullptr;
static MedianFilter<uint16_t> s_core1LoopFilter(8);

static volatile uint32_t s_core0FreeStack = 0;
static volatile uint32_t s_core0LoopTime = 0;
static volatile uint32_t s_core0MaxLoopTime = 0;
static uint32_t s_core1MaxLoopTime = 0;
static uint32_t s_lastEPDUpdate = 0;

void setup() {
  // Configure pinmodes
  pinMode(pins::kDisable5v, OUTPUT);
  pinMode(pins::kDebugLed, OUTPUT);
  pinMode(pins::kGpsReset, OUTPUT);
  digitalWrite(pins::kGpsReset, HIGH);
  pinMode(pins::kMenuBtns, INPUT);
  pinMode(pins::kChrgStat, INPUT);
  analogReadResolution(12);

  // Ensure USB mode is PWR+DATA by default
  setUSBDataOnlyMode(false);

  // Initialize LEDs
  initLEDs();
  setRGB(0, 128, 32, 0); // Power LED to loading

  // Start USB Serial
  Serial.begin(USB_BAUD);

  // Initialize E-Paper Display
  screenInit();
  drawLoadingScreen();

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
  }

  // Initialize NAU7802 ADC
  initPowerSensors();

  // Initialize GPS
  gpsInit(); 

  // Initialize RFM95 Radio
  rfmInit();

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Launch WiFi AP
  WiFi.begin(AP_SSID, AP_PASSWORD);
  WiFi.setTxPower(WIFI_TX_POWER);
  uint32_t wifiConnectStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiConnectStart < 1000) {  // 1 second timeout
    delay(500);
    Serial.print(".");
  }
 
  if (WiFi.status() == WL_CONNECTED) {
    IPAddress IP = WiFi.localIP();
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(IP);
  } else {
    Serial.println("\nWiFi connection failed - falling back to AP mode");
    WiFi.softAP(AP_SSID, AP_PASSWORD, WIFI_CHANNEL);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("Fallback AP IP address: ");
    Serial.println(IP);
  }

  // Configure Async web server
  startWebServer();
  
  // Launch background task for Core 0
  xTaskCreatePinnedToCore(
      loopAltCoreHandler,   // Task function
      "Core0Loop",          // Name of the task
      ALT_CORE_STACKS_SIZE, // Stack size
      NULL,                 // Task input parameter
      1,                    // Priority of the task
      &s_core0Task,         // Task handle
      0                     // Core number
  );

  // Set power LED to "ready"
  setRGB(0, 0, 128, 0); // Turn on power LED
  Serial.println("Kuhglocke Initialization Complete!");
}

void loop() {
  uint32_t loopStart = millis();
  
  handleGPS();
  handleReadPowerSensors();
  handleLEDs();
  
  if (analogRead(pins::kMenuBtns) > 40) {
    setLEDBrightness(255);
  } else {
    setLEDBrightness(DEFAULT_LED_BRIGHTNESS);
  }

  // Check for incoming RFM95 packets
  if (getRfmReceivedFlag()) {
    clearRfmReceivedFlag();
    onRFMReceive();
  }

  uint16_t core1LoopTime = (millis() - loopStart);
  s_core1LoopFilter.AddValue(core1LoopTime);
  if (core1LoopTime > s_core1MaxLoopTime) {
    s_core1MaxLoopTime = core1LoopTime;
  }
}

static void loopAltCoreHandler(void* pvParameters) {
  Serial.println("Starting background thread on Core0");
  while (true) {
    loopAltCore();
  }
}

static void loopAltCore() {
  uint32_t loopStart = millis();
  
  if (millis() - s_lastEPDUpdate > EPD_UPDATE_INT) {
    s_lastEPDUpdate = millis();
    updateEPD();
  }

  s_core0FreeStack = uxTaskGetStackHighWaterMark(NULL);
  s_core0LoopTime = (millis() - loopStart);
  if (s_core0LoopTime > s_core0MaxLoopTime) {
    s_core0MaxLoopTime = s_core0LoopTime;
  }
}

// Accessors for loop and core statistics
uint16_t getCore1LoopFiltered() {
  return s_core1LoopFilter.GetFiltered();
}

uint32_t getCore1MaxLoopTime() {
  return s_core1MaxLoopTime;
}

uint32_t getCore0FreeStack() {
  return s_core0FreeStack;
}

uint32_t getCore0LoopTime() {
  return s_core0LoopTime;
}

uint32_t getCore0MaxLoopTime() {
  return s_core0MaxLoopTime;
}
