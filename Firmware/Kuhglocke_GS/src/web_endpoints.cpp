#include <Arduino.h>
#include "Global.h"
#include "pinouts.h"
#include "power_sensors.h"
#include "Radio_control.h"
#include "User_interface.h"
#include "GPS.h"
#include "SD_MMC.h"
#include "SPIFFS.h"
#include <ESPAsyncWebServer.h>

// Getter prototypes from main.cpp
uint16_t getCore1LoopFiltered();
uint32_t getCore1MaxLoopTime();
uint32_t getCore0FreeStack();
uint32_t getCore0LoopTime();
uint32_t getCore0MaxLoopTime();

static AsyncWebServer s_webServer(80);

String processorDebug(const String& var) {
  if (var == "FirmwareVersion") {
    return FIRMWARE_VERSION;
  } else if (var == "CompileDate") {
    return (String(__DATE__) + ", " + String(__TIME__));
  } else if (var == "WiFiPower") {
    return String("IUnk");
  } else if (var == "WiFiChannel") {
    return String(WIFI_CHANNEL);
  }
  return String();
}

String readSDFile(String path) {
  File file = SD_MMC.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return "";
  }

  String fileContent;
  while (file.available()) {
    fileContent += (char)file.read();
  }
  file.close();
  return fileContent;
}

void configWebServer() {
  // Route for root / web page
  s_webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false, processorDebug);
  });

  // Route for debug web page
  s_webServer.on("/debug", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/debug.html", String(), false, processorDebug);
  });

  // Route for dashboard web page live updates reduce heap usage
  s_webServer.on("/debugData", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncResponseStream *res = request->beginResponseStream("text/plain");

    // Uptime,MainLoopSpeed,MaxMainLoopSpeed,FreeHeap,AmbientTemp
    res->print(millis());
    res->print(','); res->print(getCore1LoopFiltered());
    res->print(','); res->print(getCore1MaxLoopTime());
    res->print(','); res->print(ESP.getFreeHeap() / 1024.0, 1);
    res->print(','); res->print(getAmbTemperature());

    // PSUVolt,BattVolt,BattSoC,SysCurrent,BattStatus,USBMode
    res->print(','); res->print(getPSUVoltage());
    uint16_t battVolt = getBatteryVoltage();
    res->print(','); res->print(battVolt);
    res->print(','); res->print(voltToPercent(battVolt));
    res->print(','); res->print(getSystemCurrent());

    uint8_t chrgStat = getChargingStatus();
    res->print(',');
    if      (chrgStat == 0) res->print("Charging");
    else if (chrgStat == 1) res->print("Fully Charged");
    else                    res->print("Discharging");

    res->print(',');
    res->print(isUsbDataOnly() ? "Data Only" : "Power + Data");

    // FixAge,NumSat,Lat,Lon,Alt,Timestamp
    res->print(','); res->print(getGPSAge());
    res->print(','); res->print(getGPSSats());
    res->print(','); res->print(getGPSLat(), 6);
    res->print(','); res->print(getGPSLng(), 6);
    res->print(','); res->print(getGPSAlt());
    res->print(','); res->print(1);

    // SDPresent,SDCapacity,SDAvailable,LogID,SPIFFSSize,SPIFFSFree
    if (SD_MMC.cardType() == CARD_NONE) {
      res->print(",No,?,?,?");
    } else {
      res->print(",Yes");
      uint32_t sdSize = SD_MMC.totalBytes() / (1024 * 1024);
      uint32_t sdUsed = SD_MMC.usedBytes()  / (1024 * 1024);
      uint32_t sdFree = sdSize - sdUsed;
      res->print(','); res->print(sdSize);
      res->print(','); res->print(sdFree);
      res->print(','); res->print(getLogFileNumber());
    }
    res->print(','); res->print(SPIFFS.totalBytes() / 1024.0, 1);
    uint32_t freeSpace = SPIFFS.totalBytes() - SPIFFS.usedBytes();
    res->print(','); res->print(freeSpace / 1024.0, 1);

    // LoRaFreq,LoRaBand,LoRaSF,LastPingTime,LastPacket,LastRSSI,LastSNR,LastFreqErr,LastPcktValid,AFCOn
    res->print(','); res->print(getRadioFreq(), 2);
    res->print(','); res->print(getRadioBandwidth(), 2);
    res->print(','); res->print(getRadioSF());
    res->print(','); res->print(millis() - getRfmLastRFReceived());

    // Hex dump without building a String
    res->print(',');
    const uint8_t* lastPacket = getRfmLastPacket();
    for (int i = 0; i < RFM_PACKET_SIZE; i++) {
      uint8_t b = lastPacket[i];
      const char hex[] = "0123456789ABCDEF";
      res->print(hex[b >> 4]);
      res->print(hex[b & 0x0F]);
    }

    res->print(','); res->print(getRfmLastRSSI());
    res->print(','); res->print(getRfmLastSNR());
    res->print(','); res->print(getRfmLastFreqErr());
    res->print(','); res->print(isRfmLastPacketValid() ? "Yes" : "NO");
    res->print(','); res->print(false ? "Yes" : "NO"); // AFCOn placeholder

    // RcktSats,RcktLat,RcktLon,RcktAltm,RcktStatus,RcktCallsign
    res->print(','); res->print(getRocketGPSSats());
    res->print(','); res->print(getRocketLatDeg(), 6);
    res->print(','); res->print(getRocketLonDeg(), 6);
    res->print(','); res->print(getRocketAltitudeMeters(), 1);
    res->print(','); res->print(getRocketStatus(), BIN);
    res->print(','); res->print("QRET");

    res->print(','); res->print(getCurFreqOffset());
    res->print(','); res->print(getCore0FreeStack());
    res->print(','); res->print(getCore0LoopTime());
    res->print(','); res->print(getRocketVelocity());

    request->send(res);
  });

  // Route to load style.css file
  s_webServer.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });

  // Route to load terms file
  s_webServer.on("/terms", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/terms.html", "text/html");
  });

  // Route to load qret.svg file
  s_webServer.on("/qret.svg", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/qret.svg", "image/svg+xml");
  });

  // Route for frequency offset increase
  s_webServer.on("/FreqUp", HTTP_GET, [](AsyncWebServerRequest *request){
    changeFreqOffset(1000);
    request->send(200, "text/plain", "OK");
  });

  // Route for frequency offset decrease
  s_webServer.on("/FreqDown", HTTP_GET, [](AsyncWebServerRequest *request){
    changeFreqOffset(-1000);
    request->send(200, "text/plain", "OK");
  });

  // Route for reload radio
  s_webServer.on("/RadioReload", HTTP_GET, [](AsyncWebServerRequest *request){
    rfmInit();
    request->send(200, "text/plain", "OK");
  });

  // Route for downloading logs
  s_webServer.on("/log", HTTP_GET, [](AsyncWebServerRequest *request){
    const AsyncWebParameter* p = request->getParam(0);
    if (p->name() == "id") {
      String filename = "/logs/Kuhglocke_Log" + String(p->value()) + ".txt";
      File file = SD_MMC.open(filename);
      if (!file) {
        request->send(404, "text/plain", "Cannot load file.");
        return;
      }

      AsyncWebServerResponse *response = request->beginChunkedResponse("text/plain", 
        [file = std::move(file)](uint8_t *buffer, size_t maxLen, size_t index) mutable -> size_t {
          if (!file) {
            return 0;
          }
          size_t length = file.read(buffer, maxLen);
          if (length == 0) {
            file.close();
          }
          return length;
        });
      request->send(response);
      return;
    }
    request->send(404, "text/plain", "No id parameter given.");
  });

  // Route for changing radio settings
  s_webServer.on("/RadioConfig", HTTP_GET, [](AsyncWebServerRequest *request){
    const AsyncWebParameter* p = request->getParam(0);
    setRadioConfig(p->name(), p->value().toInt());
    request->send(200, "text/plain", "OK");
  });
}

void startWebServer() {
  configWebServer();
  s_webServer.begin();
}
