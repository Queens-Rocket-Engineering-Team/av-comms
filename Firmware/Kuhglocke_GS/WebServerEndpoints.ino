/*
 * File containing web server configuration.
 * 
 * To be appended to main file upon compilation.
 */



// Replaces placeholders in the Debug page's
// HTML before serving it.
// Called for each placeholder
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
}//processorDebug()



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
}//readSDFile



/*
 * Sets up all the required endpoints for
 * the async webserver
 */
void configWebServer() {

  // Route for root / web page
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false, processorDebug);
  });

  // Route for debug web page
  webServer.on("/debug", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/debug.html", String(), false, processorDebug);
  });

  // Route for dashboard web page live updates reduce heap usage
  webServer.on("/debugData", HTTP_GET, [](AsyncWebServerRequest *request){
  AsyncResponseStream *res = request->beginResponseStream("text/plain");

  // Uptime,MainLoopSpeed,MaxMainLoopSpeed,FreeHeap,AmbientTemp
  res->print(millis());
  res->print(','); res->print(core1LoopFilter.GetFiltered());
  res->print(','); res->print(maxCore1LoopTime);
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
  res->print(usbDataOnly ? "Data Only" : "Power + Data");

  // FixAge,NumSat,Lat,Lon,Alt,Timestamp
  res->print(','); res->print(gps.location.age());
  res->print(','); res->print(gps.satellites.value());
  res->print(','); res->print(gps.location.lat(), 6);
  res->print(','); res->print(gps.location.lng(), 6);
  res->print(','); res->print(gps.altitude.meters());
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
    res->print(','); res->print(logFileNumber);
  }
  res->print(','); res->print(SPIFFS.totalBytes() / 1024.0, 1);
  uint32_t freeSpace = SPIFFS.totalBytes() - SPIFFS.usedBytes();
  res->print(','); res->print(freeSpace / 1024.0, 1);

  // LoRaFreq,LoRaBand,LoRaSF,LastPingTime,LastPacket,LastRSSI,LastSNR,LastFreqErr,LastPcktValid,AFCOn
  res->print(','); res->print(freqOpts[freqSelected], 2);
  res->print(','); res->print(bandwidthOpts[bandwidthSelected], 2);
  res->print(','); res->print(spreadOpts[spreadSelected]);
  res->print(','); res->print(millis() - rfmLastRFReceived);

  // Hex dump without building a String
  res->print(',');
  for (int i = 0; i < RFM_PACKET_SIZE; i++) {
    uint8_t b = rfmLastPacket[i];
    const char hex[] = "0123456789ABCDEF";
    res->print(hex[b >> 4]);
    res->print(hex[b & 0x0F]);
  }

  res->print(','); res->print(rfmLastRSSI);
  res->print(','); res->print(rfmLastSNR);
  res->print(','); res->print(rfmLastFreqErr);
  res->print(','); res->print(rfmLastPacketValid ? "Yes" : "NO");
  res->print(','); res->print(false ? "Yes" : "NO"); // AFCOn placeholder

  // RcktSats,RcktLat,RcktLon,RcktAltm,RcktStatus,RcktCallsign
  res->print(','); res->print(rocketGPSSats);
  res->print(','); res->print(rocketGPSLat / 1000000.0, 6);
  res->print(','); res->print(rocketGPSLon / 1000000.0, 6);
  res->print(','); res->print(rocketAltitude);
  res->print(','); res->print(rocketStatus, BIN);
  res->print(','); res->print((const char*)rocketCallsign);

  res->print(','); res->print(curFreqOffset);
  res->print(','); res->print(core0FreeStack);
  res->print(','); res->print(core0LoopTime);
  res->print(','); res->print(rocketVelocity);

  request->send(res);
});


  // Route to load style.css file
  webServer.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });

  // Route to load style.css file
  webServer.on("/terms", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/terms.html", "text/html");
  });

  // Route to load qret.svg file
  webServer.on("/qret.svg", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/qret.svg", "image/svg+xml");
  });



  // Route for frequency offset increase
  webServer.on("/FreqUp", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
    curFreqOffset += 1000;
  });

  // Route for frequency offset increase
  webServer.on("/FreqDown", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
    curFreqOffset -= 1000;
  });

  // Route for reload radio
  webServer.on("/RadioReload", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
    rfmInit();
  });

  // Route for downloading logs
  webServer.on("/log", HTTP_GET, [](AsyncWebServerRequest *request){
    // Ensure valid request
    const AsyncWebParameter* p = request->getParam(0);
    if (p->name() == "id") {
      // Generate requested log file name
      String filename = "/logs/Kuhglocke_Log" + String(p->value()) +".txt";

      // Open log file
      File file = SD_MMC.open(filename);
      if (!file) {
        request->send(404, "text/plain", "Cannot load file.");
        return;
      }//if

      // Ensure the file stays open throughout the response lifecycle
      AsyncWebServerResponse *response = request->beginChunkedResponse("text/plain", 
        [file = std::move(file)](uint8_t *buffer, size_t maxLen, size_t index) mutable -> size_t {
          if (!file) {
            return 0; // File is not open, return 0 to indicate end of data
          }
          size_t length = file.read(buffer, maxLen);
          if (length == 0) {
            file.close(); // Close the file when done
          }
          return length;
        });
      request->send(response);

    }//if
    request->send(404, "text/plain", "No id paramter given.");
  });

  // Route for changing radio settings
  webServer.on("/RadioConfig", HTTP_GET, [](AsyncWebServerRequest *request){
    // Ensure valid request
    const AsyncWebParameter* p = request->getParam(0);

    if (p->name() == "bandwidth") {
      uint16_t bwInd = p->value().toInt();
      bandwidthSelected = bwInd;
    } else if (p->name() == "spreadingfactor") {
      uint16_t sfInd = p->value().toInt();
      spreadSelected = sfInd;
    } else if (p->name() == "codingrate") {
      uint16_t crInd = p->value().toInt();
      codingSelected = crInd;
    }//if

    request->send(200, "text/plain", "OK");
  });
  
}//configWebServer
