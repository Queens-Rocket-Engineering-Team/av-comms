#pragma once

#include "pinouts.h"
#include <Wire.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include <RadioLib.h>
#include <GxEPD2_BW.h> // https://github.com/ZinggJM/GxEPD2
#include "GxEPD2_display_selection_new_style.h" //TODO: Investigate how much of this is nessesary to include.
#include "GxEPD2_selection_check.h"
#include "SD_MMC.h"
#include <NAU7802_2CH.h> //https://github.com/Kenneract/NAU7802_Arduino_2CH
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include <MedianFilterLib.h> //https://github.com/luisllamasbinaburo/Arduino-MedianFilter
#include <math.h>
#include "GPS.h"
#include "Radio_control.h"
#include "Screen_control.h"
#include "User_interface.h"
#include "battery_managment.h"
#include "Global.h"
#include "Sensor_handling.h"