#include "libraries.h"

// Radio options
double freqOpts[] = {902.0, 905.4, 928.0};
uint8_t numFreqOpts = 3;

double bandwidthOpts[] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500};
uint8_t numBandwidthOpts = 10;

int32_t spreadOpts[] = {7, 8, 9, 10, 11, 12};
uint8_t numSpreadOpts = 6;

int32_t codingOpts[] = {5, 6, 7, 8};
uint8_t numCodingOpts = 4;

int32_t freqCorrectionOpts[] = {10000, 8000, 6000, 4000, 2000, 0, -2000, -4000, -6000, -8000, -10000};
uint8_t numFreqCorrectionOpts = 11;

// Selected options
uint8_t freqSelected = 1;
uint8_t bandwidthSelected = 6;
uint8_t spreadSelected = 3;
uint8_t codingSelected = 1;
uint8_t freqCorrectionSelected = 5;

// Incoming packet data
volatile uint32_t rfmLastRFReceived = 0;
volatile int16_t rfmLastRSSI = 0;
volatile float rfmLastSNR = 0;
volatile int32_t rfmLastFreqErr = 0;
byte rfmLastPacket[RFM_PACKET_SIZE] = {0};
volatile bool rfmLastPacketValid = false;

// Decoded rocket packet data
volatile int32_t rocketGPSLat = 0;
volatile int32_t rocketGPSLon = 0;
volatile uint8_t rocketGPSSats = 0;
volatile int32_t rocketAltitude = 0;
volatile uint8_t rocketStatus = 0;
volatile double rocketVelocity = 0;
volatile char rocketCallsign[7] = {'-','-','-','-','-','-','\0'};

uint32_t lastLocalGPSLog = 0;
int32_t curFreqOffset = 0;

// ⚠️ Don’t call millis() in a global initializer (see note below)
uint32_t lastEPDUpdate = 0;

// Core timing / flags
volatile uint32_t core0FreeStack = 0;
volatile uint32_t core0LoopTime = 0;
volatile uint32_t maxCore0LoopTime = 0;
uint32_t maxCore1LoopTime = 0;

volatile bool rfmReceivedFlag = false;

// Sensors / UI
uint32_t lastSensorRead = 0;
bool nauChannel = 0;
bool usbDataOnly = false;

// Indicators
uint32_t indRFFlashStart = 0;
bool indBattFlashState = true;
uint32_t indBattLastToggle = 0;

// Logging
uint16_t logFileNumber = 0;

// ================================================================================================
// ================================== \/ GLOBAL OBJECTS \/ ========================================
// ================================================================================================
// Task handle
TaskHandle_t Core0Task = nullptr;

// Sensor median filters
MedianFilter<uint32_t> vccVoltFilter(8);
MedianFilter<uint32_t> battVoltFilter(8);
MedianFilter<uint32_t> sysCurrentFilter(8); //TODO: CHANGE THIS TO AN AVG FILTER
MedianFilter<uint32_t> ambTempFilter(8);

// Core loop median filter TODO: CHANGE THIS TO AN AVG FILTER
MedianFilter<uint16_t> core1LoopFilter(8);

// GPS
TinyGPSPlus gps;

// UART1 Serial Object
HardwareSerial gpsSerial(1);

// RGB LEDs
 Adafruit_NeoPixel rgbLEDs(NUM_RGB_LEDS, pins::kRgbData, NEO_GRB + NEO_KHZ800);

// E-paper Display Objects (pick class/driver)
SPIClass epdSPI(FSPI);
GxEPD2_BW<GxEPD2_213_B74, GxEPD2_213_B74::HEIGHT>
display(GxEPD2_213_B74(pins::kEinkCs, pins::kEinkDc, pins::kEinkReset, pins::kEinkBusy));

// RFM95 Objects
SPIClass rfmSPI(HSPI);
SPISettings rfmSPISettings(RFM_SPI_CLOCK, MSBFIRST, SPI_MODE0);
static Module radioModule(pins::kRfCs, pins::kRfDio0, pins::kRfReset, pins::kRfDio1);
RFM95 radio(&radioModule);

// NAU7802 ADC object
NAU7802 nau;

// AsyncWebServer object on port 80
AsyncWebServer webServer(80);

// Global log file handle
File logFile;

