/*
 * Authors: Kennan Bays, Ethan Toste
 * Hardware: QRET Camera Module Rev1 (OCELLUS)
 * Env: Arduino 1.8.10, STM32duino 2.7.1
 * Created: ~May.31.2025
 * Updated: Aug.04.2025
 * Purpose: SRAD firmware for camera module Rev1 (OCELLUS).
*/


#include <Wire.h>
#include <SPI.h>
#include <Tone.h>
#include <SerialFlash.h>
#include <SoftwareSerial.h>
#include "pinouts.h"
#include "CANpackets.h"
#include "STM32_CAN.h" //IMPORTANT: DO NOT DOWNLOAD THE LATEST VERSION (DOWNLOAD 1.1.2)
#include <HardwareSerial.h>

// Define the serial port for Tramp protocol communication
#define VTX_CONTROL_PIN PB10  // Button control pin

#define USB_BAUD_RATE 115200
#define LIVE_CAM_BAUD_RATE 9600

// Tramp protocol command structure
struct TrampCommand {
  uint8_t header;
  uint8_t command;
  uint8_t data[4];
  uint8_t checksum;
};

// VTX Configuration
enum {
  BAND_A = 0,
  BAND_B = 1
};

const uint16_t BAND_A_FREQS[8] = {1080, 1120, 1160, 1200, 1240, 1280, 1320, 1360};
const uint16_t BAND_B_FREQS[8] = {1200, 1220, 1240, 1258, 1280, 1300, 1320, 1340};

const uint16_t POWER_LEVELS[5] = {0, 25, 200, 1000, 4000}; // PIT, 25mW, 200mW, 1000mW, 4000mW

// Current VTX settings
uint8_t currentBand = BAND_A;
uint8_t currentChannel = 4; // Default to channel 5 (1240MHz)
uint8_t currentPower = 2;   // Default to 200mW
bool pitMode = false;

//VARIABLES
//buzzer
const uint32_t BEEP_DELAY = 8000;
const uint32_t BEEP_LENGTH = 1000;
const uint32_t BEEP_FREQ = 1000;

// //canbus
const uint32_t STATUS_REPORT_DELAY = 1000; //How frequently the status report should be sent
uint32_t lastStatusReport = 0; //When the last status report sent

// //flash settings
uint16_t flashDelay = 250; // How frequently the debug LED should be toggled
uint32_t lastFlash = 0; // Last millis() the debug LED was toggle at

//physical run-cam camera settings
const uint32_t CAM_RECORD_DELAY = 360000; //5min after power on
const uint32_t CAM_POWER_TIME = 10800000; //3hrs after record start

//video transmission settings
SoftwareSerial trampSerial(UNUSED_PIN, VTX_DATA_PIN); // RX, TX
const uint8_t BAND = 0; //Band B = 1
const uint8_t CHANNEL = 2; //Channel 3 = 2 (1240 MHz)
uint8_t flightStage = 0;
uint8_t prevFlightStage = 0;
uint8_t powerLevel = 2;

//thermistor
const float Vcc = 3.3;             // STM32 3.3V (TODO: Change this to uint16 & millivolts; works faster)
const float R_FIXED = 10000.0;     // Fixed resistor value (10k)
const float BETA = 3435.0;         // B value for NRNE104F3435B2F
const float T0 = 298.15;           // Reference temp (25°C in Kelvin)
const float R0 = 10000.0;          // Thermistor resistance at 25°C
const float calibrationValue = 2;
bool powerTurnedOff = false;

// USB interface
HardwareSerial usb(USB_RX_PIN, USB_TX_PIN);

//GLOBAL VARIABLES
STM32_CAN canb( CAN1, ALT );    //CAN1 ALT is PB8+PB9
static CAN_message_t CAN_TX_msg ;
static CAN_message_t CAN_RX_msg;


void setup(){
  // Setup pinmodes
  pinMode(BUZZER_PIN_A, OUTPUT);
  pinMode(BUZZER_PIN_B, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(CAMERA_POWER_PIN, OUTPUT);
  pinMode(VTX_PWR_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
  digitalWrite(VTX_PWR_PIN, HIGH);

  // Start USB debugging  
  usb.begin(USB_BAUD_RATE);

  // Init buzzer
  initBuzzer();
  setBuzzerFreq(BEEP_FREQ);

  // Configure SPI
  SPI.setSCLK(FLASH_SCK_PIN);
  SPI.setMISO(FLASH_MISO_PIN);
  SPI.setMOSI(FLASH_MOSI_PIN);

  //LED FLASHES TO ALLOW FOR SERIAL OPEN
  for (int i=0 ; i <5 ; i++){
    digitalWrite(STATUS_LED_PIN,HIGH);
    delay(250);
    digitalWrite(STATUS_LED_PIN,LOW);
    delay(250);
  }//for

  // STARTUP BEEP
  delay(BEEP_DELAY);
  startBuzzer();
  delay(1000);
  stopBuzzer();

  analogReadResolution(12);

  usb.println("[MDE] ENTER D FOR DEBUG");
  digitalWrite(STATUS_LED_PIN,HIGH);

  uint32_t startTime = millis();
  while (!usb.available() and millis()-startTime < 5000) {}

  // Prompt for entering debug mode
  if (usb.available()) {
    byte d = usb.read();
    emptySerialBuffer();
    usb.println("[MDE] Entered Debug Mode");
    debugMode();
    while (true) {}
  }//if

  digitalWrite(STATUS_LED_PIN,LOW);

  //Canbus Setup
  //Start CANBUS
  canb.begin(); //automatic retransmission can be done using arg "true"
  canb.setBaudRate(500000); //500kbps

  // Initialize Tramp protocol serial (8N1, 9600 baud)
  Serial2.setTx(PB10);  // Explicit TX pin
  Serial2.setRx(PB11);  // Optional (not needed for Tramp TX)
  Serial2.begin(9600);
  
  delay(1000);  // Let serial ports stabilize
  
  // // Send PIT mode command with verification
  // if (setPitMode(true)) {
  //   usb.println("PIT mode activated (Green LED should be solid)");
  // } else {
  //   usb.println("Failed! Check wiring/baud rate");
  // }
}



// /*---\/---\/---\/---\/---*\
//       |  DEBUG MENU |    
// \*---/\---/\---/\---/\---*/

// Empties all bytes from incoming serial buffer.
// Used by Debug mode
void emptySerialBuffer() {
  while (usb.available()) {usb.read();}
}//emptySerialBuffer()

// Called when Debug mode is activated;
// all normal board functions will cease.
// Only purpose is to respond to serial
// commands.
void debugMode() {

  // Status LED static "ON"
  digitalWrite(STATUS_LED_PIN, HIGH);

  while (true) {
    emptySerialBuffer();
    while (!usb.available()) {}
    uint8_t cmd = usb.read();

    if(cmd == 'Z') {
      // "Identify" command; return Sucessful
      usb.println("TEST SUCESSFUL");
    }//if
    if(cmd == 'T'){
      usb.println(readThermistorTemperature());
    }
    if (cmd == 'I') {
      // "Identify" command; return board name
      usb.println(F("[MDE] CAMERA"));
    }
    if (cmd == 'F') {
      digitalWrite(VTX_PWR_PIN, LOW);
    }
    if (cmd == 'O') {
      digitalWrite(VTX_PWR_PIN, HIGH);
    }
    if (cmd == 'A') {
      usb.println(F(getFlightStatus()));
    }
    
  
  }//while

}//debugMode()


void loop(){

  //RUNCAM PHYSICAL CODE
  // Check if camera power should be given
  // TODO: Make this a bit more efficient? Only trigger at a certain time?
  if (millis() > CAM_RECORD_DELAY+CAM_POWER_TIME) {
    //turn off camera
    digitalWrite(CAMERA_POWER_PIN, LOW);
    Serial.println("CAM OFF");
  } else if (millis() > CAM_RECORD_DELAY) {
    //turn on camera
    digitalWrite(CAMERA_POWER_PIN, HIGH);
    Serial.println("CAM ON");
  }//if

  if(readThermistorTemperature() > 75 && powerTurnedOff == false){
    digitalWrite(VTX_PWR_PIN, LOW);
    powerTurnedOff = true;
  }
  else if(readThermistorTemperature() < 55 && powerTurnedOff == true){
    digitalWrite(VTX_PWR_PIN, HIGH);
    powerTurnedOff = true;
  }

  flightStage = getFlightStatus();

  //Checking if it is time to report status to CANBUS
  if(millis() - lastStatusReport > STATUS_REPORT_DELAY){
    lastStatusReport = millis();
    sendCANStatus(); //Reporting status
  }//if

//  if(flightStage == 5 || flightStage == NULL){
//    digitalWrite(VTX_PWR_PIN, LOW);
//  }else{
//    digitalWrite(VTX_PWR_PIN, HIGH);
//  }

  if(millis() == 18000000){
    digitalWrite(VTX_PWR_PIN, LOW);
  }
}


float readThermistorTemperature(){
  int adcVal = analogRead(V_THERM_PIN);   // Read ADC pin

  float voltage = adcVal * Vcc / 4095.0;     // STM32 12-bit ADC
  float R_thermistor = (voltage * R_FIXED) / (Vcc - voltage);  // Low-side formula

  float tempK = 1.0 / (1.0/T0 + (1.0/BETA) * log(R_thermistor / R0));
  float tempC = tempK - 273.15;
  tempC -= 2;
  return tempC;
}

int getFlightStatus(){
    while (canb.read(CAN_RX_msg)) {
    if (CAN_RX_msg.id == ALTIMETER_MOD_CANID+FLIGHT_STAGE_CANID) {
      //Serial.println(F("Recv FLGHT STAGE"));
      return CAN_RX_msg.buf[0];
      return flightStage;
    } 
    //if  
  } //while
}

bool setPitMode(bool enable) {
  uint8_t powerLevel = enable ? 0x00 : 0x02;
  uint8_t packet[] = {
    0xAA,
    0x02,
    powerLevel,
    0x00,
    0x00,
    0xAA ^ 0x02 ^ powerLevel
  };

  Serial2.write(packet, sizeof(packet));
  delay(50);
  
  return verifyPowerLevel(powerLevel);
}

bool verifyPowerLevel(uint8_t expectedLevel) {
  uint8_t query[] = {0xAA, 0x01, 0x00, 0x00, 0x00, 0xAB};
  Serial2.write(query, sizeof(query));
  delay(50);
  
  if (Serial2.available() >= 6) {
    uint8_t response[6];
    Serial2.readBytes(response, 6);
    return (response[2] == expectedLevel);
  }
  return false;
}

// ---CANBUS 
void sendCANStatus(){
    //Build the CANBUS message
    CAN_TX_msg.id = (CAMERA_MOD_CANID+STATUS_CANID);
    CAN_TX_msg.len = 1;

    CAN_TX_msg.buf[0] = 1;

    canb.write(CAN_TX_msg); //send
} //sendCANStatus()





//IMPORTS
// #include <Wire.h>
// #include <SPI.h>
// #include <Adafruit_Sensor.h>
// #include "Adafruit_BME680.h"
// #include <Tone.h>
// #include <SerialFlash.h>
// #include <SoftwareSerial.h>
// #include "pinouts.h"
// #include "CANpackets.h"
// #include "STM32_CAN.h"

// //VARIABLES
// //buzzer
// const uint32_t BEEP_DELAY = 9000;
// const uint32_t BEEP_LENGTH = 1000;
// const uint32_t BUZZER_TONE = 1000;
// const uint32_t BUZZER_TONE_Q = 500;

// //canbus
// const uint32_t STATUS_REPORT_DELAY = 1000; //How frequently the status report should be sent
// uint32_t lastStatusReport = 0; //When the last status report sent

// //flash settings
// uint16_t flashDelay = 250; // How frequently the debug LED should be toggled
// uint32_t lastFlash = 0; // Last millis() the debug LED was toggle at

// //physical run-cam camera settings
// const uint32_t CAM_RECORD_DELAY = 360000; //5min after power on
// const uint32_t CAM_POWER_TIME = 10800000; //3hrs after record start

// //video transmission settings
// SoftwareSerial trampSerial(-1, VTX_DATA_PIN); // RX, TX
// const uint8_t BAND = 0; //Band B = 1
// const uint8_t CHANNEL = 2; //Channel 3 = 2 (1240 MHz)
// uint8_t flightStage = 0;
// uint8_t prevFlightStage = 0;
// uint8_t powerLevel = 2;

// //thermistor
// const float THERMISTOR_NOMINAL = 10000.0; // Resistance at 25°C
// const float TEMPERATURE_NOMINAL = 25.0;   // Nominal temperature (25°C)
// const float B_COEFFICIENT = 3950.0;       // Beta coefficient (check datasheet)
// const float ADC_MAX = 4095.0;             // 12-bit ADC for most STM32
// const float MAXIMUM_ALLOWABLE_TEMP = 40.0; //Maximum Temperature of the VTX before emergency shutoff (IN CELSIUS)

// //GLOBAL VARIABLES
// STM32_CAN canb( CAN1, ALT );    //CAN1 ALT is PB8+PB9
// static CAN_message_t CAN_TX_msg ;
// static CAN_message_t CAN_RX_msg;



// void setup(){
//   // Configure pinmodes
//   pinMode(BUZZER_PIN_A, OUTPUT);
//   pinMode(STATUS_LED_PIN, OUTPUT);
//   pinMode(CAMERA_POWER_PIN, OUTPUT);

//   // // Start USB debugging
//   // // Configure I2C bus
//   // usb.begin(115200);

//   // // Configure SPI
//   // SPI.setSCLK(FLASH_SCK_PIN);
//   // SPI.setMISO(FLASH_MISO_PIN);
//   // SPI.setMOSI(FLASH_MOSI_PIN);

//   // //Configure Buzzer
//   digitalWrite(BUZZER_PIN_B,LOW);

//   //LED FLASHES TO ALLOW FOR SERIAL OPEN
//   for (int i=0 ; i <5 ; i++){
//     digitalWrite(STATUS_LED_PIN,HIGH);
//     delay(250);
//     digitalWrite(STATUS_LED_PIN,LOW);
//     delay(250);
//   }//for

//   // STARTUP BEEP
//   delay(BEEP_DELAY);
//   tone(BUZZER_PIN_A, BUZZER_TONE);
//   delay(BEEP_LENGTH);
//   noTone(BUZZER_PIN_A);
//   usb.println("STARTED");

  // usb.println("[MDE] ENTER D FOR DEBUG");
  // digitalWrite(STATUS_LED_PIN,HIGH);

  // uint32_t startTime = millis();
  // while (!usb.available() and millis()-startTime < 5000) {}

  // // Prompt for entering debug mode
  // if (usb.available()) {
  //   byte d = usb.read();
  //   emptySerialBuffer();
  //   usb.println("[MDE] Entered Debug Mode");
  //   debugMode();
  //   while (true) {}
  // }//if

  // digitalWrite(STATUS_LED_PIN,LOW);

  // //Canbus Setup
  // //Start CANBUS
  // canb.begin(); //automatic retransmission can be done using arg "true"
  // canb.setBaudRate(500000); //500kbps

  //VTX Startup (Set Frequency and Power)
  // trampSerial.begin(LIVE_CAM_BAUD_RATE);
  // delay(1000); // Wait for VTX to power up
  // sendTrampSetCommand(BAND, CHANNEL, 2); //Power Level = 2 (200mW)

  //Thermistor
  // analogReadResolution(12); // Set ADC to 12-bit resolution if available

//}//setup()



/*---\/---\/---\/---\/---*\
      |  DEBUG MENU |    
\*---/\---/\---/\---/\---*/

// Empties all bytes from incoming serial buffer.
// Used by Debug mode
// void emptySerialBuffer() {
//   while (usb.available()) {usb.read();}
// }//emptySerialBuffer()

// // Called when Debug mode is activated;
// // all normal board functions will cease.
// // Only purpose is to respond to serial
// // commands.
// void debugMode() {

//   // Status LED static "ON"
//   digitalWrite(STATUS_LED_PIN, HIGH);

//   while (true) {
//     emptySerialBuffer();

//     while (!usb.available()) {}
//     uint8_t cmd = usb.read();

//     if(cmd == 'Z') {
//       // "Identify" command; return Sucessful
//       usb.println("TEST SUCESSFUL");
//     }//if
//     if(cmd == 'T'){
//       usb.println(readThermistorTemperature());
//     }
//     if (cmd == 'I') {
//       // "Identify" command; return board name
//       usb.println(F("[MDE] CAMERA"));
//     }
  
//   }//while

// }//debugMode()



// void sendTrampSetCommand(uint8_t band, uint8_t channel, uint8_t power) {
//   // Based on ImmersionRC Tramp protocol format
//   uint8_t frame[18];

//   frame[0] = 0xAA;
//   frame[1] = 0x55;
//   frame[2] = 0x01; // Command ID: Set VTX
//   frame[3] = 0x0D; // Payload size (13 bytes)

//   // Payload
//   frame[4] = band;
//   frame[5] = channel;
//   frame[6] = power;
//   frame[7] = 0; // Pit mode off
//   frame[8] = 0x00; // freq_lsb (optional if band+channel used)
//   frame[9] = 0x00; // freq_msb
//   frame[10] = 0; // unused
//   frame[11] = 0; // unused
//   frame[12] = 0; // unused
//   frame[13] = 0; // unused
//   frame[14] = 0; // unused
//   frame[15] = 0; // unused

//   // Checksum (XOR of bytes 2–15)
//   uint8_t checksum = 0;
//   for (int i = 2; i <= 15; i++) {
//     checksum ^= frame[i];
//   }
//   frame[16] = checksum;

//   // Footer
//   frame[17] = 0xEE;

//   // Send frame
//   for (int i = 0; i < 18; i++) {
//     trampSerial.write(frame[i]);
//   }
// }

// //Calculate Temperature
// float readThermistorTemperature() {
//   // Read ADC value (0-4095 for 12-bit)
//   int adcValue = analogRead(V_THERM_PIN);
  
//   // Handle potential divide-by-zero or invalid readings
//   if (adcValue == 0) {
//     return -273.15; // Return absolute zero to indicate error
//   }
  
//   // Calculate thermistor resistance
//   float resistance = THERMISTOR_NOMINAL * ((ADC_MAX / (float)adcValue) - 1.0);
  
//   // Steinhart-Hart equation to convert to temperature
//   float steinhart = resistance / THERMISTOR_NOMINAL;    // (R/Ro)
//   steinhart = log(steinhart);                          // ln(R/Ro)
//   steinhart /= B_COEFFICIENT;                          // 1/B * ln(R/Ro)
//   steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15);   // + (1/To)
//   steinhart = 1.0 / steinhart;                         // Invert
//   float temperature = steinhart - 273.15;              // Convert to Celsius
  
//   return temperature;
// }

// // ---CANBUS 
// void sendCANStatus(){

//     //Build the CANBUS message
//     CAN_TX_msg.id = (CAMERA_MOD_CANID+STATUS_CANID);
//     CAN_TX_msg.len = 1;

//     CAN_TX_msg.buf[0] = 1;

//     canb.write(CAN_TX_msg); //send
// } //sendCANStatus()
