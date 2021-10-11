#include <RadioLib.h>
#include <avr/dtostrf.h>
#include <Adafruit_BMP085.h>
#include "SparkFun_Ublox_Arduino_Library.h"
#include "SparkFunLIS3DH.h"
#include <SPI.h>
#include <MemoryFree.h>;
#include <Adafruit_SSD1306.h>

// SX1262 LoRa Module has the following connections:
// NSS pin:   8
// DIO1 pin:  3
// NRST pin:  9
// BUSY pin:  2
SX1262 lora = new Module(8, 3, 9, 2);

#define BattPin A5
#define GpsPwr  12
#define GpsON  digitalWrite(GpsPwr, LOW);
#define GpsOFF digitalWrite(GpsPwr, HIGH);
SFE_UBLOX_GPS myGPS;
Adafruit_BMP085 bmp;
LIS3DH myIMU; //Default constructor is I2C, addr 0x19.
#define SSD1306_WHITE 1   ///< Draw 'on' pixels

//#define DEVMODE // Development mode. Uncomment to enable for debugging.

boolean airborne = false; //if you want to put the tracker on a airborne device, set this variable true;

//*********** General Settings ***********// 
uint16_t    trackerID = 1234; //change this and set a random unique trackerID to avoid conflicts if you have multiple trackers.
uint16_t    loRaTXinterval=60; //time interval (seconds) between two beacons (TX). This value is updated (increased) based on your LoRa Region settings and duty cycle.
uint8_t     measurementSystem = 0; //0 for metric (meters, km, Celcius, etc.), 1 for imperial (feet, mile, Fahrenheit,etc.) 
float       battMin=2.7;// min volts to run.

//*********** LoRa Settings ***********// 
//following TX settings should be same on the reciever module,
//otherwise RX module can not recieve LoRa packets.
boolean loRaEnabled = true;

//Following frequencies were chosen arbitrarily; and should be same on the reciever (RX) module.
//If necessary, users can select the appropriate channels according to their country regulations.
float loraFrequency = 865.2; //EU863-870
//float loraFrequency = 907.4; //US902-928

int8_t outputPower = 16; //dBm (max outputPower is 16 dBm for EU868, AS923, KR920, RU864)
//int8_t outputPower = 22; //dBm (max outputPower is 30 dBm for US915, AU915, IN865 but device limit is 22 dBm)

float loraBandWith = 125.0f; //do not change this, 125kHz is optimum for default payload size.  //https://avbentem.github.io/airtime-calculator/
uint8_t spreadingFactor = 8; ////do not change this, SF8 is optimum for default payload size.  https://www.thethingsnetwork.org/docs/lorawan/spreading-factors/
uint8_t codingRate = 5; //do not change this
uint8_t syncWord = 0x12; //(private network)
uint16_t preambleLength = 8; //do not change this
int8_t lowDataRateOptimization = 0; //do not change this
float dutyCyle = 0.01; //https://www.thethingsnetwork.org/docs/lorawan/duty-cycle.html
int8_t loRaWANHeaderSize = 0; // aka overhead size, 13 for LoRAWAN, 0 for LoRa 
int explcttHdr = 0; //1 for LoRaWAN, 0 for LoRa
int8_t CRC = 1; //do not change this

//latitude,longtitude,altitude,sattelite,speed,bearing,voltage,temp,pressure,etc. / 22 packets x 4 = 88 bytes
typedef union {
    float f[22];         // Assigning fVal.f will also populate fVal.bytes;
    unsigned char bytes[4];   // Both fVal.f and fVal.bytes share the same 4 bytes of memory.
} floatArr2Val;

floatArr2Val telemetry;

//********** GPS (uBlox) Settings ****************//
uint8_t navigationFrequencyGPS = 10; //1-10. Set the number of GPS nav solutions sent per second. 10 is max and provide 10 GPS data per second.
#define defaultMaxWait 100 //do not change this. Overriding UBlox default setting (1100) to fetch data faster from GPS module via I2C
#define getPVTmaxWait 100 //do not change this. Overriding UBlox default setting (1100) to fetch data faster from GPS module via I2C

//********** OLED Display Settings ****************//

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int displayPageID = 1;
int displayInterval = 5;
uint32_t last_display_refreshed = -10000;

//*********** Temp Veriables ***********// 

boolean gpsFixed = false;
int txCount = 1; //do not change this. counting LoRa TX messages...
uint32_t last_packet_send = -60000; //do not change this. Last LoRa packet sent time
float maxAltitude = 0;  //do not change this.
float minPressure = 101325;  //do not change this.  

float accelGX = 0;
float accelGY = 0;
float accelGZ = 0;

float minGX = 0;
float maxGX = 0;
float minGY = 0;
float maxGY = 0;
float minGZ = 0;
float maxGZ = 0;

float tempAltitude = 0;
float tempSpeed = 0;
float tempTemperature = 0;
float tempLatitude  = 0;
float tempLongitude = 0;
float tempSats = 0;
float tempHeading = 0;
float tempVoltage = 0;
float tempPressure = 0;

static char tempUnit[2] = ""; //C or F
static char speedUnit[7] = ""; //km/h or mph (mile per hour)
static char altUnit[5] = ""; //meters or feet
static char distUnit[5] = ""; //km or mile
static char measureSystem[10] = ""; //Metric or Imperial

void setup() {
  
  pinMode(A1,INPUT_PULLUP);
  if(digitalRead(A1)==LOW) while(1);

  delay(5000);

  pinMode(GpsPwr, OUTPUT);
  GpsON;
  
  SerialUSB.begin(115200);
  // Wait up to 5 seconds for serial to be opened, to allow catching
  // startup messages on native USB boards (that do not reset when
  // serial is opened).
  unsigned long start = millis();
  while (millis() - start < 5000 && !SerialUSB);

  SerialUSB.println();
  SerialUSB.println(F("Starting"));
  SerialUSB.println();

  SerialUSB.println(F("LoRa setup initiating.."));
  setupLoRa();     
  printLoRaSettings();

  SerialUSB.println(F("GPS setup initiating..(NO GPS fix yet)"));
  setupGPS_BMP();
  delay(2000);
  
  if(airborne){
    SerialUSB.println(F("Airborne mode initiated..."));       
    setupUBloxDynamicModel(); // Set the dynamic model to DYN_MODEL_AIRBORNE4g
  }

  SerialUSB.println(F("Accelerometer setup initiating.."));
  setupAccel();

  //OLED Display
  setupDisplay();
  
  freeMem();

  printDisplay(4);
  delay(7000);
}

void loop() {

if (readBatt() > battMin) {


  if (myGPS.getPVT() && (myGPS.getFixType() !=0) && (myGPS.getSIV() > 3)) {

      if(!gpsFixed){        
        SerialUSB.println(F("GPS Fixed..."));
        gpsFixed = true;
        }
        
    } 
     #if defined(DEVMODE)
          printGPSData();   
     #endif   

  updateTelemetry();
  getAccelerometerData();

  //update display
  if (millis() - last_display_refreshed > (displayInterval * 1000)) {
          printDisplay(displayPageID);
          last_display_refreshed = millis();

          ++displayPageID;

          if(displayPageID == 3) {
                displayPageID = 1;
           }
          
   }  
  
  
  if(loRaEnabled) { 
    if (millis() - last_packet_send > (loRaTXinterval * 1000)) {
      
      printDisplay(3);                
      
      sendLoraPacket();
      printLoRaSettings();
      last_packet_send = millis();
      if(loraFrequency >= 863.f && loraFrequency <= 870.f){
        calculateTransmitInterval(); 
      }
      delay(1000);
      
      printDisplay(2);
      last_display_refreshed = millis();
      
      SerialUSB.print(F("Next TX is "));
      SerialUSB.print(loRaTXinterval);
      SerialUSB.println(F(" seconds later."));
      freeMem();
    }
    
  }  

    
  } else {
    SerialUSB.println(F("Voltage is too low, please check your battery..."));
  }
  
}

void printDisplay(int pageID){

    display.clearDisplay();       
    display.setCursor(0, 0);
  
switch(pageID) {
  
  case 1:
  static char fixMessage[10] = "";
  if(myGPS.getFixType() !=0){
      sprintf(fixMessage, "%s", "Fixed");
    } else {
      sprintf(fixMessage, "%s", "Not Fixed");
      }
  
    display.println("");  
    display.print("GPS Fix : ");display.println(fixMessage);
    display.print("GPS Sats: ");display.println((int)tempSats);
    display.print("Lat     : ");display.println(tempLatitude);
    display.print("Long    : ");display.println(tempLongitude);    
    display.print("Altitude: ");display.print((int)tempAltitude);display.println(altUnit);    
    display.print("Speed   : ");display.print((int)tempSpeed);display.println(speedUnit);
    break;
  
  case 2:
    display.println("");  
    display.print("Battery :");display.print(tempVoltage); display.println("V");
    display.print("Temp    :");display.print(tempTemperature);display.println(tempUnit);
    display.print("Pressure:");display.print(tempPressure); display.println("hPa");   
    display.print("TX Count:");display.println(txCount-1);
    display.print("Next TX :");display.print(loRaTXinterval - ((millis() - last_packet_send)/1000)); display.println(" secs left");             
    break;

  case 3:
    display.println("");
    display.println("");
    display.println("LoRa Pckt Sending...");
    display.println("");
    break;

  case 4:
    display.println("");
    display.print("Freq    : ");display.print(loraFrequency);display.println("mHz");
    display.print("SF      : ");display.println(spreadingFactor);
    display.print("Bandwith: ");display.print(loraBandWith);display.println("kHz");
    display.print("Power   : ");display.print(outputPower);display.println("dBm");
    display.print("PcktSize: ");display.print(sizeof(telemetry));display.println(" bytes");    
    display.print("Measure : ");display.println(measureSystem);
      
    break;    
          
  default:
    SerialUSB.println("default");
    break;
 }

  display.display();
     
}

void setupDisplay(){
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
      SerialUSB.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }  
    display.clearDisplay();
    display.setTextSize(1); // Draw 2X-scale text
    display.setTextColor(SSD1306_WHITE);

  if(measurementSystem == 0){ //Metric    
    //meters
    sprintf(altUnit, "%s", " m");    
    //km/hour
    sprintf(speedUnit, "%s", " km/h");
    //Celsius
    sprintf(tempUnit, "%s", "C");
    //km
    sprintf(distUnit, "%s", " km");
    //Metric
    sprintf(measureSystem, "%s", "Metric");
               
    
   } else { //Imperial

    //feet
    sprintf(altUnit, "%s", " ft");    
    //mile/hour
    sprintf(speedUnit, "%s", " mph");
    //Fahrenheit
    sprintf(tempUnit, "%s", "F");
    //mile
    sprintf(distUnit, "%s", " mi");      
    //Imperial
    sprintf(measureSystem, "%s", "Imperial");
  }
    
  }

void setupAccel() {
 
//********** Acceloremeter (LSM303DLHC) Settings **************//
  myIMU.settings.adcEnabled = 1;
  myIMU.settings.accelSampleRate = 400;  //Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
  myIMU.settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
  myIMU.settings.xAccelEnabled = 1;
  myIMU.settings.yAccelEnabled = 1;
  myIMU.settings.zAccelEnabled = 1; 

  //Call .begin() to configure the IMU
  myIMU.begin();
   
  delay(100);
 
  }

 void getAccelerometerData(){

    accelGX = myIMU.readFloatAccelX();
    accelGY = myIMU.readFloatAccelY();
    accelGZ = myIMU.readFloatAccelZ();
 

    if (accelGX < minGX) {
        minGX = accelGX;
    } else if (accelGX > maxGX) {
        maxGX = accelGX;        
    }

    if (accelGY < minGY) {
        minGY = accelGY;
    } else if (accelGY > maxGY) {
        maxGY = accelGY;        
    }
      
    if (accelGZ < minGZ) {
        minGZ = accelGZ;
    } else if (accelGZ > maxGZ) {
        maxGZ = accelGZ;        
    }      


}

// Same functionality as Arduino's standard map function, except using floats
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void sendLoraPacket(){  
  

  SerialUSB.print(F("[SX1262] LoRa Radio Module Transmitting packet ... "));
  
 
  int state = lora.transmit(telemetry.bytes,sizeof(telemetry));
 

  if (state == ERR_NONE) {
    
    // the packet was successfully transmitted
    SerialUSB.println(F("success!"));

    // print measured data rate
    SerialUSB.print(F("[SX1262] Datarate:\t"));
    SerialUSB.print(lora.getDataRate());
    SerialUSB.println(F(" bps"));
    
  } else if (state == ERR_PACKET_TOO_LONG) {
    
    // the supplied packet was longer than 256 bytes
    SerialUSB.println(F("too long!"));

  } else if (state == ERR_TX_TIMEOUT) {  
    // timeout occured while transmitting packet
    SerialUSB.println(F("timeout!"));
  } else {
    // some other error occurred
    SerialUSB.print(F("failed, code "));
    SerialUSB.println(state);
  }
  lora.sleep();

  txCount++;
  
  }


void updateTelemetry() {

  if(measurementSystem == 0){ //Metric
    //meters
    tempAltitude = myGPS.getAltitude() / 1000.f;
    //km/hour
    tempSpeed = myGPS.getGroundSpeed() * 0.0036f;
    //Celsius
    tempTemperature = bmp.readTemperature();    
    
   } else { //Imperial
    //feet
    tempAltitude = (myGPS.getAltitude() * 3.2808399)  / 1000.f;
    //mile/hour
    tempSpeed = myGPS.getGroundSpeed() *  0.00223694f;
    //Fahrenheit
    tempTemperature =  (bmp.readTemperature() * 1.8f) + 32;       
  }

  tempLatitude  = myGPS.getLatitude() / 10000000.f;
  tempLongitude = myGPS.getLongitude() / 10000000.f;
  tempSats = myGPS.getSIV();

  if ((tempSats > 4) && (tempAltitude > maxAltitude)) {
    maxAltitude = tempAltitude;
  }
    
  tempHeading = myGPS.getHeading() / 100000;
  tempVoltage = readBatt();
  tempPressure = bmp.readPressure() / 100.0;

  if (tempPressure < minPressure) {
    minPressure = tempPressure;
  }  
  
  //prepare data for LoRa communcation...
  if(loRaEnabled) {
    telemetry.f[0] = tempLatitude;
    telemetry.f[1] = tempLongitude;
    telemetry.f[2] = tempAltitude;
    telemetry.f[3] = tempSats;
    telemetry.f[4] = tempHeading;
    telemetry.f[5] = tempSpeed;
    telemetry.f[6] = tempVoltage;
    telemetry.f[7] = tempTemperature;
    telemetry.f[8] = tempPressure;
    telemetry.f[9] = accelGX;
    telemetry.f[10] = accelGY;
    telemetry.f[11] = accelGZ;    
    telemetry.f[12] = minGX;   
    telemetry.f[13] = maxGX;  
    telemetry.f[14] = minGY;   
    telemetry.f[15] = maxGY;
    telemetry.f[16] = minGZ;   
    telemetry.f[17] = maxGZ;   
    telemetry.f[18] = maxAltitude;
    telemetry.f[19] = minPressure;    
    telemetry.f[20] = txCount;
    telemetry.f[21] = trackerID;
    
  }

#if defined(DEVMODE)
  SerialUSB.println(F(""));
  SerialUSB.print(F("Batt: "));
  SerialUSB.print(tempVoltage);
  SerialUSB.print(F(" Temp: "));  
  SerialUSB.print(tempTemperature);
  SerialUSB.print(F(" Press: "));
  SerialUSB.print(tempPressure);
  SerialUSB.print(F(" GX: "));
  SerialUSB.print(accelGX);  
  SerialUSB.print(F(" GY: "));
  SerialUSB.print(accelGY); 
  SerialUSB.print(F(" GZ: "));
  SerialUSB.println(accelGZ); 
#endif
  
/**   
#if defined(DEVMODE)
  SerialUSB.println(F("----"));
  SerialUSB.print(F("lat   :"));
  SerialUSB.println(telemetry.f[0]);
  SerialUSB.print(F("long  :"));
  SerialUSB.println(telemetry.f[1]);
  SerialUSB.print(F("alt   :"));
  SerialUSB.println(telemetry.f[2]);
  SerialUSB.print(F("sats  :"));
  SerialUSB.println(telemetry.f[3]);
  SerialUSB.print(F("course:"));
  SerialUSB.println(telemetry.f[4]);
  SerialUSB.print(F("speed :"));
  SerialUSB.println(telemetry.f[5]);
  SerialUSB.print(F("batt  :"));
  SerialUSB.println(telemetry.f[6]);
  SerialUSB.print(F("temp  :"));  
  SerialUSB.println(telemetry.f[7]);
  SerialUSB.print(F("press :"));
  SerialUSB.println(telemetry.f[8]);
  SerialUSB.print(F("scaledGX :"));
  SerialUSB.println(telemetry.f[9]);  
  SerialUSB.print(F("scaledGY :"));
  SerialUSB.println(telemetry.f[10]); 
  SerialUSB.print(F("scaledGZ :"));
  SerialUSB.println(telemetry.f[11]); 
  SerialUSB.print(F("minScaledGX :"));
  SerialUSB.println(telemetry.f[12]);
  SerialUSB.print(F("maxScaledGX :"));
  SerialUSB.println(telemetry.f[13]);
  SerialUSB.print(F("minScaledGY :"));
  SerialUSB.println(telemetry.f[14]);
  SerialUSB.print(F("maxScaledGY :"));
  SerialUSB.println(telemetry.f[15]);
  SerialUSB.print(F("minScaledGZ :"));
  SerialUSB.println(telemetry.f[16]);
  SerialUSB.print(F("maxScaledGZ :"));
  SerialUSB.println(telemetry.f[17]);         
  SerialUSB.print(F("maxAltitude :"));
  SerialUSB.println(telemetry.f[18]);         
  SerialUSB.print(F("minPressure :"));  
  SerialUSB.println(telemetry.f[19]);
  SerialUSB.print(F("txCount :"));  
  SerialUSB.println(telemetry.f[20]);
  SerialUSB.print(F("trackerID :"));  
  SerialUSB.println((int)telemetry.f[21]);    
  SerialUSB.println(F("----"));  
#endif
*/

}


void setupLoRa() {

  // initialize SX1262 with default settings
  SerialUSB.print(F("[SX1262] Initializing ... "));  
  int state = lora.begin();

  if (state == ERR_NONE) {
    SerialUSB.println(F("success!"));
  } else {
    SerialUSB.print(F("failed, code "));
    SerialUSB.println(state);
    while (true);
  }

  if (lora.setFrequency(loraFrequency,true) == ERR_INVALID_FREQUENCY) {
    SerialUSB.println(F("Selected frequency is invalid for this module!"));
    while (true);
  }

  // set bandwidth to 1250 kHz
  if (lora.setBandwidth(loraBandWith) == ERR_INVALID_BANDWIDTH) {
    SerialUSB.println(F("Selected bandwidth is invalid for this module!"));
    while (true);
  }

  // set spreading factor to 10
  if (lora.setSpreadingFactor(spreadingFactor) == ERR_INVALID_SPREADING_FACTOR) {
    SerialUSB.println(F("Selected spreading factor is invalid for this module!"));
    while (true);
  }

  // set coding rate to 5
  if (lora.setCodingRate(codingRate) == ERR_INVALID_CODING_RATE) {
    SerialUSB.println(F("Selected coding rate is invalid for this module!"));
    while (true);
  }

  // set LoRaWAN sync word to 0x34
  if (lora.setSyncWord(syncWord) != ERR_NONE) {
    SerialUSB.println(F("Unable to set sync word!"));
    while (true);
  }

  // set over current protection limit to 140 mA (accepted range is 45 - 240 mA)
  if (lora.setCurrentLimit(140) == ERR_INVALID_CURRENT_LIMIT) {
    Serial.println(F("Selected current limit is invalid for this module!"));
    while (true);
  }   

  // set output power to outputPower  dBm (accepted range is -17, 22 dBm)
  if (lora.setOutputPower(outputPower) == ERR_INVALID_OUTPUT_POWER) {
    SerialUSB.println(F("Selected output power is invalid for this module!"));
    while (true);
  }

  // set LoRa preamble length to 8 symbols (accepted range is 0 - 65535)
  if (lora.setPreambleLength(preambleLength) == ERR_INVALID_PREAMBLE_LENGTH) {
    SerialUSB.println(F("Selected preamble length is invalid for this module!"));
    while (true);
  }

  // enable CRC
  if (lora.setCRC(CRC) == ERR_INVALID_CRC_CONFIGURATION) {
    SerialUSB.println(F("Selected CRC is invalid for this module!"));
    while (true);
  }

  SerialUSB.println(F("All settings succesfully changed!"));  
  
  }

float readBatt() {

  float R1 = 560000.0; // 560K
  float R2 = 100000.0; // 100K
  float value = 0.0f;
  do {    
    value =analogRead(BattPin);
    value +=analogRead(BattPin);
    value +=analogRead(BattPin);
    value = value / 3.0f;
    value = (value * 3.3) / 1024.0f;
    value = value / (R2/(R1+R2));
  } while (value > 20.0);
  return value ;

}


void printLoRaSettings(){

  SerialUSB.println(F("-------------------------------LoRa Settings-----------------------------------------")); 
  SerialUSB.print(F("Freq: "));    
  SerialUSB.print(loraFrequency);                   
  SerialUSB.print(F("mHz, Bandwith: "));
  SerialUSB.print(loraBandWith);
  SerialUSB.print(F("kHz, SF: "));              
  SerialUSB.print(spreadingFactor);
  SerialUSB.print(F(", Power: "));              
  SerialUSB.print(outputPower);
  SerialUSB.print(F("dBm, Payload Size: "));
  SerialUSB.print(sizeof(telemetry));
  SerialUSB.println(F(" bytes"));      
  SerialUSB.println(F("------------------------------------------------------------------------------------"));    
  
  }

//Every radio device must be compliant with the regulated duty cycle limits. This applies to both nodes and gateways.
//In practice, this means that you should program your nodes in such a way, that they stay within the limits. 
//The easiest way to do this, is to calculate how much airtime each message consumes using one of the many 
//airtime calculators and use that information to choose a good transmit interval (duty cycle). https://www.thethingsnetwork.org/docs/lorawan/duty-cycle.html
//Following function checks and enforce duty cycle limits based on your LoRa settings so you can not break the law.

void calculateTransmitInterval(){

  if(spreadingFactor>= 11){
      lowDataRateOptimization = 1;
    }

  if (loraFrequency >=868.7 && loraFrequency <=869.2) {
        dutyCyle = 0.001;      
  } else if (loraFrequency >=869.4 && loraFrequency <=869.65) {
        dutyCyle = 0.1;      
  } else {
        dutyCyle = 0.01;         
  }

    float symbolDuration = pow(2, spreadingFactor) / (loraBandWith * 1000.f);
    float a = (8 * (sizeof(telemetry) + loRaWANHeaderSize)) - (4 * spreadingFactor) + 28 + (16 * CRC) - (20 * (1-explcttHdr));
    float b = 4 * (spreadingFactor - (2 * lowDataRateOptimization));

    int nbrSymbols = ceil(a/b) * codingRate;

    if (nbrSymbols < 0){
        nbrSymbols = 0;
      }
    nbrSymbols += 8;

    float preambuleDuration = (preambleLength + 4.25f) * symbolDuration;
    float payloadDuration = nbrSymbols * symbolDuration;
    float packetDuration = (preambuleDuration + payloadDuration) / dutyCyle;
   
    if (loRaTXinterval < (int)packetDuration) {
      
      #if defined(DEVMODE)
        SerialUSB.print(F("Updating "));
        SerialUSB.print(F("loRaTXinterval= "));    
        SerialUSB.print(loRaTXinterval);  
        SerialUSB.print(F(" to loRaTXinterval= ")); 
        SerialUSB.print((int)packetDuration);      
        SerialUSB.println(F(" seconds..."));          
      #endif      
     
        loRaTXinterval = (int)(packetDuration);
      }    

  }


void setupGPS_BMP() {
  GpsON;
  delay(100);
  Wire.begin();
  bmp.begin();

  Wire.setClock(400000);

  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    SerialUSB.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  // do not overload the buffer system from the GPS, disable UART output
  myGPS.setUART1Output(0); //Disable the UART1 port output 
  myGPS.setUART2Output(0); //Disable Set the UART2 port output
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  //myGPS.enableDebugging(); //Enable debug messages over Serial (default)

  myGPS.setNavigationFrequency(navigationFrequencyGPS);//Sadece roket için 10 yapalım.  //Set output to 10 times a second
  byte rate = myGPS.getNavigationFrequency(); //Get the update rate of this module
  //SerialUSB.print("Current GPS update rate:");
  //SerialUSB.println(rate);

  myGPS.saveConfiguration(); //Save the current settings to flash and BBR  
  
  }


void setupUBloxDynamicModel() {
    // If we are going to change the dynamic platform model, let's do it here.
    // Possible values are:
    // PORTABLE, STATIONARY, PEDESTRIAN, AUTOMOTIVE, SEA, AIRBORNE1g, AIRBORNE2g, AIRBORNE4g, WRIST, BIKE
    //DYN_MODEL_AIRBORNE4g model increases ublox max. altitude limit from 12.000 meters to 50.000 meters. 
    if (myGPS.setDynamicModel(DYN_MODEL_AIRBORNE4g) == false) // Set the dynamic model to DYN_MODEL_AIRBORNE4g
    {
      SerialUSB.println(F("***!!! Warning: setDynamicModel failed !!!***"));
    }
    else
    {
        SerialUSB.print(F("Dynamic platform model changed successfully! : "));
        SerialUSB.println(myGPS.getDynamicModel());
    }
  
  } 

void freeMem() {
  #if defined(DEVMODE)
    SerialUSB.print(F("Free RAM: ")); SerialUSB.print(freeMemory(), DEC); SerialUSB.println(F(" byte"));
  #endif

}

void printGPSData()
{
  // Calling getPVT returns true if there actually is a fresh navigation solution available.
    SerialUSB.println(F(""));
    
    byte fixType = myGPS.getFixType();
  
    SerialUSB.print(F("FixType: "));
    SerialUSB.print(fixType);    

    long latitude = myGPS.getLatitude();
    float flat = myGPS.getLatitude() / 10000000.f;
    
    SerialUSB.print(F(" Lat: "));
    SerialUSB.print(latitude);
    SerialUSB.print(F(" - "));
    SerialUSB.print(flat);    

    long longitude = myGPS.getLongitude();
    float flong = myGPS.getLongitude() / 10000000.f;    
    SerialUSB.print(F(" Long: "));
    SerialUSB.print(longitude);
    SerialUSB.print(F(" - "));
    SerialUSB.print(flong);        

    float altitude = myGPS.getAltitude() / 1000;
    SerialUSB.print(F(" Alt: "));
    SerialUSB.print(altitude);
    SerialUSB.print(F(" (m)"));

    int SIV = myGPS.getSIV();
    SerialUSB.print(F(" SIV: "));
    SerialUSB.print(SIV);

    float speed = myGPS.getGroundSpeed() * 0.0036f;
    SerialUSB.print(F(" Speed: "));
    SerialUSB.print(speed);
    SerialUSB.print(F(" (km/h)"));

    long heading = myGPS.getHeading() / 100000;
    SerialUSB.print(F(" Heading: "));
    SerialUSB.print(heading);
    SerialUSB.print(F(" (degrees )"));
    
    SerialUSB.print(" ");
    SerialUSB.print(myGPS.getYear());
    SerialUSB.print("-");
    SerialUSB.print(myGPS.getMonth());
    SerialUSB.print("-");
    SerialUSB.print(myGPS.getDay());
    SerialUSB.print(" ");
    SerialUSB.print(myGPS.getHour());
    SerialUSB.print(":");
    SerialUSB.print(myGPS.getMinute());
    SerialUSB.print(":");
    SerialUSB.print(myGPS.getSecond());
    SerialUSB.print(":");
    SerialUSB.print(myGPS.getMillisecond());    

    SerialUSB.println();
    delay(1000);

}
