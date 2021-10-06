
#include <RadioLib.h>
#include <avr/dtostrf.h>
#include <Adafruit_BMP085.h>
#include "SparkFun_Ublox_Arduino_Library.h" 
#include <SPI.h>
#include <MemoryFree.h>;
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <Wire.h>

#define d2r (M_PI / 180.0)

// SX1262 has the following connections:
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

//#define DEVMODE // Development mode. Uncomment to enable for debugging.

//*********** General Settings ***********// 
int measurementSystem = 0; //0 for metric (meters, km, Celcius, etc.), 1 for imperial (feet, mile, Fahrenheit,etc.) 
float battMin=2.7;    // min volts to run.

//********** GPS (uBlox) Settings ****************//
uint8_t navigationFrequencyGPS = 1; //1-10. Set the number of GPS nav solutions sent per second. 10 is max and provide 10 GPS data per second.

//*********** LoRa Settings ***********// 
//Following frequencies were chosen arbitrarily; and should be same on the reciever (RX) module.
//If necessary, users can select the appropriate channels according to their country regulations.

float loraFrequency = 865.2; //EU863-870
//float loraFrequency = 907.4; //US902-928

int8_t outputPower = 16; //dBm (max outputPower is 16 dBm for EU868, AS923, KR920, RU864)
//int8_t outputPower = 22; //dBm (max outputPower is 30 dBm for US915, AU915, IN865 but device limit is 22 dBm)

float loraBandWith = 125.0f; //do not change this, this is optimum for default payload size.  //https://avbentem.github.io/airtime-calculator/
uint8_t spreadingFactor = 8; ////do not change this, SF8 is optimum for default payload size.  https://www.thethingsnetwork.org/docs/lorawan/spreading-factors/
uint8_t codingRate = 5; //do not change this
uint8_t syncWord = 0x12; //(private network)
uint16_t preambleLength = 8; //do not change this
int8_t lowDataRateOptimization = 0; //do not change this
float dutyCyle = 0.01; //https://www.thethingsnetwork.org/docs/lorawan/duty-cycle.html
int8_t loRaWANHeaderSize = 0; // aka overhead size, 13 for LoRAWAN, 0 for LoRa 
int explcttHdr = 0; //1 for LoRaWAN, 0 for LoRa
int8_t CRC = 1; //do not change this

//********** OLED Display Settings ****************//

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int displayPageID = 2;
int displayInterval = 8;
uint32_t last_display_refreshed = 0;

//*********** Temp Veriables ***********// 
int rxCount = 0; //do not change this. counting LoRa RX messages...

float maxAltitude = 0;  //do not change this.
float minPressure = 0;  //do not change this.

// flag to indicate that a packet was received
volatile bool receivedFlag = false; //do not change this.

// disable interrupt when it's not needed
volatile bool enableInterrupt = true; //do not change this.

uint32_t last_packet_send = 0; //do not change this. Last LoRa packet sent time
uint32_t last_packet_received = 0; //do not change this. Last LoRa packet received time

static char tempUnit[2] = ""; //C or F
static char speedUnit[7] = ""; //km/h or mph (mile per hour)
static char altUnit[5] = ""; //meters or feet
static char distUnit[5] = ""; //km or mile
static char measureSystem[10] = ""; //Metric or Imperial

//latitude,longtitude,altitude,sattelite,speed,bearing,voltage,temp,pressure,etc. / 22 packets
typedef union {
    float f[22];         // Assigning fVal.f will also populate fVal.bytes;
    unsigned char bytes[4];   // Both fVal.f and fVal.bytes share the same 4 bytes of memory.
} floatArr2Val;

floatArr2Val telemetry;

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
  //printLoRaSettings();

  SerialUSB.println(F("GPS setup initiating..(NO GPS fix yet)"));
  setupGPS_BMP();
  delay(2000);
  
  setupDisplay();
  
}

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag(void) {
  // check if the interrupt is enabled
  if(!enableInterrupt) {
    return;
  }

  // we got a packet, set the flag
  receivedFlag = true;
}


void loop() {

if (readBatt() > battMin) {

  if(receivedFlag) {

    displayIncmng();

    // disable the interrupt service routine while
    // processing the data
    enableInterrupt = false;

    // reset flag
    receivedFlag = false;

    // you can read received data as an Arduino String if sent as string...
    //String str;
    //int state = lora.readData(str);

    // you can also read received data as byte array
    byte byteArr[sizeof(telemetry)];
    int state = lora.readData(byteArr, sizeof(telemetry));
  
    byte tempData[4];
    int x = 0;
    for (int i = 0; i < sizeof(telemetry); i++) {

          tempData[x] = byteArr[i];

          if(x == 3) {
              telemetry.f[(int)(i/4)] = *((float*)(tempData));
              x=0;
            } else {
              ++x;
              }
    }
    
    if (state == ERR_NONE) {
      // packet was successfully received
      last_packet_received = millis();

      // print data of the packet
      displayPageID = 1;
      printRXTelemetry();
      printLoRaSettings();
      ++rxCount;

    } else if (state == ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      SerialUSB.println(F("CRC error!"));
      displayIncmngError();

    } else {
      // some other error occurred
      SerialUSB.print(F("failed, code "));
      SerialUSB.println(state);

    }

    // put module back to listen mode
    lora.startReceive();

    // we're ready to receive more packets,
    // enable interrupt service routine
    enableInterrupt = true;

    freeMem();
  }  

  //update GPS data
  myGPS.getPVT();

  #if defined(DEVMODE)
     printGPSData();   
  #endif   
 
  //update oled display
  if (millis() - last_display_refreshed > (displayInterval * 1000)) {

          printDisplay(displayPageID);
          last_display_refreshed = millis();
          ++displayPageID;

          if(displayPageID == 6) {
                displayPageID = 2;
           }
   }

  
     
  } else {
    SerialUSB.println(F("Voltage is too low, please check your battery..."));
  }
  
}

void displayIncmng(){
    display.clearDisplay();       
    display.setCursor(0, 30);
    display.println("Incoming packet...");
    display.display();
  }

void displayIncmngError(){
    display.clearDisplay();       
    display.setCursor(0, 30);
    display.println("CRC Error, packet malformed..");
    display.display();
  }  

void printDisplay(int pageID){

    display.clearDisplay();       
    display.setCursor(0, 0);


    float tempAltitude = 0;
    float tempSpeed = 0;
    float tempTemperature = 0; 

    double distance = 0;

    if(measurementSystem == 0){ //Metric
      //meter
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

  if ((myGPS.getFixType() !=0) && (myGPS.getSIV() > 3)) {

    if(telemetry.f[0] ==0 || telemetry.f[1] ==0){
      distance = 0;
      } else {
      distance = haversine(telemetry.f[0], telemetry.f[1], myGPS.getLatitude() / 10000000.f, myGPS.getLongitude() / 10000000.f, measurementSystem);            
        }
  }

  //SerialUSB.println(last_packet_received);
  int lastPcktTimeSeconds = (int)((millis()-last_packet_received)/1000);
  //SerialUSB.println(lastPcktTimeSeconds);
  
  if(last_packet_received==0) {lastPcktTimeSeconds=0;}
  int lastPcktTimeMinutes = lastPcktTimeSeconds / 60 ;
  
  //SerialUSB.println(lastPcktTimeMinutes);
  //SerialUSB.println("--------------------");
  
switch(pageID) {
  case 1:
    display.println("New Packet Recieved");
    display.println("");
    display.print("ID       :");display.println((int)telemetry.f[21]);
    display.print("RX/TX    :");display.print(rxCount);display.print("/");display.println((int)telemetry.f[20]);     
    display.print("RSSI     :");display.print(lora.getRSSI()); display.println(" dBm");
    display.print("SNR      :");display.print(lora.getSNR()); display.println(" dB");
    display.print("PcktLngth:");display.print(lora.getPacketLength()); display.println(" bytes");               
    break;
  case 2:
    display.println("RX Basic Data (1/3)");
    display.println("");
    display.print("Battery  :");display.print(telemetry.f[6]); display.println("V");
    display.print("Temp     :");display.print(telemetry.f[7]);display.println(tempUnit);
    display.print("Pressure :");display.print(telemetry.f[8]); display.println("hPa");
    if(lastPcktTimeSeconds < 100) {
        display.print("Last Pckt:");display.print(lastPcktTimeSeconds); display.println(" secs ago");
      } else {
        display.print("Last Pckt:");display.print(lastPcktTimeMinutes); display.println(" mins ago");
        }
    
    break;
  case 3:
    display.println("RX GPS Data (2/3)");
    display.println("");    
    display.print("GPS Sats :");display.println((int)telemetry.f[3]);
    display.print("Speed    :");display.print(telemetry.f[5]);display.println(speedUnit);
    display.print("Heading  :");display.print((int)telemetry.f[4]);display.println(" degrees");
    display.print("Altitude :");display.print((int)telemetry.f[2]);display.println(altUnit);
    display.print("Latitude :");display.println(telemetry.f[0]);
    display.print("Longitude:");display.println(telemetry.f[1]);                    
    break;
  case 4:
    display.println("RX G Data (3/3)");
    display.println("");      
    display.print("MnX:");display.print(telemetry.f[12]);display.print(" MxX:");display.println(telemetry.f[13]);    
    display.print("MnY:");display.print(telemetry.f[14]);display.print(" MxY:");display.println(telemetry.f[15]);
    display.print("MnZ:");display.print(telemetry.f[16]);display.print(" MxZ:");display.println(telemetry.f[17]);   
    display.print("Min Press:");display.print(telemetry.f[19]);display.println("hPa");
    display.print("Max Alt:");display.print(telemetry.f[18]);display.println(altUnit);                
    break;
  case 5:
    display.println("Your Data");
    display.println("");    
    display.print("GPS Sats :");display.println((int)myGPS.getSIV());
    display.print("Speed    :");display.print(tempSpeed);display.println(speedUnit);
    display.print("Altitude :");display.print((int)tempAltitude);display.println(altUnit);
    display.print("Battery  :");display.print(readBatt()); display.println("V");
    display.print("Temp     :");display.print(tempTemperature);display.println(tempUnit);
    display.print("Distance :");display.print(distance);display.println(distUnit);          
    break;        
  default:
    SerialUSB.println("default");
    break;
 }

  display.display();
     
}

//calculate haversine distance for linear distance
double haversine(double lat1, double long1, double lat2, double long2, boolean km)
{   
    double distance = 0;
    double dlong = (long2 - long1) * d2r;
    double dlat = (lat2 - lat1) * d2r;
    double a = pow(sin(dlat/2.0), 2) + cos(lat1*d2r) * cos(lat2*d2r) * pow(sin(dlong/2.0), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));

    if (km) {
        distance = 6367 * c; //km
      } else {        
        distance = 3956 * c; //miles
        }    

    return distance;
}


void printRXTelemetry() {

  SerialUSB.println(F("-------Incoming Packet Telemetry-------"));
  SerialUSB.print(F("lat   :"));
  SerialUSB.println(telemetry.f[0]);
  SerialUSB.print(F("long  :"));
  SerialUSB.println(telemetry.f[1]);
  SerialUSB.print(F("alt   :"));
  SerialUSB.println(telemetry.f[2]);
  SerialUSB.print(F("sats  :"));
  SerialUSB.println((int)telemetry.f[3]);
  SerialUSB.print(F("heading:"));
  SerialUSB.println((int)telemetry.f[4]);
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
  SerialUSB.print(F("rxC   :"));  
  SerialUSB.println((int)telemetry.f[20]);
  SerialUSB.print(F("trackerID :"));  
  SerialUSB.println((int)telemetry.f[21]);

}

void printLoRaSettings(){

  SerialUSB.println(F("-------------------------Incoming Packet LoRa Settings ---------------------------------------------------------"));
  SerialUSB.print(F("RSSI: "));
  SerialUSB.print(lora.getRSSI());
  SerialUSB.print(F(" dBm"));
  SerialUSB.print(F(", SNR: "));
  SerialUSB.print(lora.getSNR());
  SerialUSB.print(F(" dB"));  
  SerialUSB.print(F(", Freq: "));    
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
  SerialUSB.println(F("---------------------------------------------------------------------------------------------------------------"));    
  }


void setupLoRa() {  

  // initialize SX1262 with default settings
  SerialUSB.print(F("[SX1262] LoRa Radio Module Initializing ... "));  
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

  // set output power to 14 dBm (accepted range is -17, 22 dBm)
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

  // set the function that will be called
  // when new packet is received
  lora.setDio1Action(setFlag);

  // start listening for LoRa packets
  SerialUSB.print(F("[SX1262] LoRa Radio Module Starting to listen ... "));
  state = lora.startReceive();
  if (state == ERR_NONE) {
    SerialUSB.println(F("success!"));
  } else {
    SerialUSB.print(F("failed, code "));
    SerialUSB.println(state);
    while (true);
  }
  
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
  } while (value > 10.0);
  return value ;

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
  SerialUSB.print("Current GPS update rate:");
  SerialUSB.println(rate);

  myGPS.saveConfiguration(); //Save the current settings to flash and BBR  
  
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

void printGPSData()
{
  // Calling getPVT returns true if there actually is a fresh navigation solution available.

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

void freeMem() {
  #if defined(DEVMODE)
    SerialUSB.print(F("Free RAM: ")); SerialUSB.print(freeMemory(), DEC); SerialUSB.println(F(" byte"));
  #endif

}
