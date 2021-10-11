#include <basicmac.h>
#include <hal/hal.h>
#include <Adafruit_BMP085.h>
#include "SparkFun_Ublox_Arduino_Library.h"
#include <MemoryFree.h>;
#include <CayenneLPP.h>
#include "SparkFunLIS3DH.h"
#include <Adafruit_SSD1306.h>

// SX1262 has the following connections:
#define nssPin 8
#define rstPin 9
#define dio1Pin 3
#define busyPin 2

#define BattPin A5
#define GpsPwr  12
#define GpsON  digitalWrite(GpsPwr, LOW);
#define GpsOFF digitalWrite(GpsPwr, HIGH);

SFE_UBLOX_GPS myGPS;
Adafruit_BMP085 bmp; //temp and pressure sensor
LIS3DH myIMU; //accelerometer
#define SSD1306_WHITE 1   ///< Draw 'on' pixels

//#define DEVMODE // Development mode. Uncomment to enable for debugging.

boolean airborne = false; //if you want to put the tracker on an airborne (balloon, drone, plane, etc.) device, set this variable true;
uint8_t measurementSystem = 0; //0 for metric (meters, km, Celcius, etc.), 1 for imperial (feet, mile, Fahrenheit,etc.) 

//***************************** UPDATE HERE WITH YOUR DEVICE KEYS **************************************/

//You should copy device keys from Helium or TTN Console and update following keys. Please check out: https://github.com/lightaprs/LightTracker-1.0/wiki/Adding-Device-on-Helium-Console

// This EUI must be in little-endian format, so least-significant-byte (lsb)
// first. When copying an EUI from Helium Console or ttnctl output, this means to reverse the bytes. 

static const u1_t PROGMEM DEVEUI[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //helium or ttn

// This EUI must be in little-endian format, so least-significant-byte (lsb)
// first. When copying an EUI from Helium Console or ttnctl output, this means to reverse the bytes.  

static const u1_t PROGMEM APPEUI[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //helium or ttn

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In practice, a key taken from Helium Console or ttnctl can be copied as-is.

static const u1_t PROGMEM APPKEY[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //helium or ttn
//*****************************************************************************************************/


//************************** LoRaWAN Settings ********************
//DO NOT FORGET TO CHANGE YOUR REGION, DEFAULT REGION IS EU. Uncomment your region, but comment other regions
uint8_t Lorawan_region_code = REGCODE_EU868;//For EU
//uint8_t Lorawan_region_code = REGCODE_US915;//For North America
//uint8_t Lorawan_region_code = REGCODE_AS923;//For Asia
//uint8_t Lorawan_region_code = REGCODE_AU915;//For South America and Australia
//uint8_t Lorawan_region_code = REGCODE_IN865;//For India and Pakistan

const unsigned TX_INTERVAL = 30000;  // Schedule TX every this many miliseconds (might become longer due to duty cycle limitations).
boolean gpsFixRequiredforTX = true; //do not changed this, GPS fix is required for helium mapper

//Spreading Factor automatically selected by BasicMAC LoRaWAN Library. Keep this "true" if you want to use your tracker in urban areas.
//But if you are too far from the the gateways/hotspots then change it to "false"
boolean autoSF = true; //do not change this

//latitude,longtitude,altitude,hdop / 4 packets x 4 = 16 bytes
typedef union {
    float f[4];         // Assigning fVal.f will also populate fVal.bytes;
    unsigned char bytes[4];   // Both fVal.f and fVal.bytes share the same 4 bytes of memory.
} floatArr2Val;

floatArr2Val telemetry;

u1_t os_getRegion (void) { return Lorawan_region_code; } //do not change this.

uint8_t channelNoFor2ndSubBand = 8; //do not change this. Used for US915 and AU915 / TTN and Helium
uint32_t last_packet = 0; //do not change this. Timestamp of last packet sent.

boolean ev_joined = false;

//pinmap for SX1262 LoRa module
#if !defined(USE_STANDARD_PINMAP)
const lmic_pinmap lmic_pins = {
    .nss = nssPin,
    .tx = LMIC_UNUSED_PIN,
    .rx = LMIC_UNUSED_PIN,
    .rst = rstPin,
    .dio = {/* DIO0 */ LMIC_UNUSED_PIN, /* DIO1 */ dio1Pin, /* DIO2 */ LMIC_UNUSED_PIN},
    .busy = busyPin,
    .tcxo = LMIC_CONTROLLED_BY_DIO3,
};
#endif // !defined(USE_STANDARD_PINMAP)

//************************** uBlox GPS  Settings ********************

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.

//********************************* Power Settings ******************************

int   battWait=60;    //seconds sleep if super capacitors/batteries are below battMin (important if power source is solar panel) 
float battMin=2.7;    // min Volts to TX.

//********************************* Misc Settings ******************************

int txCount = 1;
float voltage = 0;
float tempAltitudeLong = 0;
float tempAltitudeShort = 0; 
float tempSpeed = 0;
float tempTemperature = 0;
float tempLatitude  = 0;
float tempLongitude = 0;
float tempSats = 0;
float tempHeading = 0;
float tempPressure = 0;
float tempHDOP = 0;

static char tempUnit[2] = ""; //C or F
static char speedUnit[7] = ""; //km/h or mph (mile per hour)
static char altUnit[5] = ""; //meters or feet
static char distUnit[5] = ""; //km or mile
static char measureSystem[10] = ""; //km or mile
static char regionName[6] ="";

//********** OLED Display Settings ****************//

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int displayPageID = 1;
int displayInterval = 10;
uint32_t last_display_refreshed = -10000;

void setup() {
  
  pinMode(A1,INPUT_PULLUP);
  if(digitalRead(A1)==LOW) while(1);

  delay(5000); //do not change this

  pinMode(GpsPwr, OUTPUT);    
  
  SerialUSB.begin(115200);
    
  // Wait up to 5 seconds for serial to be opened, to allow catching
  // startup messages on native USB boards (that do not reset when
  // serial is opened).
  unsigned long start = millis();
  while (millis() - start < 5000 && !SerialUSB);

  SerialUSB.println();
  SerialUSB.println(F("Starting"));
  SerialUSB.println();

  GpsON;
  delay(1000);  

  SerialUSB.println(F("GPS setup"));   
  setupGPS_BMP();
  delay(2000);
  
  if(airborne){
    SerialUSB.println(F("Airborne mode initiated..."));     
    setupUBloxDynamicModel(); // Set the dynamic model to DYN_MODEL_AIRBORNE4g
    autoSF = false; //override LoRaWAN auto Spreading Factor selection setting 
  }
  
  SerialUSB.println(F("Searching for GPS fix...")); 
  setupAccel();

  //OLED Display
  setupDisplay();  
  SerialUSB.println(F("LoRaWAN OTAA Login initiated..."));     
  startJoining();
  SerialUSB.println(F("LoRaWAN OTAA Login successful..."));
  printLoRaWANSettings();

  //OLED Display
  printDisplay(5);
  delay(1000); 
  
  freeMem();
  
}

void loop() {

voltage = readBatt();

if (voltage > battMin) {

   // Let LMIC handle LoRaWAN background tasks
   os_runstep();       
    
   #if defined(DEVMODE)
     printGPSandSensorData();
   #endif  
  
  // If TX_INTERVAL passed, *and* our previous packet is not still
  // pending (which can happen due to duty cycle limitations), send
  // the next packet.
  if ((millis() - last_packet > TX_INTERVAL && !(LMIC.opmode & (OP_JOINING|OP_TXRXPEND)))){
    
     // Calling myGPS.getPVT() returns true if there actually is a fresh navigation solution available.

    if ((myGPS.getPVT() && (myGPS.getFixType() !=0) && (myGPS.getSIV() > 0)) || !gpsFixRequiredforTX) {

      updateTelemetry();
      
      printDisplay(3);
      sendLoRaWANPacket();
                 
      SerialUSB.println(F("LoRaWAN packet sent.."));
      printLoRaWANSettings();       
      freeMem();

      delay(1000);     
      displayPageID=2;
      
   
    }  

  } 

      //update display
      if (millis() - last_display_refreshed > (displayInterval * 1000)) {
          printDisplay(displayPageID);
          last_display_refreshed = millis();

          ++displayPageID;

          if(displayPageID == 3) {
                displayPageID = 1;
           }
          
       }  
  
  } else {

    SerialUSB.println(F("Voltage is too low, please check your battery..."));
    
  }
  
  delay(1000);
  
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
  
    display.print("GPS Fix : ");display.println(fixMessage);
    display.print("GPS Sats: ");display.println((int)myGPS.getSIV());
    display.print("Lat     : ");display.println(tempLatitude);
    display.print("Long    : ");display.println(tempLongitude);    
    display.print("Altitude: ");display.print((int)tempAltitudeLong);display.println(altUnit);    
    display.print("Speed   : ");display.print((int)tempSpeed);display.println(speedUnit);
    display.print("Battery : ");display.print(voltage); display.println("V");
    break;
  
  case 2:    
    display.print("Temp    :");display.print(tempTemperature);display.println(tempUnit);
    display.print("Pressure:");display.print(tempPressure); display.println("hPa");
    display.print("TX Count:");display.println(txCount-1); 
    display.print("Freq.   :");display.print(LMIC.freq/1000000.f);display.println("MHz");
    display.print("SF&BW   :");display.print(F("SF"));display.print(getSf(LMIC.rps) - SF7 + 7);display.print(F("BW"));display.println(125 << (getBw(LMIC.rps) - BW125));    
    display.print("TXPower :");display.println(LMIC.txpow + LMIC.brdTxPowOff);
    if(txCount >1) {
      display.print("Last TX :");display.print((millis() - last_packet)/1000); display.println(" secs ago");
    }    
    break;   

  case 3:
    display.println("");
    display.println("");    
    display.println("Packet Sending...");
    display.print("PcktSize: ");display.print(sizeof(telemetry));display.println(" bytes"); 
    display.println("");
    break;
    
  case 4:
    display.println("   Helium Mapper");
    display.println("");
    display.println("LoRaWAN OTAA login");
    display.println("initiated...");
    display.println("");
    display.print("Region ");display.println(regionName);   
    
    break;

  case 5:
    display.println(""); 
    display.println("LoRaWAN OTAA login");
    display.println("successful...");
    display.println("");
    display.print("Region ");display.println(regionName);   
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
    sprintf(distUnit, "%s", " m");      
    //Imperial
    sprintf(measureSystem, "%s", "Imperial");
  }
    
  }

//to join LoRaWAN network, every region requires different parameters. Please refer to : https://lora-alliance.org/resource_hub/rp2-1-0-3-lorawan-regional-parameters/
void startJoining() {
  
    // LMIC init
    os_init(nullptr);
    LMIC_reset();

    // Start join
    LMIC_startJoining();   

    //DO NOT CHANGE following code blocks unless you know what you are doing :)

    //Europe
    if(Lorawan_region_code == REGCODE_EU868) {

       sprintf(regionName, "%s", "EU868"); 

       if(!autoSF){
          //DR2 (SF10 BW125kHz)
          LMIC_setDrTxpow(2,KEEP_TXPOWADJ);                      
        } 
 

     //Japan, Malaysia, Singapore, Brunei, Cambodia, Hong Kong, Indonesia, Laos, Taiwan, Thailand, Vietnam
      } else if (Lorawan_region_code == REGCODE_AS923) {

        sprintf(regionName, "%s", "AS923"); 
        
        if(!autoSF){
          //DR2 (SF10 BW125kHz)
          //For AS923, DR2 join only since max payload limit is 11 bytes.
          LMIC_setDrTxpow(2,KEEP_TXPOWADJ);    
        }
                   

      //North and South America (Except Brazil)
      } else if (Lorawan_region_code == REGCODE_US915) {

        sprintf(regionName, "%s", "US915"); 
        if(!autoSF){
          //DR0 (SF10 BW125kHz)
          //For US915, DR0 join only since max payload limit is 11 bytes.
          LMIC_setDrTxpow(0,KEEP_TXPOWADJ);              
        }
        
       //TTN and Helium only supports second sub band (channels 8 to 15)
       //so we should force BasicMAC to initiate a join with second sub band channels.
       LMIC_selectChannel(8); 
      
      //Australia and New Zeleand   
      } else if (Lorawan_region_code == REGCODE_AU915) {

        sprintf(regionName, "%s", "AU915");

        if(!autoSF){
          //DR2 (SF10 BW125kHz)
          //For AU915, DR2 join only since max payload limit is 11 bytes.
          LMIC_setDrTxpow(2,KEEP_TXPOWADJ);                         
        }

       //TTN and Helium only supports second sub band (channels 8 to 15)
       //so we should force BasicMAC to initiate a join with second sub band channels.
       LMIC_selectChannel(8);
        
      } 

    LMIC_setAdrMode(false); //do not enable ADR (Adaptive Data Rate), does not work for every region or gateway
    LMIC_setLinkCheckMode(0);  
    
    // Make sure the first packet is scheduled ASAP after join completes
    last_packet = millis() - TX_INTERVAL;

    //OLED Display
    printDisplay(4);

    // Optionally wait for join to complete (uncomment this is you want
    // to run the loop while joining).

    while ((LMIC.opmode & (OP_JOINING))) {
        os_runstep();
      }

    
}

// Telemetry size is very important, try to keep it lower than 51 bytes. Always lower is better.
void sendLoRaWANPacket(){
    
    //Europa
    if(Lorawan_region_code == REGCODE_EU868) {

        if(!autoSF){
          if(sizeof(telemetry) < 52) {
            //DR2 (SF10 BW125kHz) max payload size is 51 bytes.
            LMIC_setDrTxpow(2,KEEP_TXPOWADJ);          
          } else if (sizeof(telemetry) < 116){
            //DR3 (SF9 BW125kHz) max payload size is 115 bytes.
            LMIC_setDrTxpow(3,KEEP_TXPOWADJ);                                        
          } else {
            //DR4 (SF8 BW125kHz) max payload size is 222 bytes.
            LMIC_setDrTxpow(4,KEEP_TXPOWADJ);           
         }
      }   
           
      //Japan, Malaysia, Singapore, Brunei, Cambodia, Hong Kong, Indonesia, Laos, Taiwan, Thailand, Vietnam
      }  else if (Lorawan_region_code == REGCODE_AS923) {

        if(!autoSF){
          if(sizeof(telemetry) < 54) {
             //DR3 (SF9 BW125kHz) max payload size is 53 bytes.
            LMIC_setDrTxpow(3,KEEP_TXPOWADJ);         
          } else if (sizeof(telemetry) < 126){
            //DR4 (SF8 BW125kHz) max payload size is 125 bytes.
            LMIC_setDrTxpow(4,KEEP_TXPOWADJ);                                        
          } else {
            //DR5 (SF7 BW125kHz) max payload size is 222 bytes.
            LMIC_setDrTxpow(5,KEEP_TXPOWADJ);        
          } 
          
        }
  

      //North and South America (Except Brazil) or Australia and New Zeleand 
      } else if (Lorawan_region_code == REGCODE_US915 || Lorawan_region_code == REGCODE_AU915) {

        if(!autoSF){
          //North and South America (Except Brazil)
          if (Lorawan_region_code == REGCODE_US915){      
            if(sizeof(telemetry) < 54) {
             //DR1 (SF9 BW125kHz) max payload size is 53 bytes.
              LMIC_setDrTxpow(1,KEEP_TXPOWADJ);         
            } else if (sizeof(telemetry) < 126){
              //DR2 (SF8 BW125kHz) max payload size is 125 bytes.
              LMIC_setDrTxpow(2,KEEP_TXPOWADJ);                                        
            } else {
              //DR3 (SF7 BW125kHz) max payload size is 222 bytes.
              LMIC_setDrTxpow(3,KEEP_TXPOWADJ);               
            }          

          //Australia and New Zeleand                       
          } else if (Lorawan_region_code == REGCODE_AU915){ 
            if(sizeof(telemetry) < 54) {
              //DR3 (SF9 BW125kHz) max payload size is 53 bytes.
              LMIC_setDrTxpow(3,KEEP_TXPOWADJ);         
            } else if (sizeof(telemetry) < 126){
              //DR4 (SF8 BW125kHz) max payload size is 125 bytes.
              LMIC_setDrTxpow(4,KEEP_TXPOWADJ);                                        
            } else {
              //DR5 (SF7 BW125kHz) max payload size is 222 bytes.
              LMIC_setDrTxpow(5,KEEP_TXPOWADJ);             
            }          
          }
          
        }
          
       //TTN and Helium only supports second sub band (channels 8 to 15)
       //so we should force BasicMAC use second sub band channels.
       LMIC_selectChannel(channelNoFor2ndSubBand);
       
       ++channelNoFor2ndSubBand;
       if(channelNoFor2ndSubBand > 15) {
          channelNoFor2ndSubBand = 8;
        }
     //India     
     } else if (Lorawan_region_code == REGCODE_IN865) {

      if(!autoSF){
        if(sizeof(telemetry) < 52) {
            //DR2 (SF10 BW125kHz) max payload size is 51 bytes.
            LMIC_setDrTxpow(2,KEEP_TXPOWADJ);          
          } else if (sizeof(telemetry) < 116){
            //DR3 (SF9 BW125kHz) max payload size is 115 bytes.
            LMIC_setDrTxpow(3,KEEP_TXPOWADJ);                                        
          } else {
            //DR4 (SF8 BW125kHz) max payload size is 222 bytes.
            LMIC_setDrTxpow(4,KEEP_TXPOWADJ);   
            
          }     
      }

    }     
      
    LMIC_setAdrMode(false);
    LMIC_setLinkCheckMode(0);
    
    LMIC_setTxData2(1, telemetry.bytes, sizeof(telemetry), 0);
    last_packet = millis();
    txCount++;
    SerialUSB.println(F("Packet queued..."));
  
  }

void setupAccel() {
  myIMU.settings.adcEnabled = 1;
  myIMU.settings.accelSampleRate = 400;  //Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
  myIMU.settings.accelRange = 16;  //Max G force readable.  Can be: 2, 4, 8, 16
  myIMU.settings.xAccelEnabled = 1;
  myIMU.settings.yAccelEnabled = 1;
  myIMU.settings.zAccelEnabled = 1;
  
  //Call .begin() to configure the IMU
  myIMU.begin();

  delay(100);

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

  myGPS.setNavigationFrequency(2);//Set output to 2 times a second. Max is 10
  byte rate = myGPS.getNavigationFrequency(); //Get the update rate of this module
  SerialUSB.print("Current update rate for GPS: ");
  SerialUSB.println(rate);

  myGPS.saveConfiguration(); //Save the current settings to flash and BBR  
  
  }

void printGPSandSensorData()
{
 
    lastTime = millis(); //Update the timer

    byte fixType = myGPS.getFixType();

    SerialUSB.print(F("FixType: "));
    SerialUSB.print(fixType);    

    int SIV = myGPS.getSIV();
    SerialUSB.print(F(" Sats: "));
    SerialUSB.print(SIV); 

    int horDop = myGPS.getHorizontalDOP();
    SerialUSB.print(F(" HDOP: "));
    SerialUSB.print(horDop);     

    float flat = myGPS.getLatitude() / 10000000.f;
    
    SerialUSB.print(F(" Lat: "));
    SerialUSB.print(flat);    

    float flong = myGPS.getLongitude() / 10000000.f;    
    SerialUSB.print(F(" Long: "));
    SerialUSB.print(flong);        

    float altitude = myGPS.getAltitude() / 1000;
    SerialUSB.print(F(" Alt: "));
    SerialUSB.print(altitude);
    SerialUSB.print(F(" (m)"));

    //float speed = myGPS.getGroundSpeed() * 0.0036f;
    //SerialUSB.print(F(" Speed: "));
    //SerialUSB.print(speed);
    //SerialUSB.print(F(" (km/h)"));

    //long heading = myGPS.getHeading() / 100000;
    //SerialUSB.print(F(" Heading: "));
    //SerialUSB.print(heading);
    //SerialUSB.print(F(" (degrees)"));
        
    SerialUSB.print(" Time: ");    
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
    
    SerialUSB.print(" Temp: ");
    SerialUSB.print(bmp.readTemperature());
    SerialUSB.print(" C");
    
    SerialUSB.print(" Press: ");    
    SerialUSB.print(bmp.readPressure() / 100.0);
    SerialUSB.print(" hPa");

    //SerialUSB.print(" Accel: ");      
    //SerialUSB.print(myIMU.readFloatAccelX());
    //SerialUSB.print(",");
    //SerialUSB.print(myIMU.readFloatAccelY()- 0.98f);
    //SerialUSB.print(",");
    //SerialUSB.print(myIMU.readFloatAccelZ());
    //SerialUSB.print(" ");
    SerialUSB.println();
   

}  

void updateTelemetry() {

  tempAltitudeLong = 0; //meters or feet
  tempAltitudeShort = 0; //km or miles
  tempSpeed = 0; //km or miles
  tempTemperature = 0; //Celcius or Fahrenheit

  if(measurementSystem == 0){ //Metric
    
    tempAltitudeLong = myGPS.getAltitude() / 1000.f; //meters
    tempAltitudeShort = tempAltitudeLong / 1000.f; //kilometers   
    tempSpeed = myGPS.getGroundSpeed() * 0.0036f; //km/hour    
    tempTemperature = bmp.readTemperature(); //Celsius   
    
   } else { //Imperial
    
    tempAltitudeLong = (myGPS.getAltitude() * 3.2808399)  / 1000.f;//feet
    tempAltitudeShort = tempAltitudeLong / 5280.f;//miles       
    tempSpeed = myGPS.getGroundSpeed() *  0.00223694f; //mile/hour    
    tempTemperature =  (bmp.readTemperature() * 1.8f) + 32; //Fahrenheit      
  }

  tempLatitude  = myGPS.getLatitude() / 10000000.f;
  tempLongitude = myGPS.getLongitude() / 10000000.f;
  tempSats = myGPS.getSIV();      
  tempHeading = myGPS.getHeading() / 100000;
  tempPressure = bmp.readPressure() / 100.0f;
  tempHDOP = myGPS.getHorizontalDOP();

  telemetry.f[0] = tempLatitude;
  telemetry.f[1] = tempLongitude;
  telemetry.f[2] = myGPS.getAltitude() / 1000.f;
  telemetry.f[3] = tempHDOP;

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

void freeMem() {
#if defined(DEVMODE)
  SerialUSB.print(F("Free RAM: ")); SerialUSB.print(freeMemory(), DEC); SerialUSB.println(F(" byte"));
#endif

}

void os_getJoinEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getNwkKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

void onLmicEvent (ev_t ev) {
    SerialUSB.print(os_getTime());
    SerialUSB.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            SerialUSB.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            SerialUSB.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            SerialUSB.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            SerialUSB.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            SerialUSB.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            SerialUSB.println(F("EV_JOINED"));
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            SerialUSB.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            SerialUSB.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            SerialUSB.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            SerialUSB.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              SerialUSB.println(F("Received ack"));
            if (LMIC.dataLen) {
              SerialUSB.print(F("Received "));
              SerialUSB.print(LMIC.dataLen);
              SerialUSB.println(F(" bytes of payload"));
            }
            break;
        case EV_LOST_TSYNC:
            SerialUSB.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            SerialUSB.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            SerialUSB.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            SerialUSB.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            SerialUSB.println(F("EV_LINK_ALIVE"));
            break;
        case EV_SCAN_FOUND:
            SerialUSB.println(F("EV_SCAN_FOUND"));
            break;
        case EV_TXSTART:
            SerialUSB.println(F("EV_TXSTART"));
            break;
        case EV_TXDONE:
            SerialUSB.println(F("EV_TXDONE"));
            break;
        case EV_DATARATE:
            SerialUSB.println(F("EV_DATARATE"));
            break;
        case EV_START_SCAN:
            SerialUSB.println(F("EV_START_SCAN"));
            break;
        case EV_ADR_BACKOFF:
            SerialUSB.println(F("EV_ADR_BACKOFF"));
            break;

         default:
            SerialUSB.print(F("Unknown event: "));
            SerialUSB.println(ev);
            break;
    }
}

void printLoRaWANSettings(){

  SerialUSB.println(F("------------------------------------------------------------------------------------"));
  SerialUSB.print(F("Region: "));    
  SerialUSB.print(regionName);     
  SerialUSB.print(F(", Freq: "));    
  SerialUSB.print(LMIC.freq/1000000.f);                   
  SerialUSB.print(F("MHz, SF"));              
  SerialUSB.print(getSf(LMIC.rps) - SF7 + 7);
  SerialUSB.print(F("BW"));   
  SerialUSB.print(125 << (getBw(LMIC.rps) - BW125));
  SerialUSB.print(F(", Power: "));              
  SerialUSB.print(LMIC.txpow + LMIC.brdTxPowOff);
  SerialUSB.print(F("dBm, Payload Size: "));
  SerialUSB.print(sizeof(telemetry));
  SerialUSB.println(F(" bytes"));      
  SerialUSB.println(F("------------------------------------------------------------------------------------"));    
  
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
        SerialUSB.print(F("Ublox Dynamic platform model changed successfully! : "));
        SerialUSB.println(myGPS.getDynamicModel());
    }
  
  } 
