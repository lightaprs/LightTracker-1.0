#include <basicmac.h>
#include <hal/hal.h>
#include <Adafruit_BMP085.h>
#include <LightTrackerGeofence.h>
#include "SparkFun_Ublox_Arduino_Library.h"
#include <MemoryFree.h>;
#include <CayenneLPP.h>
#include "SparkFunLIS3DH.h"

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


//#define DEVMODE // Development mode. Uncomment to enable for debugging.

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
const unsigned TX_INTERVAL = 60000;  // Schedule TX every this many miliseconds (might become longer due to duty cycle limitations).

//try to keep telemetry size smaller than 51 bytes if possible. Default telemetry size is 45 bytes.
CayenneLPP telemetry(51);

// The LoRaWAN region to use, automatically selected based on your location. So GPS fix is necesarry
u1_t os_getRegion (void) { return Lorawan_Geofence_region_code; } //do not change this

// GEOFENCE 
uint8_t Lorawan_Geofence_no_tx  = 0; //do not change this
uint8_t Lorawan_Geofence_region_code = _REGCODE_UNDEF; //do not change this
uint8_t Lorawan_Geofence_special_region_code = _REGCODE_UNDEF; //do not change this

uint8_t lastLoRaWANRegion = _REGCODE_UNDEF; //do not change this

boolean OTAAJoinStatus = false; //do not change this.
int channelNoFor2ndSubBand = 8; //do not change this. Used for US915 and AU915 / TTN and Helium
uint32_t last_packet = 0; //do not change this. Timestamp of last packet sent. 

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
boolean gpsFix=false; //do not change this.
boolean ublox_high_alt_mode_enabled = false; //do not change this.
boolean gpsBMPSetup=false; //do not change this.
//********************************* Power Settings ******************************
int   battWait=60;    //seconds sleep if super capacitors/batteries are below battMin (important if power source is solar panel) 
float battMin=3.5;    // min Volts to TX. (Works with 3.3V too but 3.5V is safer) 
float gpsMinVolt=4.5; //min Volts for GPS to wake up. (important if power source is solar panel) //do not change this
//********************************* Misc Settings ******************************
int txCount = 1;
float voltage = 0;

void setup() {
  
  pinMode(A1,INPUT_PULLUP);
  if(digitalRead(A1)==LOW) while(1);

  delay(5000); //do not change this

  pinMode(GpsPwr, OUTPUT);    
  GpsOFF; 
  
  SerialUSB.begin(115200);
    
  // Wait up to 5 seconds for serial to be opened, to allow catching
  // startup messages on native USB boards (that do not reset when
  // serial is opened).
  unsigned long start = millis();
  while (millis() - start < 5000 && !SerialUSB);

  SerialUSB.println();
  SerialUSB.println(F("Starting"));
  SerialUSB.println();
  freeMem();
  
}

void loop() {

voltage = readBatt();

if (((voltage > battMin) && gpsFix) || ((voltage > gpsMinVolt) && !gpsFix)) {

  if(!gpsBMPSetup){
    
    SerialUSB.println(F("GPS setup"));   
    setupGPS_BMP();
    SerialUSB.println(F("Searching for GPS fix...")); 
    setupAccel();
    gpsBMPSetup = true;
    freeMem();
    
  }

  if(gpsFix) {
    // Let LMIC handle LoRaWAN background tasks
    os_runstep();      
  }
    
  //SerialUSB.println(millis() - lastLoRaPacket);
  
  // If TX_INTERVAL passed, *and* our previous packet is not still
  // pending (which can happen due to duty cycle limitations), send
  // the next packet.
  if ((millis() - last_packet > TX_INTERVAL && !(LMIC.opmode & (OP_JOINING|OP_TXRXPEND))) || !gpsFix){
  
    GpsON;
    delay(500);

    if(!ublox_high_alt_mode_enabled){
      setupUBloxDynamicModel();
    }

     // Calling myGPS.getPVT() returns true if there actually is a fresh navigation solution available.

    if (myGPS.getPVT() && (myGPS.getFixType() !=0) && (myGPS.getSIV() > 0)) {
      gpsFix=true;     
      
      checkRegionByLocation();

      if(lastLoRaWANRegion != Lorawan_Geofence_region_code) {
          SerialUSB.println(F("Region has changed, force LoRaWAN OTAA Login")); 
          OTAAJoinStatus = false;
          lastLoRaWANRegion = Lorawan_Geofence_region_code;
        }

      if(!OTAAJoinStatus && (Lorawan_Geofence_no_tx == 0)){            
          SerialUSB.println(F("LoRaWAN OTAA Login initiated..."));     
          startJoining();
          SerialUSB.println(F("LoRaWAN OTAA Login success..."));     
          OTAAJoinStatus = true;
          freeMem();
      }
      
      updateTelemetry();
      #if defined(DEVMODE)        
        SerialUSB.print(F("Telemetry Size: "));
        SerialUSB.print(telemetry.getSize());
        SerialUSB.println(F(" bytes"));
      #endif   

      //need to save power
      if (readBatt() < 4.5) {
       GpsOFF;
       ublox_high_alt_mode_enabled = false; //gps sleep mode resets high altitude mode.
       delay(500);      
      }
      
      if (Lorawan_Geofence_no_tx == 0) {
          sendLoRaWANPacket();
          SerialUSB.println(F("LoRaWAN packet sent.."));  
        }                 
       
        freeMem();
   
    } 

      #if defined(DEVMODE)
        printGPSandSensorData();
      #endif    

  }

  //this code block protecting serial connected (3V + 3V) super caps from overcharging by powering on GPS module.
  //GPS module uses too much power while on, so if voltage is too high for supercaps, GPS ON.
  if (readBatt() > 6.5) {
       GpsON;
       delay(500);
   }
  
  } else {

    GpsOFF;
    ublox_high_alt_mode_enabled = false; //gps sleep mode resets high altitude mode.     
    delay(battWait * 1000);
    
  }
  
  delay(1000);
  
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
    if(Lorawan_Geofence_region_code == _REGCODE_EU868) {

      SerialUSB.println(F("Region EU868"));

      //A little hack for Russian region since BasicMAC does not officially support RU864-870. Tested on TTN and worked..   
      if(Lorawan_Geofence_special_region_code == _REGCODE_RU864) {
        SerialUSB.println(F("Special Region RU864"));
        LMIC_setupChannel(0, 868900000, DR_RANGE_MAP(0, 2));
        LMIC_setupChannel(0, 869100000, DR_RANGE_MAP(0, 2));      
      } 
       //DR2 (SF10 BW125kHz)
       //SF10 is better/optimum spreading factor for high altitude balloons
       LMIC_setDrTxpow(2,KEEP_TXPOWADJ);        

     //Japan, Malaysia, Singapore, Brunei, Cambodia, Hong Kong, Indonesia, Laos, Taiwan, Thailand, Vietnam
      } else if (Lorawan_Geofence_region_code == _REGCODE_AS923) {
        SerialUSB.println(F("Region AS923"));
        
     //A little hack for Korean region since BasicMAC does not officially support KR920-923. Tested on TTN and worked..
      if(Lorawan_Geofence_special_region_code == _REGCODE_KR920) {
        SerialUSB.println(F("Special Region KR920"));        
        LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(0, 2));
        LMIC_setupChannel(0, 922300000, DR_RANGE_MAP(0, 2));
        LMIC_setupChannel(0, 922500000, DR_RANGE_MAP(0, 2));
      }
           
       //DR2 (SF10 BW125kHz)
       //For AS923, DR2 join only since max payload limit is 11 bytes.
       LMIC_setDrTxpow(2,KEEP_TXPOWADJ); 

      //North and South America (Except Brazil)
      } else if (Lorawan_Geofence_region_code == _REGCODE_US915) {

        SerialUSB.println(F("Region US915"));

       //DR0 (SF10 BW125kHz)
       //For US915, DR0 join only since max payload limit is 11 bytes.
       LMIC_setDrTxpow(0,KEEP_TXPOWADJ);
       //TTN and Helium only supports second sub band (channels 8 to 15)
       //so we should force BasicMAC to initiate a join with second sub band channels.
       LMIC_selectChannel(8); 
      
      //Australia and New Zeleand   
      } else if (Lorawan_Geofence_region_code == _REGCODE_AU915) {

         SerialUSB.println(F("Region AU915"));
       //DR2 (SF10 BW125kHz)
       //For AU915, DR2 join only since max payload limit is 11 bytes.
       LMIC_setDrTxpow(2,KEEP_TXPOWADJ);
       //TTN and Helium only supports second sub band (channels 8 to 15)
       //so we should force BasicMAC to initiate a join with second sub band channels.
       LMIC_selectChannel(8);
        
      } else {
        
          LMIC_setDrTxpow(2,KEEP_TXPOWADJ);  
        
        }

    LMIC_setAdrMode(false);
    LMIC_setLinkCheckMode(0);  
    
    // Make sure the first packet is scheduled ASAP after join completes
    last_packet = millis() - TX_INTERVAL;


    // Optionally wait for join to complete (uncomment this is you want
    // to run the loop while joining).
    while ((LMIC.opmode & (OP_JOINING))) {
        os_runstep();
      }
  
    
}

// Telemetry size is very important, try to keep it lower than 51 bytes. Always lower is better.
void sendLoRaWANPacket(){
    
    //Europa
    if(Lorawan_Geofence_region_code == _REGCODE_EU868) {

        if(telemetry.getSize() < 52) {
            //DR2 (SF10 BW125kHz) max payload size is 51 bytes.
            LMIC_setDrTxpow(2,KEEP_TXPOWADJ);          
          } else if (telemetry.getSize() < 116){
            //DR3 (SF9 BW125kHz) max payload size is 115 bytes.
            LMIC_setDrTxpow(3,KEEP_TXPOWADJ);                                        
          } else {
            //DR4 (SF8 BW125kHz) max payload size is 222 bytes.
            LMIC_setDrTxpow(4,KEEP_TXPOWADJ);   
            
            }
      //Japan, Malaysia, Singapore, Brunei, Cambodia, Hong Kong, Indonesia, Laos, Taiwan, Thailand, Vietnam
      }  else if (Lorawan_Geofence_region_code == _REGCODE_AS923) {

        if(telemetry.getSize() < 54) {
             //DR3 (SF9 BW125kHz) max payload size is 53 bytes.
            LMIC_setDrTxpow(3,KEEP_TXPOWADJ);         
          } else if (telemetry.getSize() < 126){
            //DR4 (SF8 BW125kHz) max payload size is 125 bytes.
            LMIC_setDrTxpow(4,KEEP_TXPOWADJ);                                        
          } else {
            //DR5 (SF7 BW125kHz) max payload size is 222 bytes.
            LMIC_setDrTxpow(5,KEEP_TXPOWADJ);   
            
            }   

      //North and South America (Except Brazil) or Australia and New Zeleand 
      } else if (Lorawan_Geofence_region_code == _REGCODE_US915 || Lorawan_Geofence_region_code == _REGCODE_AU915) {

        //North and South America (Except Brazil)
        if (Lorawan_Geofence_region_code == _REGCODE_US915){
          
          if(telemetry.getSize() < 54) {
             //DR1 (SF9 BW125kHz) max payload size is 53 bytes.
              LMIC_setDrTxpow(1,KEEP_TXPOWADJ);         
          } else if (telemetry.getSize() < 126){
            //DR2 (SF8 BW125kHz) max payload size is 125 bytes.
            LMIC_setDrTxpow(2,KEEP_TXPOWADJ);                                        
          } else {
            //DR3 (SF7 BW125kHz) max payload size is 222 bytes.
            LMIC_setDrTxpow(3,KEEP_TXPOWADJ);               
          }          

        //Australia and New Zeleand                       
        } else if (Lorawan_Geofence_region_code == _REGCODE_AU915){
          
          if(telemetry.getSize() < 54) {
             //DR3 (SF9 BW125kHz) max payload size is 53 bytes.
            LMIC_setDrTxpow(3,KEEP_TXPOWADJ);         
          } else if (telemetry.getSize() < 126){
            //DR4 (SF8 BW125kHz) max payload size is 125 bytes.
            LMIC_setDrTxpow(4,KEEP_TXPOWADJ);                                        
          } else {
            //DR5 (SF7 BW125kHz) max payload size is 222 bytes.
            LMIC_setDrTxpow(5,KEEP_TXPOWADJ);             
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
     } else if (Lorawan_Geofence_region_code == _REGCODE_IN865) {

        if(telemetry.getSize() < 52) {
            //DR2 (SF10 BW125kHz) max payload size is 51 bytes.
            LMIC_setDrTxpow(2,KEEP_TXPOWADJ);          
          } else if (telemetry.getSize() < 116){
            //DR3 (SF9 BW125kHz) max payload size is 115 bytes.
            LMIC_setDrTxpow(3,KEEP_TXPOWADJ);                                        
          } else {
            //DR4 (SF8 BW125kHz) max payload size is 222 bytes.
            LMIC_setDrTxpow(4,KEEP_TXPOWADJ);   
            
            }
      }     
      
    LMIC_setAdrMode(false);
    LMIC_setLinkCheckMode(0);
    
    LMIC_setTxData2(1, telemetry.getBuffer(), telemetry.getSize(), 0);
    last_packet = millis();
    txCount++;
    SerialUSB.print(F("Packet queued..."));
  
  }

void setupAccel() {
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

  float tempAltitudeLong = 0; //meters or feet
  float tempAltitudeShort = 0; //km or miles
  float tempSpeed = 0; //km or miles
  float tempTemperature = 0; //Celcius or Fahrenheit

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

  //latitude,longtitude,altitude,speed,course,sattelite,battery,temp,pressure

  telemetry.reset();// clear the buffer
  telemetry.addGPS(1, myGPS.getLatitude() / 10000000.f, myGPS.getLongitude() / 10000000.f, tempAltitudeLong);   // channel 3, coordinates and altitude (meters or feet)
  telemetry.addTemperature(2, tempTemperature); // Celcius or Fahrenheit
  telemetry.addAnalogInput(3, voltage); //Battery/Supercaps voltage
  telemetry.addDigitalInput(4, myGPS.getSIV()); //GPS sattelites in view
  telemetry.addAnalogInput(5, tempSpeed);  //km/h or mile/h
  telemetry.addDigitalInput(6, myGPS.getHeading() / 100000); //course in degrees   
  telemetry.addBarometricPressure(7, bmp.readPressure() / 100.f); //pressure  
  telemetry.addAccelerometer(8,myIMU.readFloatAccelX(),myIMU.readFloatAccelY()- 0.98f,myIMU.readFloatAccelZ());   
  telemetry.addAnalogInput(9, tempAltitudeShort); //kilometers or miles

}

void checkRegionByLocation() {

  float tempLat = myGPS.getLatitude() / 10000000.f;
  float tempLong = myGPS.getLongitude() / 10000000.f;
  
  Lorawan_Geofence_position(tempLat,tempLong);
    
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
      #if defined(DEVMODE)
        SerialUSB.print(F("Dynamic platform model changed successfully! : "));
        SerialUSB.println(myGPS.getDynamicModel());
      #endif  
    }
  
  } 
