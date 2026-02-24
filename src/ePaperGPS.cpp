/**************************************************!
   GPS handling functions
***************************************************/
#include <Arduino.h>

#include "ePaperAnchorAlarm.h"
#include "ePaperGPS.h"
#include "global.h"

// stuff for optimization of gps receiver 
// presently not used.

// write message as single bytes with 5 ms pause in between
size_t sendUBX(HardwareSerial& gpsSerial, uint8_t* msg, uint8_t len) {
  size_t written = 0;
  for(int i = 0; i < len; i++) {                        
    written += gpsSerial.write(msg[i]);
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
  return written;
}  

// gets "PCAS12,10" and returns "$PCAS12,10*2F\r\n"
// checksum ist after *, and is the bitwise XOR of all characters between $ and *
void buildATGM336Message(char* msg)
{
  char checksum;

  char inMsg[200];
  char outMsg[200];
  sprintf(outstring,"buildATGM336Message msg: _%s_", msg);
  logOut(2, outstring);

  strcpy(inMsg, msg);
  sprintf(outstring,"buildATGM336Message inMsg: _%s_", inMsg);
  logOut(2, outstring);

  checksum = inMsg[0];
  
  for(int i=1; i<strlen(inMsg); i++){
    checksum = checksum ^ inMsg[i];
  }
  sprintf(outMsg,"$%s*%X\r\n", inMsg, checksum);
  strcpy(msg, outMsg);

  sprintf(outstring,"Checksum added to: _%s_ Result: _%s_", inMsg, outMsg);
  logOut(2, outstring);
}

void addChecksum(uint8_t* msg, uint8_t len) {
  uint8_t ckA = 0, ckB = 0;
  for (uint8_t i = 2; i < len - 2; i++) {
    ckA += msg[i];
    ckB += ckA;
  }
  msg[len - 2] = ckA;
  msg[len - 1] = ckB;
}

/*****************************************************************************! 
  @brief  configure GPS messages
  @details via binary UBX statement
  @details disable all NMEA messages, tehen re-enable the required ones. Disable UBX messages
  @return void
*****************************************************************************/
// these data are taken from Ublox U-Center (incl. checksums)
const char UBLOX_MSG_INIT[] PROGMEM = {
  // Disable NMEA
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32, // GxGSA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39, // GxGSV off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47, // GxVTG off

  // enable NMEA, which ever is activated (gotten from Message monitor in Ublox U-Center)
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x28,  // GGA on, on UART1
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x01,0x2F,  // GLL on, on UART1   
  //0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x02,0x36,  // GSA on, on UART1   
  //0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x03,0x3D,  // GSV on, on UART1    
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x01,0x00,0x00,0x00,0x00,0x04,0x44,  // RMC on, on UART1   
  //0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x01,0x00,0x00,0x00,0x00,0x05,0x4B,  // VTG on, on UART1   

  // Disable UBX
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0xDC, //NAV-PVT off
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9, //NAV-POSLLH off
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0, //NAV-STATUS off

  // Enable UBX
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //NAV-PVT on
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH on
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS on

  // Rate
  //0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)
};

void configureGpsMessages(HardwareSerial& gpsSerial){
  sprintf(outstring,"configureGPSMessages");
  logOut(2,outstring);
  // send configuration data in UBX protocol
  for(int i = 0; i < sizeof(UBLOX_MSG_INIT); i++) {                        
    gpsSerial.write( pgm_read_byte(UBLOX_MSG_INIT+i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
}  

/*****************************************************************************! 
  @brief  configure GPS for "pedestrian" mode (default) - data from UKHAS files
  @details via binary UBX statement
  @return void
*****************************************************************************/
// CFG-NAV5 (0x06 0x24): Get/Set Navigation Engine Settings
const char UBLOX_MODE_PEDESTRIAN[] PROGMEM = { // Pedestrian mode, high accuracy at slow speed
0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF,0x03,0x03,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x05,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x2C,
0x01,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x4A,0x75 
};

// configure GPS for "pedestrian" mode: optimized accuracy for slow speeds
void configurePedestrian(HardwareSerial& gpsSerial){
  delay(100);
  logOut(2,(char*)"configure UKHAS 'Pedestrian' Mode");
  // send configuration data in UBX protocol
  for(int i = 0; i < sizeof(UBLOX_MODE_PEDESTRIAN); i++) {                        
    gpsSerial.write( pgm_read_byte(UBLOX_MODE_PEDESTRIAN + i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
} 

/*****************************************************************************! 
  @brief  configure GPS for "portable" mode (default) - data from UKHAS files
  @details via binary UBX statement
  @return void
*****************************************************************************/
const char UBLOX_MODE_PORTABLE[] PROGMEM = { // Portable mode, default
0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF,0x00,0x03,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x05,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x2C,
0x01,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x47,0x0F 
};

void configurePortable(HardwareSerial& gpsSerial){
  delay(100);
  logOut(2,(char*)"configure UKHAS 'Portable' Mode");
  // send configuration data in UBX protocol
  for(int i = 0; i < sizeof(UBLOX_MODE_PORTABLE); i++) {                        
    gpsSerial.write( pgm_read_byte(UBLOX_MODE_PORTABLE + i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
} 

/*****************************************************************************! 
  @brief  set power saving mode - data from UKHAS files
  @details via binary UBX statement
  @return void
*****************************************************************************/
//UKHAS Wiki: Set GPS ot Power Save Mode (Default Cyclic 1s)
// CFG-RXM (0x06 0x11) : Here the GPS ist set to power save mode (0x01)
uint8_t setUKhasPSM[] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92 };
// CFG-RXM (0x06 0x11) : Here the GPS ist set to continuous mode (0x00)
uint8_t setUKhasCoM[] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91};  

void configureUKhasPSM(HardwareSerial& gpsSerial){ // power saving mode
  delay(100);
  logOut(2,(char*)"configure UKHAS 'Power Saving' Mode");
  // send configuration data in UBX protocol
  for(int i = 0; i < sizeof(setUKhasPSM); i++) {                        
    gpsSerial.write( pgm_read_byte(setUKhasPSM + i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
} 

void configureUKhasCoM(HardwareSerial& gpsSerial){ // continuous mode
  delay(100);
  logOut(2,(char*)"configure UKHAS 'Power Saving' Mode");
  // send configuration data in UBX protocol
  for(int i = 0; i < sizeof(setUKhasCoM); i++) {                        
    gpsSerial.write( pgm_read_byte(setUKhasCoM + i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
} 

/*****************************************************************************! 
  @brief  set power saving mode PM2 for 30 sec or 1 sec update
  @details via binary UBX statement
  @details 30 sec to save power, 1 sec for normal fast operation
  @return void
*****************************************************************************/

//// CFG-PM2 (0x06 0x3B) - extended Power Management configuration
//const char UBLOX_PM2_30SEC[] PROGMEM = { // configure power management to 30000 ms updatePeriod. old, possibly wrong
//  // searchPeriod: 30000 ms minAquTime: 0 ms onTime: 2000 ms
//0xB5,0x62,0x06,0x3B,0x2C,0x00,0x01,0x06,0x00,0x00,0x0E,0x90,0x42,0x01,0xE8,0x03,0x00,0x00,0x10,0x27,0x00,0x00,0x00,0x00,0x00,0x00,0x02,
//0x00,0x00,0x00,0x2C,0x01,0x00,0x00,0x4F,0xC1,0x03,0x00,0x87,0x02,0x00,0x00,0xFF,0x00,0x00,0x00,0x64,0x40,0x01,0x00,0xE6,0xAE
//};
//
//const char UBLOX_PM2_1SEC[] PROGMEM = { // configure power management to 1000 ms updatePeriod, possibly wrong
//  // searchPeriod: 1000 ms minAquTime: 0 ms onTime: 2000 ms
//0xB5,0x62,0x06,0x3B,0x2C,0x00,0x01,0x06,0x00,0x00,0x0E,0x90,0x42,0x01,0xE8,0x03,0x00,0x00,0x10,0x27,0x00,0x00,0x00,0x00,0x00,0x00,0x02,
//0x00,0x00,0x00,0x2C,0x01,0x00,0x00,0x4F,0xC1,0x03,0x00,0x87,0x02,0x00,0x00,0xFF,0x00,0x00,0x00,0x64,0x40,0x01,0x00,0xE6,0xAE   
//};

const char UBLOX_PM2_1SEC[] PROGMEM = { // configure power management to 1000 ms updatePeriod. from U-Center 15.2.26
// 1000 ms update period (E8 03), 10 sec search period, 0 minAcqTime, 0 On Time
0xB5,0x62,0x06,0x3B,0x2C,0x00,0x01,0x06,0x00,0x00,0x0E,0x10,0x42,0x01,0xE8,0x03,0x00,0x00,0x10,0x27,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x2C,0x01,0x00,0x00,0x4F,0xC1,0x03,0x00,0x86,0x02,0x00,0x00,0xFE,0x00,0x00,0x00,0x64,0x40,0x01,0x00,0x62,0xEA 
};

// CFG-PM2 (0x06 0x3B) - extended Power Management configuration
const char UBLOX_PM2_10SEC[] PROGMEM = { // configure power management to 10000 ms updatePeriod. from U-Center 15.2.26
// 10000 ms update period (10 27), 10 sec search period, 0 minAcqTime, 0 On Time
0xB5,0x62,0x06,0x3B,0x2C,0x00,0x01,0x06,0x00,0x00,0x0E,0x10,0x42,0x01,0x10,0x27,0x00,0x00,0x10,0x27,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x2C,0x01,0x00,0x00,0x4F,0xC1,0x03,0x00,0x86,0x02,0x00,0x00,0xFE,0x00,0x00,0x00,0x64,0x40,0x01,0x00,0xAE,0x76
};

void configurePM2Slow(HardwareSerial& gpsSerial){
  // send configuration data in UBX protocol
  sprintf(outstring,"configurePM2Slow");
  logOut(2,outstring);
  for(int i = 0; i < sizeof(UBLOX_PM2_10SEC); i++) {                        
    gpsSerial.write( pgm_read_byte(UBLOX_PM2_10SEC + i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
} 

void configurePM2Fast(HardwareSerial& gpsSerial){
  // send configuration data in UBX protocol
  sprintf(outstring,"configurePM2Fast");
  logOut(2,outstring);
  for(int i = 0; i < sizeof(UBLOX_PM2_1SEC); i++) {                        
    gpsSerial.write( pgm_read_byte(UBLOX_PM2_1SEC + i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
} 

// UKHAS Wiki: Switch the RF GPS section off, draws about 5mA, retains its settings, 
//             wakes on serial command.
// CFG-RST (0x06 0x04) Reset Receiver / Clear Backup Data Structures
// 0x08: controlled gps stop. 
// The receiver will not be restarted, but will stop any GPS related processing
uint8_t GPSoff[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00,0x08, 0x00, 0x16, 0x74};

// 0x09: controlled gps start
uint8_t GPSon[] =  {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00,0x09, 0x00, 0x17, 0x76};

// Abschalten des HF Teils und und der Messages vom GPS.
// TinyGPS zeigt immer noch die jeweils letzten Daten als valide an. 
void configureUKhasGPSoff(HardwareSerial& gpsSerial){
  // send configuration data in UBX protocol
  sprintf(outstring,"configureUKhasGPSoff");
  logOut(2,outstring);
  for(int i = 0; i < sizeof(GPSoff); i++) {                        
    gpsSerial.write( pgm_read_byte(GPSoff + i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
} 

// Wieder einschalten des HF Teils und der Messages des GPS. Funktioniert, braucht aber ein par sec bis wieder valide Daten kommen
void configureUKhasGPSon(HardwareSerial& gpsSerial){
  sprintf(outstring,"configureUKhasGPSon");
  logOut(2,outstring);  
  // send configuration data in UBX protocol
  for(int i = 0; i < sizeof(GPSon); i++) {                        
    gpsSerial.write( pgm_read_byte(GPSon + i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
} 

//---------------- Power save configuration as proposed in Ublox application note 
//---------------- Power Management Considerations for u-blox 7 and M8 GNSS receivers
// Three Steps:
// 1. UBX-CFG GNSS configuration to disable Glonass, Galileo, Beidou (M8N can do power save only with GPS)
// 2. UBX-CFG PM2  configuration to set desired PM Mode: on/ off or cyclic mode (we are using cyclic)
// 3. UBX-CFG RXM  configuration to set power mode to "1 - Power Save Mode"

const char UBLOX_GNSS_ONLY_GPS[] PROGMEM = { // configure CFG - GNSS to GPS satellites only
0xB5,0x62,0x06,0x3E,0x3C,0x00,0x00,0x00,0x16,0x07,0x00,0x08,0xFF,0x00,0x01,0x00,0x00,
0x01,0x01,0x00,0x03,0x00,0x01,0x00,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x01,
0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x05,
0x00,0x03,0x00,0x00,0x00,0x00,0x01,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xC8,0xDA
};

const char UBLOX_GNSS_PM2_CYCLIC[] PROGMEM = { // configure CFG-PM2 - cyclic, updatePerios 10000ms, searchPeriod 10s, minAquTime 0, OnTime 2s
0xB5,0x62,0x06,0x3B,0x2C,0x00,0x01,0x06,0x00,0x00,0x0E,0x90,0x42,0x01,0x10,0x27,0x00,
0x00,0x10,0x27,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x2C,0x01,0x00,0x00,
0x4F,0xC1,0x03,0x00,0x87,0x02,0x00,0x00,0xFF,0x00,0x00,0x00,0x64,0x40,0x01,0x00,0x32,
0x3A
};  

const char UBLOX_GNSS_RXM[] PROGMEM = { // configure CFG - GNSS to GPS satellites only
0xB5,0x62,0x06,0x11,0x02,0x00,0x08,0x01,0x22,0x92   
};

// configure power save as described in Ublox application note for power saving
void configurePowerSaveAppNote(HardwareSerial& gpsSerial)
{
  sprintf(outstring,"configurePowerSaveAppNote");
  logOut(2,outstring); 
  // send configuration data in UBX protocol: Only GPS active, not Glonass, Beidou, Galileo
  //for(int i = 0; i < sizeof(UBLOX_GNSS_ONLY_GPS); i++) {                        
  //  gpsSerial.write( pgm_read_byte(UBLOX_GNSS_ONLY_GPS + i) );
  //  delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  //}
  smartDelay(50);
  // send configuration data in UBX protocol: configure PM2 to cyclic
  for(int i = 0; i < sizeof(UBLOX_GNSS_PM2_CYCLIC); i++) {                        
    gpsSerial.write( pgm_read_byte(UBLOX_GNSS_PM2_CYCLIC + i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }  
  smartDelay(50);
  // send configuration data in UBX protocol: configure RXM to power save mode
  for(int i = 0; i < sizeof(UBLOX_GNSS_RXM); i++) {                        
    gpsSerial.write( pgm_read_byte(UBLOX_GNSS_RXM + i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }  
}

// Revert to last saved configuration (CFG-CFG)
const char UBLOX_REVERT_SAVED[] PROGMEM = {
0xB5,0x62,0x06,0x09,0x0D,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x01,0x1B,0xB1
};

void revertUbloxToLastSavedConfig(HardwareSerial& gpsSerial)
{
  sprintf(outstring,"revertUbloxToLastSavedConfig");
  logOut(2,outstring); 
  for(int i = 0; i < sizeof(UBLOX_REVERT_SAVED); i++) {                        
    gpsSerial.write( pgm_read_byte(UBLOX_REVERT_SAVED + i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }  
  smartDelay(50);
}

// Revert to default configuration (CFG-CFG)
const char UBLOX_REVERT_DEFAULT[] PROGMEM = {
0xB5,0x62,0x06,0x09,0x0D,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x01,0x19,0x98  
};

void revertUbloxToDefaultConfig(HardwareSerial& gpsSerial)
{
  sprintf(outstring,"revertUbloxToDefaultConfig");
  logOut(2,outstring); 
  for(int i = 0; i < sizeof(UBLOX_REVERT_SAVED); i++) {                        
    gpsSerial.write( pgm_read_byte(UBLOX_REVERT_SAVED + i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }  
  smartDelay(50);
}

/*****************************************************************************! 
  @brief  resetPowerSaveMode() : reset all power save stuff to a neutral status
  @details 
  @return void
*****************************************************************************/
void resetPowerSaveMode(HardwareSerial& gpsSerial)
{
  sprintf(outstring,"resetPowerSaveMode");
  logOut(2,outstring);

  #if defined NEO_6M || defined NEO_M8N
    /*
    logOut(2,(char *)"resetPowerSaveMode: cont. mode, PM2 to fast, GPS HF to ON");
    // UKHAS power save mode -> continuous mode
    configureUKhasCoM();
  
    // PM2
    configurePM2Fast();
  
    // switch on GPS HF electronics
    configureUKhasGPSon(); 
    */
   
    //revertUbloxToLastSavedConfig(); // revert Ublox GPS to last saved configuration
    revertUbloxToDefaultConfig(); // revert Ublox to default configuration
  #endif
}

/*****************************************************************************! 
  @brief  configurePowerSaveMode
  @details 
  @param  bool switchON. true:  switch all power stuff for ESP32 running
  @param                 false: switch power stuff for ESP32 deep sleep
  @return void
*****************************************************************************/
void configurePowerSaveMode(HardwareSerial& gpsSerial, bool switchON)
{
  sprintf(outstring,"configurePowerSaveMode to %d",wData.PowerSaveMode);
  logOut(2,outstring);
  if(switchON){  
    #ifdef NEO_6M
      if((wData.PowerSaveMode == MIN) ||(wData.PowerSaveMode == MID)||(wData.PowerSaveMode == MAX)){
        // remove unnecessary NMEA messages to optimize GPS performance
        smartDelay(50); // give GPS some time to start up
        configureGpsMessages(gpsSerial); // working from gpsTest
        sprintf(wData.actConfigString,"%s2.configGPSMsg\n", wData.actConfigString);
        wData.linesInConfigString++;

        // configure GPS for "pedestrian" mode: good accuracy at slow speeds
        smartDelay(50); // give GPS some time to start up
        configurePedestrian(gpsSerial);
        sprintf(wData.actConfigString,"%sconfigPedestrian\n", wData.actConfigString);
        wData.linesInConfigString++;
      }

      if(wData.PowerSaveMode == MID){
        // set power saving mode
        smartDelay(500); // give GPS some time to start up
        configureUKhasPSM(gpsSerial);
        sprintf(wData.actConfigString,"%sconfigUKhasPSM\n", wData.actConfigString);
        wData.linesInConfigString++;
      } 

      if(wData.PowerSaveMode == MAX){
        configurePM2Fast(gpsSerial);
        sprintf(wData.actConfigString,"%sconfigGPSOn\n", wData.actConfigString);
        wData.linesInConfigString++;
        smartDelay(2000); // give GPS some time to start up
      }  
    #endif

    #ifdef NEO_M8N 
      if((wData.PowerSaveMode == MIN) ||(wData.PowerSaveMode == MID)||(wData.PowerSaveMode == MAX)){
        // remove unnecessary NMEA messages to optimize GPS performance
        smartDelay(50); // give GPS some time to start up
        configureGpsMessages(gpsSerial); // working from gpsTest
        sprintf(wData.actConfigString,"%sconfigGPSMsg\n", wData.actConfigString);
        wData.linesInConfigString++;
        smartDelay(50); // give GPS some time to start up
        configurePedestrian(gpsSerial);
        sprintf(wData.actConfigString,"%sconfigPedestrian\n", wData.actConfigString);
        wData.linesInConfigString++;
      }  

      if(wData.PowerSaveMode == MID){
        // set power saving mode      
        //configurePM2Fast; 
        // sprintf(wData.actConfigString,"%sConfigPM2Fast\n", wData.actConfigString);
        // wData.linesInConfigString++;
        configurePowerSaveAppNote(gpsSerial); //!!! switches off all satellites other than GPS!
        sprintf(wData.actConfigString,"%sConfigAppNote\n", wData.actConfigString);
        wData.linesInConfigString++;
        smartDelay(50); // give GPS some time to start up
      }  

      if(wData.PowerSaveMode == MAX){
        configureUKhasGPSon(gpsSerial); // switch on GPS HF electronics
        sprintf(wData.actConfigString,"%sconfigGPSOn\n", wData.actConfigString);
        wData.linesInConfigString++;
        smartDelay(100); // give GPS some time to start up
        // then check if reception re-estabilshed. alert if not.
        boolean ret = gpsReEstablished(wData.noGpsAlertThreshold);
        if(!ret){
          wData.validGPSLocation = false;  
          wData.alertON = true;
          wData.alertReason += 8;
          strcat(wData.alertReasonString,"Invalid GPS Loc.  ");
        }
        smartDelay(500); // give GPS some time to stabilize data
      }  
    #endif

    logOut(2,(char*)"before MIN / MID / MAX");
    #ifdef ATGM336H // to be used for chinese ATGM336H module
      if((wData.PowerSaveMode == MIN) ||(wData.PowerSaveMode == MID)||(wData.PowerSaveMode == MAX)){
        //NMEA sentences off other than GGA; gll; rmc
        char pcas03[] = "$PCAS03,1,1,0,0,1,0,0,0,0,0,,,0,0*03\r\n";	/* NMEA command, write(fd, cmdbuf, strlen(cmdbuf)); */
        gpsSerial.write(pcas03, strlen(pcas03));
        //turn on GPS, Beidou, Glonass
        char pcas04[] = "$PCAS04,7*1E\r\n";	/* NMEA command, write(fd, cmdbuf, strlen(cmdbuf)); */
        gpsSerial.write(pcas04, strlen(pcas04));
        // dynamic model: pedestrian
        char pcas11[] = "$PCAS11,2*1F\r\n";	/* NMEA command, write(fd, cmdbuf, strlen(cmdbuf)); */
        gpsSerial.write(pcas11, strlen(pcas11));
      }

      if(wData.PowerSaveMode == MID){
      }  
    #endif   
  }
  else{ // switch off 
    #ifdef NEO_M8N
      if(wData.PowerSaveMode == MID){
        configurePM2Slow(gpsSerial);
        delay(50); // wait a bit for gps to adapt to new settings
      }

      if(wData.PowerSaveMode == MAX){
        configureUKhasGPSoff(gpsSerial);
        smartDelay(100); // give GPS some time to process
        }
    #endif 

    #ifdef NEO_6M
       if(wData.PowerSaveMode == MAX){
        configurePM2Slow(gpsSerial);
        delay(50); // wait a bit for gps to adapt to new settings
      }
    #endif
    logOut(2,(char*)"before MID / MAX");

    #ifdef ATGM336H // to be used for chinese ATGM336H module
      if((wData.PowerSaveMode == MID)||(wData.PowerSaveMode == MAX)){
        // receiver standby mode
        //char cmdbuf[] = "$PCAS12,10*2F\r\n";	// for 10 seconds
        //char cmdbuf[] = "$PCAS12,8*16\r\n";	  // for 8 seconds
        char cmdbuf[] = "$PCAS12,18*27\r\n";	// for 18 seconds

        //char cmdbuf[] = "$PCAS12,20*2C\r\n";	// for 20 seconds
        //char cmdbuf[] = "$PCAS12,28*24\r\n";	// for 28 seconds
        gpsSerial.write(cmdbuf, strlen(cmdbuf));
        logOut(2,(char*)"Setting ATGM336H to low power mode for 18 sec");
      }
    #endif 
  }  

  // test checksum function for ATGM336H
  //logOut(2,(char*)"before buildATGM336Message");
  //char msg[30];
  //sprintf(msg,"PCAS12,18");
  //buildATGM336Message(msg);
}

/*****************************************************************************! 
  @brief  startupGPSSafely()
  @details checks if we have a fix, if not tries to re-establish
  @details if that does not work within first time, reduce powerSaveMode to MIN and try again
  @details if that does not work: message and return
  @param uint32_t  secs1: first waiting time for GPS fix until reduction of power save mode
  @param uint32_t  secs2: second waiting time for GPS fix after reduction of power save mode
  @return true if started properly, false if not.
*****************************************************************************/
bool startupGPSSafely(HardwareSerial& gpsSerial, uint32_t  secs1, uint32_t secs2){
    sprintf(outstring,"************ startupGPSSafely: started");
    logOut(2, outstring);  
  if(gpsReEstablished(secs1)){
    sprintf(outstring,"startupGPSSafely: GPS Fix OK");
    logOut(2, outstring);
    //showWelcomeMessage(false,outstring, 1);
    return true;
  }
  else{ // gps fix not re-established within time1 secs
    sprintf(outstring,"-------- startupGPSSafely: NO Fix within %ld seconds. Retry.", secs1);
    logOut(2, outstring);  
    if(wData.PowerSaveMode > MIN){
      wData.PowerSaveMode = MIN;
      // wData.preferencesChanged = true; // ensures that new power save mode is written to preferences
      resetPowerSaveMode(gpsSerial);
      smartDelay(500);
      configurePowerSaveMode(gpsSerial, true);
      if(gpsReEstablished(secs2)){
        sprintf(outstring,"startupGPSSafely: GPS Fix OK after reducing powerSaveMode to MIN");
        logOut(2, outstring);
        //showWelcomeMessage(false,outstring, 1);
        return true;
      }
      else{
        sprintf(outstring,"startupGPSSafely: GPS Fix NOT OK after reducing powerSaveMode to MIN");
        logOut(2, outstring);
        showWelcomeMessage(true,(char*)"GPS Fix NOT OK\nafter powerSave\nset to MIN", 1);
        smartDelay(1000);
        return false;
      }  
    }
    else{
      if(gpsReEstablished(secs2)){
        sprintf(outstring,"startupGPSSafely: GPS Fix already in PowerSaveMode MIN. OK after retry");
        logOut(2, outstring);
        //showWelcomeMessage(false,outstring, 1);
        return true;
      }
      else{
        sprintf(outstring,"startupGPSSafely: GPS Fix already in PowerSaveMode MIN. NOT OK after retry");
        logOut(2, outstring);
        //showWelcomeMessage(false,outstring, 1);
        return false;
      }
    }

  }  
  return true;
}