
/****************************************************************/
// ePaperAnchorAlarm by. M. Preidel
/****************************************************************/

#include <Arduino.h>
#include <math.h>

//**** ESP32 sleep and rtc memory (which survives deep sleep)
#include "esp_sleep.h"
#include "driver/rtc_io.h"

#include <SimpleTimer.h>        // Simple Timer, used instead of Blynk timer for virtuino
SimpleTimer gpsTimer;
int gpsTimerHandle=1;

// ** rotary encoder lib
#include <ESP32Encoder.h>
#define ENCODER_CLK_PIN 22 //25
#define ENCODER_DT_PIN 27
#define ENCODER_SW_PIN 26

// button stuff from ePaperBarograf
#define BUTTON1 GPIO_NUM_26
struct Button {
  const uint8_t PIN;
  uint32_t numberKeyPresses;
  bool pressed;
};
volatile Button button1 = {ENCODER_SW_PIN, 0, false};

void ARDUINO_ISR_ATTR button1Handler();

// include bounce2 library for button debouncing
#include <Bounce2.h>
Bounce button = Bounce();

// GPIO for measurement of voltage
#define VOLTAGE_PIN 39

// TinyGPSPlus library for GPS handling
#include <TinyGPSPlus.h>
// #include <SoftwareSerial.h>

// float volt, percent;                    // battery voltage and percent fill degree

static const int RXPin = 19, TXPin = 15; //22;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;  // The TinyGPSPlus object
HardwareSerial ss(1); // The serial connection to the GPS device

// for tests latitude and longitude of start position
double startLat = 0.0;
double startLon = 0.0;
double anchorDistance_m = 23;
double anchorBearing_deg = 45.0;
double anchorLat = 0.0;
double anchorLon = 0.0;

// ADC calibration stuff
// https://www.reddit.com/r/esp32/comments/1g4ma01/esp32_s3_adc/?rdt=46482
#include <esp_adc_cal.h>

#if CONFIG_IDF_TARGET_ESP32
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_VREF
#elif CONFIG_IDF_TARGET_ESP32S3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP_FIT
#endif
#define VREF 1100  // Reeference voltag. used if no data in EFUSE

// *** lib for permanent data storage in EEPROM
// https://randomnerdtutorials.com/esp32-save-data-permanently-preferences/
// if these defines are set, preferences are written / saved to eeprom. 
// otherwise just from RTC storage (survives deep sleep)
#define WRITE_PREFERENCES
#define READ_PREFERENCES
#include <Preferences.h>
Preferences preferences; // object for preference storage in EEPROM
#define prefIDENT "AnchorAlarm1" // unique identifier for preferences

// header file mit #defines, forward declarations, "extern" declarations von woanders deklarierten globalen var's
// globale variablen und alle anderen #includes sind im .cpp file
#include "ePaperAnchorAlarm.h"
#include "global.h" // global stuff from other modules

//float volt, percent;                    // battery voltage and percent fill degree

char outstring[maxLOG_STRING_LEN];      // for serial and other debug output

//----------------------- measurement data variables -----------------------
uint32_t startTimeMillis; 

ESP32Encoder encoder;  // rotary encoder object

// measurement data, stored in RTC memory which survives the deep sleep. ESP32 has 8 K, used for this 5448 Bytes
RTC_DATA_ATTR measurementData wData;

// screen colors
RTC_DATA_ATTR uint32_t fgndColor;
RTC_DATA_ATTR uint32_t bgndColor;

// determine after how many partial updates a full update of the epaper is to be done
// 1: always full update
#ifdef LOLIN32_LITE
  #define FULL_UPDATE_INTERVAL  10
#endif

// GPIO definition for battery voltage reading
#define VOLTAGE_PIN 39

// GPIO definition for buzzer
#define BUZZER_PIN 2

// store these variables in RTC memory, which survives deep sleep. 
//RTC_DATA_ATTR uint32_t startCounter = 0; // 25920;  // total counter for starts of ESP32
//RTC_DATA_ATTR uint32_t dischgCnt = 0;    // counter for starts of ESP32 since last charge
//RTC_DATA_ATTR uint32_t prevMicrovolt = 0;
//RTC_DATA_ATTR float prevVoltage = 0;

// test control variables
bool testRotaryEncoder = false;
bool testGPSModule = true;
bool testBuzzer = true;
bool testDisplayModule = false;


/**************************************************!
   @brief    logOut()
   @details  Function create log output
   @return   void
***************************************************/
void logOut(int logLevel, char* str)
{
  char safeStr[maxLOG_STRING_LEN+10];
  static uint32_t logCnt = 0;
  // for safety: ensure zero termination well before the end of the string
  // crashes with static strings.
  // str[maxLOG_STRING_LEN-1]=0;
  if(logLevel <= logLEVEL){
    strncat(safeStr, str, maxLOG_STRING_LEN-1); // copy limited number of characters only
    sprintf(safeStr,"%ld %s",logCnt,str);
    //Serial.println(str);
    Serial.println(safeStr);
    logCnt++;
  }
}

/*************************** getBatteryCapacity ************************** */
int readBatteryVoltage(float* percent, float* volt)
{
  uint16_t readval;
  uint32_t millivolts1, millivolts2;

  esp_adc_cal_characteristics_t adc_chars;

  *percent = 100;
  pinMode(VOLTAGE_PIN, INPUT);

  // Prüft, ob Curve Fitting (TP_FIT) eFuse-Daten vorhanden sind
  /*
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP_FIT) == ESP_OK) {
     sprintf(outstring,"eFuse Kalibrierdaten (TP_FIT) vorhanden!");
  } 
  else {
     sprintf(outstring,"Keine eFuse Kalibrierdaten gefunden. Nutze Default-Werte.\n");
  }
  logOut(2,outstring);
  */

  esp_adc_cal_value_t efuse_config = esp_adc_cal_characterize(
      ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, VREF, &adc_chars);
  if (efuse_config != ESP_ADC_CAL_VAL_EFUSE_TP_FIT) {
    //logOut(2, (char*)"ALERT, ADC calibration failed");
    logOut(3, (char*)"Werkseitige ADC Curve-Fitting-Kalibrierung nicht verfügbar, nutze Default VREF.");
  }

  millivolts1 = analogReadMilliVolts(VOLTAGE_PIN);

  uint32_t raw = analogRead(VOLTAGE_PIN);
  millivolts2 = esp_adc_cal_raw_to_voltage(raw, &adc_chars);
  
  sprintf(outstring,"millivolts1: %ld raw: %ld, millivolts2: %ld",
    millivolts1, raw, millivolts2);
  logOut(3,outstring);

  // Original, works well on Lolin32
  readval = analogRead(VOLTAGE_PIN);// / 4096.0 * 7.23;      // LOLIN D32 (no voltage divider need already fitted to board.or NODEMCU ESP32 with 100K+100K voltage divider
  *volt= (float)readval / 4096.0 * 7.23;
  //float voltage = analogRead(39) / 4096.0 * 7.23;    // NODEMCU ESP32 with 100K+100K voltage divider added
  //float voltage = analogRead(A0) / 4096.0 * 4.24;    // Wemos / Lolin D1 Mini 100K series resistor added
  //float voltage = analogRead(A0) / 4096.0 * 5.00;    // Ardunio UNO, no voltage divider required
  
  *percent = 2808.3808 * pow(*volt, 4) - 43560.9157 * pow(*volt, 3) + 252848.5888 * pow(*volt, 2) - 650767.4615 * *volt + 626532.5703;
  
  if(*percent<0) *percent =0; // to ensure that the above formula does not produce negative values, as it does slightly above 3.5V

  if (*volt > 4.19) *percent = 100;
  else if (*volt <= 3.50) *percent = 0;

  //sprintf(outstring,"GPIO: %d  readval: %d Voltage: %3.2f Percent: %3.1f\n",
  //    VOLTAGE_PIN,readval, *volt,*percent);
  //logOut(2,outstring);
  return(true);
}


/**************************************************!
   @brief    handleButtonInterrupt()
   @details  interrupt handling for button pressed on rotary encoder
   @return   void
***************************************************/
volatile bool buttonEvent = false;
volatile unsigned long lastInterruptTime = 0;

void IRAM_ATTR handleButtonInterrupt() {
  unsigned long now = millis();

  // simples Entprellen
  if (now - lastInterruptTime > 400) {
    buttonEvent = true;
    lastInterruptTime = now;
    doParallelBuzzer(1,50,0); // buzzer feedback
  }
}

// possibly better, not yet used 30.1.26
void ARDUINO_ISR_ATTR button1Handler()
{
    if(!button1.pressed)  // avoid repeated pressed to be counted, entprellen
      button1.numberKeyPresses += 1;
    button1.pressed = true;
    //sprintf(outstring, "Button 1 pressed %d\n", button1.numberKeyPresses);
    //logOut(2,outstring);
}

/**************************************************!
   @brief    displayGPSInfo()
   @details  display GPS info, if available
   @return   void
***************************************************/
void displayGPSInfo()
{
  strcpy(outstring,"");

  if (gps.location.isValid())
  {
    double distanceToStart = TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(), 
      anchorLat,
      anchorLon);
    double courseToStart = TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      anchorLat,
      anchorLon);
    
    wData.actLat = gps.location.lat();
    wData.actLon = gps.location.lng();
    sprintf(outstring," GPS Sats: #%ld HDOP: %4.2f Dist2Start: %4.2f m Course2Start: %4.2f° Stored Lat/Lon: %6.4f %6.4f",
        gps.satellites.value(),gps.hdop.hdop(), distanceToStart, courseToStart, wData.actLat , wData.actLon);
    logOut(2, outstring);
  }
  else
  {
    //Serial.print(F("INVALID"));
  }

  // Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    //Serial.print(F("  Date/Time: "));
    //Serial.print(gps.date.day());
    //Serial.print(F("."));
    //Serial.print(gps.date.month());
    //Serial.print(F("."));
    //Serial.print(gps.date.year());
    sprintf(outstring,"%s Date: %02d.%02d.%04d",
        outstring, gps.date.day(),gps.date.month(),gps.date.year());
  }
  else
  {
    //Serial.print(F("INVALID"));
  }

  //Serial.print(F(" "));
  if (gps.time.isValid())
  {
    //Serial.print(F(" "));
    //if (gps.time.hour() < 10) Serial.print(F("0"));
    //Serial.print(gps.time.hour());
    //Serial.print(F(":"));
    //if (gps.time.minute() < 10) Serial.print(F("0"));
    //Serial.print(gps.time.minute());
    //Serial.print(F(":"));
    //if (gps.time.second() < 10) Serial.print(F("0"));
    //Serial.print(gps.time.second());
    //Serial.print(F("."));
    //if (gps.time.centisecond() < 10) Serial.print(F("0"));
    //Serial.print(gps.time.centisecond());
    //Serial.print(F(" "));
    sprintf(outstring,"%s Time: %02d:%02d:%02d.%02d",
        outstring, gps.time.hour(),gps.time.minute(),gps.time.second(),gps.time.centisecond());
  }
  else
  {
    //Serial.print(F("INVALID"));
  }

  logOut(2, outstring);
  //Serial.println();
}

#include <math.h>

#define EARTH_RADIUS 6371000.0  // Meter

/**************************************************!
   @brief    gps_offset()
   @details  Calculate new GPS position given start coords, distance and bearing
   @param    double lat1_deg      : start latitude in degrees
   @param    double lon1_deg      : start longitude in degrees
   @param    double distance_m    : distance to move in meters
   @param    double bearing_deg   : bearing in degrees
   @param    double *lat2_deg     : pointer to store resulting latitude in degrees
   @param    double *lon2_deg     : pointer to store resulting longitude in degrees
   @return   void
***************************************************/
void gps_offset(
    double lat1_deg,
    double lon1_deg,
    double distance_m,
    double bearing_deg,
    double *lat2_deg,
    double *lon2_deg
) {
    // Grad → Radiant
    double lat1 = lat1_deg * M_PI / 180.0;
    double lon1 = lon1_deg * M_PI / 180.0;
    double bearing = bearing_deg * M_PI / 180.0;

    // Winkelstrecke
    double delta = distance_m / EARTH_RADIUS;

    // Neue Breite
    double lat2 = asin(
        sin(lat1) * cos(delta) +
        cos(lat1) * sin(delta) * cos(bearing)
    );

    // Neue Länge
    double lon2 = lon1 + atan2(
        sin(bearing) * sin(delta) * cos(lat1),
        cos(delta) - sin(lat1) * sin(lat2)
    );

    // Radiant → Grad
    *lat2_deg = lat2 * 180.0 / M_PI;
    *lon2_deg = lon2 * 180.0 / M_PI;
}

/**************************************************!
   @brief    getGPSStartPosition()
   @details  get the initial start position from GPS, averaging several readings
   @param    double *startLat : pointer to store start latitude (Breitengrad)  
   @param    double *startLon : pointer to store start longitude (Längengrad)
   @return   void
***************************************************/
void getGPSStartPosition(double *startLat, double *startLon) // get start position from GPS
{
  Serial.println(F("Getting GPS start position..."));
  uint32_t start = millis();
  int cnt = 0, avg = 5;

  *startLat = 0;
  *startLon = 0;

  // wait up to 30 seconds for GPS fix
  while (millis() - start < 30000)
  {
    while (ss.available() > 0)
    {
      char c = ss.read();
      Serial.write(c); // uncomment to see raw GPS data
      gps.encode(c);
      if (gps.location.isUpdated())
      {
        cnt++;
        /*
        Serial.print(F("Fix #"));
        Serial.print(cnt+1);
        Serial.print(F(" Sats: "));
        Serial.print(gps.satellites.value());
        Serial.print(F(" HDOP: "));
        Serial.print(gps.hdop.hdop(), 2);   
        */
        sprintf(outstring,"Fix #%d Sats:%ld  HDOP: %4.2f", 
            cnt+1,gps.satellites.value(),gps.hdop.hdop());
        logOut(2, outstring);  
        
        *startLat += gps.location.lat();
        *startLon += gps.location.lng();
        /*
        Serial.print(cnt, 2);
        Serial.print(F("Start position acquired :"));
        Serial.print(*startLat/cnt, 6);
        Serial.print(F(", "));
        Serial.println(*startLon/cnt, 6);
        */
        sprintf(outstring,"Start position acquired in %d Lat:%6.6f, Lon:%6.6f",
          cnt, *startLat/cnt, *startLon/cnt);
        logOut(2, outstring);    
      }
      if(cnt>=avg){
        *startLat /= cnt;
        *startLon /= cnt;
        /*
        Serial.print(F("Averaged start position: "));
        Serial.print(*startLat, 6);
        Serial.print(F(", "));
        Serial.println(*startLon, 6);
        */
        sprintf(outstring,"Averaged start position Lat:%6.6f, Lon:%6.6f",
          *startLat, *startLon);
        logOut(2, outstring);  
        return;
      }
    }
  }
  Serial.println(F("Failed to acquire GPS start position within 30 seconds."));

}

/**************************************************!
   @brief    smartDelay()
   @details  This custom version of delay() ensures that the gps object is being "fed".
   @details  It should be used instead of delay() when waiting for GPS data.
   @details  however, it utilizes a blocking delay, so no other tasks can be done during this time.
   @param    unsigned long ms : delay time in milliseconds
   @return   void
***************************************************/
static void smartDelay(unsigned long ms)
{
  char ch;
  unsigned long start = millis();
  do 
  {
    while (ss.available()){
      ch = ss.read();
      //Serial.write(ch); // uncomment to see raw GPS data 
      gps.encode(ch);
    }  
  } while (millis() - start < ms);
}

/**************************************************!
   @brief    gpsHandler()
   @details  Function to handle input from gps module, called by timer
***************************************************/
void gpsHandler(){
  char ch;
  while (ss.available()){
    ch = ss.read();
    //Serial.write(ch); // uncomment to see raw GPS data 
    gps.encode(ch);
  }   
}

/**************************************************!
   @brief    gpsTest()
   @details  Function to test gps module
   @param    void 
   @return   true if gps location is valid, else false
***************************************************/
boolean gpsTest()
{
  smartDelay(1000); // wait for 1 second while feeding gps object
  displayGPSInfo();  // get and display GPS info
  return gps.location.isValid();
}

/**************************************************!
   @brief    buzzer()
   @details  Function to create a buzzer sound
   @param    uint16_t number    : how many buzzes
   @param    uint16_t duration : how long in ms for every buzz
   @param    uint16_t interval : how long in ms between buzzes
   @return   void
***************************************************/
void buzzer(uint16_t number, uint16_t duration, uint16_t interval)
{
  //logOut(2, (char*)"Start Buzzer");  
  uint16_t i;
  for(i=0; i<number; i++){
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, HIGH); // buzzer on
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW); // buzzer off
    delay(interval);
  }  
  //logOut(2, (char*)"End of Buzzer");
}    

/**************************************************!
   @brief    parallelBuzzer()
   @details  Function to create a buzzer sound in parallel thread. Self-ending
   @param    uint16_t number    : how many buzzes
   @param    uint16_t duration : how long in ms for every buzz
   @param    uint16_t interval : how long in ms between buzzes
   @return   void
***************************************************/
struct buzzerDataType
{
  uint16_t number; 
  uint16_t duration; 
  uint16_t interval;
};
buzzerDataType buzzerParam;

void parallelBuzzer(void *param)
{
  uint16_t i;
  buzzerDataType *buzzerParam ;
  for(i=0; i<((buzzerDataType*)param)->number; i++){
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, HIGH); // buzzer on
    delay(((buzzerDataType*)param)->duration);
    digitalWrite(BUZZER_PIN, LOW); // buzzer off
    delay(((buzzerDataType*)param)->interval);
  }  

  vTaskDelete(NULL);
}    

void doParallelBuzzer(uint16_t number, uint16_t duration, uint16_t interval)
{
  buzzerParam.number   = number;
  buzzerParam.duration = duration;
  buzzerParam.interval = interval;

  xTaskCreatePinnedToCore(
    parallelBuzzer,   // function
    "parallelBuzzer", // name
    2048,             // stack size
    (void *)&buzzerParam,           // parameter
    1,                // priority
    NULL,             // task handle (not needed)
    0                 // core (0 or 1) (do not use 0 if Wifi or BLE active, core 0 is busy)
  );
}


/**************************************************!
   @brief    testRotary()
   @details  Function to test rotary encoder
   @param    boolean *rotButtonPressed : state of rotary encoder button (presently disabled!)
   @param    int64_t *rotPosition : position of rotary encoder
   @return   void
***************************************************/
void testRotary(boolean *rotButtonPressed, int64_t *rotPosition)
{
  logOut(2, (char*)"testRotary: start");
  static long degrees;
  //static bool lastButtonState = HIGH;
  //static bool buttonPressed = false;

  // read encoder switch
  /*
  bool reading =  digitalRead(ENCODER_SW_PIN);
  if (reading == LOW && lastButtonState == HIGH) {
    // button pressed
    buttonPressed = !buttonPressed; // toggle state
    //if(buttonPressed){
    //  LogOut(2, "Rotary encoder button pressed: ON");
    //} else {
    //  LogOut(2, "Rotary encoder button pressed: OFF");
    //}
  }
  */
  button.update();
  if(button.fell()){
    *rotButtonPressed = true; // set button state
  }
  if(button.rose()){
    *rotButtonPressed = false; // reset button state
  }
  // read  encoder position
  *rotPosition = - encoder.getCount(); // +3600; // offset to avoid negative values
  degrees = *rotPosition * 5 % 360 ;
  sprintf(outstring, "Rotary encoder position: %lld , degrees: %ld Button: %s", *rotPosition , degrees, *rotButtonPressed ? "ON" : "OFF");
  logOut(2, outstring);

  //logOut(2, "testRotary: end");
}

/*****************************************************************************! 
  @brief  read all preferences values
  @details remember: short names (max 15 char), otherwise not stored
  @return void
*****************************************************************************/
void readPreferences()
{
    preferences.begin(prefIDENT, true);
    //----- counters etc.
    //int bytes=preferences.getBytes("prefs", prefs, sizeof(prefs));
    //----- verbosity
    if(preferences.isKey("verbosity"))
      wData.verbosity = (verbosityType)preferences.getULong("applyPCorr", d_verbosity);
    else {  // set default    
      wData.verbosity = d_verbosity;
      preferences.putULong("verbosity", wData.verbosity);
    }  
    //----- Graph weight
    if(preferences.isKey("graphWeight"))
      wData.graphWeight = (graphWeightType)preferences.getULong("graphWeight", d_graphWeight);
    else {  // set default    
      wData.graphWeight = d_graphWeight;
      preferences.putULong("graphWeight", wData.graphWeight);
    }  
    //----- Measurement interval in seconds
    if(preferences.isKey("mIntervSec"))   // sleep time target in seconds, controls the measurement
      wData.targetMeasurementIntervalSec = preferences.getLong("mIntervSec", d_targetMeasurementIntervalSec);
    else {  // set default    
      wData.targetMeasurementIntervalSec = d_targetMeasurementIntervalSec;
      preferences.putLong("mIntervSec", wData.targetMeasurementIntervalSec);
    }  
    //----- alarmDistanceM: distance from anchor to increase alarm counter
    if(preferences.isKey("alarmDistanceM"))   // alarm distance in meters
      wData.alarmDistanceM = preferences.getFloat("alarmDistanceM", d_alarmDistanceM);
    else {  // set default    
      wData.alarmDistanceM = d_alarmDistanceM;
      preferences.putFloat("alarmDistanceM", wData.alarmDistanceM);
    }  
    //----- alarmThreshold: if alarm counter is bigger than this threshold: alarm
    if(preferences.isKey("alarmThreshold"))   // alarm threshold
      wData.alarmThreshold = preferences.getLong("alarmThreshold", d_alarmThreshold);
    else {  // set default    
      wData.alarmThreshold = d_alarmThreshold;
      preferences.putLong("alarmThreshold", wData.alarmThreshold);
    }

    //----- anchorBearingDeg: direction to anchor
    if(preferences.isKey("anchorBearingDeg"))   // anchorBearingDeg
      wData.anchorBearingDeg = preferences.getFloat("anchorBearingDeg", d_anchorBearingDeg);
    else {  // set default    
      wData.anchorBearingDeg = d_anchorBearingDeg;
      preferences.putFloat("anchorBearingDeg", wData.anchorBearingDeg);
    }

    //----- anchorDistanceM: distance to anchor
    if(preferences.isKey("anchorDistanceM"))   // anchorDistanceM
      wData.anchorDistanceM = preferences.getFloat("anchorDistanceM", d_anchorDistanceM);
    else {  // set default    
      wData.anchorDistanceM = d_anchorDistanceM;
      preferences.putFloat("anchorDistanceM", wData.anchorDistanceM);
    }

    //int bytes2= preferences.getBytes("teststring2", teststring2, 80); // test
    //preferences.remove("teststring1"); // remove single key
    //preferences.clear();  // clear the namespace completely
    preferences.end(); // close the namespace

    sprintf(outstring,"Read Preferences: verbosity: %ld graphWeight: %ld Meas.IntervalSec %ld alarmDistance %3.1f alarmThreshold %ld",
            wData.verbosity, wData.graphWeight, wData.targetMeasurementIntervalSec, wData.alarmDistanceM, wData.alarmThreshold);
    logOut(2,outstring); 
    sprintf(outstring,"Read Preferences: anchorBearingDeg: %3.1f anchorDistanceM: %3.1f ",
            wData.anchorBearingDeg, wData.anchorDistanceM);
    logOut(2,outstring);         
}    

/*****************************************************************************! 
  @brief  write all preference values
  @details 
  @return void
*****************************************************************************/
void writePreferences()
{
    size_t ret1, ret11, ret12, ret13, ret2, ret3, ret4, ret5, ret6, ret7;

    ret1=preferences.begin(prefIDENT, false);
    if(!ret1){
      sprintf(outstring,"could open preferences for writing, ret: %d", ret1);
      logOut(2,outstring);
    }  
    
    // verbosity and graph weight
    ret11 =   preferences.putULong("verbosity", wData.verbosity);
    ret12 =   preferences.putULong("graphWeight", wData.graphWeight);

    // measurement interval in seconds
    ret13 =   preferences.putLong("mIntervSec", wData.targetMeasurementIntervalSec);

    //----- alarm distance and alarm threshold
    ret2 = preferences.putFloat("alarmDistanceM", wData.alarmDistanceM);
    ret3 = preferences.putLong("alarmThreshold", wData.alarmThreshold);

    //----- distance and bearing to anchor which are set initially
    ret4 = preferences.putFloat("anchorBearingDeg", wData.anchorBearingDeg);
    ret5 = preferences.putFloat("anchorDistanceM", wData.anchorDistanceM);

    //int bytes2= preferences.getBytes("teststring2", teststring2, 80); // test
    //preferences.remove("teststring1"); // remove single key
    //preferences.clear();  // clear the namespace completely
    preferences.end(); // close the namespace

    sprintf(outstring,"Wrote Preferences: verbosity: %ld graphWeight: %ld Meas.IntervalSec %ld alarmDistance %3.1f alarmThreshold %ld",
            wData.verbosity, wData.graphWeight, wData.targetMeasurementIntervalSec, wData.alarmDistanceM, wData.alarmThreshold);
    logOut(2,outstring); 
    sprintf(outstring,"Read Preferences: anchorBearingDeg: %3.1f anchorDistanceM: %3.1f ",
            wData.anchorBearingDeg, wData.anchorDistanceM);
    logOut(2,outstring);       
    sprintf(outstring,"Wrote Preferences: ret values: %d %d %d %d %d %d %d %d \n", 
                ret1, ret11, ret12, ret13, ret2, ret3, ret4, ret5);
    logOut(2,outstring); 
}


/*****************************************************************************! 
  @brief  writeCounterPreferences()
  @details writes the counter related preference data
  @return void
*****************************************************************************/
/*
void writeCounterPreferences()
{
  int ret11, ret12, ret13;
  bool writing_counter_prefs = false;
  writing_counter_prefs = true;
  sprintf(outstring,"writeCounterPreferences: startCounter %ld dischgCnt %ld prevVoltage %f prevMicrovolt %ld\n", 
     startCounter, dischgCnt, prevVoltage, prevMicrovolt),
  logOut(2,outstring);
  // open preferences namespace in rw mode mode and write preferences infos.
  preferences.begin(prefIDENT, false);

  // counters and previous microvolts
  ret11 =   preferences.putULong("startCounter", startCounter);  
  ret12 =   preferences.putULong("dischargeCnt", dischgCnt);
  ret13 =   preferences.putULong("prevMicrovolt", prevMicrovolt);
  preferences.end(); // close the namespace
  sprintf(outstring,"Write Counter preferences startCounter: %ld %d dischgCnt: %ld %d prevMicrovolt: %ld %d\n", 
    startCounter, ret11, dischgCnt, ret12, prevMicrovolt, ret13);
  logOut(2,outstring);        
}
*/

/*****************************************************************************! 
  @brief  setup routine
  @details 
  @return void
*****************************************************************************/
void setup()
{
  startTimeMillis = millis(); // remember time when woken up
  Serial.begin(115200);   // set speed for serial monitor

  logOut(2,(char*)"**********************************************************");
  sprintf(outstring,"* %s %s - %s ",PROGNAME, VERSION, BUILD_DATE);
  logOut(2,outstring);
  logOut(2,(char*)"**********************************************************");

  #ifdef READ_PREFERENCES
    // get data from EEPROM using preferences library in readonly mode
    sprintf(outstring," before readPreferences()");
    logOut(2,outstring);
    readPreferences();
  #endif

  // increment start counter
  wData.startCounter++;

   // initialize rotary encoder
   ESP32Encoder::useInternalWeakPullResistors = puType::up;
   encoder.attachHalfQuad(ENCODER_CLK_PIN, ENCODER_DT_PIN);
   encoder.setCount(0);

   /*
   // initialize encoder switch at pin ENCODER_SW_PIN
   pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
   // prepare de-bounced button
   button.attach(ENCODER_SW_PIN);
   button.interval(5);
  */
  // button via interrupt
  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
  attachInterrupt(
    digitalPinToInterrupt(ENCODER_SW_PIN),
    handleButtonInterrupt,
    FALLING
  );

  // initialize the display
  logOut(2,(char*)"before initDisplay()");
  initDisplay(wData.startCounter, FULL_UPDATE_INTERVAL); 

  // Serial port for GPS module
  ss.begin(9600, SERIAL_8N1, RXPin, TXPin); // RX, TX
  gpsTimerHandle = gpsTimer.setInterval(200L, gpsHandler); // set gps test timer to 100 milliseconds

  if(wData.currentMode == MODE_STARTED){ // tests only with fresh start, to prevent this stuff when waking up after sleep
    // short display test
    if(testDisplayModule)
      testDisplay();  

    // show welcome message on ePaper
    char show[30];
    sprintf(show,"AnchorAlarm %s", VERSION);
    showWelcomeMessage(true, show);
    showWelcomeMessage(false, (char*)"Buzzer Test");

    if(testBuzzer)        // buzzer test
      //buzzer(1,100,500); 
      doParallelBuzzer(1,100,500);
    if(testDisplayModule) // display test, original from WeAct (manufacturer of the ePaper display)
      testDisplayWeAct();
    if(testBuzzer){       // buzzer test
      logOut(2,(char*)"Testing buzzer");
      for(int i=0; i<5; i++)
      {
         //buzzer(1,100,500-i*75); // buzzer test
         doParallelBuzzer(1,100,500);
         delay(1000); // needed for parallel buzzer
      } 
    }  


    boolean buttonPressed = false; 
    int64_t encoderPos = 0;
    if(testRotaryEncoder){
      showWelcomeMessage(false, (char*)"Encoder Test");
      for(int i=0;i<30;i++) {
        testRotary(&buttonPressed, &encoderPos );
        delay(500);
      }  
    }  

    if(testGPSModule){
      showWelcomeMessage(false, (char*)"GPS Test");
      logOut(2,(char*)"GPS serial started.");
      getGPSStartPosition(&startLat, &startLon); // get start position from GPS

      gps_offset(
        startLat,
        startLon,
        anchorDistance_m,
        anchorBearing_deg,
        &anchorLat,
        &anchorLon
      );
      Serial.print(F("Calculated anchor position: "));
      Serial.print(anchorLat, 6); 
      Serial.print(F(", "));
      Serial.println(anchorLon, 6);
    }
  }  // if just started

} // setup

/*****************************************************************************! 
  @brief  handleExt0Wakeup()
  @details handle wakeup by EXT0 wakeup source = button via GPIO 
  @return void
*****************************************************************************/
void handleExt0Wakeup()
{
    // if woken up by button remember this, may be alert acknowledge
    wData.buttonPressed = true;

    // if in alert mode
    if(!wData.alertON){
      // do something tbd
    } // !alertON
}

/*****************************************************************************! 
  @brief    print_wakeup_reason
  @details  determine wakeup reason
  @param  void
  @return wakeup reason, or 0 if not just woken up
*****************************************************************************/
// https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/
uint32_t print_wakeup_reason() 
{
  esp_sleep_wakeup_cause_t wakeup_reason;
  static int calls = 0;

  if(calls++ == 0){
    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason) {
      case ESP_SLEEP_WAKEUP_EXT0:     logOut(2,(char*)"Wakeup caused by external signal using RTC_IO"); break;
      case ESP_SLEEP_WAKEUP_EXT1:     logOut(2,(char*)"Wakeup caused by external signal using RTC_CNTL"); break;
      case ESP_SLEEP_WAKEUP_TIMER:    logOut(2,(char*)"Wakeup caused by timer"); break;
      case ESP_SLEEP_WAKEUP_TOUCHPAD: logOut(2,(char*)"Wakeup caused by touchpad"); break;
      case ESP_SLEEP_WAKEUP_ULP:      logOut(2,(char*)"Wakeup caused by ULP program"); break;
      default:                        sprintf(outstring,"Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
                                      logOut(2, outstring);
    }
    return(wakeup_reason);
  }
  return(0);
}

/*****************************************************************************! 
  @brief    gotoDeepSleep: routine to enter deep sleep
  @details  wakeup possible by GPIO specified. 
  @param  deepSleepTime  time to to into deep sleep
  @return void
*****************************************************************************/
void gotoDeepSleep(gpio_num_t button, uint64_t deepSleepTime)
{
  //**********  TEST override
  // sleeptime = 60 * SECONDS - 1000*(millis()-startTimeMillis);
  uint32_t am = millis();
  sprintf(outstring,"Hibernating for target %ld sec: %lld usec ", 
          wData.targetMeasurementIntervalSec, deepSleepTime);  
  logOut(2,outstring);
  if(deepSleepTime > maxSleeptimeSafetyLimit){
    sprintf(outstring,"Hibernating time %lld usec above safety limit. Reducing to %ld usec    ", 
          deepSleepTime, maxSleeptimeSafetyLimit);  
    logOut(2,outstring);
  }

  // shut down display
  endDisplay(1); // mode 0: power off, mode 1: hibernate
  //1: display.hibernate();  // danach wird beim wieder aufwachen kein Reset des Screens gemacht.
  //0: display.powerOff(); // danach wird beim wieder aufwachen ein voller Reset des Screens gemacht

  // Initiate sleep
  esp_sleep_enable_timer_wakeup(deepSleepTime);   // define sleeptime for timer wakeup
  // and via external pin. only one seems possible.
  // https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/
  #ifdef LOLIN32_LITE
    //esp_sleep_enable_ext0_wakeup(button, HIGH); // enable wakeup via button1 HIGH
    esp_sleep_enable_ext0_wakeup(button, LOW); // enable wakeup via button1 LOW
  #endif  
  #ifdef CROW_PANEL
    esp_sleep_enable_ext0_wakeup(button, LOW); // enable wakeup via button1
  #endif  
  //rtc_gpio_pullup_dis(button);  //Configure pullup/downs via RTCIO to LOW during deepsleep
  //rtc_gpio_pulldown_en(button); // EXT0 resides in the same power domain (RTC_PERIPH) as the RTC IO pullup/downs.
  rtc_gpio_pullup_en(button);  //Configure pullup/downs via RTCIO to HIGH during deepsleep
  rtc_gpio_pulldown_dis(button); // EXT0 resides in the same power domain (RTC_PERIPH) as the RTC IO pullup/downs.
    
  esp_deep_sleep_start();                               // go to sleep
}

/*****************************************************************************! 
  @brief  handleParamChange - Main worker routine to handle single parameter changes
  @details This routine handles the adaptation of a single parameter via rotary encode
  @param  int trianglePos : position of triangle indicating which parameter to change. 
  @param  0: anchor bearing; 1: anchor distance, 2: alarm coount; 3:alarm distance
  @return boolean : true if parameter change ongoing, false if exit parameter change mode

*****************************************************************************/
boolean handleParamChange(int trianglePos)
{
  switch(trianglePos){
    case 0: // anchor bearing
      {
        int64_t encoderPos = encoder.getCount();
        if(encoderPos != 0){
          wData.anchorBearingDeg += -encoderPos * 5; // each step is 5 degrees
          if(wData.anchorBearingDeg < 0)
            wData.anchorBearingDeg += 360;
          wData.anchorBearingDeg = ((int)wData.anchorBearingDeg) % 360; // wrap around
          wData.preferencesChanged = true; // flag to indicate that a preference value has been changed
          wData.graphBufferClearingNeeded = true; //change that makes clearing of graph buffer necessary
          sprintf(outstring,"handleParamChange: Anchor bearing changed to %3.0f degrees", wData.anchorBearingDeg);
          logOut(2, outstring);
          encoder.setCount(0); // reset encoder count
          drawInputData(); // redraw input data
        }  // encoder
      }
      break;
    case 1: // anchor distance
      {
        int64_t encoderPos = encoder.getCount();
        if(encoderPos != 0){
          wData.anchorDistanceM += -encoderPos; // each step is 1 meter
          if(wData.anchorDistanceM < 5)
            wData.anchorDistanceM = 5; // minimum 5 meters
          wData.preferencesChanged = true; // flag to indicate that a preference value has been changed  
          wData.graphBufferClearingNeeded = true; //change that makes clearing of graph buffer necessary
          sprintf(outstring,"handleParamChange: Anchor distance changed to %3.0f meters", wData.anchorDistanceM);
          logOut(2, outstring);
          encoder.setCount(0); // reset encoder count
          drawInputData(); // redraw input data
        }  // encoder
      }
      break;
    case 2: // alarm count
      {
        int64_t encoderPos = encoder.getCount();
        if(encoderPos != 0){
          wData.alarmThreshold += -encoderPos; // each step is 1
          if(wData.alarmThreshold < 1)
            wData.alarmThreshold = 1; // minimum 1
          wData.preferencesChanged = true; // flag to indicate that a preference value has been changed  
          sprintf(outstring,"handleParamChange: Alarm count changed to %ld", wData.alarmThreshold);
          logOut(2, outstring);
          encoder.setCount(0); // reset encoder count
          drawInputData(); // redraw input data
        }  // encoder
      } 
      break;
    case 3: // alarm distance
      {
        int64_t encoderPos = -encoder.getCount();
        if(encoderPos != 0){
          wData.alarmDistanceM += encoderPos; // each step is 1 meter
          if(wData.alarmDistanceM < 1)
            wData.alarmDistanceM = 1; // minimum 1 meter
          wData.preferencesChanged = true; // flag to indicate that a preference value has been changed  
          wData.graphBufferClearingNeeded = true; //change that makes clearing of graph buffer necessary
          sprintf(outstring,"handleParamChange: Alarm distance changed to %3.0f meters", wData.alarmDistanceM);
          logOut(2, outstring);
          encoder.setCount(0); // reset encoder count
          drawInputData(); // redraw input data
        }  // encoder
      } 
      break;   
    case 4: // sleep time
      {
        int64_t encoderPos = -encoder.getCount();
        if(encoderPos != 0){
          wData.targetMeasurementIntervalSec += 5*encoderPos; // each step is 5 seconds
          if(wData.targetMeasurementIntervalSec < 1)
            wData.targetMeasurementIntervalSec = 1; // minimum 1 second
          wData.preferencesChanged = true; // flag to indicate that a preference value has been changed  
          sprintf(outstring,"handleParamChange: targetMeasurementIntervalSec to %ld [s]", wData.targetMeasurementIntervalSec);
          logOut(2, outstring);
          encoder.setCount(0); // reset encoder count
          drawInputData(); // redraw input data
        }  // encoder
      }
      break; 
    case 5: // detail verbosity
      {
        int64_t encoderPos = -encoder.getCount();
        if(encoderPos != 0){
          // Drehwert-basierte Abfrage -  nich so gut für ENUM-Werte, springt meist einen zu weit
          //wData.verbosity = (enum verbosityType)(wData.verbosity + (verbosityType)encoderPos); // each step is 1
          //if(wData.verbosity > HI) 
          //  wData.verbosity = LO; // handle runover, valid are 0..2
          //if(wData.verbosity < LO) 
          //  wData.verbosity = HI; // handle runover, valid are 0..2  

          // einfach nur auf +/- dämpft das besser
          if(encoderPos > 0)
            wData.verbosity = (enum verbosityType)(wData.verbosity + 1); 
          else if(encoderPos < 0)
            wData.verbosity = (enum verbosityType)(wData.verbosity - 1); 
          if(wData.verbosity > HI) 
            wData.verbosity = LO; // handle runover, valid are 0..2
          if(wData.verbosity < LO) 
            wData.verbosity = HI; // handle runover, valid are 0..2        

          wData.preferencesChanged = true; // flag to indicate that a preference value has been changed  
          sprintf(outstring,"handleParamChange: verbosity to %ld [s]", wData.verbosity);
          logOut(2, outstring);
          encoder.setCount(0); // reset encoder count
          drawInputData(); // redraw input data
        }  // encoder
      }
      break; 
    case 6: // graph weight  
      {
        int64_t encoderPos = -encoder.getCount();
        if(encoderPos != 0){
          // Drehwert-basierte Abfrage -  nich so gut für ENUM-Werte, springt meist einen zu weit
          //wData.graphWeight = (enum graphWeightType)(wData.graphWeight + (graphWeightType)encoderPos); // each step is 1 
          //if(wData.graphWeight > HEAVY) 
          //  wData.graphWeight = MINIM; // handle runover, valid are 0..2
          //if(wData.graphWeight < MINIM) 
          //  wData.graphWeight = HEAVY; // handle runover, valid are 0..2  

          // einfach nur auf +/- dämpft das besser
          if(encoderPos > 0)
            wData.graphWeight = (enum graphWeightType)(wData.graphWeight + 1); 
          else if(encoderPos < 0)
            wData.graphWeight = (enum graphWeightType)(wData.graphWeight - 1); 
          if(wData.graphWeight > HEAVY) 
            wData.graphWeight = MINIM; // handle runover, valid are 0..2
          if(wData.graphWeight < MINIM) 
            wData.graphWeight = HEAVY; // handle runover, valid are 0..2                


          wData.preferencesChanged = true; // flag to indicate that a preference value has been changed  
          sprintf(outstring,"handleParamChange: graphWeight to %ld [s]", wData.graphWeight);
          logOut(2, outstring);
          encoder.setCount(0); // reset encoder count
          drawInputData(); // redraw input data
        }  // encoder
      }
      break; 
    case 7: // exit parameter change mode
      {
        logOut(2, (char*)"handleParamChange: !!!!!!!!! EXIT !!!!!!!!!");
        return false;
      }
      break;   
    default:
      sprintf(outstring,"handleParamChange: default %d in case statement",trianglePos);
      logOut(2, outstring);
  } 
  return true;
}

/*****************************************************************************! 
  @brief  doWork - Routine main worker routine
  @details 
  @return void
*****************************************************************************/

// helper function to clear drawing buffer for historic positions
// which remain only valid as long as anchor position, alarm radius etc are not changed
void clearGraphBuffer(int callerID)
{
  sprintf(outstring,"clearing data buffer. Caller: %d", callerID);
  logOut(2,outstring);
  wData.drawCount = 0;
  wData.graphBufferClearingNeeded = false;
  for (int i=0; i<maxDrawBufferLen; i++){
    wData.drawBuffer[i][0] = 0;
    wData.drawBuffer[i][1] = 0;
    }
}

// helper function to populate the wData structure
void updatewData() 
{
  wData.actLat = gps.location.lat();
  wData.actLon = gps.location.lng();
  wData.actHDOP= gps.hdop.hdop();
  wData.SatCnt = gps.satellites.value();

  // calculate distance between actual position and anchor position
  wData.actAnchorDistanceM = TinyGPSPlus::distanceBetween(wData.actLat,wData.actLon, wData.anchorLat, wData.anchorLon);
  // calculate bearing from first to second position. Here: from anchor to actual position of boat
  wData.actAnchorBearingDeg = TinyGPSPlus::courseTo(wData.anchorLat, wData.anchorLon, wData.actLat,wData.actLon);

  // first check for distance from anchor
  // calc alert counter, based on deviation
  // 10% over adds 1, 30% over adds 3, 100% over adds 10
  if(wData.actAnchorDistanceM > wData.alarmDistanceM)
     wData.alertCount += 10.0* (wData.actAnchorDistanceM / wData.alarmDistanceM -1.0);
  else
    wData.alertCount = 0.0;

  wData.alertON = false;  // start out with no alert
  strcpy(wData.alertReasonString,"None");
  wData.alertReason = 0;

  if(wData.alertCount >= wData.alarmThreshold){       // alarm if alert counter above threshold
    wData.alertON = true;
    wData.alertReason += 1;
    strcpy(wData.alertReasonString, "Anchor Distance");
  }  

  if(wData.batteryVoltage < BAT_VOLTAGE_THRESHOLD){   // or alarm if battery voltage too low 
    wData.alertON = true;
    wData.alertReason += 2;
    strcpy(wData.alertReasonString, "Battery Voltage Low");
  }  

  if(wData.batteryPercent < BAT_PERCENT_THRESHOLD){ 
    wData.alertReason += 4;
    wData.alertON = true;
    strcpy(wData.alertReasonString, "Battery Percent Low");
  }

  if(!gps.location.isValid()){                        // or alarm if no GPS reception   
    wData.validGPSLocation = false;  
    wData.alertON = true;
    wData.alertReason += 8;
    strcpy(wData.alertReasonString, "Invalid GPS Location");
  }    
}

void doRoutineWork()
{
  int cnt = 0;
  uint32_t ret;
  boolean gpsValid = false, gpsUpdated = false;
  static int trianglePos = 0; // position of triangle indicator, serves as line selector
  static boolean handleParamchanges = false; // if true, handle parameter changes per line
  uint32_t targetSleepUSec;               // target sleep time in microseconds
  static uint32_t menuStartTimer = 0;     // millis to remember time when menu has been started

  // determine reason for wakeup. if timer wakeup: continue with measurements. 
  // if EXT0 wakeup (button pressed): handleExt0Wakeup()
  ret = print_wakeup_reason(); // determine reason for wakeup
  if(ret == ESP_SLEEP_WAKEUP_EXT0)
    handleExt0Wakeup();

  readBatteryVoltage(&wData.batteryPercent, &wData.batteryVoltage);                  // Auslesen der Batteriespannung  
  //wData.batteryPercent = volt;
  //wData.batteryPercent = percent;

  sprintf(outstring,"doRoutineWork in wData.currentMode:  %d", wData.currentMode);
  logOut(2, outstring);
  delay(100);

  switch(wData.currentMode){
    case MODE_STARTED:    // 0 just switched on
      { 
        // includes clear screen & setting of font/rotation. 
        // Otherwise following messages will show small and rotated within previous screen
        sprintf(outstring,"AnchorAlarm %s", VERSION);
        showWelcomeMessage(true, outstring);
        
        sprintf(outstring,"Wait for GPS Fix");
        showWelcomeMessage(false, outstring);
        do{
          sprintf(outstring,"doRoutineWork: MODE_STARTED, waiting for GPS fix... %d", cnt);
          logOut(2, outstring);
          gpsValid = gpsTest();
        } 
        while(!gpsValid && cnt++<120); // wait up to 120 seconds for gps fix
        if(gpsValid){
          sprintf(outstring,"Valid GPS Fix");
          showWelcomeMessage(false, outstring);
        }
        else{
          sprintf(outstring,"NO GPS Fix");
          showWelcomeMessage(false, outstring);
        }  
        //smartDelay(1000);
        setScreenParameters();
        drawInputMainScreen();
        drawInputHeader();
        drawInputData();
        if(gpsValid){
          wData.currentMode = MODE_SELECTLINE; // to to input mode if gps is valid
          menuStartTimer = millis();            // remember timer when gone to change parameter mode
        }  
      }
      break;
    case MODE_SELECTLINE:    // 1 input mode to set position of triangle
      {
        int64_t encoderPos;
        sprintf(outstring,"doRoutineWork: MODE_SELECTLINE ... %d", cnt);
        logOut(2, outstring);
        //gpsValid = gpsTest();
        gpsValid=gps.location.isValid();
        gpsUpdated=gps.location.isUpdated();
        if(gpsValid && gpsUpdated){
          wData.actLat = gps.location.lat();
          wData.actLon = gps.location.lng();
          if(!wData.buttonPressed) // quicker if button pressed
            drawInputHeader();
        }
        
        if(!wData.buttonPressed){ // quicker if button pressed
          drawTriangle(true,INPUTDATA_X_POS, trianglePos * INPUTDATA_FONT_SIZE + INPUTDATA_Y_POS); // draw triangle pointing to input data box
          //delay(200); // wait 0,2 seconds
          drawTriangle(false,INPUTDATA_X_POS, trianglePos * INPUTDATA_FONT_SIZE + INPUTDATA_Y_POS); // draw triangle pointing to input data box
        
          encoderPos = encoder.getCount();
          if(encoderPos != 0){
            if(encoderPos > 0)
              trianglePos --; // decrease
            else if(encoderPos < 0)
              trianglePos ++; // increase
            if(trianglePos < 0) // wrap around
              trianglePos = NUM_INPUTDATA_LINES-1;  // max of 8 positions 0..7
            trianglePos = (trianglePos) % NUM_INPUTDATA_LINES; // 8 positions
            sprintf(outstring,"doRoutineWork: Encoder triggered. encoderPos: %ld New trianglePos: %d", (int32_t)encoderPos, trianglePos);
            logOut(2, outstring);
            encoder.setCount(0); // reset encoder count
          }  // encoder
        } // button

        if(wData.buttonPressed){ // button state is checked in main loop
          // wData.buttonPressed = false; // reset button pressed state. not done here, since we want to use it in next state
          sprintf(outstring,"doRoutineWork: MODE_SELECTLINE: Button pressed. Selected trianglePos: %d", trianglePos);
          logOut(2, outstring);
          wData.currentMode = MODE_CHANGEPARAM; // go to change param mode
          menuStartTimer = millis();            // remember timer when gone to change parameter mode
        }
        // safeguard against accidental keypress: if too long in menu, go to routine mode and do measurements
        if((millis() - menuStartTimer) > MAX_MILLIS_IN_MENU){
          wData.currentMode = MODE_RUNNING; // go to measurement mode
          // get anchor position, project it and clear graph buffer if not yet available
          //if(wData.actLat < 0.01 && wData.actLon < 0.01){
          if(wData.anchorLat < 0.01 && wData.anchorLon < 0.01){  
            getGPSStartPosition(&wData.actLat, &wData.actLon); // get averaged boat start position from GPS
            sprintf(outstring,"doRoutineWork: MODE_CHANGEPARAM: Exiting via timer. Current GPS position Lat: %6.4f Lon: %6.4f", 
                wData.actLat, wData.actLon);  
            logOut(2, outstring);
            // project anchor distance and bearing from start position to get anchor position
            gps_offset(wData.actLat, wData.actLon, wData.anchorDistanceM, wData.anchorBearingDeg, &wData.anchorLat, &wData.anchorLon);
            sprintf(outstring,"doRoutineWork: MODE_CHANGEPARAM: Exiting via timer. Anchor position set to Lat: %6.4f Lon: %6.4f", 
                wData.anchorLat, wData.anchorLon);
            logOut(2, outstring);
            //initialize draw buffer
            if(wData.graphBufferClearingNeeded)
              clearGraphBuffer(2);
            updatewData(); // populate wData structure for first run, then no update of gps data expected
          }
          else{
           sprintf(outstring,"leaving menu via timer, re-use existing anchor position %6.6f %6.6f", 
              wData.anchorLat, wData.anchorLon);
            logOut(2, outstring);    
          }
        }
      }       
      break;
    case MODE_CHANGEPARAM:    // 2 input mode to set a specific parameter, defined by last triangePos
      {
        if(!handleParamchanges) // do not draw triangle if already in param change mode, takes too long
        {
          sprintf(outstring,"doRoutineWork: MODE_CHANGEPARAM. wData.Button: %d handleParamchanges: %d", 
              wData.buttonPressed, handleParamchanges);
          logOut(2, outstring);
          drawTriangle(true,INPUTDATA_X_POS, trianglePos * INPUTDATA_FONT_SIZE + INPUTDATA_Y_POS); // draw triangle
        }
        // if button pressed again, and not yet in param change mode, go to param change mode
        if(wData.buttonPressed && !handleParamchanges){
          handleParamchanges = true;
          wData.buttonPressed = false; // reset button pressed state  
          drawTriangle(true,INPUTDATA_X_POS, trianglePos * INPUTDATA_FONT_SIZE + INPUTDATA_Y_POS); // draw triangle once
        }  
        // if button pressed again, and already in param change mode, go back to select line mode
        if(wData.buttonPressed && handleParamchanges){
          handleParamchanges = false;
          wData.buttonPressed = false; // reset button pressed state  
          wData.currentMode = MODE_SELECTLINE; // go to change param mode
          menuStartTimer = millis();            // remember timer when gone to change parameter mode
        } 

        if(handleParamchanges){
          boolean ret = handleParamChange(trianglePos); // handle parameter change within a line
          if(!ret){ // exit parameter change mode, go to running mode - anchor alarm active
            handleParamchanges = false;
            // get anchor position from current GPS position
            getGPSStartPosition(&wData.actLat, &wData.actLon); // get averaged boat start position from GPS
            sprintf(outstring,"doRoutineWork: MODE_CHANGEPARAM: Exiting. Current GPS position Lat: %6.4f Lon: %6.4f", 
                wData.actLat, wData.actLon);  
            logOut(2, outstring);
            // project anchor distance and bearing from start position to get anchor position
            gps_offset(wData.actLat, wData.actLon, wData.anchorDistanceM, wData.anchorBearingDeg, &wData.anchorLat, &wData.anchorLon);
            sprintf(outstring,"doRoutineWork: MODE_CHANGEPARAM: Exiting. Anchor position set to Lat: %6.4f Lon: %6.4f", 
                wData.anchorLat, wData.anchorLon);
            logOut(2, outstring);
            //initialize draw buffer
            if(wData.graphBufferClearingNeeded)
              clearGraphBuffer(1);
            updatewData(); // populate wData structure for first run, then no update of gps data expected
            wData.currentMode = MODE_RUNNING; // go to change param mode
          }
        }  
        // safeguared against accicental keypress: if too long in menu, go to routine mode and do measurements
        if(millis() - menuStartTimer > MAX_MILLIS_IN_MENU)
          wData.currentMode = MODE_RUNNING; // go to measurement mode
      }
      break;  
    case MODE_RUNNING:    // anchor alarm active
      {
        if(!wData.buttonPressed){ // button pressed means: quiet alarm, back to menu
          double startLat = 0.0; double startLon = 0.0;
          sprintf(outstring,"doRoutineWork: MODE_RUNNING ... %d", cnt++);
          logOut(2, outstring);
          
          // we need a bit of time to get the GPS data
          for(int i=0; i<3; i++)
          {  
            if((gps.location.isUpdated() && gps.location.isValid())) 
              break;
            smartDelay(1000);  
          }    
          
          updatewData();
          sprintf(outstring,"doRoutineWork: MODE_RUNNING: Current GPS position Lat: %6.6f Lon: %6.6f", 
              wData.actLat, wData.actLon);                
          logOut(2, outstring);
          
          sprintf(outstring,"doRoutineWork: MODE_RUNNING: Current distance anchor-boat: %4.2f m Bearing anchor-boat: %4.2f°", 
              wData.actAnchorDistanceM, wData.actAnchorBearingDeg); 
          logOut(2, outstring);
          //} if 
          drawWatchScreen();
          
          if(wData.alertON){    
            buzzer(3, 100,50);
            //doParallelBuzzer(3, 100,50);
            sprintf(outstring,"!!!!!!!!!!!! ALERT Reason %d %s !!!!!!!!!!!!",wData.alertReason, wData.alertReasonString);
            logOut(2,outstring);
            smartDelay(1000);
          }  
          else{
            // sleep for a time: switch off display, deep sleep for ESP32, leave gps receiving
            targetSleepUSec= wData.targetMeasurementIntervalSec * SECONDS;
            gotoDeepSleep(BUTTON1, targetSleepUSec); // go to deep sleep. parameters: sleeptime in us, button to wakeup from  
            //smartDelay(5000);
          } // else alert on


        }
        else{ // button pressed
          wData.buttonPressed = false; // reset button pressed state  
          wData.alertON = false;       // reset alarm
          wData.currentMode = MODE_STARTED; // go to start mode
        }    
      }
      break;
    default:  
    {
      sprintf(outstring,"doRoutineWork: default %d in case statement", trianglePos);
      logOut(2, outstring);
    }
  }  

  #ifdef WRITE_PREFERENCES
  // write counter preferences
  // if(startCounter % d_counterWriteInterval == 0){
  //   writeCounterPreferences();
  // }
  // write all preferences, incl. counter
    if(wData.preferencesChanged){
      writePreferences();
      wData.preferencesChanged = false;
    }
  #endif
} // doRoutineWork


/*****************************************************************************! 
  @brief  checkButton()    
  @details 
  @return void
*****************************************************************************/
void checkButton()
{
  if (buttonEvent) {
    buttonEvent = false;
    if(!wData.buttonPressed){
      logOut(2, (char*)"Interrupt: Rotary encoder button released.");
      wData.buttonPressed = true;
      //buzzer(1,50,0); // buzzer feedback
      doParallelBuzzer(1,50,0);
    } 
    sprintf(outstring,"Interrupt: Button wurde gedrückt");
    logOut(2, outstring);
  }
} 

/*****************************************************************************! 
  @brief  main loop
  @details 
  @return void
*****************************************************************************/
void loop() 
{
  // Update the Bounce instance (YOU MUST DO THIS EVERY LOOP)
  /*
  button.update(); // for button debouncing
  int buttonState = button.read();
  if(buttonState == LOW ){
    logOut(2, (char*)"Rotary encoder button pressed.");
    //buzzer(1,50,0); // buzzer feedback
    wData.buttonPressed = true;
  }
  */
  checkButton();

  doRoutineWork();
  gpsTimer.run(); // run gps timer to get data from gps module
};