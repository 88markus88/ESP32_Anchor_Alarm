
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

// TinyGPSPlus library for GPS handling
#include <TinyGPSPlus.h>
// #include <SoftwareSerial.h>



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
#define VREF 0

// *** lib for permanent data storage in EEPROM
// https://randomnerdtutorials.com/esp32-save-data-permanently-preferences/
// if these defines are set, preferences are written / saved to eeprom. 
// otherwise just from RTC storage (survives deep sleep)
#define WRITE_PREFERENCES
#define READ_PREFERENCES
#include <Preferences.h>
Preferences preferences; // object for preference storage in EEPROM
#define prefIDENT "ePaperAnchorAlarm" // unique identifier for preferences

// header file mit #defines, forward declarations, "extern" declarations von woanders deklarierten globalen var's
// globale variablen und alle anderen #includes sind im .cpp file
#include "ePaperAnchorAlarm.h"
#include "global.h" // global stuff from other modules

float volt, percent;                    // battery voltage and percent fill degree

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
RTC_DATA_ATTR uint32_t startCounter = 0; // 25920;  // total counter for starts of ESP32
RTC_DATA_ATTR uint32_t dischgCnt = 0;    // counter for starts of ESP32 since last charge
RTC_DATA_ATTR uint32_t prevMicrovolt = 0;
RTC_DATA_ATTR float prevVoltage = 0;

// test control variables
bool testRotaryEncoder = true;
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
   @brief    testRotary()
   @details  Function to test rotary encoder
   @param    boolean *rotButtonPressed : state of rotary encoder button
   @param    int64_t *rotPosition : position of rotary encoder
   @return   void
***************************************************/
void testRotary(boolean *rotButtonPressed, int64_t *rotPosition)
{
  //logOut(2, "testRotary: start");
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
  initDisplay(startCounter, FULL_UPDATE_INTERVAL); 

  // Serial port for GPS module
  ss.begin(9600, SERIAL_8N1, RXPin, TXPin); // RX, TX
  gpsTimerHandle = gpsTimer.setInterval(200L, gpsHandler); // set gps test timer to 100 milliseconds

  if(wData.currentMode == MODE_STARTED){ //populate missing data from defaults
    logOut(2,(char*)"===== populating defaults");
    wData.anchorBearingDeg= d_anchorBearingDeg;
    wData.anchorDistanceM = d_anchorDistanceM; 
    wData.alertThreshold  = d_alertThreshold; 
    wData.alarmDistanceM  = d_alarmDistanceM;  
    wData.drawCount       = 0;
  }

  if(wData.currentMode == MODE_STARTED){ // tests only with fresh start, to prevent this stuff when waking up after sleep
    // short display test
    if(testDisplayModule)
      testDisplay();  

    if(testBuzzer)        // buzzer test
      buzzer(1,100,500); 
    if(testDisplayModule) // display test, original from WeAct (manufacturer of the ePaper display)
      testDisplayWeAct();
    if(testBuzzer){       // buzzer test
      for(int i=0; i<5; i++)
      {
         buzzer(1,100,500-i*75); // buzzer test
      } 
    }  

    if(testGPSModule){
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
          wData.alertThreshold += -encoderPos; // each step is 1
          if(wData.alertThreshold < 1)
            wData.alertThreshold = 1; // minimum 1
          sprintf(outstring,"handleParamChange: Alarm count changed to %ld", wData.alertThreshold);
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
          sprintf(outstring,"handleParamChange: Alarm distance changed to %3.0f meters", wData.alarmDistanceM);
          logOut(2, outstring);
          encoder.setCount(0); // reset encoder count
          drawInputData(); // redraw input data
        }  // encoder
      } 
      break;   
    case 4: // exit parameter change mode
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
  @brief  doWork - Test main worker routine
  @details 
  @return void
*****************************************************************************/

void doTestWork()
{
  boolean buttonPressed = false; 
  int64_t encoderPos = 0;

  if(testRotaryEncoder)
    testRotary(&buttonPressed, &encoderPos );
  if(testGPSModule )
    gpsTest();
} // doTestWork

/*****************************************************************************! 
  @brief  doWork - Routine main worker routine
  @details 
  @return void
*****************************************************************************/

void updatewData() // helper function to populate the wData structure
{
  wData.actLat = gps.location.lat();
  wData.actLon = gps.location.lng();
  wData.actHDOP= gps.hdop.hdop();
  wData.SatCnt = gps.satellites.value();

  // calculate distance between actual position and anchor position
  wData.actAnchorDistanceM = TinyGPSPlus::distanceBetween(wData.actLat,wData.actLon, wData.anchorLat, wData.anchorLon);
  // calculate bearing from first to second position. Here: from anchor to actual position of boat
  wData.actAnchorBearingDeg = TinyGPSPlus::courseTo(wData.anchorLat, wData.anchorLon, wData.actLat,wData.actLon);

  // calc alert counter, based on deviation
  // 10% over adds 1, 30% over adds 3, 100% over adds 10
  if(wData.actAnchorDistanceM > wData.alarmDistanceM)
     wData.alertCount += 10.0* (wData.actAnchorDistanceM / wData.alarmDistanceM -1.0);
  else
    wData.alertCount = 0.0;
  if(wData.alertCount >= wData.alertThreshold)     // alarm!
    wData.alertON = true;
  else
    wData.alertON = false;
}

void doRoutineWork()
{
  int cnt = 0;
  uint32_t ret;
  boolean gpsValid = false, gpsUpdated = false;
  static int trianglePos = 0; // position of triangle indicator, serves as line selector
  static boolean handleParamchanges = false; // if true, handle parameter changes per line
  uint32_t targetSleepUSec;               // target sleep time in microseconds

  // determine reason for wakeup. if timer wakeup: continue with measurements. 
  // if EXT0 wakeup (button pressed): handleExt0Wakeup()
  ret = print_wakeup_reason(); // determine reason for wakeup
  if(ret == ESP_SLEEP_WAKEUP_EXT0)
    handleExt0Wakeup();

  switch(wData.currentMode){
    case MODE_STARTED:    // just started
      { 
        do{
          sprintf(outstring,"doRoutineWork: MODE_STARTED, waiting for GPS fix... %d", cnt);
          logOut(2, outstring);
          gpsValid = gpsTest();
        } 
        while(!gpsValid && cnt++<120); // wait up to 120 seconds for gps fix
        setScreenParameters();
        drawInputMainScreen();
        drawInputHeader();
        drawInputData();
        if(gpsValid)
          wData.currentMode = MODE_SELECTLINE; // to to input mode if gps is valid
      }
      break;
    case MODE_SELECTLINE:    // input mode to set position of triangle
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
              trianglePos = NUM_INPUTDATA_LINES-1;  // max of five positions 0..4
            trianglePos = (trianglePos) % NUM_INPUTDATA_LINES; // five positions
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
        }
      }
      break;
    case MODE_CHANGEPARAM:    // input mode to set a specific parameter, defined by last triangePos
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
            wData.drawCount = 0;
            for (int i=0; i<maxDrawBufferLen; i++){
              wData.drawBuffer[i][0] = 0;
              wData.drawBuffer[i][1] = 0;
            }
            updatewData(); // populate wData structure for first run, then no update of gps data expected
            wData.currentMode = MODE_RUNNING; // go to change param mode
          }
        }  
      }
      break;  
    case MODE_RUNNING:    // anchor alarm active
      {
        if(!wData.buttonPressed){
          double startLat = 0.0; double startLon = 0.0;
          sprintf(outstring,"doRoutineWork: MODE_RUNNING ... %d", cnt++);
          logOut(2, outstring);
          if(gps.location.isUpdated() && gps.location.isValid()){ 
            //wData.actLat = gps.location.lat();
            //wData.actLon = gps.location.lng();
            //wData.actHDOP= gps.hdop.hdop();
            //wData.SatCnt = gps.satellites.value();
            updatewData();
            sprintf(outstring,"doRoutineWork: MODE_RUNNING: Current GPS position Lat: %6.6f Lon: %6.6f", 
                wData.actLat, wData.actLon);                
            logOut(2, outstring);
            // calculate distance between actual position and anchor position
            //wData.actAnchorDistanceM = TinyGPSPlus::distanceBetween(wData.actLat,wData.actLon, wData.anchorLat, wData.anchorLon);
            // calculate bearing from first to second position. Here: from anchor to actual position of boat
            //wData.actAnchorBearingDeg = TinyGPSPlus::courseTo(wData.anchorLat, wData.anchorLon, wData.actLat,wData.actLon);

            // calc alert counter, based on deviation
            // 10% over adds 1, 30% over adds 3, 100% over adds 10
            /*
            if(wData.actAnchorDistanceM > wData.alarmDistanceM)
               wData.alertCount += 10.0* (wData.actAnchorDistanceM / wData.alarmDistanceM -1.0);
            else
              wData.alertCount = 0.0;
            if(wData.alertCount >= wData.alertThreshold)     // alarm!
              wData.alertON = true;
            else
              wData.alertON = false;
            */  
            if(wData.alertON){    
              buzzer(3, 100,50);
              smartDelay(1000);
            }  
            else{
              // sleep for a time: switch off display, deep sleep for ESP32, leave gps receiving
              targetSleepUSec= wData.targetMeasurementIntervalSec * SECONDS;
              gotoDeepSleep(BUTTON1, targetSleepUSec); // go to deep sleep. parameters: sleeptime in us, button to wakeup from  
              //smartDelay(5000);
            }

            sprintf(outstring,"doRoutineWork: MODE_RUNNING: Current distance anchor-boat: %4.2f m Bearing anchor-boat: %4.2f°", 
                wData.actAnchorDistanceM, wData.actAnchorBearingDeg); 
            logOut(2, outstring);
          } 
          drawWatchScreen();
        }
        else{ // button pressed
          wData.buttonPressed = false; // reset button pressed state  
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

} // doRoutineWork


/*****************************************************************************! 
  @brief  checkButon()    
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
      buzzer(1,50,0); // buzzer feedback
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
    buzzer(1,50,0); // buzzer feedback
    wData.buttonPressed = true;
  }
  */
  checkButton();

  doRoutineWork();
  gpsTimer.run(); // run gps timer to get data from gps module
};