
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

// interrupt handling for button press
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

// display GPS info
void displayGPSInfo()
{
  //Serial.print(F("Location: ")); 

  if (gps.location.isValid())
  {
    Serial.print(F(" Sats: "));
    Serial.print(gps.satellites.value());
    Serial.print(F(" HDOP: "));
    Serial.print(gps.hdop.hdop(), 2);   
    
    Serial.print(F(" Location: ")); 
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);

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
    Serial.print(F("  Dist2Start: "));
    Serial.print(distanceToStart, 2); 
    Serial.print(F(" m  Course2Start: "));
    Serial.print(courseToStart, 2);     

    wData.actLat = gps.location.lat();
    wData.actLon = gps.location.lng();
    sprintf(outstring," Stored Lat/Lon: %6.4f %6.4f",wData.actLat , wData.actLon);
    logOut(2, outstring);
  }
  else
  {
    //Serial.print(F("INVALID"));
  }

  // Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(F("  Date/Time: "));
    Serial.print(gps.date.day());
    Serial.print(F("."));
    Serial.print(gps.date.month());
    Serial.print(F("."));
    Serial.print(gps.date.year());
  }
  else
  {
    //Serial.print(F("INVALID"));
  }

  //Serial.print(F(" "));
  if (gps.time.isValid())
  {
    Serial.print(F(" "));
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
    Serial.print(F(" "));
  }
  else
  {
    //Serial.print(F("INVALID"));
  }

  Serial.println();
}

#include <math.h>

#define EARTH_RADIUS 6371000.0  // Meter

// Calculate new GPS position given start coords, distance and bearing
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

// get the initial start position from GPS
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
        Serial.print(F("Fix #"));
        Serial.print(cnt+1);
        Serial.print(F(" Sats: "));
        Serial.print(gps.satellites.value());
        Serial.print(F(" HDOP: "));
        Serial.print(gps.hdop.hdop(), 2);   
        
        *startLat += gps.location.lat();
        *startLon += gps.location.lng();
        Serial.print(cnt, 2);
        Serial.print(F("Start position acquired :"));
        Serial.print(*startLat/cnt, 6);
        Serial.print(F(", "));
        Serial.println(*startLon/cnt, 6);
      }
      if(cnt>=avg){
        *startLat /= cnt;
        *startLon /= cnt;
        Serial.print(F("Averaged start position: "));
        Serial.print(*startLat, 6);
        Serial.print(F(", "));
        Serial.println(*startLon, 6);
        return;
      }
    }
  }
  Serial.println(F("Failed to acquire GPS start position within 30 seconds."));

}

// This custom version of delay() ensures that the gps object
// is being "fed".
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
   @param    void 
   @return   void
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

  // short display test
  if(testDisplayModule)
    testDisplay();  
  if(testBuzzer)
    buzzer(1,100,500); // buzzer test
  if(testDisplayModule)
    testDisplayWeAct();
  if(testBuzzer){  
    // buzzer(2,500,100); // buzzer test
    // buzzer test

    for(int i=0; i<5; i++)
    {
       buzzer(1,100,500-i*75); // buzzer test
      //buzzer(3,100,500); // buzzer test
    } 
    //buzzer(3,300,1000); // buzzer test
  }  

  // Serial port for GPS module
  ss.begin(9600, SERIAL_8N1, RXPin, TXPin); // RX, TX
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
  gpsTimerHandle = gpsTimer.setInterval(200L, gpsHandler); // set gps test timer to 100 milliseconds
  
} // setup


/*****************************************************************************! 
  @brief  handleParamChange - Test main worker routine
  @details This routine handles the adaptation of a single parameter via rotary encode
  @param  int trianglePos : position of triangle indicating which parameter to change. 
  @param  0: anchor bearing; 1: anchor distance, 2: alarm coount; 3:alarm distance
  @return void
*****************************************************************************/
void handleParamChange(int trianglePos)
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
          wData.alertCount += -encoderPos; // each step is 1
          if(wData.alertCount < 1)
            wData.alertCount = 1; // minimum 1
          sprintf(outstring,"handleParamChange: Alarm count changed to %ld", wData.alertCount);
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
    default:
      logOut(2, (char*)"handleParamChange: default in case statement");
  } 
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

void doRoutineWork()
{
  int cnt = 0;
  boolean gpsValid = false, gpsUpdated = false;
  static int trianglePos = 0; // position of triangle indicator, serves as line selector
  static boolean handleParamchanges = false; // if true, handle parameter changes per line

  switch(wData.currentMode){
    case MODE_STARTED:    // just started
      { 
        do{
          sprintf(outstring,"doRoutineWork: MODE_STARTED, waiting for GPS fix... %d", cnt);
          logOut(2, outstring);
          gpsValid = gpsTest();
        } 
        while(!gpsValid && cnt++<120); // wait up to 120 seconds for gps fix
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
              trianglePos = 3;  // max of four positions 0..3
            trianglePos = (trianglePos) % 4; // four positions
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
          handleParamChange(trianglePos); // handle parameter change within a line
        }
      }
      break;  
    default:  
    {
      sprintf(outstring,"doRoutineWork: default in case statement ... %d", cnt);
      logOut(2, outstring);
    }
  }  

} // doRoutineWork



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
  if (buttonEvent) {
    buttonEvent = false;
    if(!wData.buttonPressed){
      logOut(2, (char*)"Interrupt: Rotary encoder button released.");
      wData.buttonPressed = true;
      buzzer(1,50,0); // buzzer feedback
    } 
    Serial.println("Interrupt: Button wurde gedrückt");
  }

  doRoutineWork();
  gpsTimer.run(); // run gps timer
};