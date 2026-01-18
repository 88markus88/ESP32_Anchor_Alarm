
/****************************************************************/
// ePaperAnchorAlarm by. M. Preidel
/****************************************************************/

//**** ESP32 sleep and rtc memory (which survives deep sleep)
#include "esp_sleep.h"
#include "driver/rtc_io.h"

// ** rotary encoder lib
#include <ESP32Encoder.h>
#define ENCODER_CLK_PIN 25
#define ENCODER_DT_PIN 27
#define ENCODER_SW_PIN 26


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
   @brief    testRotary()
   @details  Function to test rotary encoder
   @param    void
   @return   void
***************************************************/
void testRotary()
{
  //logOut(2, "testRotary: start");
  static long degrees;
  static bool lastButtonState = HIGH;
  static bool buttonPressed = false;

  // read encoder switch
  bool reading = digitalRead(ENCODER_SW_PIN);
  if (reading == LOW && lastButtonState == HIGH) {
    // button pressed
    buttonPressed = !buttonPressed; // toggle state
    //if(buttonPressed){
    //  LogOut(2, "Rotary encoder button pressed: ON");
    //} else {
    //  LogOut(2, "Rotary encoder button pressed: OFF");
    //}
  }

  // read  encoder position
  long pos = encoder.getCount();
  degrees = pos *10 % 360 ;
  sprintf(outstring, "Rotary encoder position: %ld , degrees: %ld Button: %s", pos, degrees, buttonPressed ? "ON" : "OFF");
  logOut(2, outstring);

  delay(100);
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
   encoder.setCount(3600);

   // initialize encoder switch at pin ENCODER_SW_PIN
   pinMode(ENCODER_SW_PIN, INPUT_PULLUP);


} // setup

/*****************************************************************************! 
  @brief  doWork - main worker routine
  @details 
  @return void
*****************************************************************************/

void doWork()
{
  testRotary();
} // doWork


/*****************************************************************************! 
  @brief  main loop
  @details 
  @return void
*****************************************************************************/
void loop() 
{
  doWork();
};