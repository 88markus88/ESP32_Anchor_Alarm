// this file contains function prototypes and external definitions of variables needed by other modules

#ifndef _global_H
#define _global_H

/***************** global defines */
#define logLEVEL 2              // any output with log level <= this number is logged
#define maxLOG_STRING_LEN 240   // max len of logstring

#define maxDrawBufferLen 200  // max length of draw buffer for screen data

extern RTC_DATA_ATTR uint32_t fgndColor;
extern RTC_DATA_ATTR uint32_t bgndColor;

#define maxSleeptimeSafetyLimit 2000000000 // safety limit for deep sleep: no sleep above 2000 sec

//*************** global global variables ******************/
extern char outstring[maxLOG_STRING_LEN];

enum mainLoopMode {
  MODE_STARTED = 0,
  MODE_SELECTLINE = 1,
  MODE_CHANGEPARAM = 2,
  MODE_RUNNING = 3,
  MODE_ALARN = 4
};

// defaults
#define d_anchorBearingDeg 45 
#define d_anchorDistanceM  23
#define d_alertThreshold   10 
#define d_alarmDistanceM   40  

// Battery Thresholds for alert
#define BAT_VOLTAGE_THRESHOLD 3.6
#define BAT_PERCENT_THRESHOLD 10

struct measurementData
{
  // admin stuff
  bool justInitialized;   // indicator for the fact that software has just been initialized (test data)
  bool dataPresent;       // for simulation. do not create simulation data if this is true
  int32_t graphicsType; // determine which graph is shown  0: pressure, 1: temperature, 2: humidity
  int32_t startCounter;  // total counter for starts of ESP32
  int32_t dischgCnt;    // counter for starts of ESP32 since last charge
  struct timeval lastMeasurementTimestamp;  // time value when last measurement has been taken
  struct timeval last2MeasurementTimestamp;  // time value when measurement before last has been taken
  float actSecondsSinceLastMeasurement;     // seconds elapsed since last measurement, before last deep sleep
  bool applyInversion;   // if true, white on black. otherwise black on white
  
  bool buttonPressed;    // remembers if button has been pressed to acknowledge an alert
  bool alertON;          // remembers if an alert has been triggered
  mainLoopMode currentMode; // current main loop mode

  int32_t targetMeasurementIntervalSec=20;    // sleep time target in seconds, controls the measurement
  int32_t lastTargetSleeptime;             // last target standard sleep time in seconds
  int64_t lastActualSleeptimeAfterMeasUsec;         // this is the number in usec actually used to set the sleep timer after last measurement
  int64_t lastActualSleeptimeNotMeasUsec;         // this is the number in usec actually used to set the sleep timer when no measurement

  float anchorBearingDeg;      // anchor angle in degrees
  float anchorDistanceM;       // anchor distance in meters
  int32_t alertThreshold;       // number of position deviations before alarm is triggered
  float alarmDistanceM;        // alarm distance in meters

  double anchorLat;               // anchor latitude
  double anchorLon;               // anchor longitude

  int32_t drawCount;              // counter for display updates
  int drawBuffer[maxDrawBufferLen][2];         // buffer for display data
  double actLat;                  // actual latitude
  double actLon;                  // actual longitude
  float actAnchorBearingDeg;        // anchor angle in degrees
  float actAnchorDistanceM;       // anchor distance in meters
  double actHDOP;                 // GPS HDOP
  int32_t SatCnt;                 // GPS visible satellites
  double alertCount;              // counter for alert condition

  float batteryVoltage;
  float batteryPercent;
};
extern RTC_DATA_ATTR measurementData wData;

//*************** function prototypes ******************/
void testDisplay();
void testDisplayWeAct();
void initDisplay(int startCounter, int fullInterval); // start display
void endDisplay(int mode);                  // power off display if mode =0 else hibernate
void logOut(int logLevel, char* str);
void buzzer(uint16_t number, uint16_t duration, uint16_t interval);
//void getInputMainScreen();
void drawInputMainScreen();
void drawInputHeader();
void drawInputData();
void drawTriangle(bool visible, int16_t xpos, int16_t ypos);
void drawWatchScreen();
void setScreenParameters();
void doParallelBuzzer(uint16_t number, uint16_t duration, uint16_t interval);

boolean gpsTest();

#endif // _global_H