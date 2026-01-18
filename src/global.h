// this file contains function prototypes and external definitions of variables needed by other modules

#ifndef _global_H
#define _global_H

/***************** global defines */
#define logLEVEL 2              // any output with log level <= this number is logged
#define maxLOG_STRING_LEN 240   // max len of logstring

extern RTC_DATA_ATTR uint32_t fgndColor;
extern RTC_DATA_ATTR uint32_t bgndColor;

//*************** global global variables ******************/
extern char outstring[maxLOG_STRING_LEN];

struct measurementData
{
  // admin stuff
  bool justInitialized;   // indicator for the fact that software has just been initialized (test data)
  bool dataPresent;       // for simulation. do not create simulation data if this is true
  int32_t graphicsType; // determine which graph is shown  0: pressure, 1: temperature, 2: humidity
  int32_t startCounter;    // total counter for starts of ESP32
  int32_t dischgCnt;    // counter for starts of ESP32 since last charge
  struct timeval lastMeasurementTimestamp;  // time value when last measurement has been taken
  struct timeval last2MeasurementTimestamp;  // time value when measurement before last has been taken
  float actSecondsSinceLastMeasurement;     // seconds elapsed since last measurement, before last deep sleep

  bool buttonPressed;    // remembers if button has been pressed to acknowledge an alert
  bool alertON;          // remembers if an alert has been triggered

  int32_t targetMeasurementIntervalSec;    // sleep time target in seconds, controls the measurement
  int32_t lastTargetSleeptime;             // last target standard sleep time in seconds
  int64_t lastActualSleeptimeAfterMeasUsec;         // this is the number in usec actually used to set the sleep timer after last measurement
  int64_t lastActualSleeptimeNotMeasUsec;         // this is the number in usec actually used to set the sleep timer when no measurement

  float batteryVoltage;
  float batteryPercent;
};
extern RTC_DATA_ATTR measurementData wData;

//*************** function prototypes ******************/


#endif // _global_H