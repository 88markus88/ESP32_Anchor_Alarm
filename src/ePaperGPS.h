#ifndef _ePaperAnchorAlarmGraphics_H
#define _ePaperAnchorAlarmGraphics_H

//*************** function prototypes ******************/
bool startupGPSSafely(HardwareSerial& gpsSerial, uint32_t  secs1, uint32_t secs2);
void configurePowerSaveMode(HardwareSerial& gpsSerial, bool switchON);
void resetPowerSaveMode(HardwareSerial& gpsSerial);


#endif // _ePaperAnchorAlarmGraphics_H