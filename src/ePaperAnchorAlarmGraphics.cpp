/**************************************************!
   main drawing functions
***************************************************/

#include <Arduino.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

// Screen parameters
#include "screenParameters.h"

// GxEPD2 ePaper library
#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>

#ifdef LOLIN32_LITE 
  // Connections for LOLIN D32 
  static const uint8_t EPD_BUSY = 4;  // to EPD BUSY
  static const uint8_t EPD_CS   = 5;  // to EPD CS
  static const uint8_t EPD_RST  = 16; // to EPD RST
  static const uint8_t EPD_DC   = 17; // to EPD DC
  static const uint8_t EPD_SCK  = 18; // to EPD CLK
  static const uint8_t EPD_MOSI = 23; // to EPD DIN
  // display class definition  for GxRPD  from class template
  // MP 21.12.24: my standard. Works ok in full refresh, not in partial refresh
  GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> display(GxEPD2_154_D67(/*CS*/ EPD_CS, /*DC=*/ EPD_DC, /*RST=*/ EPD_RST, /*BUSY=*/ EPD_BUSY)); // GDEH0154D67

#endif // LOLIN32_LITE 

//***************** general libraries ****************************
// Genutzte Schriften importieren
// https://learn.adafruit.com/adafruit-gfx-graphics-library/using-fonts
// https://docs.espressif.com/projects/arduino-esp32/en/latest/tutorials/preferences.html#workflow 
#include <Fonts/FreeMonoBold24pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMono24pt7b.h>
#include <Fonts/FreeMono18pt7b.h>
#include <Fonts/FreeMono12pt7b.h>
#include <Fonts/FreeMono9pt7b.h>

#include "ePaperAnchorAlarm.h"
#include "global.h"

// platformio libdeps: olikraus/U8g2_for_Adafruit_GFX@^1.8.0
#include <U8g2_for_Adafruit_GFX.h>
U8G2_FOR_ADAFRUIT_GFX u8g2Fonts;  // Select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall

// Using fonts:
// u8g2_font_helvB08_tf
// u8g2_font_helvB10_tf
// u8g2_font_helvB12_tf
// u8g2_font_helvB14_tf
// u8g2_font_helvB18_tf
// u8g2_font_helvB24_tf

//****************** file global variables ********************************/

//+++++++++++++++++++++++++++ clear screen using partial update ++++++++++++++
//void clearScreenPartialUpdate()
//{
//  display.setPartialWindow(0, 0, display.width(), display.height()); 
//  
//  do{
//    display.fillScreen(bgndColor);
//  }while(display.nextPage());
//}  
//
////+++++++++++++++++++++++++++ clear screen using full update ++++++++++++++
//void clearScreenFullUpdate()
//{
//  display.setFullWindow();
//  do{
//    display.fillScreen(bgndColor);
//  }while(display.nextPage());
//};  

// initialize display, taken from setup()
void initDisplay(int startCounter, int fullInterval)
{
  #ifdef CROW_PANEL
    epdPower(HIGH);

    #ifdef TEST_CROW_PANEL
      epdInit();
      logOut(2,"Testing panel");
      epdTest();
      delay(3000);
    #endif // TEST_CROW_PANEL  
  #endif // CROW_PANEL  
  // full refresh of epaper every fullInterval's time. 1: every time
  if (startCounter % fullInterval == 0)
  {
    logOut(2,(char*)"+++++++ Full window clearing");
    display.init(115200, true, 2, false); // initial = true  for first start
    display.setFullWindow();
  }
  else{
    logOut(2,(char*)"------- Partial window clearing");
    display.init(115200, false, 2, false); // initial = false for subsequent starts
    display.setPartialWindow(0, 0, display.width(), display.height());
  }

  if(wData.applyInversion){
    fgndColor = GxEPD_WHITE;
    bgndColor = GxEPD_BLACK;
  }
  else{
    fgndColor = GxEPD_BLACK;
    bgndColor = GxEPD_WHITE;    
  }
  // prepare display colors (adafruit)
  display.setTextColor(fgndColor, bgndColor);  

  // prepare u8g2 fonts
  u8g2Fonts.begin(display); // connect u8g2 procedures to Adafruit GFX
  u8g2Fonts.setFontMode(1);                  // use u8g2 transparent mode (this is default)
  u8g2Fonts.setFontDirection(0);             // left to right (this is default)
  u8g2Fonts.setForegroundColor(fgndColor);   // apply Adafruit GFX color
  u8g2Fonts.setBackgroundColor(bgndColor);   // apply Adafruit GFX color
  u8g2Fonts.setFont(u8g2_font_helvB10_tf);   // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall

}

// power off display
void endDisplay(int mode)
{
    if(mode ==0)  
      display.powerOff(); // danach wird beim wieder aufwachen ein voller Reset des Screens gemacht
    else
      display.hibernate();  // danach wird beim wieder aufwachen kein Reset des Screens gemacht.
}

// display test data - home made
void testDisplay()
{
  int y;  
  //display.setRotation(1);            // Display um 90° drehen
  display.setRotation(3);              // Display um 270° drehen
  display.setTextColor(fgndColor);     // Schriftfarbe Schwarz
  display.setFont(&FreeMonoBold18pt7b);// Schrift definieren

  sprintf(outstring, "Start of testDisplay() ");
  logOut(2,outstring);

  display.firstPage();
  do{ 
    y=12;
    // Titel schreiben
    display.setCursor(0, y);
    display.setFont(&FreeMonoBold9pt7b);
    #ifdef VERSION
      sprintf(outstring, "ePaperAnchorAlarm %s",VERSION);
    #else
      sprintf(outstring,"ePaperAnchorAlarm");
    #endif  
    display.print(outstring);
  }while (display.nextPage());
}

//------------------ begin weact demo functions ----------------------
const char HelloWorld[] = "Hello World!";
const char HelloWeACtStudio[] = "WeAct Studio";

void helloWorld()
{
  display.setRotation(1);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);
  int16_t tbx, tby; uint16_t tbw, tbh;
  display.getTextBounds(HelloWorld, 0, 0, &tbx, &tby, &tbw, &tbh);
  // center the bounding box by transposition of the origin:
  uint16_t x = ((display.width() - tbw) / 2) - tbx;
  uint16_t y = ((display.height() - tbh) / 2) - tby;
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, y-tbh);
    display.print(HelloWorld);
    display.setTextColor(display.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);
    display.getTextBounds(HelloWeACtStudio, 0, 0, &tbx, &tby, &tbw, &tbh);
    x = ((display.width() - tbw) / 2) - tbx;
    display.setCursor(x, y+tbh);
    display.print(HelloWeACtStudio);
  }
  while (display.nextPage());
}

void helloFullScreenPartialMode()
{
  //Serial.println("helloFullScreenPartialMode");
  const char fullscreen[] = "full screen update";
  const char fpm[] = "fast partial mode";
  const char spm[] = "slow partial mode";
  const char npm[] = "no partial mode";
  display.setPartialWindow(0, 0, display.width(), display.height());
  display.setRotation(1);
  display.setFont(&FreeMonoBold9pt7b);
  if (display.epd2.WIDTH < 104) display.setFont(0);
  display.setTextColor(GxEPD_BLACK);
  const char* updatemode;
  if (display.epd2.hasFastPartialUpdate)
  {
    updatemode = fpm;
  }
  else if (display.epd2.hasPartialUpdate)
  {
    updatemode = spm;
  }
  else
  {
    updatemode = npm;
  }
  // do this outside of the loop
  int16_t tbx, tby; uint16_t tbw, tbh;
  // center update text
  display.getTextBounds(fullscreen, 0, 0, &tbx, &tby, &tbw, &tbh);
  uint16_t utx = ((display.width() - tbw) / 2) - tbx;
  uint16_t uty = ((display.height() / 4) - tbh / 2) - tby;
  // center update mode
  display.getTextBounds(updatemode, 0, 0, &tbx, &tby, &tbw, &tbh);
  uint16_t umx = ((display.width() - tbw) / 2) - tbx;
  uint16_t umy = ((display.height() * 3 / 4) - tbh / 2) - tby;
  // center HelloWorld
  display.getTextBounds(HelloWorld, 0, 0, &tbx, &tby, &tbw, &tbh);
  uint16_t hwx = ((display.width() - tbw) / 2) - tbx;
  uint16_t hwy = ((display.height() - tbh) / 2) - tby;
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(hwx, hwy);
    display.print(HelloWorld);
    display.setCursor(utx, uty);
    display.print(fullscreen);
    display.setCursor(umx, umy);
    display.print(updatemode);
  }
  while (display.nextPage());
  //Serial.println("helloFullScreenPartialMode done");
}

void showPartialUpdate()
{
  // some useful background
  helloWorld();
  // use asymmetric values for test
  uint16_t box_x = 10;
  uint16_t box_y = 15;
  uint16_t box_w = 70;
  uint16_t box_h = 20;
  uint16_t cursor_y = box_y + box_h - 6;
  if (display.epd2.WIDTH < 104) cursor_y = box_y + 6;
  float value = 13.95;
  uint16_t incr = display.epd2.hasFastPartialUpdate ? 1 : 3;
  display.setFont(&FreeMonoBold9pt7b);
  if (display.epd2.WIDTH < 104) display.setFont(0);
  display.setTextColor(GxEPD_BLACK);
  // show where the update box is
  for (uint16_t r = 0; r < 4; r++)
  {
    display.setRotation(r);
    display.setPartialWindow(box_x, box_y, box_w, box_h);
    display.firstPage();
    do
    {
      display.fillRect(box_x, box_y, box_w, box_h, GxEPD_BLACK);
      //display.fillScreen(GxEPD_BLACK);
    }
    while (display.nextPage());
    delay(2000);
    display.firstPage();
    do
    {
      display.fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE);
    }
    while (display.nextPage());
    delay(1000);
  }
  //return;
  // show updates in the update box
  for (uint16_t r = 0; r < 4; r++)
  {
    display.setRotation(r);
    display.setPartialWindow(box_x, box_y, box_w, box_h);
    for (uint16_t i = 1; i <= 10; i += incr)
    {
      display.firstPage();
      do
      {
        display.fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE);
        display.setCursor(box_x, cursor_y);
        display.print(value * i, 2);
      }
      while (display.nextPage());
      delay(500);
    }
    delay(1000);
    display.firstPage();
    do
    {
      display.fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE);
    }
    while (display.nextPage());
    delay(1000);
  }
}


// display test data - weAct Demo
void testDisplayWeAct(){
  display.init(115200,true,50,false);
  helloWorld();
  helloFullScreenPartialMode();
  delay(500);
  if (display.epd2.hasFastPartialUpdate)
  {
    showPartialUpdate();
    delay(100);
  }
  display.hibernate();
}

/*****************************************************************************! 
  @brief showWelcomeMessage - initial screen display for input
  @details 
  @return void
*****************************************************************************/
void showWelcomeMessage(boolean clearScreen, char* nextMessage)
{
  int x, y;
  // box parameters for partial screen update
  int box_x; 
  int box_y;
  uint16_t box_w;
  uint16_t box_h;
  static int line = 1;
  char displstring[200];       

  display.setFont(&FreeMonoBold9pt7b);// set font
  // display.setFont(NULL); // set default 5x7 font
  display.setTextColor(GxEPD_BLACK);  // set text color
  display.setRotation(SCREEN_ROTATION);//and screen rotation
  
  if(clearScreen){
    display.setFullWindow();
    
    line = 1;    
    x=0; y= line*HEADER_FONT_SIZE-1;
    display.firstPage();
    do{
      display.fillScreen(GxEPD_WHITE);

      display.setCursor(x, y);
      display.print(nextMessage);  
    } while (display.nextPage());
  }
  else{ // not clearScreen
    box_x=0; 
    box_w=SCREEN_WIDTH-1;
    box_y=(line-1)*HEADER_FONT_SIZE; 
    box_h=7*HEADER_FONT_SIZE;
    display.setPartialWindow(box_x, box_y, box_w, box_h);
    display.firstPage();
    do{
      display.fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE); 
      x=0; y= (line)*HEADER_FONT_SIZE-1;
      display.setCursor(x, y);
      display.setCursor(x, y);
      display.print(nextMessage);                 
    } while (display.nextPage());     
  }  
  sprintf(displstring,"showWelcomeMessage line: %d x: %d y: %d box_x: %d box_y :%d box_w: %d box_h: %d Msg: _%s_\n", 
                                                line,  x,    y,        box_x,    box_y,    box_w,    box_h,   nextMessage);
  logOut(2, displstring);   
  line++;
}

/*****************************************************************************! 
  @brief  drawInputMainScreen - initial screen display for input
  @details 
  @return void
*****************************************************************************/
void drawInputMainScreen()
{
  int x, y;

  display.setFullWindow();
  display.firstPage();
  do
  {
    x=0; y= HEADER_FONT_SIZE;
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, y);
    #ifdef VERSION
      sprintf(outstring, "AnchorAlarm %s",VERSION);
    #else
      sprintf(outstring,"ePaperAnchorAlarm");
    #endif  
    display.print(outstring);

    x=0; y=2*INPUTDATA_FONT_SIZE;
    display.setCursor(x, y);
    sprintf(outstring,"%6.5f %6.5f",wData.actLat,wData.actLon);
    display.print(outstring);
    //x=0; y=3*INPUTDATA_FONT_SIZE;
    //display.setCursor(x, y);
    //sprintf(outstring,"Bat:%4.2fV %3.0f%%",wData.batteryVoltage,wData.batteryPercent);
    //logOut(2, outstring);
    //display.print(outstring);
    x=0; y=INPUTDATA_FIRST_LINE*INPUTDATA_FONT_SIZE;
    display.setCursor(x, y);
    sprintf(outstring,"Anchor Bearg:");
    display.print(outstring);
    x=0; y=(INPUTDATA_FIRST_LINE+1)*INPUTDATA_FONT_SIZE;
    display.setCursor(x, y);
    sprintf(outstring,"Anchor Dist :");
    display.print(outstring);
    x=0; y=(INPUTDATA_FIRST_LINE+2)*INPUTDATA_FONT_SIZE;
    display.setCursor(x, y);
    sprintf(outstring,"Alert Thresh:");
    display.print(outstring);
    x=0; y=(INPUTDATA_FIRST_LINE+3)*INPUTDATA_FONT_SIZE;
    display.setCursor(x, y);
    sprintf(outstring,"Alert Dist  :");
    display.print(outstring);
    x=0; y=(INPUTDATA_FIRST_LINE+4)*INPUTDATA_FONT_SIZE;
    display.setCursor(x, y);
    sprintf(outstring,"Sleep Time  :");
    display.print(outstring);
    x=0; y=(INPUTDATA_FIRST_LINE+5)*INPUTDATA_FONT_SIZE;
    display.setCursor(x, y);
    sprintf(outstring,"Detail Info :");
    display.print(outstring);
    x=0; y=(INPUTDATA_FIRST_LINE+6)*INPUTDATA_FONT_SIZE;
    display.setCursor(x, y);
    sprintf(outstring,"Graph Weight:");
    display.print(outstring);
    x=0; y=(INPUTDATA_FIRST_LINE+7)*INPUTDATA_FONT_SIZE;
    display.setCursor(x, y);
    sprintf(outstring,"Power Saving:");
    display.print(outstring);

    x=0; y=(INPUTDATA_FIRST_LINE+8)*INPUTDATA_FONT_SIZE;
    display.setCursor(x, y);
    sprintf(outstring,"Exit        :");
    display.print(outstring);
  }
  while (display.nextPage());

  // write prompt
  display.setCursor(0, 0);
}

/*****************************************************************************! 
  @brief  drawTriangle - draw on screen, used for menu 
  @details 
  @param visible - true: draw triangle, false: clear triangle
  @param xpos - x position of triangle upper left corner
  @param ypos - y position of triangle upper left corner
  @return void
*****************************************************************************/
void drawTriangle(bool visible,  int16_t xpos, int16_t ypos)
{
  int16_t height = INPUTDATA_FONT_SIZE, width = 8;
  display.setPartialWindow(xpos, ypos, width, height);

  //sprintf(outstring,"drawTriangle at %d,%d visible=%d",xpos,ypos,visible);
  //logOut(2, outstring);

  display.firstPage();
  do {
    if (visible) {
      display.fillTriangle(
        xpos, ypos,   // Spitze oben
        xpos, ypos+height,   // links unten
        xpos + width, ypos + height/2,  // rechts unten
        GxEPD_BLACK
      );
    } else {
      display.fillScreen(GxEPD_WHITE);
    }
  } while (display.nextPage());
}

/*****************************************************************************! 
  @brief  setScreenParameters
  @details init screen and set font, text color, rotation
  @return void
*****************************************************************************/
void setScreenParameters()
{
  display.init(115200,true,50,false); // init display and clear screen
  display.setFont(&FreeMonoBold9pt7b);// set font
  // display.setFont(NULL); // set default 5x7 font
  display.setTextColor(GxEPD_BLACK);  // set text color
  display.setRotation(SCREEN_ROTATION);//and screen rotation
}  

/*****************************************************************************! 
  @brief  drawInputMainHeader - display first four lines as partial update
  @details 
  @return void
*****************************************************************************/
void drawInputHeader(){
  int x, y;
  // partial update of upper part of screen
  display.firstPage();
  do
  {
    display.setPartialWindow(HEADER_X_POS, HEADER_Y_POS, HEADER_WIDTH, HEADER_HEIGHT);
    display.fillRect(HEADER_X_POS, HEADER_Y_POS, HEADER_WIDTH, HEADER_HEIGHT, GxEPD_WHITE);
    
    x=0; y=1*HEADER_FONT_SIZE-1;
    display.setCursor(x, y);
    #ifdef VERSION
      sprintf(outstring, "AnchorAlarm %s",VERSION);
    #else
      sprintf(outstring,"ePaperAnchorAlarm");
    #endif  
    display.print(outstring); 
    
    x=0; y=2*HEADER_FONT_SIZE-1;
    display.setCursor(x, y);
    sprintf(outstring,"%6.6f %6.6f",wData.actLat,wData.actLon);
    logOut(2, outstring);
    display.print(outstring);
    
    //x=0; y=3*HEADER_FONT_SIZE-1;
    //display.setCursor(x, y);
    //sprintf(outstring,"Bat:%4.2fV %3.0f%%",wData.batteryVoltage,wData.batteryPercent);
    //logOut(2, outstring);
    //display.print(outstring);
  }
  while (display.nextPage());
}

/*****************************************************************************! 
  @brief  drawInput Data - display data elements in right position
  @details 
  @return void
*****************************************************************************/
void drawInputData(){
  int x, y;
  // box parameters for partial screen update
  uint16_t box_x = INPUTDATA_X_POS;
  uint16_t box_y = INPUTDATA_Y_POS;
  uint16_t box_w = INPUTDATA_WIDTH;
  uint16_t box_h = INPUTDATA_HEIGHT;
  display.firstPage();
  do
  {
    display.setPartialWindow(box_x, box_y, box_w, box_h);
    display.fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE);
    x=150; y=(INPUTDATA_FIRST_LINE+0)*INPUTDATA_FONT_SIZE-1;
    display.setCursor(x, y);
    sprintf(outstring,"%3.0f deg",wData.anchorBearingDeg);
    //sprintf(outstring,"%3.0f deg",(wData.anchorBearingDeg)+cnt%360);
    display.print(outstring);
    x=150; y=(INPUTDATA_FIRST_LINE+1)*INPUTDATA_FONT_SIZE-1;
    display.setCursor(x, y);
    sprintf(outstring,"%3.0f m",wData.anchorDistanceM);
    //sprintf(outstring,"%3.0f m",wData.anchorDistanceM+cnt%100);
    display.print(outstring);
    x=150; y=(INPUTDATA_FIRST_LINE+2)*INPUTDATA_FONT_SIZE-1;
    display.setCursor(x, y);
    sprintf(outstring,"%3d",wData.alarmThreshold);
    //sprintf(outstring,"%3d",cnt);
    display.print(outstring);
    x=150; y=(INPUTDATA_FIRST_LINE+3)*INPUTDATA_FONT_SIZE-1;
    display.setCursor(x, y);
    sprintf(outstring,"%3.0f",wData.alarmDistanceM);
    //sprintf(outstring,"%3d",cnt);
    display.print(outstring);

    x=150; y=(INPUTDATA_FIRST_LINE+4)*INPUTDATA_FONT_SIZE-1;
    display.setCursor(x, y);
    sprintf(outstring,"%3ld",wData.targetMeasurementIntervalSec);
    //sprintf(outstring,"%3d",cnt);
    display.print(outstring);
    x=150; y=(INPUTDATA_FIRST_LINE+5)*INPUTDATA_FONT_SIZE-1;
    display.setCursor(x, y);
    switch(wData.verbosity){
      case LO:
        sprintf(outstring,"LOW");
      break;
      case MED:
        sprintf(outstring,"MED");
      break;
      case HI:
        sprintf(outstring,"HIGH");
      break;
    }
    //sprintf(outstring,"%3d",wData.verbosity);
    //sprintf(outstring,"%3d",cnt);
    display.print(outstring);
    
    x=150; y=(INPUTDATA_FIRST_LINE+6)*INPUTDATA_FONT_SIZE-1;
    display.setCursor(x, y);
    switch(wData.graphWeight){
      case MINIM:
        sprintf(outstring,"MIN");
      break;
      case MEDIUM:
        sprintf(outstring,"MED");
      break;
      case HEAVY:
        sprintf(outstring,"HVY");
      break;
    }
    //sprintf(outstring,"%3d",wData.graphWeight);
    //sprintf(outstring,"%3d",cnt);
    display.print(outstring);    

    x=150; y=(INPUTDATA_FIRST_LINE+7)*INPUTDATA_FONT_SIZE-1;
    display.setCursor(x, y);
    switch(wData.PowerSaveMode){
      case MINIM:
        sprintf(outstring,"MIN");
      break;
      case MEDIUM:
        sprintf(outstring,"MID");
      break;
      case HEAVY:
        sprintf(outstring,"MAX");
      break;
    }
    //sprintf(outstring,"%3d",wData.graphWeight);
    //sprintf(outstring,"%3d",cnt);
    display.print(outstring);    
  }
  while (display.nextPage());
}


/*****************************************************************************! 
  @brief  drawWatchScreen - display main watch screen
  @details 
  @return void
*****************************************************************************/
void drawWatchScreen(){
  int x, y;
  int16_t tbx, tby; uint16_t tbw, tbh; // variables for getting bounds of text fields

  setScreenParameters(); // main parameters including color, font, rotation
  display.setFullWindow();
  display.firstPage();
  do
  {
    x=0; y= HEADER_FONT_SIZE;

    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, y);
    #ifdef VERSION
      sprintf(outstring, "AnchorAlarm %s",VERSION);
    #else
      sprintf(outstring,"ePaperAnchorAlarm");
    #endif  

    if(wData.alertON){
      sprintf(outstring,"!!!!! ALARM %d !!!! ", wData.alertReason);
    }
    display.print(outstring);

    if(wData.alertON){
      x=0; y= SCREEN_HEIGHT/2;
      sprintf(outstring,"%s", wData.alertReasonString);
      display.setCursor(x, y);
      display.print(outstring);
    }

    if(wData.verbosity == HI){
      x=0; y=2 * HEADER_FONT_SIZE;
      display.setCursor(x, y);
      sprintf(outstring,"%6.6f %6.6f",wData.anchorLat,wData.anchorLon);
      display.print(outstring);

      x=0; y=3 * HEADER_FONT_SIZE;
      display.setCursor(x, y);
      sprintf(outstring,"S:%d %3.2f",wData.SatCnt, wData.actHDOP);
      display.print(outstring);
      
      sprintf(outstring,"AC:%3.1f",wData.alertCount);
      display.getTextBounds(outstring, 0, 0, &tbx, &tby, &tbw, &tbh); // center right
      x=SCREEN_WIDTH - tbw-3; y=3 * HEADER_FONT_SIZE;
      display.setCursor(x, y);
      display.print(outstring);

      x=0; y=4 * HEADER_FONT_SIZE;
      display.setCursor(x, y);
      sprintf(outstring,"%3.2fV",wData.batteryVoltage);
      display.print(outstring);

      sprintf(outstring,"%3.0f%%",wData.batteryPercent);
      display.getTextBounds(outstring, 0, 0, &tbx, &tby, &tbw, &tbh); // center right
      x=SCREEN_WIDTH - tbw-3; y=4 * HEADER_FONT_SIZE;
      display.setCursor(x, y);
      display.print(outstring);

      sprintf(outstring,"%2.0fm",wData.alarmDistanceM);
      display.getTextBounds(outstring, 0, 0, &tbx, &tby, &tbw, &tbh); // center right
      x=SCREEN_WIDTH - tbw-3; y=5 * HEADER_FONT_SIZE;
      display.setCursor(x, y);
      display.print(outstring);

      x=0; y=5 * HEADER_FONT_SIZE;
      display.setCursor(x, y);
      sprintf(outstring,"%ld",wData.startCounter);
      display.print(outstring);

      x=0; y=6 * HEADER_FONT_SIZE;
      display.setCursor(x, y);
      sprintf(outstring,"%ld",wData.noAlertsSounded);
      display.print(outstring);

      /* // moved to after population of arrays and recalculation
      x=0; y=SCREEN_HEIGHT - 2*HEADER_FONT_SIZE;
      display.setCursor(x, y);
      sprintf(outstring,"%3.1fm",wData.stdDevX);
      display.print(outstring);      

      sprintf(outstring,"%3.1fm",wData.stdDevY);
      display.getTextBounds(outstring, 0, 0, &tbx, &tby, &tbw, &tbh); // center right
      x=SCREEN_WIDTH - tbw-3; y=SCREEN_HEIGHT - 2*HEADER_FONT_SIZE;
      display.setCursor(x, y);
      display.print(outstring);
      */

      x=0; y=SCREEN_HEIGHT - HEADER_FONT_SIZE;
      display.setCursor(x, y);
      sprintf(outstring,"%3.1fm",wData.actAnchorDistanceM);
      display.print(outstring);      

      sprintf(outstring,"%3.0f*",wData.actAnchorBearingDeg);
      display.getTextBounds(outstring, 0, 0, &tbx, &tby, &tbw, &tbh); // center right
      x=SCREEN_WIDTH - tbw-3; y=SCREEN_HEIGHT - HEADER_FONT_SIZE;
      display.setCursor(x, y);
      display.print(outstring);

      x=0; y=SCREEN_HEIGHT - 1;
      display.setCursor(x, y);
      sprintf(outstring,"%6.6f %6.6f",wData.actLat,wData.actLon);
      display.print(outstring);
    } // verbosity HI
    
    if(wData.verbosity == MED){
      x=0; y=2 * HEADER_FONT_SIZE;
      display.setCursor(x, y);
      sprintf(outstring,"Sat:%d %3.2f",wData.SatCnt, wData.actHDOP);
      display.print(outstring);
      
      sprintf(outstring,"%3.1f",wData.alertCount);
      display.getTextBounds(outstring, 0, 0, &tbx, &tby, &tbw, &tbh); // center right
      x=SCREEN_WIDTH - tbw-3; y=2 * HEADER_FONT_SIZE;
      display.setCursor(x, y);
      display.print(outstring);

      x=0; y=3 * HEADER_FONT_SIZE;
      display.setCursor(x, y);
      sprintf(outstring,"%3.2fV",wData.batteryVoltage);
      display.print(outstring);

      sprintf(outstring,"%3.0f%%",wData.batteryPercent);
      display.getTextBounds(outstring, 0, 0, &tbx, &tby, &tbw, &tbh); // center right
      x=SCREEN_WIDTH - tbw-3; y=3 * HEADER_FONT_SIZE;
      display.setCursor(x, y);
      display.print(outstring);

      sprintf(outstring,"%2.0f",wData.alarmDistanceM);
      display.getTextBounds(outstring, 0, 0, &tbx, &tby, &tbw, &tbh); // center right
      x=SCREEN_WIDTH - tbw-3; y=4 * HEADER_FONT_SIZE;
      display.setCursor(x, y);
      display.print(outstring);

      x=0; y=4 * HEADER_FONT_SIZE;
      display.setCursor(x, y);
      sprintf(outstring,"%ld",wData.startCounter);
      display.print(outstring);

      x=0; y=SCREEN_HEIGHT - 1;
      display.setCursor(x, y);
      sprintf(outstring,"%4.1fm",wData.actAnchorDistanceM);
      display.print(outstring);      

      sprintf(outstring,"%3.0f*",wData.actAnchorBearingDeg);
      display.getTextBounds(outstring, 0, 0, &tbx, &tby, &tbw, &tbh); // center right
      x=SCREEN_WIDTH - tbw-3; y=SCREEN_HEIGHT - 1;
      display.setCursor(x, y);
      display.print(outstring);
    } // verbosity MED

    if(wData.verbosity == LO){
      x=0; y=2 * HEADER_FONT_SIZE;
      display.setCursor(x, y);
      sprintf(outstring,"%3.2fV",wData.batteryVoltage);
      display.print(outstring);

      sprintf(outstring,"%3.0f%%",wData.batteryPercent);
      display.getTextBounds(outstring, 0, 0, &tbx, &tby, &tbw, &tbh); // center right
      x=SCREEN_WIDTH - tbw-3; y=2 * HEADER_FONT_SIZE;
      display.setCursor(x, y);
      display.print(outstring);

      sprintf(outstring,"AC:%3.1f",wData.alertCount);
      display.getTextBounds(outstring, 0, 0, &tbx, &tby, &tbw, &tbh); // center right
      x=SCREEN_WIDTH - tbw-3; y=3 * HEADER_FONT_SIZE;
      display.setCursor(x, y);
      display.print(outstring);

      sprintf(outstring,"%2.0fm",wData.alarmDistanceM);
      display.getTextBounds(outstring, 0, 0, &tbx, &tby, &tbw, &tbh); // center right
      x=SCREEN_WIDTH - tbw-3; y=4 * HEADER_FONT_SIZE;
      display.setCursor(x, y);
      display.print(outstring);

      x=0; y=SCREEN_HEIGHT - 1;
      display.setCursor(x, y);
      sprintf(outstring,"Sat:%ld",wData.SatCnt);
      display.print(outstring);

      sprintf(outstring,"HDOP:%3.2f",wData.actHDOP);
      display.getTextBounds(outstring, 0, 0, &tbx, &tby, &tbw, &tbh); // center right
      x=SCREEN_WIDTH - tbw-3; y=SCREEN_HEIGHT - 1;
      display.setCursor(x, y);
      display.print(outstring);
    }

    // alarm area circle and center mark
    display.drawCircle(CIRCLE_CENTER_X, CIRCLE_CENTER_Y, CIRCLE_RADIUS, GxEPD_BLACK);
    display.drawFastHLine(CIRCLE_CENTER_X - CIRCLE_RADIUS/5, CIRCLE_CENTER_Y, 2 * CIRCLE_RADIUS/5, GxEPD_BLACK);
    display.drawFastVLine(CIRCLE_CENTER_X, CIRCLE_CENTER_Y - CIRCLE_RADIUS/5, 2 * CIRCLE_RADIUS/5, GxEPD_BLACK);

    // make lines bigger
    if(wData.graphWeight == HEAVY) {
      display.drawFastHLine(CIRCLE_CENTER_X - CIRCLE_RADIUS/5, CIRCLE_CENTER_Y - 1, 2 * CIRCLE_RADIUS/5, GxEPD_BLACK);
      display.drawFastVLine(CIRCLE_CENTER_X + 1, CIRCLE_CENTER_Y - CIRCLE_RADIUS/5, 2 * CIRCLE_RADIUS/5, GxEPD_BLACK);
      display.drawFastHLine(CIRCLE_CENTER_X - CIRCLE_RADIUS/5, CIRCLE_CENTER_Y + 1, 2 * CIRCLE_RADIUS/5, GxEPD_BLACK);
      display.drawFastVLine(CIRCLE_CENTER_X - 1, CIRCLE_CENTER_Y - CIRCLE_RADIUS/5, 2 * CIRCLE_RADIUS/5, GxEPD_BLACK);
    }
    
    // cardinal marks
    x=CIRCLE_CENTER_X - 5; y = CIRCLE_CENTER_Y - CIRCLE_RADIUS + HEADER_FONT_SIZE;
    display.setCursor(x, y);
    display.print("N");
    x=CIRCLE_CENTER_X - 5; y= CIRCLE_CENTER_Y + CIRCLE_RADIUS -5;
    display.setCursor(x, y);
    display.print("S");
    x=CIRCLE_CENTER_X - CIRCLE_RADIUS + HEADER_FONT_SIZE /2 - 3; y = CIRCLE_CENTER_Y + HEADER_FONT_SIZE/2 -3;
    display.setCursor(x, y);
    display.print("W");
    x=CIRCLE_CENTER_X + CIRCLE_RADIUS - HEADER_FONT_SIZE ; y= CIRCLE_CENTER_Y + HEADER_FONT_SIZE/2 - 3;
    display.setCursor(x, y);
    display.print("E");

    // draw all points (old data positions) from draw buffer
    // three pixels to improve visibility
    sprintf(outstring,"drawing buffer. drawCount: %ld", wData.drawCount);
    logOut(2, outstring);
    for(int i=0; i< ((wData.drawCount<maxDrawBufferLen)? wData.drawCount : maxDrawBufferLen); i++){
      x=wData.drawBuffer[i][0];
      y=wData.drawBuffer[i][1];
      if(x<SCREEN_WIDTH-2 && y<SCREEN_HEIGHT-2 && x>1 && y>1){ // do not draw beyond screen
        // small circle
        if(wData.graphWeight == HEAVY)
          display.fillCircle(x,y,1,GxEPD_BLACK); 
        // 3 dots - smaller
        if(wData.graphWeight == MEDIUM){
          display.drawPixel(x, y, GxEPD_BLACK); 
          display.drawPixel(1+x, y, GxEPD_BLACK); 
          display.drawPixel(x, 1+ y, GxEPD_BLACK); 
        }  
        if(wData.graphWeight == MINIM)
          display.drawPixel(x, y, GxEPD_BLACK); 
      }
    }

    // draw boat position
    float d = wData.actAnchorDistanceM * CIRCLE_RADIUS / wData.alarmDistanceM;
    float beta = wData.actAnchorBearingDeg;
    int xOff = (int)(0.5+  d * sin(beta * DEG_TO_RAD));
    int yOff = (int)(0.5+ -d * cos(beta * DEG_TO_RAD));
    x=CIRCLE_CENTER_X + xOff;
    y=CIRCLE_CENTER_Y + yOff;
    if(x<SCREEN_WIDTH-9 && y<SCREEN_HEIGHT-9 && x> 8 && y > 8){ // do not draw beyond screen
      display.fillCircle(x, y, 8, GxEPD_WHITE);
      display.fillCircle(x, y, 6, GxEPD_BLACK);
      display.fillCircle(x, y, 4, GxEPD_WHITE); 
      display.fillCircle(x, y, 2, GxEPD_BLACK); 
    }
    sprintf(outstring,"Cnt: %d d: %.1f m b: %.1f deg xOff: %d yOff: %d", 
      wData.drawCount, wData.actAnchorDistanceM, wData.actAnchorBearingDeg, xOff, yOff );
    logOut(2, outstring);

    int posX = CIRCLE_CENTER_X + xOff;;
    int posY = CIRCLE_CENTER_Y + yOff;

    // write point to draw buffer for later use, but only if not yet in buffer to save space
    boolean pointFound = false;
    for (int i=0; i< maxDrawBufferLen-1; i++){
      if(wData.drawBuffer[i][0] == posX && wData.drawBuffer[i][1] == posY){
        // point already in buffer
        pointFound = true;
        break;
      } 
      // fehler korr 1.2.26, dies raus: wData.drawBuffer[i][1] = wData.drawBuffer[i+1][1];
    }
    if(!pointFound){
      wData.drawCount++;
      int idx = (wData.drawCount-1)%maxDrawBufferLen;
      wData.drawBuffer[idx][0] = posX;
      wData.drawBuffer[idx][1] = posY;
    }
    
    // now, with buffer updated, recalculated standard deviations
    calcGraphStdDevXY();
    if(wData.verbosity==HI){ // and display them
      x=0; y=SCREEN_HEIGHT - 2*HEADER_FONT_SIZE;
      display.setCursor(x, y);
      sprintf(outstring,"%3.1fm",wData.stdDevX);
      display.print(outstring);      

      sprintf(outstring,"%3.1fm",wData.stdDevY);
      display.getTextBounds(outstring, 0, 0, &tbx, &tby, &tbw, &tbh); // center right
      x=SCREEN_WIDTH - tbw-3; y=SCREEN_HEIGHT - 2*HEADER_FONT_SIZE;
      display.setCursor(x, y);
      display.print(outstring);
    }  
  }
  while (display.nextPage());
}