#include "Arduino.h"
#include "Monitor.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#include <TouchScreen.h>

#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
// For the Arduino Uno, Duemilanove, Diecimila, etc.:
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7
// For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).

// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

// Color definitions
#define ILI9341_BLACK       0x0000      /*   0,   0,   0 */
#define ILI9341_NAVY        0x000F      /*   0,   0, 128 */
#define ILI9341_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define ILI9341_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define ILI9341_MAROON      0x7800      /* 128,   0,   0 */
#define ILI9341_PURPLE      0x780F      /* 128,   0, 128 */
#define ILI9341_OLIVE       0x7BE0      /* 128, 128,   0 */
#define ILI9341_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define ILI9341_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define ILI9341_BLUE        0x001F      /*   0,   0, 255 */
#define ILI9341_GREEN       0x07E0      /*   0, 255,   0 */
#define ILI9341_CYAN        0x07FF      /*   0, 255, 255 */
#define ILI9341_RED         0xF800      /* 255,   0,   0 */
#define ILI9341_MAGENTA     0xF81F      /* 255,   0, 255 */
#define ILI9341_YELLOW      0xFFE0      /* 255, 255,   0 */
#define ILI9341_WHITE       0xFFFF      /* 255, 255, 255 */
#define ILI9341_ORANGE      0xFD20      /* 255, 165,   0 */
#define ILI9341_GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define ILI9341_PINK        0xF81F

/******************* UI details */
// 480x320
// X max : 320
// Y max : 480
#define LIDAR_SIZE 200
#define BUTTON_W 200
#define BUTTON_H 150
#define BUTTON_TEXTSIZE 5



// text box where numbers go
#define TEXT_X 10
#define TEXT_Y 10
#define TEXT_W 300
#define TEXT_H 50
#define TEXT_TSIZE 3
#define TEXT_TCOLOR ILI9341_BLUE
// the data (phone #) we store in the textfield
#define TEXT_LEN 16
char textfield[TEXT_LEN+1] = "";
uint8_t textfield_i=0;

#define YP A2  // must be an analog pin, use "An" notation!
#define XM A3  // must be an analog pin, use "An" notation!
#define YM 8   // can be a digital pin
#define XP 9   // can be a digital pin

#define TS_MINX 130
#define TS_MAXX 905

#define TS_MINY 75
#define TS_MAXY 930
// We have a status line for like, is FONA working
#define STATUS_X 10
#define STATUS_Y 65



Adafruit_GFX_Button buttons[2];
/* create 15 buttons, in classic candybar phone style */
char buttonlabels[3][10] = {"Go", "Show Map", "Show Data"};
uint16_t buttoncolors[3] = {ILI9341_DARKGREEN, ILI9341_DARKGREY, ILI9341_DARKGREY};

#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
// If using the shield, all control and data lines are fixed, and
// a simpler declaration can optionally be used:
// Adafruit_TFTLCD tft;

// screen state machine

Monitor::Monitor(int state){
  _state = state;
}

void Monitor::setup(){
  tft.reset();
  uint16_t identifier = tft.readID();
  tft.begin(identifier);
  tft.setRotation(0);
  tft.fillScreen(BLACK);
  // 480x320
  buttons[0].initButton(&tft, 160, 350, BUTTON_W, BUTTON_H,
  ILI9341_WHITE, buttoncolors[0], ILI9341_WHITE, buttonlabels[0], BUTTON_TEXTSIZE); // x, y, w, h, outline, fill, text
  buttons[0].drawButton();
  buttons[1].initButton(&tft, 160, 80, BUTTON_W, BUTTON_H/2,
  ILI9341_WHITE, buttoncolors[1], ILI9341_WHITE, buttonlabels[1], 2); // x, y, w, h, outline, fill, text
  buttons[1].drawButton();
}

bool Monitor::run(int print,int X, int Y, int Z){
  if(_state == 1){
    digitalWrite(13, HIGH);
    TSPoint p = ts.getPoint();
    digitalWrite(13, LOW);
    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    tft.setCursor(146,12);
    tft.setTextColor(TEXT_TCOLOR, ILI9341_YELLOW);
    tft.setTextSize(6);
    tft.print("~~~~");
    tft.setCursor(146,12);
    tft.print(print);
    if (p.z > 10 && p.z < 1000){
      int temp = p.x;
      p.x = p.y;
      p.y = temp;
      p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());
      p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());
      if(buttons[0].contains(p.x,p.y)){
        _state = 2;
        tft.fillScreen(ILI9341_DARKGREY);
      }
      else if(buttons[1].contains(p.x, p.y)&&p.x>10){
        _state = 3;
        tft.fillScreen(ILI9341_DARKGREY);
        tft.setCursor(146,12);
        tft.setTextColor(TEXT_TCOLOR, ILI9341_DARKGREY);
        tft.setTextSize(6);
        tft.print('^');
        tft.fillCircle(160, 240, LIDAR_SIZE, BLACK);
        tft.fillCircle(160,240,  5, BLUE);
      }
    }
  }
  else if(_state == 2){
    tft.setCursor(30,100);
    tft.setTextSize(5);
    tft.setTextColor(TEXT_TCOLOR, ILI9341_DARKGREY);
    tft.print("          ");
    tft.setCursor(30,100);
    tft.print("X:");
    tft.print(X);
    tft.setCursor(30,200);
    tft.print("          ");
    tft.setCursor(30,200);
    tft.print("Y:");
    tft.print(Y);
    tft.setCursor(30,300);
    tft.print("          ");
    tft.setCursor(30,300);
    tft.print("Z:");
    tft.print(Z);
    return true;
  }
  // 480x320
  else if(_state == 3){
      // for(int i = 0; i < 180; i++){
      // tft.fillCircle(Px[i], Py[i], 1, BLACK);
      // Px[i] = ((LIDAR_SIZE-3)*x[i])/max + 160;
      // Py[i] = 240 - ((LIDAR_SIZE-3)*y[i])/max;
      // Px[i] = ((int)((double)((double)(LIDAR_SIZE-3)/(double)max)*x[i])) + 160;
      // Py[i] = 240 - ((int)((double)((double)(LIDAR_SIZE-3)/(double)max)*y[i]));
      // tft.fillCircle(Px[i], Py[i], 1, ILI9341_LIGHTGREY);
      // }
    tft.setCursor(140,50);
    tft.setTextSize(3);
    tft.setTextColor(TEXT_TCOLOR, ILI9341_DARKGREY);
    tft.setCursor(30,450);
    tft.setTextColor(TEXT_TCOLOR, ILI9341_DARKGREY);
    tft.setTextSize(3);
    tft.print("~~~~~");
    tft.setCursor(30,450);
    tft.print(print);
  }
  return false;

}

    // tft.setCursor(TEXT_X + 2, TEXT_Y+10);    //show position
    // tft.setTextColor(TEXT_TCOLOR, ILI9341_BLACK);
    // tft.setTextSize(TEXT_TSIZE);
    // tft.print(p.x);
    // tft.setCursor(TEXT_X + 2, TEXT_Y+40);
    // tft.print(p.y);
    // tft.setCursor(TEXT_X + 2, TEXT_Y+70);
    // tft.print(p.z);
