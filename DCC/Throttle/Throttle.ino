#include <SPI.h>
#include "Ucglib.h"
#include "rs485c.h"
#include "xpc.h"
#include "keys.h"
#include "appstub.h"

// debugprint of soft serial?
// AltSoftSerial zal niet werken want pin8/9 in gebruik voor keys

#define XPNET_MY_ADDRESS (3)
uint8_t xpc_Address = XPNET_MY_ADDRESS; // voorlopig static

#define RS485_DIRECTION_PIN      A3     // opgelet : dit is specifiek Throttle!!  

// temp stuff from ui.cpp (CommandStation version)
#define DCC_MINSPEED 2 // 0 en 1 zijn stops, 0 = STOP, 1 = EMERGENCY STOP
#define DCC_MAXSPEED 127
// dit is volgens DCC128
#define DIRECTION_FORWARD 0x80
#define DIRECTION_REVERSE 0x0
#define DIRECTION_BIT     0x80

uint16_t ui_CurLoc = 3; // loc addr geselecteerd in UI
uint8_t ui_CurSpeed = 0 | DIRECTION_FORWARD;
uint32_t speedkeyLastMillis;
xpcFastClock_t fastClock;
bool ui_Redraw = true;
uint32_t uiRedrawLastMillis;

// display
Ucglib_ILI9341_18x240x320_HWSPI ucg(/*cd=*/ 14 , /*cs=*/ 16, /*reset=*/ 15);

static void drawClock () {
  char myclock[] = "00:00";
  uint16_t w = ucg.getStrWidth(myclock) + 20;
  ucg.setColor(0,0,100);
  ucg.drawBox(320-w,5,w,20);
  ucg.setColor(255,255,255); // terug wit
  ucg.setPrintPos(320-w+10,5);
  if (fastClock.hour < 10) ucg.print('0'); // leading zero
  ucg.print(fastClock.hour);ucg.print(':');
  if (fastClock.minute < 10) ucg.print('0'); // leading zero  
  ucg.print(fastClock.minute);
} // drawClock

static void drawSpeed() {
  ucg.setColor(0,0,0);
  ucg.drawBox(120,80,80,80);
  ucg.setColor(255,255,0);
  ucg.drawCircle(160,120,40,UCG_DRAW_ALL);
  ucg.setPrintPos(125,115);
  if (ui_CurSpeed & DIRECTION_BIT) { // forward
    ucg.print(ui_CurSpeed & 0x7F);
    ucg.print('>');
  }
  else { // reverse
    ucg.print('<');
    ucg.print(ui_CurSpeed & 0x7F);
  }
} // drawSpeed

static void drawEventText(char *msg) {
  uint16_t x,w;
  w = ucg.getStrWidth(msg);
  if (w > 320) w = 320;
  x = 160 - (w >> 1); // center text
  ucg.setColor(0,0,0);
  ucg.drawBox(0,220,319,20);
  ucg.setColor(255,255,0);
  ucg.setPrintPos(x,220);
  ucg.print(msg);
} // drawEventText

void setup_display() {
  ucg.begin(UCG_FONT_MODE_TRANSPARENT);
  ucg.setFont(ucg_font_helvB18_hr);
  ucg.setColor(0,255,255,255); // 0 = wit
  ucg.setColor(1,0,0,0); // 1 = bg, zwart
  ucg.setColor(2,255,255,0); // 2 = oranje
  ucg.setColor(3,0,0,100); // 3 = beetje blauw
  ucg.clearScreen(); // niet nodig
  ucg.setFontPosTop(); // dit heeft impact op de kaderkes in widgets.cpp
  ucg.setRotate270();

  ucg.setPrintPos(5,5);
  ucg.print("XP:"); ucg.print(xpc_Address);

  ucg.setPrintPos(5,25);
  ucg.print("LOC:3");

  // clock
  drawClock();

  // speed
  drawSpeed();

  // event
  drawEventText("start!");

} // setup_display

void ui_Update() {
  if (ui_Redraw && ((millis() - uiRedrawLastMillis) > 300)) {
    ui_Redraw = false;
    uiRedrawLastMillis = millis();
    drawSpeed();
  }
}

void setup() 
{
  keys_Init();
  setup_display();

  init_rs485(RS485_DIRECTION_PIN);
  xpc_Init(xpc_Address);

} // setup

void loop() 
{
  keys_Update();
  xpc_Run();
  ui_Update(); // test

  // todo : UI
} // loop


static bool handleSpeedKeys (key_t key)
{
  uint8_t keyCode, keyEvent;
  keyCode = key & KEYCODEFILTER;
  keyEvent = key & KEYEVENTFILTER;
  
  bool keyHandled = false;
  uint8_t speedStep, curSpeed, dirBit;
  
  // als we snel aan de knop draaien gaat de speed sneller vooruit
  if ((millis() - speedkeyLastMillis) > 50) speedStep = 1;
  else if ((millis() - speedkeyLastMillis) > 30) speedStep = 3;
  else speedStep = 8;
  speedkeyLastMillis = millis();
      
  curSpeed = ui_CurSpeed & 0x7F; // remove direction bit
  dirBit = ui_CurSpeed & DIRECTION_BIT;
  if (keyCode == KEY_ROTUP)
  {
      curSpeed += speedStep;
      if (curSpeed < DCC_MINSPEED) curSpeed = DCC_MINSPEED; // hiermee ga je van 0 naar 2
      if (curSpeed > DCC_MAXSPEED) curSpeed = DCC_MAXSPEED;
      ui_CurSpeed = curSpeed | dirBit;
      keyHandled = true;
  }
  else if (keyCode == KEY_ROTDOWN)
  {
      curSpeed -= speedStep;
      if ((curSpeed < DCC_MINSPEED) || (curSpeed > DCC_MAXSPEED)) curSpeed = 0;
      ui_CurSpeed = curSpeed | dirBit;
      keyHandled = true;
  }
  else if (keyCode == KEY_ENTER) // de switch op de rotary encoder
  {
      if (curSpeed)
      {
          ui_CurSpeed = dirBit; // zero speed, but leave direction bit
      }
      else
      {
          // enter drukken bij stilstand togglet de rijrichting
          ui_CurSpeed ^= DIRECTION_BIT; 
      }
      keyHandled = true;
  }
  if (keyHandled)
  {
    app_SetLocSpeed(ui_CurLoc,ui_CurSpeed);
    ui_Redraw = true;
  }
  return (keyHandled);
} // handleSpeedKeys

void keys_Handler (key_t key)
{
  uint8_t keyCode, keyEvent;
  bool keyHandled = false;

  keyCode = key & KEYCODEFILTER;
  keyEvent = key & KEYEVENTFILTER;
  
  // voorlopig de key-up events negeren
  if (keyEvent == KEYEVENT_UP) return;

  // draaiknop hier afhandelen (voorlopig)
  if ((keyCode == KEY_ROTUP) || (keyCode == KEY_ROTDOWN) || (keyCode == KEY_ENTER))
    keyHandled = handleSpeedKeys (key);
  if (keyHandled) return;
  
  /*
  keyHandled = ui_doHomeMenu (key);
  if (keyHandled) return;
  keyHandled = ui_doLocMenu (key);
  if (keyHandled) return;
  keyHandled = ui_doTurnoutMenu (key);
  if (keyHandled) return;
  keyHandled = ui_doAccMenu (key);
  if (keyHandled) return;
  */
  
  if (!keyHandled) {
    // whatever
  }
  
} // keys_Handler

// temp : xpc callback implementaties hier
void xpc_EventNotify( xpcEvent_t xpcEvent ) {
  char *msg = NULL;
  switch (xpcEvent) {
    case XPEVENT_POWER_ON : 
      msg = "normal operations";
      break;
    case XPEVENT_POWER_OFF :
      msg = "power off (short?)";
      break;
    case XPEVENT_LOC_STOLEN : // todo :dit komt precies ook als we zelf een loc stelen ??
      msg = "loc stolen";
      break;
    case XPEVENT_SERVICE_MODE_ON :
      msg = "service mode on";
      break;
    default:
      break;
  }
  if (msg) drawEventText(msg);
} // xpc_EventNotify

void xpc_LocStolenNotify(uint16_t stolenLocAddress) {
  drawEventText("LOC STOLEN!");
}

void xpc_FastClockNotify (xpcFastClock_t *newFastClock) {
  fastClock = *newFastClock;
  drawClock();
}
