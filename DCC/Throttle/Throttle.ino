#include <SPI.h>
#include "Ucglib.h"
#include "rs485c.h"
#include "xpc.h"
#include "keys.h"

#define RS485_DIRECTION_PIN      A3     // opgelet : dit is specifiek Throttle!!
#define XPNET_MY_ADDRESS (3)
uint8_t xpc_Address = XPNET_MY_ADDRESS; // voorlopig static


#define EVENT_UI_UPDATE (EVENT_KEY_LASTEVENT + 1)

// display
#define DISPLAY_X_SIZE  320
#define DISPLAY_Y_SIZE  240

// UI refresh settings
#define DISPLAY_MANUAL_REFRESH_DELAY  200 // avoid a display refresh on every rotary key event
#define DISPLAY_AUTO_REFRESH_DELAY    500 // polling status changes to be shown on display (current, clock changes over xpnet, ...)

#define UI_MAX_LOC_ADDRESS      999 // can't show more than 3 digits loc address on display for now
#define UI_MAX_TURNOUT_ADDRESS  999 // can't show more than 3 digits turnout address on display for now

// alle ui pages
#define UISTATE_HOME_PAGE1      0
#define UISTATE_HOME_PAGE2      1 
#define UISTATE_RUN_MAIN        2
#define UISTATE_RUN_LOC_CHANGE  3
#define UISTATE_RUN_LOC_FUNCS   4
#define UISTATE_RUN_TURNOUTS    5
#define UISTATE_PROG_PAGE1      8
#define UISTATE_TEST_PAGE1      9
#define UISTATE_SETUP_PAGE1     10

#define DCC_MINSPEED 2 // 0 en 1 zijn stops, 0 = STOP, 1 = EMERGENCY STOP
#define DCC_MAXSPEED 127
// dit is volgens DCC128
#define DIRECTION_FORWARD 0x80
#define DIRECTION_REVERSE 0x0
#define DIRECTION_BIT     0x80

// state
// command station state
// TODO : CS distinguishes PROG_OFF, PROG_OK, PROG_ERROR : needed here?
// the detailed prog state will only be communicated to us if throttle is programming
// and will have to be requested with xpc_send_ServiceModeResultsRequest
typedef enum {
  UNKNOWN, RUN_OKAY, RUN_OFF, RUN_STOP, PROG
} commandStationStatus_t;

// loc state
typedef struct {
  uint8_t speed;
  uint8_t owned:1;          // loc controlled by throttle (or stolen)
  uint8_t speedChanged:1;   // flag to indicate display refresh needed
  uint8_t funcsChanged:1;   // flag to indicate display refresh needed
  uint8_t refresh:1;        // flag to force a loc data refresh over xpnet
  uint16_t address;
  uint32_t funcs;
} locBuffer_t;

// a local copy of turnout positions (1byte/decoder in xpnet accessory format)
// TURNOUTBUFFER_SIZE consecutive decoder addresses starting at firstDecoderAddress
#define TURNOUTBUFFER_SIZE          8 // keep multiple of 2, or this rubbish code won't work properly
#define TURNOUTBUFFER_REFRESH_DELAY 30000
typedef struct {
  uint8_t firstDecoderAddress; // 8-bit in xpnet
  uint8_t data[TURNOUTBUFFER_SIZE];
  uint32_t lastRefreshMillis;
} turnoutBuffer_t;

// organizing regular xpnet requests to command station
#define XPC_REQUESTS_RETRY_DELAY  500
// list of requests that throttle will systematically send to CS
typedef enum {
  IDLE, REQ_CS_STATUS, REQ_LOC_INFO, REQ_ACC_INFO
} xpcRequestState_t;

typedef struct {
  xpcRequestState_t state;
  uint32_t reqLastMillis;
  uint32_t respLastMillis;
  uint8_t idx; // multi-use
} xpcRequests_t;

// the lcd
static Ucglib_ILI9341_18x240x320_HWSPI lcd(/*cd=*/ 14 , /*cs=*/ 16, /*reset=*/ 15);

// app status
static commandStationStatus_t commandStationStatus;
static xpcFastClock_t fastClock;
static locBuffer_t curLoc; // the loc controlled by the throttle
static locBuffer_t newLoc; // a shadow structure used to signal changes, and used for changing locs in UI

static turnoutBuffer_t turnoutBuffer;
static xpcRequests_t xpcRequests;

// UI status
static uint8_t ui_State;
static bool ui_Redraw = true;
static uint32_t uiUpdateLastMillis; // controlling manual & automatic display refreshes
static uint16_t ui_NewLocAddress; // loc addr geselecteerd in UI
static bool ui_CurLocSpeedDoDelayedRefresh = true;
static uint8_t curStartFunc = 0;
static uint8_t curHighlightFunc = 0;
static uint16_t curStartTurnout = 0; // turnouts counted from 0, but displayed from 1 (like JMRI)
static uint16_t curHighlightTurnout = 0;
static uint16_t curTurnoutPositions = 0xFFFF; // poll turnout positions, but only update screen if a turnout changed position (xpnet)
static bool turnoutPositionsChanged = true; // flag to trigger display update TODO : needs better solution
static uint32_t speedkeyLastMillis;

// ui fixed text in progmem
// TODO : betere oplossing dan spaties in flash te stockeren om de tekst te laten passen met de keys
static const char navHomePage1[] PROGMEM    = "main   pwr        test      >";
static const char navHomePage2[] PROGMEM    = "prog  setup        >";
static const char navRunMain[] PROGMEM      = "menu    fx         loc    acc";
static const char navRunLocChange[] PROGMEM = "back    <          >       OK";
static const char navRunLocFuncOrTurnoutChange[] PROGMEM = "back    <          >    toggle";
static const char navTest[] PROGMEM         = "back  sig1        sig2  DB-TX";
static const char navPowerPage[] PROGMEM    = "back  main        prog";
static const char navDummy[] PROGMEM        = "back";
static const char defaultLocName[] PROGMEM  = "[no name] ";
static const char evtLocStolenText[] PROGMEM          = "Loc Stolen! ";
static const char evtMainTrackOkText[] PROGMEM        = "Main OK!    ";
static const char evtMainEmergencyStopText[] PROGMEM  = "Main STOP!  ";
static const char evtTracksOffText[] PROGMEM          = "Tracks OFF! ";
static const char evtProgTrackOkText[] PROGMEM        = "Progr. Mode!";
static const char evtProgTrackShortText[] PROGMEM     = "Prog Short! ";
static const char evtProgErrorText[] PROGMEM          = "Prog Error! ";
static const char evtConnectionErrorText[] PROGMEM    = "xpnet connection error";
static const char evtConnectionOkText[] PROGMEM       = "xpnet OK!";
static const char mnuPowerHelpText[] PROGMEM          = "switch tracks on/off";

// incoming events (key+ui update events) are first passed to the active menu handler
// unhandled keys are then handled by the main key handler or discarded (only 1 active menu at the time)
static bool (*ui_ActiveMenuHandler)(uint8_t event, uint8_t code);
static bool ui_HomeMenuHandler (uint8_t event, uint8_t code);
static bool ui_RunMenuHandler (uint8_t event, uint8_t code);
static bool ui_PowerMenuHandler (uint8_t event, uint8_t code);
static bool ui_TestMenuHandler (uint8_t event, uint8_t code);
//static bool ui_SetupMenuHandler (uint8_t event, uint8_t code);
//static bool ui_ProgMenuHandler (uint8_t event, uint8_t code);
//static bool ui_EventHandler (uint8_t event, uint8_t code);
static bool ui_LocSpeedHandler (uint8_t event, uint8_t code); // generic loc speed handling with rotary key, used by all menus that don't use the rotary key differently

/*****************************************************************************/
/*    HELPER FUNCTIONS FOR LOCAL TURNOUT BUFFER                              */
/*****************************************************************************/
static uint8_t turnout_GetStatus (uint16_t turnoutAddress) {
  uint8_t decoderAddress;
  uint8_t turnoutState;
  uint8_t bitPos;

  decoderAddress = turnoutAddress >> 2;
  if ((decoderAddress < turnoutBuffer.firstDecoderAddress) ||
      (decoderAddress >= (turnoutBuffer.firstDecoderAddress + TURNOUTBUFFER_SIZE))) {
    // don't have data in the turnoutBuffer to show
    return TURNOUT_STATE_INVALID;
  }
  bitPos = (turnoutAddress & 0x3) << 1;
  turnoutState = turnoutBuffer.data[decoderAddress - turnoutBuffer.firstDecoderAddress]; // 4 turnouts
  turnoutState = (turnoutState >> bitPos) & 0x3; // i & 0x3 : turnoutId 0,1,2,3 in the decoderAddress
  return turnoutState;
} // turnout_GetStatus

// update turnoutbuffer with this status
// return true if status has changed
static bool turnout_SetStatus (uint16_t turnoutAddress, uint8_t turnoutStatus) {
  uint8_t decoderAddress;
  uint8_t bitPos, mask;
  uint8_t curData, newData;
  bool retval = false;
  decoderAddress = turnoutAddress >> 2;
  // turnoutAddress not in turnoutBuffer -> shouldn't happen
  if ((decoderAddress < turnoutBuffer.firstDecoderAddress) ||
      (decoderAddress >= (turnoutBuffer.firstDecoderAddress + TURNOUTBUFFER_SIZE))) {
    return false;
  }

  bitPos = (turnoutAddress & 0x3) << 1;
  mask = ~(0x3 << bitPos);
  curData = turnoutBuffer.data[decoderAddress - turnoutBuffer.firstDecoderAddress]; // current byte
  newData = (curData & mask) | (turnoutStatus << bitPos);
  if (newData != curData) retval = true;
  turnoutBuffer.data[decoderAddress - turnoutBuffer.firstDecoderAddress] = newData;

  return retval;
} //turnout_SetStatus

// clear data in turnout buffer
static void turnout_ClearBuffer () {
  for (uint8_t i=0;i<TURNOUTBUFFER_SIZE;i++)
    turnoutBuffer.data[i] = 0x00;
} // turnout_ResetBuffer

static void turnout_TriggerBufferRefresh() {
  turnoutBuffer.lastRefreshMillis = millis() - TURNOUTBUFFER_REFRESH_DELAY;
} // turnout_TriggerBufferRefresh

/*****************************************************************************/
/*    HELPER FUNCTIONS FOR DISPLAY WRITING                                   */
/*****************************************************************************/
static void display_Init() {
  lcd.begin(UCG_FONT_MODE_TRANSPARENT); // UCG_FONT_MODE_SOLID
  lcd.setFont(ucg_font_helvB18_hr);
  lcd.setColor(0,255,255,255); // 0 = wit
  lcd.setColor(1,0,0,0); // 1 = bg, zwart
  lcd.clearScreen(); // niet nodig
  lcd.setFontPosTop();
  lcd.setRotate270();
} // display_Init

// graphic lcd : line = yPos (0..240), clear is done by drawing a black box of ySize=20
static void clearLine (uint8_t line, uint8_t startPos=0) {
  lcd.setColor(0,0,0);
  lcd.drawBox(startPos,line,DISPLAY_X_SIZE-startPos,25);
  lcd.setColor(255,255,255);
} // clearLine

// current loc speed in a yellow circle
static void ui_ShowLocSpeed(uint8_t dccSpeed) {
  uint8_t dccDirectionForward = dccSpeed & DIRECTION_FORWARD;
  lcd.setColor(0,0,0);
  lcd.drawBox(120,80,80,80);
  lcd.setColor(255,255,0);
  lcd.drawCircle(160,120,40,UCG_DRAW_ALL);
  lcd.setPrintPos(125,113);

  if (!dccDirectionForward) {
    lcd.write('<');
  }
  dccSpeed = dccSpeed & 0x7F;
  if (dccSpeed < 100) lcd.write('0');
  if (dccSpeed < 10) lcd.write('0');
  lcd.print(dccSpeed);
  if (dccDirectionForward) {
      lcd.write('>');
  }
  lcd.setColor(255,255,255);
} // ui_ShowLocSpeed

// op lijn 1, teken 12-15, of 10-15 (long addr)
// 4 digits, default position (12,0)->(15,0) : "aaa:" with leading zeroes
// TODO : not enough screen space to show locAddress > 999 (UI_MAX_LOC_ADDRESS)
static void ui_ShowLocAddress (uint16_t locAddress, uint8_t xPos=5, uint8_t yPos=30) {
  lcd.setPrintPos(xPos, yPos);
  lcd.print("LOC:");
  if (locAddress > UI_MAX_LOC_ADDRESS) { // long addr
    locAddress = 0; // show an invalid address for now
  }
  if (locAddress < 100) lcd.write('0');
  if (locAddress < 10) lcd.write('0');
  lcd.print(locAddress);
  lcd.write(':');
} // ui_ShowLocAddress

// shows locname at current cursor position, max len characters
// unused characters are filled with spaces
static void ui_ShowLocName(uint8_t *locName, uint8_t len) {
  uint8_t i = 0;
  while (locName[i] && (i < len)) {
    lcd.write(locName[i]);
    i++;
  }
  
  while (i < len) {
    lcd.write(' ');
    i++;
  }
} // ui_ShowLocName

// show 8 funcs allFuncs[startFunc,startFunc+7], highlight allFuncs[highlightFunc] or no highlight if highlightFunc outside [startFunc,startFunc+7] (eg. 0xRFF)
// display position: (180,30)
// ofwel SOLID fontmode gebruiken, ofwel eerst de box clearen, anders overschrijven de letters elkaar
static void ui_ShowLocFuncs (uint32_t allFuncs, uint8_t startFunc, uint8_t highlightFunc) { // startFunc : multiple of 8
  uint8_t cur8Funcs;
  startFunc = startFunc & 0xF8; // multiple of 8
  cur8Funcs = (allFuncs >> startFunc) & 0xFF;
  lcd.setColor(0,0,0);
  lcd.drawBox(180,30,319-180,25); // clear the drawing area
  lcd.setColor(255,255,255);
  lcd.setPrintPos(180,30);
  for (uint8_t func=0;func<8;func++) {
    uint8_t funcActive = (cur8Funcs >> func) & 0x1;
    if ((startFunc+func) == highlightFunc) {
      if (funcActive) lcd.write('X');
      else lcd.write('O');
    }
    else {
      if (funcActive) lcd.write('x');
      else lcd.write('o');
    }
  }
} // ui_ShowLocFuncs

// show 8 turnouts [startTurnout,startTurnout+7], highlight [startTurnout,startTurnout+7] or 0xFFFF for no highlight
// display position: (12,2)->(19,2)
static void ui_ShowTurnouts (uint16_t startTurnout, uint16_t highlightTurnout) { // startTurnout : multiple of 8
  uint8_t tbIdx;
  uint8_t decoderAddress; 
  startTurnout = startTurnout & 0xF8; // multiple of 8
  // turnoutBuffer stores data for decoders [turnoutBuffer.firstDecoderAddress, turnoutBuffer.firstDecoderAddress + TURNOUTBUFFER_SIZE -1]
  decoderAddress = startTurnout >> 2;
  if ((decoderAddress < turnoutBuffer.firstDecoderAddress) ||
      (decoderAddress >= (turnoutBuffer.firstDecoderAddress + TURNOUTBUFFER_SIZE))) {
    // don't have data in the turnoutBuffer to show -> shouldn't happen
    return; 
  }

  // now show data for decoderAddress & decoderAddress+1
  lcd.setColor(0,0,0);
  lcd.drawBox(180,55,319-180,25); // clear the drawing area
  lcd.setColor(255,255,255);
  lcd.setPrintPos(180,55);

  for (uint8_t i=0;i<8;i++) {
    uint8_t turnoutState;
    turnoutState = turnout_GetStatus(startTurnout+i);
    if ((startTurnout+i) == highlightTurnout) {
      if (turnoutState == TURNOUT_STATE_CLOSED) lcd.write('C');
      else if (turnoutState == TURNOUT_STATE_THROWN) lcd.write('T');
      else lcd.write('?');
    }
    else {
      if (turnoutState == TURNOUT_STATE_CLOSED) lcd.write('c');
      else if (turnoutState == TURNOUT_STATE_THROWN) lcd.write('t');
      else lcd.write('.');
    }
  }
} // ui_ShowTurnouts

static void ui_ShowClock () {
  char myclock[] = "00:00";
  uint16_t w = lcd.getStrWidth(myclock) + 20;

  lcd.setFontMode(UCG_FONT_MODE_SOLID);
  lcd.setColor(1,0,0,100);
  lcd.setPrintPos(320-w+10,5);
  if (fastClock.hour < 10) lcd.write('0'); // leading zero
  lcd.print(fastClock.hour);lcd.write(':');
  if (fastClock.minute < 10) lcd.write('0'); // leading zero  
  lcd.print(fastClock.minute);
  lcd.setColor(1,0,0,0);
  lcd.setFontMode(UCG_FONT_MODE_TRANSPARENT);
} // ui_ShowClock

// TODO : voorlopig enkel te gebruiken met flash strings
static void ui_ShowEventText(const char *eventText) {
  // TODO : getStrWidth werkt niet met flash strings
  /*
  uint16_t x,w;
  w = lcd.getStrWidth(eventText); 
  if (w > 320) w = 320;
  x = 160 - (w >> 1); // center text
  */
  lcd.setColor(0,0,0);
  lcd.drawBox(0,190,319,25); // TODO : w=20 leek niet genoeg
  lcd.setColor(255,127,0);
  //lcd.setPrintPos(x,190);
  lcd.setPrintPos(0,190);
  lcd.print((__FlashStringHelper*)eventText);
  lcd.setColor(255,255,255);
} // ui_ShowEventText

// navbar bottom screen in a lightblue background
// TODO : align text with buttons
static void ui_ShowNavText (const char *navText) {
  lcd.setColor(0,0,255);
  lcd.drawBox(0,215,319,25); // TODO : w=20 leek niet genoeg
  lcd.setColor(0,255,255,0);
  //lcd.setColor(1,0,0,255);
  lcd.setPrintPos(0,215);
  lcd.print((__FlashStringHelper*)navText);
  //lcd.setColor(1,0,0,0);
} // ui_ShowNavText

// show on top line in center, white text on green/red/yellow background
// TODO : pictos ?
static void ui_ShowCsStatus() {
  const char *txt;
  lcd.setPrintPos(80,5); // TODO : fixed x for now, text not centered
  switch (commandStationStatus) {
    case RUN_OKAY:
      lcd.setColor(0,100,0);
      txt = "CS:OK";
      break;
    case RUN_STOP:
      lcd.setColor(255,0,0);
      txt = "CS:STOP";
      break;
    case RUN_OFF:
      lcd.setColor(255,0,0);
      txt = "CS:OFF";
      break;
    case PROG:
      lcd.setColor(100,100,0);
      txt = "CS:PROG";
      break;
    case UNKNOWN:
      lcd.setColor(100,100,0);
      txt = "CS:????";
      break;
  }
  lcd.drawBox(80,5,120,25); // 120 = experimental 
  lcd.setColor(255,255,255);
  lcd.print(txt);
} // ui_ShowCsStatus

/*****************************************************************************/
/*    HELPER FUNCTIONS FOR INTERFACING WITH COMMAND STATION                  */
/*****************************************************************************/
static void ui_SetLocSpeed (uint16_t locAddress, uint8_t locSpeed) {
  if (locSpeed != curLoc.speed) {
    curLoc.speed = locSpeed;
    curLoc.owned = 1; // steal the loc
    xpc_send_LocSetSpeedRequest(locAddress,locSpeed);
    // we krijgen geen feedback van xpnet, dus we gaan ervan uit dat dit is gelukt
  }
} // ui_SetLocSpeed

// 1 bit tegelijk, we kunnen toch maar 1 bit tegelijk togglen in de UI
// func 0 = light
// f1 ->f28 
// on = 1, off = 0
// retval = APP_xxx constant
void ui_SetLocFunction (uint16_t locAddress, uint8_t func, uint32_t allFuncs) {
  uint8_t funcByte, funcGroup;

  if (func > 28) return;

  curLoc.owned = 1; // steal the loc
  if ((func >= 0) && (func <=4)) {
    funcGroup = 1;
    funcByte = (uint8_t) (allFuncs & 0x1F);
    funcByte = ((funcByte & 0x1) << 4) | ((funcByte & 0x1E) >> 1);
  }
  else if ((func >= 5) && (func <=8)) {
    funcGroup = 2;
    funcByte = (uint8_t) ((allFuncs >> 5) & 0x0F);
  }
  else if ((func >= 9) && (func <=12)) {
    funcGroup = 3;
    funcByte = (uint8_t) ((allFuncs >> 9) & 0x0F);
  }
  else if ((func >= 13) && (func <=20)) {
    funcGroup = 4;
    funcByte = (uint8_t) ((allFuncs >> 13) & 0xFF);
  }
  else if ((func >= 21) && (func <=28)) {
    funcGroup = 5;
    funcByte = (uint8_t) ((allFuncs >> 21) & 0xFF);
  }               
  xpc_send_LocSetFuncRequest(locAddress,funcGroup, funcByte);
} // ui_SetLocFunction

static void ui_ToggleTurnout (uint16_t turnoutAddress) {
  uint8_t turnoutPosition = 0; // 0/1  = coil
  uint8_t turnoutStatus;

  turnoutStatus = turnout_GetStatus(turnoutAddress);
  if (turnoutStatus == TURNOUT_STATE_CLOSED) {
    turnoutStatus = TURNOUT_STATE_THROWN;
    turnoutPosition = 1;
  }
  else // THROWN, UNKNOWN, INVALID -> CLOSED
    turnoutStatus = TURNOUT_STATE_CLOSED;
  turnout_SetStatus(turnoutAddress, turnoutStatus); // update local turnout buffer
  xpc_send_SetTurnoutRequest(turnoutAddress, turnoutPosition); // send command to CS

} // ui_ToggleTurnout

static void ui_SetExtendedAccessory (uint16_t decoderAddress, uint8_t signalId, uint8_t signalAspect) {
  // pass straight on to xpnet
  xpc_send_SetSignalAspectRequest(decoderAddress,signalId,signalAspect);
} // ui_SetExtendedAccessory

/*****************************************************************************/
/*    UI PAGE HANDLERS                                                       */
/*****************************************************************************/
static bool ui_HomeMenuHandler (uint8_t event, uint8_t code) {
  uint8_t keyCode;

  // ui events
  if ((event == EVENT_UI_UPDATE) && code) { // do only manual refresh
    clearLine(30);
    clearLine(55);
    clearLine(215);
    if (ui_State == UISTATE_HOME_PAGE1) ui_ShowNavText(navHomePage1);
    else if (ui_State == UISTATE_HOME_PAGE2) ui_ShowNavText(navHomePage2);
    return true;
  }

  // handle key events
  keyCode = code;

  // don't handle key up/longdown & rotary key
  if ((keyCode == KEY_ROTARY) || (keyCode == KEY_ENTER) ||
      (event == EVENT_KEY_UP) || (event == EVENT_KEY_LONGDOWN))
    return false;

  if (ui_State == UISTATE_HOME_PAGE1) {
    if (keyCode == KEY_1) {
      ui_State = UISTATE_RUN_MAIN;
      ui_ActiveMenuHandler = ui_RunMenuHandler;
    }
    else if (keyCode == KEY_2) {
      ui_ActiveMenuHandler = ui_PowerMenuHandler;
    }
    else if (keyCode == KEY_3) {
      ui_State = UISTATE_TEST_PAGE1;
      ui_ActiveMenuHandler = ui_TestMenuHandler;
    }
    else if (keyCode == KEY_4) ui_State = UISTATE_HOME_PAGE2;
  }
  else if (ui_State == UISTATE_HOME_PAGE2)  {
    if (keyCode == KEY_1) {
      ui_State = UISTATE_PROG_PAGE1;
      ui_ActiveMenuHandler = ui_TestMenuHandler;
      //ui_ActiveMenuHandler = ui_ProgMenuHandler; 
    }
    else if (keyCode == KEY_2) {
      ui_State = UISTATE_SETUP_PAGE1;
      ui_ActiveMenuHandler = ui_TestMenuHandler;
      //ui_ActiveMenuHandler = ui_SetupMenuHandler;
    }
    else ui_State = UISTATE_HOME_PAGE1;
  }
  return (true);
} // ui_HomeMenuHandler

bool ui_RunMenuHandler (uint8_t event, uint8_t code) {
  uint8_t keyCode;
  if (event == EVENT_UI_UPDATE) {
    // 1. auto+manual refresh
    // update loc speed/functions -> if loc stolen, update on every auto-refresh to keep display in sync with external control
    if (curLoc.speedChanged || code || ui_CurLocSpeedDoDelayedRefresh) {
      ui_ShowLocSpeed(curLoc.speed);
      ui_CurLocSpeedDoDelayedRefresh = false;
      curLoc.speedChanged = false;
    }

    // 2. auto refresh only
    if (!code) {
      if (ui_State != UISTATE_RUN_LOC_CHANGE) {
        if (!curLoc.owned) { // our loc is stolen, poll its status at auto-refresh interval
          newLoc.address = curLoc.address; // need the newLoc shadow structure for that, because that's where the response data are stored
          newLoc.refresh = 1;
        }
      }

      if (curLoc.funcsChanged) {
        ui_ShowLocFuncs(curLoc.funcs,curStartFunc,0xFF); // no function highlighted
        curLoc.funcsChanged = false;
      }

      if (turnoutPositionsChanged) {
        ui_ShowTurnouts(curStartTurnout, 0xFFFF); // no highlights
        turnoutPositionsChanged = false;
      }

      // TODO other data to auto-refresh (from xpnet eg.)?
      return true; // auto-refresh is completed here
    }
    // 3. manual refresh only
    if (ui_State == UISTATE_RUN_MAIN) {
      ui_ShowLocFuncs(curLoc.funcs,curStartFunc,0xFF); // no function highlighted
      ui_ShowTurnouts(curStartTurnout,0xFFFF); // no turnouts highlighted
      ui_ShowNavText(navRunMain);
    }
    else if (ui_State == UISTATE_RUN_LOC_FUNCS) {
      //lcd.setCursor(8,1);
      //lcd.write('F');lcd.print(curHighlightFunc);lcd.write(':');
      ui_ShowLocFuncs(curLoc.funcs,curStartFunc,curHighlightFunc);
      ui_ShowTurnouts(curStartTurnout,0xFFFF); // no turnouts highlighted
      ui_ShowNavText(navRunLocFuncOrTurnoutChange);
    }
    else if (ui_State == UISTATE_RUN_TURNOUTS) {
      ui_ShowLocFuncs(curLoc.funcs,curStartFunc,0xFF); // no function highlighted
      //clearLine(2); // remove notification text
      //lcd.setCursor(7,2);
      //lcd.write('W');lcd.print(curHighlightTurnout+1);lcd.write(':');
      ui_ShowTurnouts(curStartTurnout,curHighlightTurnout);
      ui_ShowNavText(navRunLocFuncOrTurnoutChange);
    }
    else if (ui_State == UISTATE_RUN_LOC_CHANGE) {
      // TODO complete
      ui_ShowNavText(navRunLocChange);
    }
    return true;
  }

  // 2. handle key events
  keyCode = code;

  // don't handle key up/longdown & rotary key
  if ((keyCode == KEY_ROTARY) || (keyCode == KEY_ENTER) ||
      (event == EVENT_KEY_UP) || (event == EVENT_KEY_LONGDOWN))
    return false;

  if (ui_State == UISTATE_RUN_MAIN) {
    if (keyCode == KEY_1) { // menu
      ui_State = UISTATE_HOME_PAGE1;
      ui_ActiveMenuHandler = ui_HomeMenuHandler;
    }
    else if (keyCode == KEY_2) ui_State = UISTATE_RUN_LOC_FUNCS;
    else if (keyCode == KEY_3) ui_State = UISTATE_RUN_LOC_CHANGE;
    else if (keyCode == KEY_4) ui_State = UISTATE_RUN_TURNOUTS;
  }
  else if (ui_State == UISTATE_RUN_LOC_FUNCS) {
    if (keyCode == KEY_1) ui_State = UISTATE_RUN_MAIN; //back
    else if (keyCode == KEY_2) { // highlight prev func
      curHighlightFunc-= 1;
      if (curHighlightFunc > 27) curHighlightFunc = 27;
      curStartFunc = curHighlightFunc & 0xF8;
    }
    else if (keyCode == KEY_3) { // highlight next func
      curHighlightFunc+= 1;
      if (curHighlightFunc > 27) curHighlightFunc = 0;
      curStartFunc = curHighlightFunc & 0xF8;
    }
    else if (keyCode == KEY_4) { // toggle function
      curLoc.funcs ^= ((uint32_t) 0x1 << curHighlightFunc); // toggle func bit
      ui_SetLocFunction(curLoc.address,curHighlightFunc,curLoc.funcs);
    }
  }

  else if (ui_State == UISTATE_RUN_LOC_CHANGE) {
    if (keyCode == KEY_1) { // back
      ui_State = UISTATE_RUN_MAIN;
    }
    else if (keyCode == KEY_2) { // prev
      if (ui_NewLocAddress > 1)
        ui_NewLocAddress -= 1;
    }
    else if (keyCode == KEY_3){ // next
      if (ui_NewLocAddress < UI_MAX_LOC_ADDRESS) // can't show more than 3 digits loc address on display for now
        ui_NewLocAddress += 1;
    }
    else if (keyCode == KEY_4) { // OK
      // bevestig de nieuwe loc selectie
      //TODO lb_ReleaseLoc(curLoc.address);
      curLoc.address = ui_NewLocAddress;
      curStartFunc = 0;
      curHighlightFunc = 0;
      ui_State = UISTATE_RUN_MAIN;
    }  
  }

  else if (ui_State == UISTATE_RUN_TURNOUTS) {
    if (keyCode == KEY_1) ui_State = UISTATE_RUN_MAIN; //back
    else if (keyCode == KEY_2) { // highlight prev turnout
      curHighlightTurnout-= 1;
      if (curHighlightTurnout > UI_MAX_TURNOUT_ADDRESS) curHighlightTurnout = UI_MAX_TURNOUT_ADDRESS;
      curStartTurnout = curHighlightTurnout & 0xFFF8; // per 8
    }
    else if (keyCode == KEY_3) { // highlight next func
      curHighlightTurnout+= 1;
      if (curHighlightTurnout > UI_MAX_TURNOUT_ADDRESS) curHighlightTurnout = 0;
      curStartTurnout = curHighlightTurnout & 0xF8;
    }
    else if (keyCode == KEY_4) { // toggle function
      // TODO : activate true/false met DOWN/UP event
      ui_ToggleTurnout (curHighlightTurnout);
    }
    if (turnout_GetStatus(curStartTurnout) == TURNOUT_STATE_INVALID) {
      // need to update turnout buffer with a new firstDecoderAddress
      turnout_ClearBuffer();
      turnoutBuffer.firstDecoderAddress = (curStartTurnout>>2) - ((curStartTurnout>>2) % TURNOUTBUFFER_SIZE); // biggest multiple of TURNOUTBUFFER_SIZE smaller than decoderAddress
      turnout_TriggerBufferRefresh();
    }
  }
  return true;
} // ui_RunMenuHandler

// switch on/off power to main/prog tracks 
static bool ui_PowerMenuHandler (uint8_t event, uint8_t code) {
  uint8_t keyCode;
  bool keyHandled = false;

  // ui events
  if ((event == EVENT_UI_UPDATE) && code) { // do only manual refresh
    clearLine(30);
    clearLine(55);
    clearLine(215);
    lcd.setColor(255,255,255); // wit
    lcd.setPrintPos(0,30);
    lcd.print ((__FlashStringHelper*)mnuPowerHelpText);
    ui_ShowCsStatus();
    ui_ShowNavText(navPowerPage);
    return true;
  }

  // handle key events
  keyCode = code;

  // don't handle key up/longdown & rotary key
  if ((keyCode == KEY_ROTARY) || (keyCode == KEY_ENTER) ||
      (event == EVENT_KEY_UP) || (event == EVENT_KEY_LONGDOWN))
    return false;

  if (keyCode == KEY_1) {
    ui_State = UISTATE_HOME_PAGE1;
    ui_ActiveMenuHandler = ui_HomeMenuHandler;
    keyHandled = true;
  }
  // toggle main track
  else if (keyCode == KEY_2) { // toggle main track
    if (commandStationStatus == RUN_OKAY) xpc_send_PowerOffRequest();
    else xpc_send_PowerOnRequest();
    // TODO bizarre stuff here
    // we keep keyhandled = false and avoid the screen refresh here 
    // otherwise we seem to be missing the xpnet broadcasts that follow immediately after indicating the status change (power on/off)
    // missing uart interrupts? does this mean that ucglib disables interrupts for a short time???
  }
  else if (keyCode == KEY_3) { // toggle prog track
    // TODO : there is no simple xpnet command, but we could initiate a dummy CV read/write to trigger service mode?
  }
  return (keyHandled);
} // ui_PowerMenuHandler

// a dummy menu for now
static bool ui_TestMenuHandler (uint8_t event, uint8_t code) {
  uint8_t keyCode;

  // ui events
  if ((event == EVENT_UI_UPDATE) && code) { // do only manual refresh
    clearLine(30);
    lcd.setColor(255,255,255); // wit
    lcd.setPrintPos(0,30);
    lcd.print ("menu not implemented");

    ui_ShowNavText (navTest);
    return true;
  }

  // handle key events
  keyCode = code;

  // don't handle key up/longdown & rotary key
  if ((keyCode == KEY_ROTARY) || (keyCode == KEY_ENTER) ||
      (event == EVENT_KEY_UP) || (event == EVENT_KEY_LONGDOWN))
    return false;

  // any key returns to home menu
  ui_State = UISTATE_HOME_PAGE1;
  ui_ActiveMenuHandler = ui_HomeMenuHandler;
  return (true);
} // ui_TestMenuHandler

// updates the current loc speed
static bool ui_LocSpeedHandler (uint8_t keyEvent, uint8_t keyCode) {
  bool keyHandled = false;
  uint8_t curSpeed = 0 | DIRECTION_FORWARD;
  uint8_t speedStep, dirBit;

  // ignore key up/longdown, ignore keypad keys
  if ((keyEvent == EVENT_KEY_UP) || (keyEvent == EVENT_KEY_LONGDOWN) ||
      ((keyCode != KEY_ROTARY) && (keyCode != KEY_ENTER)))
    return false;
  
  // als we snel aan de knop draaien gaat de speed sneller vooruit
  if ((millis() - speedkeyLastMillis) > 50) speedStep = 1;
  else if ((millis() - speedkeyLastMillis) > 30) speedStep = 3;
  else speedStep = 8;

  curSpeed = curLoc.speed;
      
  dirBit = curSpeed & DIRECTION_BIT;
  curSpeed = curSpeed & 0x7F; // remove direction bit
  if (keyEvent == EVENT_ROTARY_UP) {
    curSpeed += speedStep;
    if (curSpeed < DCC_MINSPEED) curSpeed = DCC_MINSPEED; // hiermee ga je van 0 naar 2
    if (curSpeed > DCC_MAXSPEED) curSpeed = DCC_MAXSPEED;
    curSpeed = curSpeed | dirBit; // append direction bit
    keyHandled = true;
  }
  else if (keyEvent == EVENT_ROTARY_DOWN) {
    curSpeed -= speedStep;
    if ((curSpeed < DCC_MINSPEED) || (curSpeed > DCC_MAXSPEED)) curSpeed = 0;
    curSpeed = curSpeed | dirBit; // append direction bit
    keyHandled = true;
  }
  else if (keyCode == KEY_ENTER) { // de switch op de rotary encoder
    if (curSpeed) curSpeed = dirBit; // zero speed, but leave direction bit (TODO : or do emergency stop?)
    else { // enter drukken bij stilstand togglet de rijrichting
      curSpeed = dirBit ^ DIRECTION_BIT; 
    }
    keyHandled = true;
  }
  if (keyHandled) {
    ui_SetLocSpeed(curLoc.address,curSpeed);
    if ((millis() - speedkeyLastMillis) > DISPLAY_MANUAL_REFRESH_DELAY) { // vermijden dat bij elke rot-key een refresh gebeurt, want dan werkt de speedup feature niet
      ui_CurLocSpeedDoDelayedRefresh = false;
      ui_ShowLocSpeed (curSpeed); // don't do ui_Redraw=true, to avoid a complete display redraw on every speed change
      // klopt dat bij alle menus? (dwz als menus de rotkey niet afhandelen accepteren ze ook dat locspeed wordt getoond op een fixed location)
    }
    else { 
      ui_CurLocSpeedDoDelayedRefresh = true;
    }
    speedkeyLastMillis = millis();
  }
  return (keyHandled);
} // ui_LocSpeedHandler

void ui_Init() {
  display_Init(); // incl font mode, fg & bg color etc

  ui_State = UISTATE_RUN_MAIN;
  ui_Redraw = true; // TODO : waarom niet nodig in CS?
  ui_ActiveMenuHandler = ui_RunMenuHandler;

  lcd.setPrintPos(5,5);
  lcd.print("XP:"); lcd.print(xpc_Address);

  lcd.setPrintPos(5,30);
  lcd.print("LOC:3");

  ui_ShowClock(); // clock
  ui_ShowCsStatus();
} // ui_Init

void ui_Update() {
  // TODO : does the display have backlight?
  /*
  if (backlightOn && ((millis() - triggerBacklightLastMillis) > BACKLIGHTOFF_DELAY)) {
    lcd.setBacklight(0);
    backlightOn = false;
  }
  */

  // too early for a display refresh
  if ((!ui_Redraw) && ((millis() - uiUpdateLastMillis) < DISPLAY_MANUAL_REFRESH_DELAY))
    return;
  
  // do the auto-refresh part
  if ((ui_Redraw) || ((millis() - uiUpdateLastMillis) > DISPLAY_AUTO_REFRESH_DELAY)) {
    ui_ActiveMenuHandler(EVENT_UI_UPDATE,(uint8_t) ui_Redraw); // handler can decide if it performs auto-refresh or not

    uiUpdateLastMillis = millis();
    ui_Redraw = false;
  }
} // ui_Update

// all key events arrive here first
void keys_Handler (uint8_t keyEvent, uint8_t keyCode) {
  bool keyHandled = false;

  // TODO THROTTLE : hebben we dit?
  //triggerBacklight();

  // menu handles the key
  keyHandled = ui_ActiveMenuHandler (keyEvent, keyCode);
  if (keyHandled) { // for simplicity each handled key triggers a display refresh (ie. also if UP/LONGDOWN are handled!)
    ui_Redraw = true;
    return;
  } 

  // if rotary key is not yet handled by a menu, we use it as a loc speed dial
  if ((keyCode == KEY_ROTARY) || (keyCode == KEY_ENTER)) {
    keyHandled = ui_LocSpeedHandler (keyEvent, keyCode);
    // note : don't ui_Redraw here to avoid a complete display redraw on every speed change, 
    // ui_LocSpeedHandler updates its part of the display
  }

  // if we come here, key event is not handled
} // keys_Handler

/*****************************************************************************/
/*    XPC CALLBACKS                                                          */
/*****************************************************************************/
/*
void xpc_MessageNotify (uint8_t *msg) {
  // TODO REMOVE
  if (msg[0] == 0xE3) {
    lcd.setColor(0,0,0);
    lcd.drawBox(0,95,150,25);
    lcd.setColor(255,255,255);
    lcd.setPrintPos(0,95);
    for (uint8_t i=0;i<4;i++) {
      lcd.print(msg[i],HEX);
      lcd.write(' ');
    }
  }
}
*/

void xpc_EventNotify( xpcEvent_t xpcEvent ) {
  const char *msg = NULL;
  // TODO REMOVE
  /*
  lcd.setColor(0,0,0);
  lcd.drawBox(0,120,25,25);
  lcd.setColor(255,255,255);
  lcd.setPrintPos(0,120);
  lcd.print(xpcEvent);
  */

  if (xpcEvent == XPEVENT_CONNECTION_ERROR) msg = evtConnectionErrorText;
  else if (xpcEvent == XPEVENT_CONNECTION_OK) msg = evtConnectionOkText;
  // XPEVENT_RX_TIMEOUT, XPEVENT_RX_ERROR, XPEVENT_MSG_ERROR, XPEVENT_TX_ERROR 
  // TODO : also notify as xpnet error?

  if (msg) ui_ShowEventText(msg);
  else { // the other events change the command station status, we show that differently
    commandStationStatus_t newStatus = UNKNOWN;
    switch (xpcEvent) {
      case XPEVENT_POWER_ON : 
        newStatus = RUN_OKAY;
        break;
      case XPEVENT_POWER_OFF :
        newStatus = RUN_OFF;
        break;
      case XPEVENT_EMERGENCY_STOP :
        newStatus = RUN_STOP;
        break;
      case XPEVENT_SERVICE_MODE_ON :
        msg = evtProgTrackOkText;
        newStatus = PROG;
        break;
      default:
        break;
    }
    if ((newStatus != UNKNOWN) && (newStatus != commandStationStatus)) {
      commandStationStatus = newStatus;
      ui_ShowCsStatus(); 
    }
  }
} // xpc_EventNotify

void xpc_LocStolenNotify(uint16_t stolenLocAddress) {
  if (curLoc.address == stolenLocAddress) { // should always be true, because throttle handles only 1 loc
    curLoc.owned = 0; // reset this flag, can be used to monitor the loc status before stealing back
    ui_ShowEventText(evtLocStolenText);
  }
} // xpc_LocStolenNotify

void xpc_FastClockNotify (xpcFastClock_t *newFastClock) {
  fastClock = *newFastClock;
  ui_ShowClock();
} // xpc_FastClockNotify

void xpc_CommandStationStatusResponse (uint8_t csStatus) {
  commandStationStatus_t newStatus = UNKNOWN;
  xpcRequests.respLastMillis = millis();

  if (csStatus == COMMANDSTATION_STATUS_POWER_ON) newStatus = RUN_OKAY;
  else if (csStatus == COMMANDSTATION_STATUS_POWER_OFF) newStatus = RUN_OFF;
  else if (csStatus == COMMANDSTATION_STATUS_EMERGENCY_STOP) newStatus = RUN_STOP;
  else if (csStatus == COMMANDSTATION_STATUS_SERVICE_MODE_ON) newStatus = PROG;
  if (newStatus != commandStationStatus) {
    commandStationStatus = newStatus;
    ui_ShowCsStatus();
  }
} // xpc_CommandStationStatusResponse

// dccSpeed128 : de speed wordt steeds in DCC128 teruggemeld, ook al wordt de loc momenteel volgens andere speedsteps gestuurd
// de xp client gaat zelf altijd DCC128 gebruiken
// locFuncs_f0_f12 : 1 bit per functie, bit0=functie0 (licht), bit1=functie1, ... tot f12
// isLocFree : true = niet in gebruik door een andere xpclient
// store the return data in the shadow structure 'newLoc'
void xpc_LocGetInfoResponse(uint8_t dccSpeed128, uint16_t locFuncs_f0_f12, bool isLocFree) {
  xpcRequests.respLastMillis = millis();

  newLoc.refresh = 0;
  newLoc.owned = !isLocFree;
  if (dccSpeed128 != newLoc.speed) newLoc.speedChanged = 1;
  if (locFuncs_f0_f12 != newLoc.funcs & 0x1FFF) newLoc.funcsChanged = 1;
  newLoc.speed = dccSpeed128;
  newLoc.funcs = (newLoc.funcs & 0xFFFFE000) + locFuncs_f0_f12;
  if (newLoc.address == curLoc.address) { // copy data in curLoc too
    if (dccSpeed128 != curLoc.speed) curLoc.speedChanged = 1;
    if (locFuncs_f0_f12 != curLoc.funcs & 0x1FFF) curLoc.funcsChanged = 1;
    curLoc.speed = dccSpeed128;
    curLoc.funcs = (curLoc.funcs & 0xFFFFE000) + locFuncs_f0_f12;
    curLoc.refresh = 0;
  }
} // xpc_LocGetInfoResponse

// here arrives all info concerning turnout changes & feedback decoder input changes
// turnouts : only storing the 8 turnouts shown on display
// if curStartTurnout changes, we request the data for these turnouts with xpc_send_AccessoryDecoderInfoRequest
void xpc_AccessoryDecoderInfoResponse (uint8_t decoderAddress, uint8_t decoderBits, uint8_t decoderBitsMask, uint8_t decoderType) {
  if (decoderType == DECODERTYPE_FEEDBACK_DECODER)
    return; // TEMP, not interested for now
  
  xpcRequests.respLastMillis = millis(); // the callback could also be a broadcast, but OK for now (we could check if the decoderAddress corresponds with the last request..)
  turnoutBuffer.lastRefreshMillis = millis();
  
  // so we have info about a turnout decoder
  // for turnouts we store the 2bits/turnout in turnoutBuffer = 1 byte/accessory decoder 
  // if decoderAddress is in the range defined by turnoutBuffer.startAddress + TURNOUTBUFFER_SIZE
  if ((decoderAddress >= turnoutBuffer.firstDecoderAddress) && 
      (decoderAddress < (turnoutBuffer.firstDecoderAddress + TURNOUTBUFFER_SIZE))) {
    uint8_t curTurnoutBufferData, newTurnoutBufferData;
    curTurnoutBufferData = turnoutBuffer.data[decoderAddress - turnoutBuffer.firstDecoderAddress];
    newTurnoutBufferData = (curTurnoutBufferData & ~decoderBitsMask) | decoderBits;

    // TODO : not sure if this is the correct place to trigger a display update
    if (((decoderAddress>>1) == (curStartTurnout>>3)) && // concerns a turnout currently on display
       (newTurnoutBufferData != curTurnoutBufferData)) {  // data changed
      turnoutPositionsChanged = true; // set flag to trigger display update
    }
    turnoutBuffer.data[decoderAddress - turnoutBuffer.firstDecoderAddress] = newTurnoutBufferData;
  }
} // xpc_AccessoryDecoderInfoResponse

/*****************************************************************************/
/*    MAIN                                                                   */
/*****************************************************************************/

void setup() {
  rs485_Init(RS485_DIRECTION_PIN);
  xpc_Init(xpc_Address);
  keys_Init();
  ui_Init ();

  commandStationStatus = UNKNOWN; // until we receive a broadcast or a response from xpc_send_CommandStationStatusRequest
  curLoc.address = 3;
  curLoc.funcs = 0;
  curLoc.owned = 0;
  curLoc.speed = 0 | DIRECTION_FORWARD;
  newLoc.address = 3;
  newLoc.funcs = 0;
  newLoc.owned = 0;
  newLoc.speed = 0 | DIRECTION_FORWARD;
  newLoc.refresh = 1; // trigger initial refresh

  turnout_TriggerBufferRefresh(); // trigger initial refresh
} // setup

void loop() {
  xpc_Run();
  keys_Update();
  ui_Update();

  // xpnet data refresh/polling in a simple fixed queue fashion
  switch (xpcRequests.state) {
    case IDLE:
      if (commandStationStatus == UNKNOWN) {
        xpcRequests.state = REQ_CS_STATUS;
        xpcRequests.reqLastMillis = millis();
        xpc_send_CommandStationStatusRequest();
      }
      else if (newLoc.refresh) { // UI requests a data refresh
        xpcRequests.state = REQ_LOC_INFO;
        xpcRequests.reqLastMillis = millis();
        xpc_send_LocGetInfoRequest(newLoc.address);
      }
      else if ((millis() - turnoutBuffer.lastRefreshMillis) > TURNOUTBUFFER_REFRESH_DELAY) {
        xpcRequests.state = REQ_ACC_INFO;
        xpcRequests.reqLastMillis = millis();
        xpcRequests.idx = 0;
        xpc_send_AccessoryDecoderInfoRequest(turnoutBuffer.firstDecoderAddress, 0);
      }
      break;
    case REQ_CS_STATUS :
      if (xpcRequests.respLastMillis > xpcRequests.reqLastMillis) { // response received
        xpcRequests.state = IDLE;
      }
      else if ((millis() - xpcRequests.reqLastMillis) > XPC_REQUESTS_RETRY_DELAY) { // need a retry
        xpcRequests.reqLastMillis = millis();
        xpc_send_CommandStationStatusRequest();
      }
      break;
    case REQ_LOC_INFO :
      if (xpcRequests.respLastMillis > xpcRequests.reqLastMillis) { // response received
        xpcRequests.state = IDLE;
      }
      else if ((millis() - xpcRequests.reqLastMillis) > XPC_REQUESTS_RETRY_DELAY) { // need a retry
        xpcRequests.reqLastMillis = millis();
        xpc_send_LocGetInfoRequest(newLoc.address);
      }
      break;
    case REQ_ACC_INFO : 
      if (xpcRequests.respLastMillis > xpcRequests.reqLastMillis) { // response received
        xpcRequests.idx+= 1; 
        // we need 2 requests per decoder address (idx & 0x1) : even/odd nibble
        if ((xpcRequests.idx>>1) < TURNOUTBUFFER_SIZE) {
          xpcRequests.reqLastMillis = millis();
          xpc_send_AccessoryDecoderInfoRequest(turnoutBuffer.firstDecoderAddress +(xpcRequests.idx>>1), xpcRequests.idx & 0x1);
        }
        else { // all done
          xpcRequests.state = IDLE;
        }
      }
      else if ((millis() - xpcRequests.reqLastMillis) > XPC_REQUESTS_RETRY_DELAY) { // need a retry
        xpcRequests.reqLastMillis = millis();
        xpc_send_AccessoryDecoderInfoRequest(turnoutBuffer.firstDecoderAddress +(xpcRequests.idx>>1), xpcRequests.idx & 0x1);
      }
      break;
  }
} // loop
