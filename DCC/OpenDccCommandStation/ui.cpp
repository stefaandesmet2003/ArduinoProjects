#include "Arduino.h"
#include "config.h"
#include "keys.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "ui.h"
#include "status.h" // status & fastclock
#include "organizer.h" // loc & turnout commands
#include "database.h" // for loc database access
#include "xpnet.h" // send xpnet event for loc stolen
#include "accessories.h" // turnout status

//#include "programmer.h" // for CV programming via UI
// TODO : toch events gebruiken voor turnout updates ipv polling ?
// om screen refreshes te verminderen
// screen refresh = i2c blocking, dus geen xpnet traffic

// the organizer keeps track of each xpnet device
// 0 = invalid xpnet slot (broadcast), so we can use this here to identify the local UI
// original opendcc used slot 0 = PC (LENZ intf)
#define LOCAL_UI_SLOT (0)
#define UI_MAX_LOC_ADDRESS      999 // can't show more than 3 digits loc address on display for now
#define UI_MAX_TURNOUT_ADDRESS  999 // can't show more than 3 digits turnout address on display for now

#define EVENT_UI_UPDATE (EVENT_KEY_LASTEVENT + 1)

// SDS2021 TODO : weg??
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

// dit is volgens DCC128
#define DIRECTION_FORWARD 0x80
#define DIRECTION_REVERSE 0x0
#define DIRECTION_BIT     0x80


// for the lcd display
// note: the whole UI code here is developed for a 20x4 char LCD
#define DISPLAY_X_SIZE  20
#define DISPLAY_Y_SIZE  4
#define BACKLIGHTOFF_DELAY  10000

#define ARROW_RIGHT_CHAR \x7E
#define ARROW_LEFT_CHAR \x7F

#define STR_(X) #X      // this converts to string
#define STR(X) STR_(X)  // this makes sure the argument is expanded before converting to string

// own glyphs
#define GLYPH_LAMP_ON_NORMAL            (uint8_t) 0x00
#define GLYPH_LAMP_OFF_NORMAL           (uint8_t) 0x01
#define GLYPH_LAMP_ON_HIGHLIGHT         (uint8_t) 0x02
#define GLYPH_LAMP_OFF_HIGHLIGHT        (uint8_t) 0x03
#define GLYPH_TURNOUT_CLOSED_NORMAL     (uint8_t) 0x04
#define GLYPH_TURNOUT_THROWN_NORMAL     (uint8_t) 0x05
#define GLYPH_TURNOUT_CLOSED_HIGHLIGHT  (uint8_t) 0x06
#define GLYPH_TURNOUT_THROWN_HIGHLIGHT  (uint8_t) 0x07

// these glyphs are in ROM
#define ARROW_RIGHT                     (uint8_t) 0x7E
#define ARROW_LEFT                      (uint8_t) 0x7F
#define FULL_BLOCK                      (uint8_t) 0xFF

// UI refresh settings
#define DISPLAY_MANUAL_REFRESH_DELAY  200 // avoid a display refresh on every rotary key event
#define DISPLAY_AUTO_REFRESH_DELAY    500 // polling status changes to be shown on display (current, clock changes over xpnet, ...)

static LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
// Create a set of new characters
static const uint8_t char0[] PROGMEM = { 0x00, 0x0E, 0x1F, 0x1F, 0x1F, 0x0E, 0x0E, 0x00 }; // lampke aan 6hoog
static const uint8_t char1[] PROGMEM = { 0x00, 0x0E, 0x11, 0x11, 0x11, 0x0E, 0x0E, 0x00 }; // lampke uit 6hoog
static const uint8_t char2[] PROGMEM = { 0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x0E, 0x0E }; // lampke aan, 8 hoog
static const uint8_t char3[] PROGMEM = { 0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E, 0x0E }; // lampke uit, 8 hoog
static const uint8_t char4[] PROGMEM = { 0x00, 0x04, 0x0E, 0x15, 0x04, 0x04, 0x04, 0x00 }; // wissel recht -> 6 hoog
static const uint8_t char5[] PROGMEM = { 0x00, 0x1E, 0x06, 0x0A, 0x0A, 0x08, 0x08, 0x00 }; // wissel schuin -> 6 hoog
static const uint8_t char6[] PROGMEM = { 0x1F, 0x1B, 0x11, 0x0A, 0x1B, 0x1B, 0x1B, 0x1F }; // wissel recht invers
static const uint8_t char7[] PROGMEM = { 0x1F, 0x01, 0x19, 0x15, 0x15, 0x17, 0x17, 0x1F }; // wissel schuin invers
static const uint8_t *const charBitmap[] PROGMEM = {char0,char1,char2,char3,char4,char5,char6,char7};

// backlight control
static uint32_t triggerBacklightLastMillis;
static bool backlightOn = true; // reduce i2c accesses

// for the UI
typedef uint8_t ui_State_t;
static ui_State_t ui_State;
static bool ui_Redraw = true;
static uint32_t uiUpdateLastMillis; // controlling manual & automatic display refreshes
static uint16_t ui_CurLocAddress, ui_NewLocAddress; // loc addr geselecteerd in UI
static bool ui_CurLocSpeedDoDelayedRefresh = true;
uint8_t curStartFunc = 0;
uint8_t curHighlightFunc = 0;
uint16_t curStartTurnout = 0; // turnouts counted from 0, but displayed from 1 (like JMRI)
uint16_t curHighlightTurnout = 0;
uint16_t curTurnoutPositions = 0xFFFF; // poll turnout positions, but only update screen if a turnout changed position (xpnet)
uiEvent_t uiEvent; // events to show on display

// ui fixed text in progmem
static const char navHomePage1[] PROGMEM = "main  pwr test   >  ";
static const char navHomePage2[] PROGMEM = "prog setup  >";
static const char navRunMain[] PROGMEM = "menu  fx  loc  acc  ";
static const char navRunLocChange[] PROGMEM = "back   " STR(ARROW_LEFT_CHAR) "   " STR(ARROW_RIGHT_CHAR) "   OK  ";
static const char navRunLocFuncOrTurnoutChange[] PROGMEM = "back   " STR(ARROW_LEFT_CHAR) "   " STR(ARROW_RIGHT_CHAR) "  toggle";
static const char navTest[] PROGMEM = "back sig1 sig2 DB TX";
static const char navPowerPage[] PROGMEM = "back main prog      ";
static const char defaultLocName[] PROGMEM = "[no name] ";
static const char evtLocStolenText[] PROGMEM          = "Loc Stolen! ";
static const char evtMainTrackOkText[] PROGMEM        = "Main OK!    ";
static const char evtMainEmergencyStopText[] PROGMEM  = "Main STOP!  ";
static const char evtTracksOffText[] PROGMEM          = "Tracks OFF! ";
static const char evtMainTrackShortText[] PROGMEM     = "Main Short! ";
static const char evtProgTrackOkText[] PROGMEM        = "Progr. Mode!";
static const char evtProgTrackShortText[] PROGMEM     = "Prog Short! ";
static const char evtProgErrorText[] PROGMEM          = "Prog Error! ";
static const char evtExternalStopText[] PROGMEM       = "Extern STOP!";
static const char mnuPowerHelpText[] PROGMEM          = "switch tracks on/off";

// incoming events (key+ui update events) are first passed to the active menu handler
// unhandled keys are then handled by the main key handler or discarded (only 1 active menu at the time)
static bool (*ui_ActiveMenuHandler)(uint8_t event, uint8_t code);
static bool ui_HomeMenuHandler (uint8_t event, uint8_t code);
static bool ui_RunMenuHandler (uint8_t event, uint8_t code);
static bool ui_PowerMenuHandler (uint8_t event, uint8_t code);
static bool ui_TestMenuHandler (uint8_t event, uint8_t code);
static bool ui_SetupMenuHandler (uint8_t event, uint8_t code);
static bool ui_ProgMenuHandler (uint8_t event, uint8_t code);
static bool ui_EventHandler (uint8_t event, uint8_t code);
static bool ui_LocSpeedHandler (uint8_t event, uint8_t code); // generic loc speed handling with rotary key, used by all menus that don't use the rotary key differently

/*****************************************************************************/
/*    HELPER FUNCTIONS FOR DISPLAY WRITING                                   */
/*****************************************************************************/
static void lcd_Init() {
  int charBitmapSize = (sizeof(charBitmap ) / sizeof (charBitmap[0]));

  lcd.begin(20,4); // initialize the lcd 
  // Switch on the backlight
  //lcd.setBacklight(1); // overbodig, want by default al aan

  // init special characters
  for (int i = 0; i < charBitmapSize; i++) {
    uint8_t aBuffer[8];
    memcpy_P ((void*)aBuffer,(void*)pgm_read_word(&(charBitmap[i])),8); // eerst data copiëren van flash->sram
    lcd.createChar (i, aBuffer);
  }
  lcd.home();
} // lcd_Init

// keys & events trigger the backlight
// UI update will switch off the backlight after BACKLIGHTOFF_DELAY if no activity
static void triggerBacklight() {
  triggerBacklightLastMillis = millis();
  if (!backlightOn) {
    backlightOn = true;
    lcd.setBacklight(1);
  }
} // triggerBacklight

// line : 0,1,2,3 : clear this line
// startPos : clear line from startPos (or from current cursor position if startPos is invalid) to end of the line, 
// note : more flexible were if we could clear the remainder of a line, but we have no possibility to get the current cursor position from the display
static void clearLine (uint8_t line, uint8_t startPos=0) {
  if (line >= DISPLAY_Y_SIZE) return;
  if (startPos < DISPLAY_X_SIZE)
    lcd.setCursor(startPos,line);
  for (uint8_t x=startPos;x<DISPLAY_X_SIZE;x++)
    lcd.write(' ');
} // clearLine

// TODO : welke waarde aanduiden voor 14/28/127 steps?
// 4 digits default position (16,0)->(19,0) : "<sss" or "sss>" with leading zeroes
static void ui_ShowLocSpeed (uint8_t dccSpeed, uint8_t xPos=16, uint8_t yPos=0) {
  uint8_t dccDirectionForward = dccSpeed & DIRECTION_FORWARD;
  lcd.setCursor(xPos,yPos);  
  if (!dccDirectionForward){
    lcd.write(ARROW_LEFT);
  }
  dccSpeed = dccSpeed & 0x7F;
  if (dccSpeed < 100) lcd.write('0');
  if (dccSpeed < 10) lcd.write('0');
  lcd.print(dccSpeed);
  if (dccDirectionForward) {
      lcd.write(ARROW_RIGHT);
  }
} // ui_ShowLocSpeed

// op lijn 1, teken 12-15, of 10-15 (long addr)
// 4 digits, default position (12,0)->(15,0) : "aaa:" with leading zeroes
// TODO : not enough screen space to show locAddress > 999 (UI_MAX_LOC_ADDRESS)
static void ui_ShowLocAddress (uint16_t locAddress, uint8_t xPos=12, uint8_t yPos=0) {
  lcd.setCursor(xPos,yPos);
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
// display position: (12,1)->(19,1)
static void ui_ShowLocFuncs (uint32_t allFuncs, uint8_t startFunc, uint8_t highlightFunc) { // startFunc : multiple of 8
  uint8_t cur8Funcs;
  startFunc = startFunc & 0xF8; // multiple of 8
  cur8Funcs = (allFuncs >> startFunc) & 0xFF;
  lcd.setCursor(12,1);
  for (uint8_t func=0;func<8;func++) {
    uint8_t funcActive = (cur8Funcs >> func) & 0x1;
    if ((startFunc+func) == highlightFunc) {
      if (funcActive) lcd.write(GLYPH_LAMP_ON_HIGHLIGHT);
      else lcd.write(GLYPH_LAMP_OFF_HIGHLIGHT);
    }
    else {
      if (funcActive) lcd.write(GLYPH_LAMP_ON_NORMAL);
      else lcd.write(GLYPH_LAMP_OFF_NORMAL);
    }
  }
} // ui_ShowLocFuncs

// show 8 turnouts [startTurnout,startTurnout+7], highlight [startTurnout,startTurnout+7] or 0xFFFF for no highlight
// display position: (12,2)->(19,2)
static void ui_ShowTurnouts (uint16_t startTurnout, uint16_t highlightTurnout) { // startTurnout : multiple of 8
  startTurnout = startTurnout & 0xF8; // multiple of 8
  lcd.setCursor(12,2);
  for (uint8_t turnout=0;turnout<8;turnout++) {
    uint8_t turnoutState = turnout_GetStatus(startTurnout+turnout);
    if ((startTurnout+turnout) == highlightTurnout) {
      if (turnoutState == TURNOUT_STATE_CLOSED) lcd.write(GLYPH_TURNOUT_CLOSED_HIGHLIGHT);
      else if (turnoutState == TURNOUT_STATE_THROWN) lcd.write(GLYPH_TURNOUT_THROWN_HIGHLIGHT);
      else lcd.write(FULL_BLOCK);
    }
    else {
      if (turnoutState == TURNOUT_STATE_CLOSED) lcd.write(GLYPH_TURNOUT_CLOSED_NORMAL);
      else if (turnoutState == TURNOUT_STATE_THROWN) lcd.write(GLYPH_TURNOUT_THROWN_NORMAL);
      else lcd.write('.');
    }
  }
} // ui_ShowTurnouts

// test
// 5 digits (6,0) - (10,0) : x.yyA or xxxmA
static void ui_ShowCurrent() {
  int a7, mA;

  a7 = analogRead(A7);
  if (a7>=4) mA = (a7-4)*8; // experimental conversion
  else mA = 0;

  lcd.setCursor(6,0);
  
  // display formatting
  if (mA > 1000) { // a rudimentary implementation to replace snprintf
    uint8_t tens = 0;
    uint8_t thousands = 0;
    while (mA > 1000) {
      mA -= 1000;
      thousands += 1;
    }
    while (mA > 100) {
      mA -= 100;
      tens += 10;
    }
    while (mA > 10) {
      mA -= 10;
      tens += 1;
    }
    lcd.print(thousands); lcd.write('.');
    if (tens<10) lcd.write('0');
    lcd.print(tens); lcd.write('A');
  }
  else {
    if (mA < 100) lcd.write(' ');
    if (mA < 10) lcd.write(' ');
    lcd.print(mA);lcd.print("mA");
  }
} // ui_ShowCurrent

// 5 digits (0,0) -> (4,0)
static void ui_ShowClock() {
  lcd.setCursor(0,0);
  if (fast_clock.hour < 10) lcd.write('0');
  lcd.print(fast_clock.hour);lcd.write(':');
  if (fast_clock.minute < 10) lcd.write('0');
  lcd.print(fast_clock.minute);
} // ui_ShowClock

// use (0,2) -> (11,2), and clear the remainder of the line
static void ui_ShowEventText(const char *eventText) {
  lcd.setCursor(0,2);
  // TODO text length check -> for now we use flash strings of equal size
  lcd.print((__FlashStringHelper*)eventText);

} // ui_ShowEventText

// show command station state on the notification line (2)
static void ui_ShowCsState() {
  const char *evtText = NULL;
  switch (opendcc_state) {
    case RUN_OKAY:
      evtText = evtMainTrackOkText;
      break;
    case RUN_STOP:
      evtText = evtMainEmergencyStopText;
      break;
    case RUN_OFF:
    case PROG_OFF: // TODO : no differences between these states, both tracks are off
      evtText = evtTracksOffText;
      break;
    case PROG_OKAY:
      evtText = evtProgTrackOkText;
      break;
    case PROG_ERROR:
      evtText = evtProgErrorText;
      break;
  }
  if (evtText)
    ui_ShowEventText(evtText);
} // ui_ShowCsState

/*****************************************************************************/
/*    HELPER FUNCTIONS FOR INTERFACING WITH ORGANIZER                        */
/*****************************************************************************/
/*
 * do_loco_speed gebruikt altijd DCC128! (intern organizer formaat)
 * het formaat dat op de rail wordt gezet hangt af van de loco database (eeprom) -> 06/2021 : is nu ook default DCC128
 * en dcc_default_format == DCC128 (in config.h) voor nieuwe loc addressen
 * met do_loco_speed_f kan je toch nog aansturen met een ander formaat, maar dan moet je eerst 'convert_speed_from_rail' :
 * speed128 = convert_speed_from_rail(speed,format)
 * en dan do_loco_speed_f (speed128,format)
 * wat op rail komt moet door de decoder ondersteund worden, maar is ok voor de mijne (doen zowel DCC28 & DCC128) 
 * dus momenteel geen nood om vanuit UI een ander formaat kunnen instellen, en do_loco_speed_f te gebruiken
 */

/*
The optional NMRA 128 speed step mode is available for both decoders and command stations that support it. 
Both the decoder and the command station controlling it must support this feature. 
A decoder switches to 128 speed step mode automatically at the track level when a it receives a 128 speed step command from the command station. 
The decoder will switch back to 14 or 28 speed step mode automatically when it receives a command in that speed command format. 
Unlike 14 and 28 speed step commands, the decoder does not need to pre-programmed to enable 128 speed step mode operation. 
It is always on, ready to be used at any time.
If a decoder capable of only 28 speed steps is on the layout, 
it will ignore the 128 mode and function at the 28 step mode.
That is why the 28 step mode is often referred to as 28/128.
*/

// #define DCC_SHORT_ADDR_LIMIT   112  (in config.h)
// als locAddr > DCC_SHORT_ADDR_LIMIT, gaat de organiser DCC msgs voor long addr gebruiken
// dus : locAddr < 112 -> short addr naar loc-decoder, > 112 --> long addr naar loc-decoder
// dus : locSpeed in het locobuffer format (128 steps), dwz 0 = stop, 1= noodstop, 2..127 = speedsteps, msb = richting, 1=voorwaarts, 0=achterwaarts
static void ui_SetLocSpeed (uint16_t locAddr, uint8_t locSpeed) {
  unsigned char retval;
  if (organizer_IsReady()) {
    retval = do_loco_speed (LOCAL_UI_SLOT,locAddr, locSpeed);
    if (retval & ORGZ_STOLEN)
      xpnet_SendLocStolen(orgz_old_lok_owner,locAddr);
  }
} // ui_SetLocSpeed

// TODO : kan beter, locobuffer heeft al een uint32_t met alle functiebits
// maar UI doet enkel toggle van 1 bit tegelijk
// door de onderliggende implementatie van functie groups (dcc & xpnet) is dat een pain in the ass
// we moeten hier telkens alle bits uit de grpX hebben want do_loco_func_grpX overschrijft alle functie bits in de groep!!!
// dus ofwel moeten we die bits uit locobuffer opvragen, ofwel uit local copy (*)
// func is de functie die moet gezet worden, maar dus alle functies in dezelfde groep worden meegezet
// func 0 = light
// f1 ->f28 
// on = 1-bit, off = 0-bit
static void ui_SetLocFunction (uint16_t locAddr, uint8_t func, uint32_t allFuncs) {
  uint8_t retval;
  if (!organizer_IsReady()) // can't send anything to organizer for now
    return;

  if (func==0)
    retval= do_loco_func_grp0 (LOCAL_UI_SLOT,locAddr, allFuncs & 0xFF); // grp0 = f0 = fl
  else if ((func >=1) && (func <= 4))
    retval= do_loco_func_grp1 (LOCAL_UI_SLOT,locAddr, (allFuncs >> 1)); // grp1 = f1..f4
  else if ((func >=5) && (func <= 8))
    retval= do_loco_func_grp2 (LOCAL_UI_SLOT,locAddr, (allFuncs >> 5)); // grp2 = f5..f8
  else if ((func >=9) && (func <= 12))
    retval= do_loco_func_grp3 (LOCAL_UI_SLOT,locAddr, (allFuncs >> 9)); // grp3 = f9..f12
#if (DCC_F13_F28 == 1)        
  else if ((func >=13) && (func <= 20))
    retval= do_loco_func_grp4 (LOCAL_UI_SLOT,locAddr, (allFuncs >> 13));
  else if ((func >=21) && (func <= 28))
    retval= do_loco_func_grp5 (LOCAL_UI_SLOT,locAddr, (allFuncs >> 21));
#endif
  if (retval & ORGZ_STOLEN)
    xpnet_SendLocStolen(orgz_old_lok_owner,locAddr);
} // ui_SetLocFunction

// TODO : return value needed to handle failed do_accessory(...) cmd?
static void ui_ToggleTurnout (uint16_t turnoutAddress, bool activate) {
  bool retval;
  uint8_t turnoutStatus;
  uint8_t coil;

  if (!organizer_IsReady()) // can't send anything to organizer for now
    return;

  turnoutStatus = turnout_GetStatus(turnoutAddress); // current turnout position, info from accessoryBuffer
  // on activate for a toggle, the coil to activate is turnoutStatus & 0x1 : 
  // turnoutStatus=00=unknown -> activate coil 0 (green)
  // turnoutStatus=01=green -> activate coil 1 (red)
  // turnoutStatus=10=red -> activate coil 0 (green)
  // on !activate, the coil to disactivate is (turnoutStatus & 0x1) ^0x1;
  // turnoutStatus=01=green -> disactivate coil 0
  // turnoutStatus=10=red -> disactivate coil 1
  coil = (uint8_t) (!activate);
  coil = (coil^turnoutStatus) & 0x1;
  retval = do_accessory(turnoutAddress,coil,activate); // retval==0 means OK
  if (activate && (retval==0)) { // only notify the 'on' command, not the 'off'
    unsigned char tx_message[3];
    tx_message[0] = 0x42;
    turnout_getInfo(turnoutAddress,&tx_message[1]);
    xpnet_SendMessage(FUTURE_ID, tx_message); // feedback broadcast
  }
} // ui_ToggleTurnout

static void ui_SetExtendedAccessory (uint16_t decoderAddress, uint8_t signalId, uint8_t signalAspect) {
  if (!organizer_IsReady())
    return;
  do_signal_accessory(decoderAddress,signalId, signalAspect); // retval==0 means OK
} // ui_SetExtendedAccessory

/*****************************************************************************/
/*    HELPER FUNCTIONS FOR INTERFACING WITH PROGRAMMER                       */
/*****************************************************************************/
// TODO : not used for now
/*
void app_TestCVRead() {
  // read CV1
  programmer_CvDirectRead(1);
}

uint8_t app_GetProgResults (uint16_t &cv, uint8_t &cvdata) {
  uint8_t retval;
  
  if (prog_event.busy)
    return 1;
  retval = (uint8_t) prog_result;
  cv = prog_cv;
  cvdata = prog_data;
  return (retval);
} // app_GetProgResults
*/

/*****************************************************************************/
/*    UI PAGE HANDLERS                                                       */
/*****************************************************************************/
static bool ui_HomeMenuHandler (uint8_t event, uint8_t code) {
  uint8_t keyCode;

  // ui events
  if ((event == EVENT_UI_UPDATE) && code) { // do only manual refresh
    clearLine(1);
    clearLine(2);
    lcd.setCursor(0,3);
    if (ui_State == UISTATE_HOME_PAGE1) lcd.print ((__FlashStringHelper*)navHomePage1);
    else if (ui_State == UISTATE_HOME_PAGE2) {
      lcd.print((__FlashStringHelper*)navHomePage2);
      clearLine(3,13);
    }
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
      ui_ActiveMenuHandler = ui_ProgMenuHandler; 
    }
    else if (keyCode == KEY_2) {
      ui_State = UISTATE_SETUP_PAGE1;
      ui_ActiveMenuHandler = ui_SetupMenuHandler;
    }
    else ui_State = UISTATE_HOME_PAGE1;
  }
  return (true);
} // ui_HomeMenuHandler

bool ui_RunMenuHandler (uint8_t event, uint8_t code) {
  bool keyHandled = false;
  uint8_t keyCode;
  uint8_t curLocSpeed = 0 | DIRECTION_FORWARD;
  uint32_t curLocFuncs = 0;
  uint8_t curLocSlot = LOCAL_UI_SLOT;
  locomem *curLocData = NULL; // retrieving existing data from locobuffer

  // refresh current loc data from locobuffer
  if (!lb_GetEntry(ui_CurLocAddress, &curLocData)) { // existing entry in locobuffer
    curLocSpeed = curLocData->speed;
    curLocFuncs = curLocData->funcs;
    curLocSlot = curLocData->slot;
  }

  // 1. handle ui update (code parameter == true if manual refresh, false if auto refresh)
  if (event == EVENT_UI_UPDATE) {
    // 1. auto+manual refresh
    // check loc stolen
    if (curLocSlot != LOCAL_UI_SLOT) {
      if (!uiEvent.locStolen) {
        uiEvent.locStolen = 1;
        ui_ShowEventText(evtLocStolenText);
      }
    }
    else uiEvent.locStolen = 0;
    // update loc speed/functions -> if loc stolen, update on every auto-refresh to keep display in sync with external control
    if (uiEvent.locStolen || code || ui_CurLocSpeedDoDelayedRefresh) {
      ui_ShowLocSpeed(curLocSpeed);
      ui_ShowLocFuncs(curLocFuncs,curStartFunc,0xFF); // no function highlighted
      ui_CurLocSpeedDoDelayedRefresh = false;
    }
    // status notification
    // not in event handler for now, we only need them on the main page
    if (uiEvent.statusChanged) {
      ui_ShowCsState();
      uiEvent.statusChanged = 0;
    }

    // 2. auto refresh only
    if (!code) {
      // check if turnouts changed over xpnet (we don't have notifies for the turnouts)
      // beetje lullig voorlopig, want hieronder hebben we nog eens per state de manual update..
      uint16_t newTurnoutPositions = 0;
      for (uint8_t i=0;i<8;i++)
        newTurnoutPositions += ((uint16_t) turnout_GetStatus(curStartTurnout+i)) << 2*i;
      // not optimal, we could just update the changed glyph, but who cares
      // or we could highlight the modified turnouts, etc. fancy fancy
      if (newTurnoutPositions != curTurnoutPositions) {
        if (ui_State == UISTATE_RUN_TURNOUTS) ui_ShowTurnouts(curStartTurnout,curHighlightTurnout);
        else if (ui_State != UISTATE_RUN_LOC_CHANGE) ui_ShowTurnouts(curStartTurnout,0xFFFF); // no turnouts highlighted
        curTurnoutPositions = newTurnoutPositions;
      }
      // TODO other data to auto-refresh (from xpnet eg.)?
      return true; // auto-refresh is done here
    }

    // 3. manual refresh only
    // retrieve locName from eeprom database, don't do this on every display refresh
    uint8_t locName[LOK_NAME_LENGTH];
    uint8_t dbRetval;

    ui_ShowLocAddress(ui_CurLocAddress);
    // need to retrieve locName from eeprom
    dbRetval = database_GetLocoName(ui_CurLocAddress, locName);
    lcd.setCursor (0,1);
    if (dbRetval) ui_ShowLocName(locName,12);
    else lcd.print((__FlashStringHelper*)defaultLocName);

    if (ui_State == UISTATE_RUN_MAIN) {
      ui_ShowLocFuncs(curLocFuncs,curStartFunc,0xFF); // no function highlighted
      ui_ShowCsState(); // show Command Station status on the notification line (2)
      ui_ShowTurnouts(curStartTurnout,0xFFFF); // no turnouts highlighted
      lcd.setCursor(0,3);
      lcd.print((__FlashStringHelper*) navRunMain);
    }
    else if (ui_State == UISTATE_RUN_LOC_FUNCS) {
      lcd.setCursor(8,1);
      lcd.write('F');lcd.print(curHighlightFunc);lcd.write(':');
      ui_ShowLocFuncs(curLocFuncs,curStartFunc,curHighlightFunc);
      ui_ShowTurnouts(curStartTurnout,0xFFFF); // no turnouts highlighted
      lcd.setCursor(0,3);
      lcd.print((__FlashStringHelper*) navRunLocFuncOrTurnoutChange);
    }
    else if (ui_State == UISTATE_RUN_TURNOUTS) {
      ui_ShowLocFuncs(curLocFuncs,curStartFunc,0xFF); // no function highlighted
      clearLine(2); // remove notification text
      lcd.setCursor(7,2);
      lcd.write('W');lcd.print(curHighlightTurnout+1);lcd.write(':');
      ui_ShowTurnouts(curStartTurnout,curHighlightTurnout);
      lcd.setCursor(0,3);
      lcd.print((__FlashStringHelper*) navRunLocFuncOrTurnoutChange);
    }
    else if (ui_State == UISTATE_RUN_LOC_CHANGE) {
      uint8_t locName[LOK_NAME_LENGTH]; // retrieving loc name from eeprom database
      uint8_t retval;
      locomem *newLocData; // retrieving existing data from locobuffer
      uint32_t newLocFuncs = 0;
      uint8_t newLocSpeed = 0;
      uint8_t newLocActive = 0;
      locName[0] = '\0';
      lcd.setCursor(0,1);
      lcd.write('?');
      retval = database_GetLocoName(ui_NewLocAddress,locName);
      if (retval) ui_ShowLocName(locName,12);
      else lcd.print((__FlashStringHelper*)defaultLocName);
      clearLine(1,11); // clear remainder of current line

      retval = lb_GetEntry(ui_NewLocAddress, &newLocData);
      if (!retval) { // loc is in locobuffer (retval=0)
        newLocFuncs = newLocData->funcs;
        newLocSpeed = newLocData->speed;
        newLocActive = newLocData->active;
      }
      ui_ShowLocFuncs(newLocFuncs,0,0xFF); // show F0..F7 for this loc, no function highlighted

      lcd.setCursor(0,2);
      lcd.write('?');
      ui_ShowLocAddress(ui_NewLocAddress,1,2);
      ui_ShowLocSpeed(newLocSpeed,5,2);
      clearLine(2,9);
      lcd.setCursor(10,2);
      if (retval || (!newLocActive)) // not in locobuffer, or not active in locobuffer
        lcd.print ("FREE");
      else if (newLocData->slot == LOCAL_UI_SLOT)
        lcd.print("IN USE[CS]");
      else {
        lcd.print("IN USE[ ");
        lcd.print(newLocData->slot);
        lcd.print("]");
      }
      lcd.setCursor(0,3);
      lcd.print((__FlashStringHelper*) navRunLocChange);
    }
    return true;
  }

  // 2. handle key events
  keyCode = code;

  // don't handle key up/longdown & rotary key
  if ((keyCode == KEY_ROTARY) || (keyCode == KEY_ENTER) ||
      (event == EVENT_KEY_UP) || (event == EVENT_KEY_LONGDOWN))
    return false;

  keyHandled = true;

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
      curLocFuncs ^= ((uint32_t) 0x1 << curHighlightFunc); // toggle func bit
      ui_SetLocFunction(ui_CurLocAddress,curHighlightFunc,curLocFuncs);
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
      lb_ReleaseLoc(ui_CurLocAddress); // TODO : is this a good choice to automatically release the old loc address?
      ui_CurLocAddress = ui_NewLocAddress;
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
      ui_ToggleTurnout (curHighlightTurnout, true);
    }
  }
  return (keyHandled); // true
} // ui_RunMenuHandler

// switch on/off power to main/prog tracks 
static bool ui_PowerMenuHandler (uint8_t event, uint8_t code) {
  uint8_t keyCode;


  // ui events
  if ((event == EVENT_UI_UPDATE) && code) { // do only manual refresh
    clearLine(1);
    lcd.setCursor(0,1);
    lcd.print ((__FlashStringHelper*)mnuPowerHelpText);
    ui_ShowCsState();
    lcd.setCursor(0,3);
    lcd.print ((__FlashStringHelper*)navPowerPage);
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
  }
  // toggle tracks
  // TODO: this is basic because the status.cpp logic doesn't allow both tracks to be active simultaneously
  else if (keyCode == KEY_2) { // toggle main track
    if (opendcc_state == RUN_OKAY) status_SetState(RUN_OFF);
    else status_SetState(RUN_OKAY);
  }
  else if (keyCode == KEY_3) { // toggle prog track
    if (opendcc_state == PROG_OKAY) status_SetState(PROG_OFF);
    else status_SetState(PROG_OKAY);
  }
  return (true);
} // ui_PowerMenuHandler

uint8_t signalHeads[2];
static bool ui_TestMenuHandler (uint8_t event, uint8_t code) {
  uint8_t keyCode;

  if ((event == EVENT_UI_UPDATE) && code) { // do only manual refresh
    for (uint8_t i=0;i<4;i++)
      clearLine(i);
    lcd.setCursor(0,0);
    lcd.print("TEST ");
    lcd.setCursor(0,1);
    lcd.print("Test funcs ");
    lcd.setCursor(0,2);
    for (uint8_t c=0;c<8;c++) lcd.write(c); // print all custom glyphs
    lcd.setCursor(0,3); lcd.print((__FlashStringHelper*) navTest);
    return true;
  }

  // handle key events
  keyCode = code;

  // don't handle key up/longdown & rotary key
  if ((keyCode == KEY_ROTARY) || (keyCode == KEY_ENTER) ||
      (event == EVENT_KEY_UP) || (event == EVENT_KEY_LONGDOWN))
    return false;

  if (keyCode == KEY_1) {
    ui_State = UISTATE_RUN_MAIN; // back // eventueel een long event gebruiken om direct terug te keren
    ui_ActiveMenuHandler = ui_RunMenuHandler;
  }
  else if (keyCode == KEY_2) {
    // toggle seinbeeld 0 op ext acc decoder adres 1
    ui_SetExtendedAccessory (1,0, signalHeads[0]); // (decoderAddress, signalHead,signalAspect)
    signalHeads[0] = (signalHeads[0] + 1) % 9; // loop through aspects 0..8
  }
  else if (keyCode == KEY_3) {
    // toggle seinbeeld 1 op ext acc decoder adres 1
    ui_SetExtendedAccessory (1,1, signalHeads[1]); // (decoderAddress, signalHead,signalAspect)
    signalHeads[1] = (signalHeads[1] + 1) % 9; // loop through aspects 0..8
  }
  else if (keyCode == KEY_4) {
    // test loco database transmission
    database_StartTransfer();
  }
  return true;
} // ui_TestMenuHandler

static bool ui_SetupMenuHandler (uint8_t event, uint8_t code) {
  bool keyHandled = false;
  uint8_t keyCode;

  if ((event == EVENT_UI_UPDATE) && code) { // do only manual refresh
    lcd.setCursor(0,0);
    lcd.print("SETUP ");
    lcd.setCursor(0,2); lcd.print("scherm niet af!");
    clearLine(3);
    lcd.setCursor(0,3); lcd.print("back");
    return true;
  }

  // handle key events
  keyCode = code;

  // don't handle key up/longdown & rotary key
  if ((keyCode == KEY_ROTARY) || (keyCode == KEY_ENTER) ||
      (event == EVENT_KEY_UP) || (event == EVENT_KEY_LONGDOWN))
    return false;

  // dummy -> go back to home
  ui_State = UISTATE_RUN_MAIN;
  ui_ActiveMenuHandler = ui_RunMenuHandler;
  keyHandled = true;

  return (keyHandled);
} // ui_SetupMenuHandler

static bool ui_ProgMenuHandler (uint8_t event, uint8_t code) {
  bool keyHandled = false;
  uint8_t keyCode;

  // handle display events
  if ((event == EVENT_UI_UPDATE) && code) { // do only manual refresh
    lcd.setCursor(0,0);
    lcd.print("PROG ");
    lcd.setCursor(0,2); lcd.print("scherm niet af!");
    clearLine(3);
    lcd.setCursor(0,3); lcd.print("back");
    return true;
  }

  // handle key events
  keyCode = code;

  // don't handle key up/longdown & rotary key
  if ((keyCode == KEY_ROTARY) || (keyCode == KEY_ENTER) ||
      (event == EVENT_KEY_UP) || (event == EVENT_KEY_LONGDOWN))
    return false;

  // dummy -> go back to home
  ui_State = UISTATE_RUN_MAIN;
  ui_ActiveMenuHandler = ui_RunMenuHandler;
  keyHandled = true;

  return (keyHandled);
} // ui_ProgMenuHandler

// status events handler
// .statusChanged is currently not handled here, only used in ui_RunMenuHandler
uiEvent_t uiEventCopy; // TODO TEMP!!
static bool ui_EventHandler (uint8_t event, uint8_t code) {
  uint8_t keyCode;
  // handle display events
  if ((event == EVENT_UI_UPDATE) && code) { // do only manual refresh
    if ((uiEvent.mainShort) || (uiEvent.progShort) || (uiEvent.extStop)) {
      uiEventCopy = uiEvent; //temp, om nadien onderscheid te maken ts main short & prog short... beetje vies ja
      triggerBacklight();
      clearLine(3);
      lcd.setCursor(15,3);
      lcd.print("OK");
      
      if (uiEvent.mainShort) ui_ShowEventText(evtMainTrackShortText);
      else if (uiEvent.progShort) ui_ShowEventText(evtProgTrackShortText);
      else if (uiEvent.extStop) ui_ShowEventText(evtExternalStopText);

      uiEvent.mainShort = 0;
      uiEvent.progShort = 0;
      uiEvent.extStop = 0;
      ui_ActiveMenuHandler = ui_EventHandler;
    }

    if (uiEvent.clockChanged) {
      ui_ShowClock();
      uiEvent.clockChanged = 0;
    }
    return true;
  }

  // handle key events
  keyCode = code;

  // don't handle key up/longdown & rotary key
  if ((keyCode == KEY_ROTARY) || (keyCode == KEY_ENTER) ||
      (event == EVENT_KEY_UP) || (event == EVENT_KEY_LONGDOWN))
    return false;

  // any key handles the event, and we go back to UISTATE_RUN_MAIN (no memory where we came from when the event occurred)
  t_opendcc_state newState = RUN_OKAY;
  if (uiEventCopy.progShort) newState = PROG_OKAY;
  status_SetState(newState);
  ui_State = UISTATE_RUN_MAIN;
  ui_ActiveMenuHandler = ui_RunMenuHandler;

  return true;
} // ui_EventHandler

#define UI_NUM_SPEED_STEPS 128 // eventueel een eeprom setup variable van maken (28 steps)
// define UI_NUM_SPEED_STEPS 28
#define DCC_MINSPEED        2 // 0 en 1 zijn stops, 0 = STOP, 1 = EMERGENCY STOP
#define DCC_MAXSPEED        127
uint32_t speedkeyLastMillis;
// updates the current loc speed
static bool ui_LocSpeedHandler (uint8_t keyEvent, uint8_t keyCode) {
  bool keyHandled = false;
  uint8_t curSpeed = 0 | DIRECTION_FORWARD;
  uint8_t speedStep, dirBit;
  locomem *curLocData;

  // ignore key up/longdown, ignore keypad keys
  if ((keyEvent == EVENT_KEY_UP) || (keyEvent == EVENT_KEY_LONGDOWN) ||
      ((keyCode != KEY_ROTARY) && (keyCode != KEY_ENTER)))
    return false;
  
  // als we snel aan de knop draaien gaat de speed sneller vooruit
  if ((millis() - speedkeyLastMillis) > 50) speedStep = 1;
  else if ((millis() - speedkeyLastMillis) > 30) speedStep = 3;
  else speedStep = 8;

  if (!lb_GetEntry(ui_CurLocAddress, &curLocData)) { // existing entry in locobuffer
    curSpeed = curLocData->speed;
  }  
      
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
    ui_SetLocSpeed(ui_CurLocAddress,curSpeed);
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

/***************************************************************************************************************/
/*    PUBLIC FUNCTIONS
/***************************************************************************************************************/
void ui_Init () {
  lcd_Init();
  uiEvent.clockChanged = 1; // force a clock display at first ui_Update()
  uiEvent.statusChanged = 1; // force a clock display at first ui_Update()

  ui_State = UISTATE_RUN_MAIN;
  ui_CurLocAddress = 3; // default loc address ofwel te vervangen door een saved state in eeprom
  ui_NewLocAddress = 3;
  ui_ActiveMenuHandler = ui_RunMenuHandler;
} // UI_Init

void ui_Update () {
  if (backlightOn && ((millis() - triggerBacklightLastMillis) > BACKLIGHTOFF_DELAY)) {
    lcd.setBacklight(0);
    backlightOn = false;
  }

  // check events (short circuit, clock change) -> doesn't wait for refresh delay
  ui_EventHandler(EVENT_UI_UPDATE, 1);

  // too early for a display refresh
  if ((!ui_Redraw) && ((millis() - uiUpdateLastMillis) < DISPLAY_MANUAL_REFRESH_DELAY))
    return;
  
  // do the auto-refresh part
  if ((ui_Redraw) || ((millis() - uiUpdateLastMillis) > DISPLAY_AUTO_REFRESH_DELAY)) {
    ui_ShowCurrent(); // for test
    ui_ActiveMenuHandler(EVENT_UI_UPDATE,(uint8_t) ui_Redraw); // handler can decide if it performs auto-refresh or not
    uiUpdateLastMillis = millis();
    ui_Redraw = false;
  }

} // ui_Update

// all key events arrive here first
void keys_Handler (uint8_t keyEvent, uint8_t keyCode) {
  bool keyHandled = false;

  triggerBacklight();

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
