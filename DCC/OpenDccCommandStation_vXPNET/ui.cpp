#include "Arduino.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#include "keys.h"
#include "ui.h"
#include "appstub.h"

#define FOCUS_PIJLKE 0x7E

// ui events
#define UIEVENT_SHORT
#define UIEVENT_LOCTAKEN
#define UIEVENT_NEWXPDEVICE
#define UIEVENT_XPDEVICELEFT
// to do :complete!

// eventueel andere zoals nummer toetsen op een remote control etc.

// alle ui states
#define UISTATE_HOME_PAGE1      0
#define UISTATE_HOME_PAGE2      1 

#define UISTATE_LOC_DO_SPEED    2
#define UISTATE_LOC_DO_ADDR     3
#define UISTATE_LOC_DO_FUNCS    4
#define UISTATE_TURNOUT_PAGE1   5
#define UISTATE_ACC_PAGE1       7
#define UISTATE_PROG_PAGE1      8
#define UISTATE_SETUP_PAGE1     9

#define UISTATE_DEFAULT        UISTATE_LOC_DO_SPEED

#define UISTATE_EVENT_MAINSHORT 100
#define UISTATE_EVENT_EXTSTOP   101
#define UISTATE_EVENT_PROGSHORT 102

// dit is volgens DCC128
#define DIRECTION_FORWARD 0x80
#define DIRECTION_REVERSE 0x0
#define DIRECTION_BIT     0x80
typedef uint8_t ui_State_t;

ui_State_t ui_State;
bool ui_Redraw = true;
uint8_t ui_Page; // om door de functies & wissels te scrollen
uint16_t ui_CurLoc, ui_NewLoc; // loc addr geselecteerd in UI
uint32_t ui_CurLocFuncs; // bit field met functies 0 (light) tot 28;
uint8_t ui_CurSpeed = 0 | DIRECTION_FORWARD ;
char ui_locName[8]; // holds a string with a locname to display (improve this!)
// dcc128 convention : highest bit in uint8_t speed is direction

extern LiquidCrystal_I2C lcd;
char emptyLine[] = "                    ";

bool ui_doHomeMenu (uint8_t key) {
  uint8_t keyCode, keyEvent;
  keyCode = key & KEYCODEFILTER;
  keyEvent = key & KEYEVENTFILTER;

  bool keyHandled = false;
  if (ui_State == UISTATE_HOME_PAGE1) {
    ui_Page = 0;
    keyHandled = true;
    if (keyCode == KEY_KEY1) ui_State = UISTATE_LOC_DO_SPEED;
    else if (keyCode == KEY_KEY2) ui_State = UISTATE_TURNOUT_PAGE1;
    else if (keyCode == KEY_KEY3) ui_State = UISTATE_ACC_PAGE1;
    else if (keyCode == KEY_KEY4) ui_State = UISTATE_HOME_PAGE2;
    else keyHandled = false;
  }
  else if (ui_State == UISTATE_HOME_PAGE2)  {
    keyHandled = true;
    if (keyCode == KEY_KEY1) ui_State = UISTATE_PROG_PAGE1;
    else if (keyCode == KEY_KEY2) ui_State = UISTATE_SETUP_PAGE1;
    else if (keyCode == KEY_KEY3) ui_State = UISTATE_SETUP_PAGE1; // temp,sds
    else if (keyCode == KEY_KEY4) ui_State = UISTATE_HOME_PAGE1;
    else keyHandled = false;
  }
  return (keyHandled);

} // ui_doHomeMenu

bool ui_doLocMenu (uint8_t key) {
  uint8_t keyCode, keyEvent;
  keyCode = key & KEYCODEFILTER;
  keyEvent = key & KEYEVENTFILTER;

  bool keyHandled = false;
  
  if ((ui_State == UISTATE_LOC_DO_SPEED) || (ui_State == UISTATE_LOC_DO_FUNCS)) {
    uint8_t func;
    if (keyCode == KEY_KEY2) {
      func = ui_Page<<1; // even functie op de ui_page
      keyHandled = true;
    }
    else if (keyCode == KEY_KEY3) {
      func = (ui_Page<<1) + 1; // oneven functie op de ui_page
      keyHandled = true;
    }           
    if (keyHandled) {
      ui_CurLocFuncs ^= (0x1 << func); // toggle func bit
      app_SetFunc(ui_CurLoc,func,(ui_CurLocFuncs >> func) & 0x1);
      return keyHandled;
    }
  }
  
  if (ui_State == UISTATE_LOC_DO_SPEED) {
    keyHandled = true;
    if (keyCode == KEY_KEY1) ui_State = UISTATE_LOC_DO_ADDR; // back
    else if (keyCode == KEY_KEY4) {
      if (ui_Page < 13) ui_Page++; // dan kan je tot func 27
      ui_State = UISTATE_LOC_DO_FUNCS;
    }
    else keyHandled = false;
  }
  else if (ui_State == UISTATE_LOC_DO_FUNCS) {
    keyHandled = true;
    if (keyCode == KEY_KEY1) {
      ui_Page=0;    
      ui_State = UISTATE_LOC_DO_SPEED; //back
    }
    else if (keyCode == KEY_KEY4) { // verder scrollen in de funcs : todo!
        ui_Page++;
    }            
    else keyHandled = false;
  }
  else if (ui_State == UISTATE_LOC_DO_ADDR) {
    keyHandled = true;
    if (keyCode == KEY_KEY1) ui_State = UISTATE_HOME_PAGE1; //back
    else if (keyCode == KEY_KEY2) // up
        ui_NewLoc = app_GetNextLoc(ui_NewLoc);
    else if (keyCode == KEY_KEY3) // down
        ui_NewLoc = app_GetPrevLoc(ui_NewLoc);
    else if (keyCode == KEY_KEY4) {
        // bevestig de nieuwe loc selectie
        ui_CurLoc = ui_NewLoc;
        ui_State = UISTATE_LOC_DO_SPEED;
        ui_CurLocFuncs = app_GetFuncs(ui_CurLoc);
        if (ui_CurLocFuncs = 0xFFFF) ui_CurLocFuncs = 0; // voor een loc die nog niet in de locobuffer zit
    }  
    else keyHandled = false;
  }
  return (keyHandled);
} // ui_doLocMenu

bool ui_doTurnoutMenu (uint8_t key)
{
  uint8_t keyCode, keyEvent;
  keyCode = key & KEYCODEFILTER;
  keyEvent = key & KEYEVENTFILTER;
  
  bool keyHandled = false;
  
  if (ui_State == UISTATE_TURNOUT_PAGE1) {
      keyHandled = true;
      if (keyCode == KEY_KEY1) {
          if (ui_Page) ui_Page--;
          else ui_State = UISTATE_HOME_PAGE1; // back // eventueel een long event gebruiken om direct terug te keren
      }
      else if (keyCode == KEY_KEY2) app_ToggleAccessory (ui_Page << 1); // wissel 1 = turnoutAddr 0
      else if (keyCode == KEY_KEY3) app_ToggleAccessory ((ui_Page << 1)+1); // wissel 2 = turnoutAddr 1
      else if (keyCode == KEY_KEY4) ui_Page++; // door de 255 pagina's scrollen, daarmee hebben we 512 wissels
      else keyHandled = false;
  }
  return (keyHandled);
} // ui_doTurnoutMenu

bool ui_doAccMenu (uint8_t key)
{
  uint8_t keyCode, keyEvent;
  bool keyHandled = false;

  keyCode = key & KEYCODEFILTER;
  keyEvent = key & KEYEVENTFILTER;
  if (ui_State == UISTATE_ACC_PAGE1) {
      ui_State = UISTATE_DEFAULT;
      keyHandled = true;
  }
  return (keyHandled);
} // ui_doAccMenu

static void ui_ShowLocFuncs (uint8_t page) // page 0 : F0+F1, page 1 : F2+F3 ...
{
  for (uint8_t func = page<<1;func<(page+1)<<1;func++) {
    lcd.print("F");
    lcd.print(func);
    if ((ui_CurLocFuncs >> func) & 0x1) // func is active
        lcd.print("*");
    else lcd.print(" ");
    lcd.print(" ");
    if (func < 10)
        lcd.print(" ");
  }
  // strPtr[0] = 0x0; // glyph lampke aan
  // strPtr[0] = 0x1; // glyphlampke uit
        
} // ui_ShowLocFuncs

// temp
extern uint32_t app_Turnouts;
static void ui_ShowTurnoutPos (uint8_t page) // page 0 : W0+W1, page 1 : W2+W3 ...
{
  for (uint8_t turnout = page<<1;turnout<(page+1)<<1;turnout++) {
    lcd.print("W");
    lcd.print(turnout+1); // wisseladressen vanaf 1 tellen op de display, zoals JMRI
    if ((app_Turnouts >> turnout) & 0x1) // func is active
        lcd.write(0x4); // speciaal teken wissel recht
    else lcd.write(0x5); // speciaal teken wissel gebogen
    lcd.print(" ");
    if (turnout < 10)
        lcd.print(" ");
  }
} // ui_ShowTurnoutPos

// todo : hoe + en -, welke waarde aanduiden voor 14/28/127 steps?
// laatste 4 tekens op lijn 1
static void ui_ShowSpeed (uint8_t dccSpeed) {
  char tmpSpeedStr[5];
  uint8_t dccDirectionForward = dccSpeed & DIRECTION_FORWARD;
  lcd.setCursor(16,0);  
  if (!dccDirectionForward) lcd.print("<");
  snprintf(tmpSpeedStr,4,"%03d",dccSpeed & 0x7F);
  lcd.print(tmpSpeedStr);
  if (dccDirectionForward) {
      lcd.setCursor(19,0);
      lcd.print(">");
  }
} // ui_ShowSpeed

// op lijn 1, teken 12-15, of 10-15 (long addr)
static void ui_ShowAddr (uint16_t myAddr) {
  char tmpAddrStr[6];
  if (myAddr > 999) { // long addr
    lcd.setCursor(10,0); // 5 digits address
    snprintf(tmpAddrStr,6,"%05d",myAddr);
  }
  else {
    lcd.setCursor(12,0); // 3 digits address
    snprintf(tmpAddrStr,4,"%03d",myAddr);
  }
  lcd.print(tmpAddrStr);
  lcd.print(":");
} // ui_ShowSpeed

// line : 0,1,2,3
static void clearLine (int line)
{
  lcd.setCursor(0,line);
  lcd.print(emptyLine);
}
/***************************************************************************************************************/
/*    PUBLIC FUNCTIONS
/***************************************************************************************************************/

void ui_Init () {
  ui_State = UISTATE_LOC_DO_SPEED;
  ui_CurLoc = 3; // default loc address ofwel te vervangen door een saved state in eeprom
  ui_CurLocFuncs = 0; // app_GetFuncs werkt hier niet, want er zit nog geen loc in de locobuffer
} // UI_Init

uint32_t updateLastMillis;
bool backlightOn = true; // reduce i2c accesses
#define REFRESH_DELAY       200
#define BACKLIGHTOFF_DELAY  10000

void ui_Update () {
  if (backlightOn && ((millis() - updateLastMillis) > BACKLIGHTOFF_DELAY)) {
    lcd.setBacklight(0);
    backlightOn = false;
  }

  // vermijden dat bij elke rot-key een refresh gebeurt, want dan werkt de speedup feature niet
  if ( (!ui_Redraw) || ((millis() - updateLastMillis) < REFRESH_DELAY))
      return;

  lcd.setBacklight(1);
  backlightOn = true;
  ui_Redraw = false;
  updateLastMillis = millis();

  ui_ShowAddr(ui_CurLoc);
  ui_ShowSpeed (ui_CurSpeed);

  if ((ui_State == UISTATE_HOME_PAGE1) || (ui_State == UISTATE_HOME_PAGE2)) { 
    // lijn 1
    lcd.setCursor(0,0);
    lcd.print("HOME");

    clearLine(1);
    clearLine(2);
    
    // lijn 4
    lcd.setCursor(0,4);
    if (ui_State == UISTATE_HOME_PAGE1) lcd.print("LOC   WIS  ACC   ->");
    else if (ui_State == UISTATE_HOME_PAGE2) {
        lcd.setCursor(0,4);
        lcd.print("PROG  SETUP       ->");
    }
    return;
  }
  else if ((ui_State == UISTATE_LOC_DO_SPEED) 
        || (ui_State == UISTATE_LOC_DO_FUNCS)
        || (ui_State == UISTATE_LOC_DO_ADDR)) {
    lcd.setCursor(0,0);
    lcd.print("LOC ");
    clearLine(1);
    lcd.setCursor (0,1);
    app_GetLocName(ui_CurLoc,ui_locName); // hoeft dit elke keer??
    lcd.print(ui_locName);
    //todo : track status op ln 3
    clearLine(2);
    lcd.setCursor (0,2);lcd.print("MAIN OK");

    if ( (ui_State == UISTATE_LOC_DO_SPEED) || (ui_State == UISTATE_LOC_DO_FUNCS)) {
      lcd.setCursor (0,3);
      lcd.print("back  ");
      ui_ShowLocFuncs(ui_Page);
      lcd.print(" ->");
      return;
    }
    else if (ui_State == UISTATE_LOC_DO_ADDR) {
      lcd.setCursor(0,1); 
      lcd.print("?");
      lcd.print(ui_NewLoc);
      lcd.print(':');
      app_GetLocName(ui_NewLoc,ui_locName); 
      lcd.print(ui_locName);
      lcd.setCursor(0,2); lcd.print("kies nieuwe loc:");
      lcd.setCursor (0,3); lcd.print("back  up  down  OK");
      return;
    }
  }
  else if (ui_State == UISTATE_TURNOUT_PAGE1) {
    lcd.setCursor(0,0);
    lcd.print("WIS ");
    
    lcd.setCursor(0,1); lcd.print(emptyLine);
    lcd.setCursor(0,2); lcd.print(emptyLine);
    
    clearLine(3);
    lcd.setCursor (0,3); 
    lcd.print("back  ");
    ui_ShowTurnoutPos(ui_Page);
    lcd.print(" ->");

    return;
  }
  else if (ui_State == UISTATE_ACC_PAGE1) {
    lcd.setCursor(0,0);
    lcd.print("ACC ");
    lcd.setCursor(0,2); lcd.print("scherm niet af!");
    clearLine(3);
    lcd.setCursor(0,3); lcd.print("back");
    return;
  }
  else if (ui_State == UISTATE_PROG_PAGE1) {
    lcd.setCursor(0,0);
    lcd.print("PROG ");
    lcd.setCursor(0,2); lcd.print("scherm niet af!");
    lcd.setCursor(0,3); lcd.print("back");
    return;
  }
  else if (ui_State == UISTATE_SETUP_PAGE1) {
    lcd.setCursor(0,0);
    lcd.print("SETUP ");
    lcd.setCursor(0,2); lcd.print("scherm niet af!");
    lcd.setCursor(0,3); lcd.print("back");
    return;
  }
  else if ( (ui_State == UISTATE_EVENT_MAINSHORT) ||
            (ui_State == UISTATE_EVENT_PROGSHORT) ||
            (ui_State == UISTATE_EVENT_EXTSTOP)) {
    clearLine(0);clearLine(1);clearLine(2);clearLine(3);
    lcd.setCursor(17,3);
    lcd.print("OK");
    lcd.setCursor(5,1);
    if (ui_State == UISTATE_EVENT_MAINSHORT) lcd.print("MAIN SHORT");
    else if (ui_State == UISTATE_EVENT_PROGSHORT) lcd.print("PROG SHORT");
    else if (ui_State == UISTATE_EVENT_EXTSTOP) lcd.print("EXTERNE STOP");
  }
} // ui_Update

void hwEvent_Handler (hwEvent_t event) {
  ui_Redraw = true; // trigger het display
  if (event == HWEVENT_MAINSHORT) ui_State = UISTATE_EVENT_MAINSHORT;
  else if (event == HWEVENT_PROGSHORT) ui_State = UISTATE_EVENT_PROGSHORT;
  else if (event == HWEVENT_EXTSTOP) ui_State = UISTATE_EVENT_EXTSTOP;
    
} // hwEvent_Handler

// ontvangt de KEY_ROTDOWN & KEY_ROTUP & KEY_ENTER events
// en produceert een ui_CurSpeed voor de app interface
// return : keyhandled
#define UI_NUM_SPEED_STEPS 128 // eventueel een eeprom setup variable van maken (28 steps)
// define UI_NUM_SPEED_STEPS 28
#define DCC_MINSPEED 2 // 0 en 1 zijn stops, 0 = STOP, 1 = EMERGENCY STOP
#define DCC_MAXSPEED 127
uint32_t speedkeyLastMillis;

static bool handleSpeedKeys (key_t key) {
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
  if (keyCode == KEY_ROTUP) {
    curSpeed += speedStep;
    if (curSpeed < DCC_MINSPEED) curSpeed = DCC_MINSPEED; // hiermee ga je van 0 naar 2
    if (curSpeed > DCC_MAXSPEED) curSpeed = DCC_MAXSPEED;
    ui_CurSpeed = curSpeed | dirBit;
    keyHandled = true;
  }
  else if (keyCode == KEY_ROTDOWN) {
    curSpeed -= speedStep;
    if ((curSpeed < DCC_MINSPEED) || (curSpeed > DCC_MAXSPEED)) curSpeed = 0;
    ui_CurSpeed = curSpeed | dirBit;
    keyHandled = true;
  }
  else if (keyCode == KEY_ENTER) { // de switch op de rotary encoder
    if (curSpeed) ui_CurSpeed = dirBit; // zero speed, but leave direction bit
    else {
        // enter drukken bij stilstand togglet de rijrichting
        ui_CurSpeed ^= DIRECTION_BIT; 
    }
    keyHandled = true;
  }
  if (keyHandled) app_SetSpeed(ui_CurLoc,ui_CurSpeed); 
  return (keyHandled);
} // handleSpeedKeys

void keys_Handler (key_t key) {
  uint8_t keyCode, keyEvent;
  bool keyHandled = false;

  keyCode = key & KEYCODEFILTER;
  keyEvent = key & KEYEVENTFILTER;
  
  // voorlopig de key-up events negeren
  if (keyEvent == KEYEVENT_UP) return;
  
  ui_Redraw = true; // bij elke key de ui redraw vragen
  // voorlopig events hier afhandelen
  if ((ui_State == UISTATE_EVENT_MAINSHORT) || (ui_State == UISTATE_EVENT_PROGSHORT) || (ui_State == UISTATE_EVENT_EXTSTOP)) {
    if ((keyCode = KEY_ENTER) || (keyCode = KEY_KEY1) || (keyCode = KEY_KEY2) || (keyCode = KEY_KEY3) || (keyCode = KEY_KEY4)) {
      app_SetCSState(RUN_OKAY);
      ui_State = UISTATE_DEFAULT; // DO_LOC_SPEED
    }
    return; // dus kunnen we verder niets doen zolang dit event niet is opgelost
  }

  // draaiknop hier afhandelen (voorlopig)
  if ((keyCode == KEY_ROTUP) || (keyCode == KEY_ROTDOWN) || (keyCode == KEY_ENTER)) keyHandled = handleSpeedKeys (key);
  if (keyHandled) return;
  
  keyHandled = ui_doHomeMenu (key);
  if (keyHandled) return;
  keyHandled = ui_doLocMenu (key);
  if (keyHandled) return;
  keyHandled = ui_doTurnoutMenu (key);
  if (keyHandled) return;
  keyHandled = ui_doAccMenu (key);
  if (keyHandled) return;
  
  if (!keyHandled)
  {
      // whatever
  }
  
} // keys_Handler
