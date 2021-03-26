#include <SPI.h>
#include "Ucglib.h"

#include "widgets.h"
#include "encoder.h"


#define MENU_MAIN_ID 1
#define MENU_LOC_ID 2

//
#define BUTTON_DIR_ID     0 // also index in bButtons array!!
#define BUTTON_F1_ID      1
#define BUTTON_F2_ID      2
#define BUTTON_F3_ID      3
#define BUTTON_F4_ID      4
#define BUTTON_CANCEL_ID  6
#define BUTTON_OK_ID      7

//the menu handler
Menu curMenu; // there can only be 1 menu
uint8_t curMenuID;
Widget* lstWidgets[10]; // list used for all menus

// widgets
Label lbLocAddress(0,0,"3");
Speedo spSpeedo1 (30,30);

// 5 general purpose buttons - functions redefined per menu (voorlopig geen malloc)
ButtonLed bButtons[5];
Button bButtonCancel(20,25,"Cancel");
Button bButtonOK(20,25,"OK");

// display
Ucglib_ILI9341_18x240x320_HWSPI ucg(/*cd=*/ 14 , /*cs=*/ 16, /*reset=*/ 15);

// the useful data controlled by this UI
char locString[] = "LOK : xx";
uint16_t locID = 3; // default
int locSpeed = 0;

void setup() 
{
  Serial.begin(9600);

  // setup menu
  curMenuID = MENU_MAIN_ID;
  setup_Menu(MENU_MAIN_ID);

  // setup display
  ucg.begin(UCG_FONT_MODE_TRANSPARENT);
  ucg.setFont(ucg_font_helvB18_hr);
  ucg.setColor(0,255,255,255);
  ucg.clearScreen();
  ucg.setFontPosTop(); // dit heeft impact op de kaderkes in widgets.cpp
  ucg.setRotate270();

  curMenu.Draw();
  // setup rotary encoder
  encoder_setup();
  encoder_setCallback(encoderCallBack);
  
} // setup

void setup_Menu (uint8_t menuID)
{
  switch(menuID)
  {
    case MENU_MAIN_ID :
      setup_MainMenu();
      break;
    case MENU_LOC_ID :
      setup_LocMenu();
      break;
    default:
      break;
  }
}
void setup_MainMenu()
{
  sprintf (locString,"LOK : %02d",locID);
  lbLocAddress.SetLabel(locString);
  lbLocAddress.SetPos(10,30);
  lstWidgets[0] = &lbLocAddress;
  lstWidgets[1] = &spSpeedo1;
  curMenu.SetMenu(lstWidgets,2);
  curMenu.SetCallback(menuCallback, MENU_MAIN_ID); 
  curMenuID = MENU_MAIN_ID;
  spSpeedo1.SetPos(120,65);
  spSpeedo1.SetSpeed(locSpeed);
  spSpeedo1.SetCallback(speedoCallback,3); // 3 = lokID bv.
}

void setup_LocMenu ()
{
  lbLocAddress.SetPos(10,30);
  sprintf (locString,"LOK : %02d",locID);
  lbLocAddress.SetLabel(locString);

  //redefine the generic buttons
  bButtons[0].SetPos(10,160);
  bButtons[0].SetLabel("<>");
  bButtons[0].SetCallback(buttonCallback,BUTTON_DIR_ID);
  bButtons[1].SetPos(70,160);
  bButtons[1].SetLabel("F1");
  bButtons[1].SetCallback(buttonCallback,BUTTON_F1_ID);
  bButtons[2].SetPos(130,160);
  bButtons[2].SetLabel("F2");
  bButtons[2].SetCallback(buttonCallback,BUTTON_F2_ID);
  bButtons[3].SetPos(190,160);
  bButtons[3].SetLabel("F3");
  bButtons[3].SetCallback(buttonCallback,BUTTON_F3_ID);
  bButtons[4].SetPos(250,160);
  bButtons[4].SetLabel("F4");
  bButtons[4].SetCallback(buttonCallback,BUTTON_F4_ID);

  bButtonCancel.SetPos(50,200);
  bButtonCancel.SetCallback(buttonCallback,BUTTON_CANCEL_ID);
  bButtonOK.SetPos(200,200);
  bButtonOK.SetCallback(buttonCallback,BUTTON_OK_ID);

  lstWidgets[0] = &lbLocAddress;
  for (int i=0;i<5;i++)
  {
    lstWidgets[i+1] = &bButtons[i];
  }
  lstWidgets[6] = &bButtonCancel;
  lstWidgets[7] = &bButtonOK;
  curMenu.SetMenu(lstWidgets,8);
  curMenu.SetCallback(menuCallback, MENU_LOC_ID); 
  
}
void buttonCallback(uint8_t buttonID) // buttonID is how we use the arg for button callbacks
{
  uint8_t funcState;
  if (curMenuID == MENU_LOC_ID)
  {
    switch (buttonID)
    {
      case BUTTON_DIR_ID :
        Serial.println("toggle DIR");
        funcState = bButtons[buttonID].GetState();
        bButtons[buttonID].SetState(!funcState);
        // send command to the CommandStation
        break;
      case BUTTON_F1_ID :
        funcState = bButtons[buttonID].GetState();
        bButtons[buttonID].SetState(!funcState);
        Serial.println("toggle F1");
        break;
      case BUTTON_F2_ID :
        funcState = bButtons[buttonID].GetState();
        bButtons[buttonID].SetState(!funcState);
        Serial.println("toggle F2");
        break;
      case BUTTON_F3_ID :
        funcState = bButtons[buttonID].GetState();
        bButtons[buttonID].SetState(!funcState);
        Serial.println("toggle F3");
        break;
      case BUTTON_F4_ID :
        funcState = bButtons[buttonID].GetState();
        bButtons[buttonID].SetState(!funcState);
        Serial.println("toggle F4");
        break;
      case BUTTON_CANCEL_ID :
        curMenuID = MENU_MAIN_ID; // go back to main menu
        setup_Menu(curMenuID);
        break;
      case BUTTON_OK_ID :
        curMenuID = MENU_MAIN_ID; // go back to main menu
        setup_Menu(curMenuID);
        break;
    }
  }
  else
  {
    // ...
  }
} // buttonCallback

void menuCallback (uint8_t menuID) // menuID is how we use the arg for button callbacks
{
  switch (menuID)
  {
    case MENU_MAIN_ID :
      // press any key to leave main menu and go to setup menus 
      curMenuID = MENU_LOC_ID;
      setup_Menu(curMenuID);
      break;
    case MENU_LOC_ID : 
      //unhandled key in LOC menu?;
      break;
    default :
      Serial.println("menuCallback error : unknown menuID!");
  }
}

void speedoCallback (uint8_t speedoID)
{
  locSpeed = spSpeedo1.GetSpeed();
  //Serial.print("Speedo callback : speed changed to ");
  //Serial.println(spSpeedo1.GetSpeed());
}

void loop() 
{
  char kar;
  uint8_t key;
  if (Serial.available())
  {
    kar = Serial.read();
    switch (kar)
    {
      case 'e':
      case 'E': 
        key = KEY_ENTER;
        break;
      case '+' : 
        key = KEY_ROTATE_RIGHT;
        break;
      case '-' : 
        key = KEY_ROTATE_LEFT;
        break;
      default : 
        key = KEY_NONE;
        break;
    }
    // pass the key to the menu (hier moet nog een app-level komen, die switcht tussen menus)
    if (key!= KEY_NONE)
    {
      curMenu.Key(key);
      curMenu.Draw();
    }
  }
  // trigger callbacks from the rotary encoder
  encoder_loop();

}

void encoderCallBack(uint8_t event)
{
  uint8_t nbrOfClicks = event >> 2;
  uint8_t key;
  switch (event & 0x3)
  {
    case EVENT_UP :
      key = KEY_ROTATE_RIGHT;
      nbrOfClicks = event >> 2;
      Serial.println("<+>");
      break;
    case EVENT_DOWN : 
      key = KEY_ROTATE_LEFT;
      nbrOfClicks = event >> 2;
      Serial.println("<->");
      break;
    case EVENT_BUTTON :
      key = event & 0xFC;
      nbrOfClicks = 1;
      if (key == BUTTON_ENCODER_SWITCH)
      {
        Serial.print("<E>");
        key = KEY_ENTER;
      }
        
      else if (key == BUTTON_RED)
      {
        // een beetje hack - nog opkuisen, menu-keys en keys op de remote zijn verschillende defines
        // buuuh!
        // en de emergency knop wordt nog niet in de ui verwerkt, dus doen we het voorlopig hier als test
        // er gebeurt wel geen refresh
        Serial.print("<R>");
        locSpeed = 0;
        spSpeedo1.SetSpeed(locSpeed);
        curMenu.Draw();
        key= KEY_NONE;
      }
      break;
  }
  if (key != KEY_NONE)
  {
    for (int i=0; i<nbrOfClicks;i++)
    {
      curMenu.Key(key); 
    }
    curMenu.Draw();
  }
  
}


