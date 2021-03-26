#include "widgets.h"
#include "Ucglib.h" // for the widget graphics
extern Ucglib_ILI9341_18x240x320_HWSPI ucg;

Widget::Widget(int ix, int iy, char *clabel)
{
  x = ix;
  y = iy;
  label = clabel;
  cbFunc = NULL;
  focus = -1; // default is een widget dat geen focus kan krijgen
}

void Widget::SetCallback (CallBackFunc ptrFunc, uint8_t funcArg)
{
  cbFunc = ptrFunc;
  cbArg = funcArg;
}

void Widget::Draw()
{
  if (redraw_required)
  {
    ucg.drawString(x,y, 0, label);
    ucg.drawRFrame(x-2,y-2,ucg.getStrWidth(label)+6,18+6,3); // font height+6
    redraw_required = false;
  }
}

Button::Button(int ix,int iy, char *clabel) : Widget (ix,iy,clabel)
{
  focus = 0;
}


void Button::Draw()
{
  if (redraw_required)
  {
    ucg.drawString(x,y, 0, label);
    if (focus==1)
    {
      // rode kader voor element in focus
      ucg.setColor(255,0,0);
    }
    ucg.drawRFrame(x-2,y-2,ucg.getStrWidth(label)+6,18+6,3); // font height+6
    ucg.setColor(255,255,255);
    redraw_required = false;    
  }
    
  //Serial.print("Button::Draw ");
  //Serial.print(label);
  //Serial.print(" - focus = ");
  //Serial.println(focus);
}

// return 0 = key unhandled, 1 = key handled
uint8_t Button::Key(uint8_t keyval)
{
  uint8_t retval = 0;
  switch(keyval)
  {
    case KEY_NONE : 
      retval = 1;
      break;
    case KEY_ENTER :
      if (cbFunc != NULL)
      {
        (cbFunc)(cbArg);
      }
      retval = 1;
      break;
  }
  return retval;
}

void ButtonLed::Draw()
{
  if (redraw_required)
  {
    if (on==1)
    {
      // invert colors : white bg, black text
      ucg.drawRBox(x-2,y-2,ucg.getStrWidth(label)+6,18+6,3);
      ucg.setColor(0,0,0);
      ucg.drawString(x,y, 0, label);
      ucg.setColor(255,255,255);
    }
    else
    {
      // normal colors : black bg, white text
      ucg.setColor(0,0,0);
      ucg.drawRBox(x-2,y-2,ucg.getStrWidth(label)+6,18+6,3);
      ucg.setColor(255,255,255);
      ucg.drawString(x,y, 0, label);
    }
    if (focus==1)
    {
      // rode kader voor element in focus
      ucg.setColor(255,0,0);
    }
    ucg.drawRFrame(x-2,y-2,ucg.getStrWidth(label)+6,18+6,3); // font height+6
    ucg.setColor(255,255,255);
    redraw_required = false;
  }
  
  //Serial.print("ButtonLed ");
  //Serial.print(label);
  //Serial.print(" ,strwidth = ");
  //Serial.println(ucg.getStrWidth(label));
}
void Label::Draw()
{
  Widget::Draw();
 
  //Serial.print("Label: ");
  //Serial.print(label);
  //Serial.print(" - focus = ");
  //Serial.println(focus);
}

Speedo::Speedo(int x, int y) : Widget (x,y,"speedo")
{
  speed = 0; 
  focus = 0;
}

void Speedo::Draw()
{
  char spdString[15];
  ucg_int_t strwidth;
  
  if (redraw_required)
  {
    sprintf(spdString,"Speed : %03d",speed);
    strwidth = ucg.getStrWidth(spdString);
    // voorlopig een boring speedo : gewoon een text box
    ucg.setColor(0,0,0);
    ucg.drawRBox(x-2,y-2,strwidth+6,18+6,3); // clear the text box, because text has changed
    ucg.setColor(255,255,255);
    ucg.drawRFrame(x-2,y-2,strwidth+6,18+6,3);
    
    ucg.drawString(x,y, 0, spdString);
  }
  
  //Serial.print("Speed : ");
  //Serial.print(speed);
  //Serial.print(" - focus = ");
  //Serial.println(focus);
  
}

uint8_t Speedo::Key(uint8_t keyValue)
{
  uint8_t retval = 0;
  // todo : boundary checks, als er veel keys komen , de speed sneller verhogen, ...
  switch(keyValue)
  {
    case KEY_ROTATE_RIGHT :
      speed++;
      retval = 1;
      break;
    case KEY_ROTATE_LEFT :
      speed--;
      retval = 1;
      break;
  }
  if (cbFunc != NULL)
  {
    (cbFunc)(cbArg);
  }
  if (retval == 1)
    redraw_required = true;
  return retval;
} // Speedo::Key

Menu::Menu()
{
  lst = NULL;
  idFocus = -1;
  nbrWidgets = 0;
}

void Menu::SetMenu(Widget* wlst[], int nbrOfWidgets)
{
  lst = wlst;
  nbrWidgets = nbrOfWidgets;
  idFocus = -1; // invalid, no widget has focus
  
  // set focus on 1st widget that can have focus
  int i=0;
  while (i < nbrWidgets)
  {
    if (lst[i]->GetFocus() != -1)
    {
      lst[i]->SetFocus(true);
      idFocus = i;
      break;
    }
    i++;
  }
  redraw_required = true;
}

void Menu::Draw()
{
  if (!nbrWidgets)
    // no menu attached, can't draw anything
    return;

  if (redraw_required)
  {
    ucg.clearScreen();
    redraw_required = false;
  }
  
  for (int i=0;i<nbrWidgets;i++)
  {
    lst[i]->Draw();
  }
} // Menu::Draw

uint8_t Menu::Key(uint8_t keyValue)
{
  uint8_t keyHandled = 0;
  uint8_t idNextFocus;

  if (!nbrWidgets)
    // no menu attached, can't handle the key
    return 0;
  
  keyHandled = lst[idFocus]->Key(keyValue);
  
  // try handling the key in the menu
  if (!keyHandled)
  {
    // rotate keys move focus up and down
    if (keyValue == KEY_ROTATE_RIGHT)
    {
      // pass focus to next widget
      idNextFocus = idFocus;
      do
      {
        idNextFocus = (idNextFocus+1) % nbrWidgets;
      } while (lst[idNextFocus]->GetFocus() == -1);
    }
    else if (keyValue == KEY_ROTATE_LEFT)
    {
      idNextFocus = idFocus;
      do
      {
        idNextFocus = (idNextFocus+nbrWidgets-1) % nbrWidgets; // avoid going below 0, modulo doesn't work like we want it here
      } while (lst[idNextFocus]->GetFocus() == -1);
    }
    if ((keyValue == KEY_ROTATE_RIGHT) || (keyValue == KEY_ROTATE_LEFT))
    {
      lst[idFocus]->SetFocus(false);
      lst[idNextFocus]->SetFocus(true);
      idFocus = idNextFocus;
      keyHandled = 1;
    }
  }
  if (!keyHandled)
  {
    // unhandled key in menu --> call the callback
    // TODO !! dat is bullshit, want de callback weet nu niet welke key het was!!
    if (cbFunc != NULL)
    {
      (cbFunc)(cbArg);
    }
  }
  return (keyHandled);
  
} //Menu::Key

void Menu::SetCallback (CallBackFunc ptrFunc, uint8_t funcArg)
{
  cbFunc = ptrFunc;
  cbArg = funcArg;
}
