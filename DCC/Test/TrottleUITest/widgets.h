#ifndef __WIDGETS_H_
#define __WIDGETS_H_

#include <arduino.h>

#define KEY_NONE          0
#define KEY_ENTER         1
#define KEY_ROTATE_LEFT   2
#define KEY_ROTATE_RIGHT  3
#define KEY_FUNC1         4
#define KEY_FUNC2         5
#define KEY_FUNC3         6
#define KEY_FUNC4         7

typedef void (*CallBackFunc)(uint8_t arg);

class Widget 
{
  protected:
    int x, y;
    char *label;
    int8_t focus; // 0 = focus off, 1 = focus on, -1 = focus disabled (for labels eg.)
    CallBackFunc cbFunc;  
    uint8_t cbArg; // pass this argument when cbFunc has to be called
    bool redraw_required = true;

  public:
    Widget(int x, int y, char *label);
    void SetPos( int ix, int iy){ x=ix; y=iy; }
    void SetLabel(char *clabel){label = clabel;redraw_required=true;} // change label text after construction
    void SetFocus(uint8_t focusVal){focus = focusVal;redraw_required=true;}
    int8_t GetFocus(){return focus;}
    void SetCallback(CallBackFunc cbFunc, uint8_t cbArg);
    virtual uint8_t Key (uint8_t keyVal) {return 0;} // default : return key unhandled
    virtual void Draw() = 0;
}; //Widget

class Button : public Widget
{
  public:
    Button() : Widget (0,0,""){focus=0;}
    Button(int x, int y, char *label);
    virtual void Draw();
    uint8_t Key (uint8_t keyVal);
}; // Button

class ButtonLed : public Button
{
  uint8_t on;
  public:
    ButtonLed() : Button(){on=0;}
    void SetState(uint8_t stateOn){on=stateOn; redraw_required=true;}
    uint8_t GetState() {return on;}
    void Draw();
};

class Label : public Widget
{
  public:
    Label(int ix,int iy, char *clabel) : Widget (ix,iy,clabel){}
    void Draw();
}; // Label

// wijzigt een externe variable "speed"
class Speedo : public Widget
{
  int speed;
  
  public:
    Speedo(int x, int y);
    void SetSpeed (int speedVal){speed = speedVal;}
    int GetSpeed (){return speed;};
    void Draw();
    uint8_t Key (uint8_t keyVal);
};

class Menu
{
  Widget **lst;
  uint8_t nbrWidgets;
  int8_t idFocus; // welk widget heeft focus
  CallBackFunc cbFunc;
  uint8_t cbArg; // pass this argument when cbFunc has to be called
  bool redraw_required = true; // do a clearscreen and redraw all widgets
  
  public:
    Menu(void);
    void SetMenu (Widget* lst[], int nbrOfWidgets);
    void SetCallback(CallBackFunc cbFunc, uint8_t cbArg);
    uint8_t Key(uint8_t keyVal);
    void Draw();
}; // Menu


#endif
