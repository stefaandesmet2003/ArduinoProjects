#include <arduino.h>
#include "encoder.h"

/*
 * CLK = pin D2
 * DT = pin D3
 * SW = pin D7
 * als DT achterloopt op CLK -> wijzerzin turn
 * als DT voorloopt op CLK -> tegenwijzerzin turn
 */

volatile int turns; // aantal turns gedetecteerd vooraleer de main loop er iets mee doet
static encoder_CallBackFunc encoder_cb = NULL;

#define PinCLK        3   // Used for generating interrupts using CLK signal
#define PinDT         4   // Used for reading DT signal
#define PinSW         5   // Used for the push button switch
#define PinRedButton  18  // A4 = rode emergency stop

/*
 * de INT1 interrupt aangeroepen bij elke change van CLK
 * 
  */
void isr () 
{
  int clk, dt;
  clk = digitalRead(PinCLK);
  dt = digitalRead(PinDT);
  if (clk == dt)
    turns--;
  else
    turns++;
}


void encoder_setup()
{
 pinMode(PinCLK,INPUT_PULLUP);
 pinMode(PinDT,INPUT_PULLUP);  // geen pullups van de arduino gebruiken, er zitten al pullups op de module
 pinMode(PinSW,INPUT_PULLUP); // er zit geen pullup weerstand op de module voor de SWITCH
 pinMode(PinRedButton,INPUT_PULLUP);

 attachInterrupt (1,isr,CHANGE);   // interrupt 0 is always connected to pin 2 on Arduino UNO, int1 on pin3
}

void encoder_setCallback(encoder_CallBackFunc cb)
{
  encoder_cb = cb;
}

// vanuit deze functie worden de event callbacks gemaakt
void encoder_loop ()  {
  int copyTurns = 0;

  cli();
  if (turns != 0)
  {
    // isr heeft een beweging gedetecteerd
    copyTurns = turns;
    turns = 0;
    sei();
  }
 sei();
 
 if ((encoder_cb) && (copyTurns !=0))
  {
    if (copyTurns > 0)
    {
      (encoder_cb)(EVENT_UP | (min(copyTurns,15) << 2)); // we beperken het aantal turns tot 15, allicht nog teveel!
    }
    else
    {
      (encoder_cb)(EVENT_DOWN | (min(-copyTurns,15) << 2)); // we beperken het aantal turns tot 15, allicht nog teveel!
    }
  }

 // werkt polling van de switch of gebruiken we beter een pin change interrupt??

 // TODO! keyup/keydown
 // nu gaat deze code events genereren zolang de knop is ingedrukt!
  if (!(digitalRead(PinSW))) 
  {
    // check if pushbutton is pressed
    (encoder_cb)(EVENT_BUTTON | BUTTON_ENCODER_SWITCH);
  }

  if (!(digitalRead(PinRedButton))) 
  {
    // check if pushbutton is pressed
    (encoder_cb)(EVENT_BUTTON | BUTTON_RED); 
  }
 
}


