#include "Arduino.h"
#include "keys.h"

volatile int turns; // aantal turns gedetecteerd vooraleer de main loop er iets mee doet
long encoderMillis = 0;

typedef enum {UP, DEBOUNCING_DOWN, DOWN, LONG_DOWN, DEBOUNCING_UP} debounceState_t;

uint8_t keypins[NUMBER_OF_DEBOUNCED_KEYS] = KEYPINS;
typedef struct
{
  //uint8_t Pin; // index in deze array is dezelfde als in keys[i].Pin === keypins[i]
  debounceState_t State;
  uint32_t LastMillis;
  //uint8_t Key; // index in deze array is de keycode
} debouncedKey_t;


// de index in deze array komt overeen met de keycode 
debouncedKey_t keys[NUMBER_OF_DEBOUNCED_KEYS]; // de enter toets op de rotary encoder en de rode/groene knop

static void detect_keys (void);

// aangeroepen bij elke change van CLK
void isr () 
{
  int clk; 
  int dt;
  encoderMillis = millis();
  clk = digitalRead(PIN_ROT_CLK); 
  dt = digitalRead(PIN_ROT_DT);
  if (clk == dt)
    turns--;
  else
    turns++;
}

// aangeroepen bij elke falling edge van CLK
void isrFalling () 
{
  int dt;
  encoderMillis = millis();
  dt = digitalRead(PIN_ROT_DT);
  if (0 == dt)
    turns--;
  else
    turns++;
}

void keys_Init (void)  
{
  pinMode(PIN_ROT_CLK,INPUT); // geen pullup van de arduino gebruiken, er zitten al 10K pullup op de module
  pinMode(PIN_ROT_DT,INPUT);  // geen pullup van de arduino gebruiken, er zitten al 10K pullup op de module
  attachInterrupt (1,isr,CHANGE);   // interrupt 0 is always connected to pin 2 on Arduino UNO, int1 to pin 3 (throttle hw)

  // de drukknoppen
  for (int i=0; i < NUMBER_OF_DEBOUNCED_KEYS; i++)
  {
      keys[i].State = UP;
      pinMode(keypins[i],INPUT_PULLUP); // er zit geen pullup weerstand op de module voor de SWITCH
  }
  
} // keys_Init

void keys_Update (void)
{
  int copyTurns = 0;
  uint8_t key;

  cli();
  if (copyTurns != turns)
  {
    // isr heeft een beweging gedetecteerd
    copyTurns = turns;
    turns = 0;
    sei();
    if (copyTurns > 0) key = KEY_ROTUP;
    else if (copyTurns < 0) key = KEY_ROTDOWN;
    if (keys_Handler)
        keys_Handler (key | KEYEVENT_NONE );
  }
  sei();

  // de button keys pollen en het debouncing state machine laten werken
  detect_keys ();
 
} // keys_Update

key_t keys_GetState (key_t key)
{
    key_t keyCode;
    keyCode = key & KEYCODEFILTER;
    if ( keyCode < NUMBER_OF_DEBOUNCED_KEYS )
    {
        return (keys[keyCode].State);
    }
    else
        return (keyCode | KEYEVENT_NONE);
} // keys_GetState


/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
static void detect_keys (void)
{
  int keycode;
  for (keycode=0;keycode<NUMBER_OF_DEBOUNCED_KEYS;keycode++)
  {
    switch(keys[keycode].State)
    {
      case UP : 
        if (digitalRead(keypins[keycode]) == LOW )
        {
          keys[keycode].LastMillis = millis();
          keys[keycode].State = DEBOUNCING_DOWN;
        }
        break;
      case DEBOUNCING_DOWN :
        if (digitalRead(keypins[keycode]) != LOW )
        {
          keys[keycode].State = UP;
        }
        else if ( (millis() - keys[keycode].LastMillis) > DEBOUNCE_DELAY )
        {
          keys[keycode].State = DOWN;
          if (keys_Handler)
            keys_Handler ( keycode | KEYEVENT_DOWN);
        }
        break;
      case DOWN :
        if (digitalRead(keypins[keycode]) != LOW )
        {
          keys[keycode].State = DEBOUNCING_UP;
          keys[keycode].LastMillis = millis();
        }
        else if ( (millis() - keys[keycode].LastMillis) > LONGPRESS_DELAY )
        {
          keys[keycode].State = LONG_DOWN;
          if (keys_Handler)
            keys_Handler (keycode | KEYEVENT_LONGDOWN);
        }
        break;
      case LONG_DOWN :
        if (digitalRead(keypins[keycode]) != LOW )
        {
          keys[keycode].State = DEBOUNCING_UP;
          keys[keycode].LastMillis = millis();
        }
        break;
      case DEBOUNCING_UP :
        if (digitalRead(keypins[keycode]) == LOW )
        {
          keys[keycode].LastMillis = millis();
        }
        else if ( (millis() - keys[keycode].LastMillis) > DEBOUNCE_DELAY )
        {
          keys[keycode].State = UP;
          if (keys_Handler)
            keys_Handler (keycode | KEYEVENT_UP);
        }
        break;
    }
  }
} // detect_keys
