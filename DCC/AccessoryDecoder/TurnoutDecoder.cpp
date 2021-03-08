#include "Arduino.h"
#include "AccessoryDecoder.h" // hardware config
#include "TurnoutDecoder.h"
#include "Timer.h"
#include "NmraDcc.h"

extern Timer timer;
int8_t timerId;

extern NmraDcc  Dcc ;
uint8_t pulse_duration[4];
uint8_t decoderOutputs[8] = {PIN_OUTPUT0,PIN_OUTPUT1,PIN_OUTPUT2,PIN_OUTPUT3,
                             PIN_OUTPUT4,PIN_OUTPUT5,PIN_OUTPUT6,PIN_OUTPUT7}; // deze global zou beter in accessoryDecoder.ino staan zeker?

uint16_t getMyAddr(void); // uit nmraDcc.cpp --> in de Dcc class steken?

//void turnout_TimerCallback (void); // voorlopig laten we de timer library het werk doen
uint8_t turnoutPositions[4]; // 0 = rechtdoor, 1 = afslaan; todo : aanpassen als er DCC commando's binnenkomen

void turnout_Init( void )
{
  int i;

  pulse_duration[0] = Dcc.getCV (CV_TimeOnOutput1);
  pulse_duration[1] = Dcc.getCV (CV_TimeOnOutput2);
  pulse_duration[2] = Dcc.getCV (CV_TimeOnOutput3);
  pulse_duration[3] = Dcc.getCV (CV_TimeOnOutput4);

  // set output mode on outputs
  for (i=0;i<8;i++)
    pinMode(decoderOutputs[i], OUTPUT);
  // TODO : if saved state, herstel deze state uit de CV's
  
} // turnout_Init


// OutputAddr : 0..7
// State = 1 : activeer output, 0 = deactiveer output (voorlopig gebeurt dit automatisch door de handler)
// FLAGS_MY_ADDRESS_ONLY : afhankelijk daarvan enkel own address, 
// in deze config enkel op MY_ADDRESS reageren!!
void turnout_Handler( uint16_t Addr, uint16_t BoardAddr, uint8_t OutputAddr, uint8_t State)
{
  uint8_t pairedOutputAddr = OutputAddr ^ 0x1; // laagste bit flippen
  Serial.print("turnout_Handler: ") ;
  Serial.print(Addr,DEC) ;
  Serial.print(',');
  Serial.print(BoardAddr,DEC) ;
  Serial.print(',');
  Serial.print(OutputAddr,DEC) ;
  Serial.print(',');
  Serial.println(State, HEX) ;
  /* voorlopig uit, nog checken of dit klopt en niet met Addr+1 of Addr-1 of zo
  if !(getMyAddr() == Addr)
  {
    return;
  }
  */
  if (State)
  {
    Serial.print("[");
    Serial.print (millis());
    Serial.print("] : ");
    Serial.print(decoderOutputs[pairedOutputAddr]);
    Serial.println(" = LOW");
    Serial.print("[");
    Serial.print (millis());
    Serial.print("] : ");
    Serial.print(decoderOutputs[OutputAddr]);
    Serial.println(" = HIGH");
    digitalWrite(decoderOutputs[pairedOutputAddr], LOW); // eerste de andere coil deactiveren!
    digitalWrite(decoderOutputs[OutputAddr], HIGH);
    timerId = timer.pulseImmediate(decoderOutputs[OutputAddr], pulse_duration[OutputAddr>>1] * 50L, HIGH);
    timer.oscillate(PIN_PROGLED, LED_FAST_FLASH, LOW, 1); // 1 led flashes ter bevestiging van een turnout commando

  }
  else
  {
    /* voorlopig gaan we de off commands negeren, want jmri stuurt die te snel */
    /*
    Serial.print("[");
    Serial.print (millis());
    Serial.print("] : ");
    Serial.print(decoderOutputs[OutputAddr]);
    Serial.println(" = LOW");
    digitalWrite(decoderOutputs[OutputAddr], LOW);
    */
  }
  
} // turnout_Handler

// 0 = OK, 1 = NOK
uint8_t turnout_FactoryResetCV ( void )
{
  int i;
  if (!Dcc.isSetCVReady())
    return 1;
    
  for (i=0; i < 4; i++)
  {
    Dcc.setCV( CV_TimeOnOutput1+i, DEFAULT_TIMEONOUTPUT);
  }
  return 0;
  
} // turnout_FactoryResetCV 

// turnoutId = 0..3, manuele bediening met de knoppen
void turnout_ManualToggle (uint8_t turnoutId)
{
    Serial.print ("manual toggle : ");
    Serial.println(turnoutPositions[turnoutId]);
    turnoutPositions[turnoutId] = (!turnoutPositions[turnoutId]) & 0x1;
    turnout_Handler(getMyAddr(),0,(turnoutId << 1) + turnoutPositions[turnoutId],1);
} // turnout_ManualToggle



