#include "Arduino.h"
#include "AccessoryDecoderSTM8.h" // hardware config
#include "TurnoutDecoder.h"
#include "Timer.h"
#include "NmraDcc.h"

int8_t timerId;

uint8_t pulse_duration[4];
uint8_t decoderOutputs[8] = {PIN_OUTPUT0,PIN_OUTPUT1,PIN_OUTPUT2,PIN_OUTPUT3,
                             PIN_OUTPUT4,PIN_OUTPUT5,PIN_OUTPUT6,PIN_OUTPUT7}; // deze global zou beter in accessoryDecoder.ino staan zeker?

uint16_t getMyAddr(void); // uit nmraDcc.cpp --> in de Dcc class steken?

//void turnout_TimerCallback (void); // voorlopig laten we de timer library het werk doen
uint8_t turnoutPositions[4]; // 0 = rechtdoor, 1 = afslaan; todo : aanpassen als er DCC commando's binnenkomen

void turnout_Init() {
  int i;

  pulse_duration[0] = DCC_getCV (CV_TimeOnOutput1);
  pulse_duration[1] = DCC_getCV (CV_TimeOnOutput2);
  pulse_duration[2] = DCC_getCV (CV_TimeOnOutput3);
  pulse_duration[3] = DCC_getCV (CV_TimeOnOutput4);

  // set output mode on outputs
  for (i=0;i<8;i++)
    pinMode(decoderOutputs[i], OUTPUT);
  // TODO : if saved state, herstel deze state uit de CV's
} // turnout_Init


// OutputId : 0..7
// activate = true : activate output, 0 = deactivate output (voorlopig gebeurt dit automatisch door de handler)
// FLAGS_MY_ADDRESS_ONLY : not used for now, so we filter here
void turnout_Handler(uint16_t decoderAddress, uint8_t outputId, bool activate) {
  uint8_t pairedOutputId = outputId ^ 0x1; // laagste bit flippen
  #ifdef DEBUG
    Serial_print_s("turnout_Handler: ");
    Serial_print_u(decoderAddress);
    Serial_print_s(",");
    Serial_print_u(outputId);
    Serial_print_s(",");
    Serial_println_ub(activate,HEX);
  #endif

  // filter own address or not? remove filter if decoder needs to respond to multiple addresses
  // alternatively set FLAGS_MY_ADDRESS_ONLY, to enable filtering by nmradcc lib
  if ((getMyAddr() != decoderAddress) && (decoderAddress != 511)) { // 511 = broadcast
    return;
  }

  if (activate) {
    #ifdef DEBUG
      Serial_print_s("[");
      Serial_print_u (millis());
      Serial_print_s("] : ");
      Serial_print_u(decoderOutputs[pairedOutputId]);
      Serial_println_s(" = LOW");
      Serial_print_s("[");
      Serial_print_u(millis());
      Serial_print_s("] : ");
      Serial_print_u(decoderOutputs[outputId]);
      Serial_println_s(" = HIGH");
    #endif
    digitalWrite(decoderOutputs[pairedOutputId], LOW); // eerste de andere coil deactiveren!
    digitalWrite(decoderOutputs[outputId], HIGH);
    timerId = Timer_pulseImmediate(decoderOutputs[outputId], pulse_duration[outputId>>1] * 50L, HIGH);
    Timer_oscillate(PIN_PROGLED, LED_FAST_FLASH, HIGH, 1); // 1 led flashes ter bevestiging van een turnout commando
  }
  else {
    // voorlopig gaan we de off commands negeren, want jmri stuurt die te snel na de on
    // digitalWrite(decoderOutputs[outputId], LOW);
    /*
    #ifdef DEBUG
      Serial_print_s("[");
      Serial_print_u (millis());
      Serial_print_s("] : ");
      Serial_print_u(decoderOutputs[outputId]);
      Serial_println_s(" = LOW");
    #endif
    */
  }
} // turnout_Handler

// 0 = OK, 1 = NOK
uint8_t turnout_FactoryResetCV () {
  int i;
  if (!DCC_isSetCVReady())
    return 1;
    
  for (i=0; i < 4; i++) {
    DCC_setCV(CV_TimeOnOutput1+i, DEFAULT_TIMEONOUTPUT);
  }
  return 0;
  
} // turnout_FactoryResetCV 

// turnoutId = 0..3, manuele bediening met de knoppen
void turnout_ManualToggle (uint8_t turnoutId) {
  #ifdef DEBUG
    Serial_println_s ("manual!");
  #endif
  turnoutPositions[turnoutId] = (!turnoutPositions[turnoutId]) & 0x1;
  turnout_Handler(getMyAddr(),(turnoutId << 1) + turnoutPositions[turnoutId],true);
} // turnout_ManualToggle
