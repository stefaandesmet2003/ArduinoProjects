#include "Arduino.h"
#include "AccessoryDecoder.h" // hardware config
#include "TurnoutDecoder.h"
#include "Timer.h"
#include "NmraDcc.h"

extern Timer timer;
int8_t timerId;

extern NmraDcc  Dcc;
uint8_t pulseDuration[4];
uint8_t decoderOutputs[8] = {PIN_OUTPUT0,PIN_OUTPUT1,PIN_OUTPUT2,PIN_OUTPUT3,
                             PIN_OUTPUT4,PIN_OUTPUT5,PIN_OUTPUT6,PIN_OUTPUT7}; // deze global zou beter in accessoryDecoder.ino staan zeker?

uint16_t getMyAddr(void); // uit nmraDcc.cpp --> in de Dcc class steken?

//void turnout_TimerCallback (void); // voorlopig laten we de timer library het werk doen
uint8_t turnoutPositions[4]; // 0 = rechtdoor, 1 = afslaan; todo : aanpassen als er DCC commando's binnenkomen

void turnout_Init() {
  int i;

  pulseDuration[0] = Dcc.getCV (CV_TimeOnOutput1);
  pulseDuration[1] = Dcc.getCV (CV_TimeOnOutput2);
  pulseDuration[2] = Dcc.getCV (CV_TimeOnOutput3);
  pulseDuration[3] = Dcc.getCV (CV_TimeOnOutput4);

  // set output mode on outputs
  for (i=0;i<8;i++)
    pinMode(decoderOutputs[i], OUTPUT);
  // TODO : if saved state, herstel deze state uit de CV's
  #ifdef DEBUG
    for (int i=0;i<4;i++) {
      Serial.print("pulseDuration output ");Serial.print(i);
      Serial.print(" : ");Serial.print(pulseDuration[i]*50);Serial.println("ms");
    }
  #endif

} // turnout_Init


// OutputId : 0..7
// activate = true : activate output, 0 = deactivate output (voorlopig gebeurt dit automatisch door de handler)
// FLAGS_MY_ADDRESS_ONLY : not used for now, so we filter here
void turnout_Handler(uint16_t decoderAddress, uint8_t outputId, bool activate) {
  uint8_t pairedOutputId = outputId ^ 0x1; // laagste bit flippen
  #ifdef DEBUG
    Serial.print("turnout_Handler: ");
    Serial.print(decoderAddress);
    Serial.print(',');
    Serial.print(outputId);
    Serial.print(',');
    Serial.println(activate);
  #endif

  // not using FLAGS_MY_ADDRESS_ONLY in nmradcc lib
  // so we get all basic accessory packets here, 
  // and need to filter the decoderAddresses we are interested in
  if ((getMyAddr() != decoderAddress) && (decoderAddress != 511)) { // 511 = broadcast
    return;
  }

  // particular case of emergency off (RCN-213)
  if ((decoderAddress == 511) && (outputId == 3)) {
    for (int i=0;i<8;i++) digitalWrite(decoderOutputs[i],LOW); // all outputs off
    timer.oscillate(PIN_PROGLED, LED_FAST_FLASH, LOW, 1); // 1 led flashes ter bevestiging van een turnout commando
  return;
  }

  if (activate) { // 'on' command
    #ifdef DEBUG
      Serial.print("[");
      Serial.print (millis());
      Serial.print("] : ");
      Serial.print(decoderOutputs[pairedOutputId]);
      Serial.println(" = LOW");
      Serial.print("[");
      Serial.print (millis());
      Serial.print("] : ");
      Serial.print(decoderOutputs[outputId]);
      Serial.println(" = HIGH");
    #endif
    digitalWrite(decoderOutputs[pairedOutputId], LOW); // eerste de andere coil deactiveren!
    digitalWrite(decoderOutputs[outputId], HIGH);
    if (pulseDuration[outputId>>1]) // switch output auto-off after pulseDuration
      timerId = timer.pulseImmediate(decoderOutputs[outputId], pulseDuration[outputId>>1] * 50L, HIGH);
    timer.oscillate(PIN_PROGLED, LED_FAST_FLASH, LOW, 1); // 1 led flashes ter bevestiging van een turnout commando

  }
  else { // 'off' command
    // according to nmra spec, if pulseDuration == 0, outputs stay on until the 'off' command is received
    // this doesn't work well with JMRI that sends the 'off' command too fast for marklin turnouts -> use pulseDuration CV's
    if (pulseDuration[outputId>>1] == 0) { // no auto-off on the output, so switch off now
      digitalWrite(decoderOutputs[outputId], LOW);
    }
    #ifdef DEBUG
      Serial.print("[");
      Serial.print (millis());
      Serial.print("] : ");
      Serial.print(decoderOutputs[outputId]);
      Serial.println(" = LOW");
    #endif
  }
} // turnout_Handler

// 0 = OK, 1 = NOK
uint8_t turnout_FactoryResetCV () {
  int i;
  if (!Dcc.isSetCVReady())
    return 1;
    
  for (i=0; i < 4; i++) {
    Dcc.setCV( CV_TimeOnOutput1+i, DEFAULT_TIMEONOUTPUT);
  }
  return 0;
  
} // turnout_FactoryResetCV 

// turnoutId = 0..3, manuele bediening met de knoppen
void turnout_ManualToggle (uint8_t turnoutId, bool activate) {
  #ifdef DEBUG
    Serial.print ("manual toggle : ");
    Serial.print(turnoutPositions[turnoutId]);
    if (activate) Serial.println(",ON");
    else Serial.println(",OFF");
  #endif 
  if (activate)
    turnoutPositions[turnoutId] = (!turnoutPositions[turnoutId]) & 0x1;
  turnout_Handler(getMyAddr(),(turnoutId << 1) + turnoutPositions[turnoutId],activate);
} // turnout_ManualToggle
