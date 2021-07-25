#include "Arduino.h"
#include "AccessoryDecoder.h" // hardware config
#include "TurnoutDecoder.h"
#include "Timer.h"
#include "NmraDcc.h"

extern Timer timer;
static int8_t timerId;

extern NmraDcc  Dcc;
static uint8_t pulseDuration[4];
const static uint8_t decoderOutputs[8] = OUTPUTPINS;

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
  for (i=0;i<4;i++)
    turnoutPositions[i] = 0;
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

  // no dcc packet filtering in nmradcc lib
  // so we get all basic accessory packets here, 
  // and need to filter the decoderAddresses we are interested in
  if ((decoderAddress != getDecoderAddress()) && (decoderAddress != 511)) { // 511 = broadcast
    return;
  }

  // particular case of emergency off (RCN-213)
  if ((decoderAddress == 511) && (outputId == 3)) {
    for (int i=0;i<8;i++) digitalWrite(decoderOutputs[i],LOW); // all outputs off
    timer.oscillate(PIN_PROGLED, LED_FAST_FLASH, LOW, 1); // 1 led flashes ter bevestiging van een turnout commando
  return;
  }

  if (activate) { // 'on' command
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
      timer.oscillate(PIN_PROGLED, LED_FAST_FLASH, LOW, 1); // 1 led flashes ter bevestiging van een turnout commando
    }
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

// turnoutId = 0..3, manual toggling with the buttons
void turnout_ManualToggle (uint8_t turnoutId, bool activate) {
  #ifdef DEBUG
    Serial.print ("manual toggle : ");
    Serial.print(turnoutPositions[turnoutId]);
    if (activate) Serial.println(",ON");
    else Serial.println(",OFF");
  #endif
  if (activate)
    turnoutPositions[turnoutId] = (!turnoutPositions[turnoutId]) & 0x1;
  turnout_Handler(getDecoderAddress(),(turnoutId << 1) + turnoutPositions[turnoutId],activate);
} // turnout_ManualToggle

// handler for 8 unpaired outputs with on/off functions (lights, decoupler rails, etc)
// SOFTWAREMODE_OUTPUT_DECODER in CV33
// the decoder occupies 2 successive dcc accessory decoder addresses : CV1/9 & CV1/9 + 1
// outputId : 0..7 : 
// CV1/9    : O,1 map onto output 0; 2,3 onto output 1; 4,5 onto output 2; 6,7 onto output 3
// CV1/9 +1 : O,1 map onto output 4; 2,3 onto output 5; 4,5 onto output 6; 6,7 onto output 7
// activate = true on even outputId: activate output, true on odd outputId : deactive output
// activate = false : ignored
// this is how JMRI controls lights (light on = 'coil' 0 (even outputId), light off = 'coil' 1 (odd outputId)
void output_Handler(uint16_t decoderAddress, uint8_t outputId, bool activate) {
  uint8_t physicalOutput;
  #ifdef DEBUG
    Serial.print("output_Handler: ");
    Serial.print(decoderAddress);
    Serial.print(',');
    Serial.print(outputId);
    Serial.print(',');
    Serial.println(activate);
  #endif


  // no dcc packet filtering in nmradcc lib
  // so we get all basic accessory packets here, 
  // and need to filter the decoderAddresses we are interested in
  if ((decoderAddress != getDecoderAddress()) && 
      (decoderAddress != getDecoderAddress() + 1) && 
      (decoderAddress != 511)) { // 511 = broadcast
    return;
  }

  // particular case of emergency off (RCN-213)
  if ((decoderAddress == 511) && (outputId == 3)) {
    for (int i=0;i<8;i++) digitalWrite(decoderOutputs[i],LOW); // all outputs off
    timer.oscillate(PIN_PROGLED, LED_FAST_FLASH, LOW, 1); // 1 led flashes ter bevestiging van een turnout commando
  return;
  }
  physicalOutput = outputId >> 1;
  if (decoderAddress == getDecoderAddress() + 1) physicalOutput =+ 4;

  if (activate) { // 'on' command
    digitalWrite(decoderOutputs[physicalOutput], (outputId & 0x1) == 0); // green (even) coil = output on, red (odd) coil = output off
    // TODO : nog checken of dit zin heeft om 4 pulseDurations te gebruiken voor 8 outputs; of eventueel andere CV's?
    /*
    if (pulseDuration[outputId>>1]) // switch output auto-off after pulseDuration
      timerId = timer.pulseImmediate(decoderOutputs[physicalOutput], pulseDuration[outputId>>1] * 50L, HIGH);
    */
    timer.oscillate(PIN_PROGLED, LED_FAST_FLASH, LOW, 1); // 1 led flashes ter bevestiging van een turnout commando
  }
  else { // 'off' command
    // ignore for this decoder type
  }
} // output_Handler

// in SOFTWAREMODE_OUTPUT_DECODER mode only outputs 0..3 can be manually toggled, because we have only 3 buttons
void output_ManualToggle (uint8_t outputId) {
  uint8_t pin = decoderOutputs[outputId];
  #ifdef DEBUG
    Serial.print ("manual toggle output ");
    Serial.println(outputId);
  #endif

  digitalWrite(pin,!digitalRead(pin));
  timer.oscillate(PIN_PROGLED, LED_FAST_FLASH, LOW, 1); // 1 led flashes ter bevestiging van een turnout commando

} // output_ManualToggle
