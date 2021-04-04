/*
 * we beginnen met een simpele feedback decoder met 8 inputs D2->D9
 * met CV delay zoals LR101
 * 
 */

#include "rs485c.h"
#include "xpc.h"

#define DEBUG
#ifdef DEBUG
// for debug prints or command line input
#include <AltSoftSerial.h>
AltSoftSerial mySerial;
//#define SSerialRX 8   //Serial Receive pin --> is fixed in AltSoftSerial
//#define SSerialTX 9   //Serial Transmit pin --> is fixed in AltSoftSerial
#endif

#define RS485_DIRECTION_PIN   4     // hw specific!
#define XPNET_MY_ADDRESS (5)
#define MAX_NBR_INPUTS  16

#define LED_BLINK_SLOW_MILLIS 1000
#define LED_BLINK_FAST_MILLIS 200

typedef struct {
  uint8_t nbrInputs; // hardware configuration
  uint8_t myAddress; // 1 feedback decoder address per 8 inputs, 16 inputs has 2 successive addresses
  struct {
    uint8_t pinValue:1;
    uint8_t feedbackValue:1; // a delayed version of pinValue
    uint8_t activeLow:1;     // default is active high
    uint8_t pin:5;           // arduino pin nr
  } inputs[MAX_NBR_INPUTS];
  uint8_t feedbackDelayMs[MAX_NBR_INPUTS]; // configurable delay
  uint32_t lastChangeMillis[MAX_NBR_INPUTS];
  bool xpnetConnectionOk; // if no connection we shouldn't send messages, because it will end up blocking the TxBuffer

} decoderState_t;

typedef enum {
  LED_OFF, LED_ON, LED_BLINK_SLOW, LED_BLINK_FAST
} ledState_t;

static decoderState_t decoderState;
static ledState_t ledState;
bool decoderNotifyChange = false;
uint32_t blinkMillis;

// a static config for now not using CV's/eeprom
static void decoder_init(void) {
  uint8_t pins[] = {2,3,5,6,7,8,9,10}; // careful : D4 used for RS485

  decoderState.nbrInputs = 8; // 8 or 16 supported
  decoderState.myAddress = 65;
  decoderState.xpnetConnectionOk = true;

  for (int i=0;i<8;i++) {
    pinMode(pins[i],INPUT_PULLUP);
    decoderState.inputs[i].activeLow = 1;
    decoderState.inputs[i].pin = pins[i];
    decoderState.inputs[i].pinValue = digitalRead(pins[i]);
    decoderState.inputs[i].feedbackValue = decoderState.inputs[i].pinValue;
  }
  for (int i=0;i<8;i++) {
    decoderState.feedbackDelayMs[i] = 50; // 50ms
  }
  ledState = LED_BLINK_SLOW;
} // decoder_init

// send the proprietary 0x72 message to command station
static void decoder_sendNotify(void) {
  uint8_t fbData = 0;
  for (int i=0;i<8;i++) {
    uint8_t fbBit = (decoderState.inputs[i].feedbackValue != decoderState.inputs[i].activeLow);
    fbData |= (fbBit << i);
  }
  xpc_send_AccessoryDecoderInfoNotify(decoderState.myAddress,fbData);
  if (decoderState.nbrInputs == 8) return;

  // if (decoderState.nbrInputs == 16) ->  send the 2nd decoder address data
  fbData = 0;
  for (int i=0;i<8;i++) {
    uint8_t fbBit = (decoderState.inputs[i+8].feedbackValue != decoderState.inputs[i+8].activeLow);
    fbData |= (fbBit << i);
  }
  xpc_send_AccessoryDecoderInfoNotify(decoderState.myAddress+1,fbData);
} // decoder_sendNotify

void setup() 
{
#ifdef DEBUG
  // set the data rate for the SoftwareSerial port
  mySerial.begin(19200);
  mySerial.println("Feedback decoder!");
#endif 

  decoder_init();
  init_rs485(RS485_DIRECTION_PIN);
  xpc_Init(XPNET_MY_ADDRESS);

  // led state indication
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);


} // setup

void loop() 
{
  uint32_t now = millis();
  for (int i=0;i<decoderState.nbrInputs;i++) {
    uint8_t pinValue = digitalRead(decoderState.inputs[i].pin);
    if (pinValue != decoderState.inputs[i].pinValue) {
      decoderState.lastChangeMillis[i] = now;
      decoderState.inputs[i].pinValue = pinValue;
    }
    if ((decoderState.inputs[i].feedbackValue != decoderState.inputs[i].pinValue) &&
        ((now - decoderState.lastChangeMillis[i]) > decoderState.feedbackDelayMs[i])) {
      decoderState.inputs[i].feedbackValue = pinValue;
      decoderNotifyChange = true;
    }
  } // for
  if (decoderState.xpnetConnectionOk && decoderNotifyChange) {
    // send xpnet message
    
    decoder_sendNotify();
    decoderNotifyChange = false;
  #ifdef DEBUG
    mySerial.println("ntf!");
  #endif 
    
  }

  xpc_Run();
  /*
  switch (ledState) {
    case LED_OFF :
      digitalWrite(LED_BUILTIN,LOW);
      break;
    case LED_ON :
      digitalWrite(LED_BUILTIN,HIGH);
      break;
    case LED_BLINK_SLOW :
      if ((now - blinkMillis) > LED_BLINK_SLOW_MILLIS) {
        digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
        blinkMillis = now;
      }
      break;
    case LED_BLINK_FAST :
      if ((now - blinkMillis) > LED_BLINK_FAST_MILLIS) {
        digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
        blinkMillis = now;
      }
      break;
  }
  */
} // loop

void xpc_MessageNotify (uint8_t *msg) {
  (void) msg;
  // any received message reset the connectionOK flag
  decoderState.xpnetConnectionOk = true;
} // xpc_MessageNotify

void xpc_EventNotify( xpcEvent_t xpcEvent )
{
  #ifdef DEBUG
    mySerial.print("xpcEvent : ");
    mySerial.println(xpcEvent);
  #endif 

  switch (xpcEvent) {
    case XPEVENT_POWER_OFF : // main track disabled
    case XPEVENT_MAIN_SHORT : // main track short circuit
    case XPEVENT_PROG_SHORT : // prog track short circuit
    case XPEVENT_SERVICE_MODE_ON : // service mode entry
      ledState = LED_BLINK_FAST;
      break;
    case XPEVENT_POWER_ON : // normal operation resumed
      ledState = LED_BLINK_SLOW;
      // slow blink
      break;
    case XPEVENT_CONNECTION_ERROR : // haven't received a call byte during CONNECTION_TIMEOUT; this could be due to programming mode?
      decoderState.xpnetConnectionOk = false;
      ledState = LED_OFF;
      break;
    case XPEVENT_TX_ERROR : // xp-master signalled a data error
    case XPEVENT_BUSY : // xp-master signalled "command station busy"
    case XPEVENT_UNKNOWN_COMMAND : // xp-master signalled an unknown command -> if this happens, there is a coding bug
      decoderNotifyChange = true; // force a retransmit
      break;
    case XPEVENT_MSG_ERROR : // rxBuffer overrun
    case XPEVENT_RX_ERROR : // xor error in received message, msg discarded. application needs to resend the last request
    case XPEVENT_RX_TIMEOUT : // not all msg bytes received within RX_TIMEOUT
    case XPEVENT_LOC_STOLEN :
      // don't cares
      break;
  }
} // xpc_EventNotify
