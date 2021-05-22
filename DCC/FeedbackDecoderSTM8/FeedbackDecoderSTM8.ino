/*
 * inputs are default active low
 *  -> avr/stm8 local inputs with internal pullup
 *  -> expanded inputs are pulled up internally with 100uA
 *  -> the feedback bit is then '1' if the pin value is LOW
 *  .activeHigh = foresee option to define inputs as active high (not used for now)
 * 
 * 1 common feedback delay for all inputs
 * CV33 : # expanders, default = 1
 * CV34 : feedback delay in ms, default = 50
 * CV35 : xpnet address, default = 5
 * 
 * CV33 : expander 1 must have address 0x20, expander 2 : 0x21, etc, up to 0x27 (8 expanders possible)
 * max inputs = 8 local + 8x8 expanded inputs = 72
 * the feedback decoder occupies multiple dcc addresses : 1 + #expanders (CV33)
 *  -> 1 dcc address for the local inputs (the address in CV1/9)
 *  -> 1 extra dcc address per expander (8 inputs)
 * 
 * TODO:
 * -> for the moment the complete code doesn't fit in 8kB -> temporarily removed DCC part
 * -> check dat DCC address < 256 bij cv programming, want dat is de limiet voor xpnet
 * -> decoderState.xpnetConnectionOk = false tijdens service mode??
 *    zal de CommandStation de feedback verwerken tijdens service mode??
 * -> for STM8 blinking led & i2c are mutually exclusive! (same as signalDecoder todo)
 */

#include "rs485c.h"
#include "xpc.h"
#include "FeedbackDecoderSTM8.h"
#include "Wire.h"

#undef INCLUDE_DCC // temp
#ifdef INCLUDE_DCC
#include "NmraDcc.h"

#define VersionId (0x1)
//factory defaults
static const CVPair FactoryDefaultCVs [] = {
  {CV_ACCESSORY_DECODER_ADDRESS_LSB, 10}, // ipv 1 voor een accessory decoder
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, 0},
  {CV_VERSION_ID, VersionId},
  {CV_MANUFACTURER_ID, MAN_ID_DIY},
  {CV_DECODER_CONFIGURATION,0x80}, // basic accessory decoder (bit5=0)!
  {CV_NUMBER_OF_EXPANDERS, 1},
  {CV_FEEDBACK_DELAY_MS, 50},
  {CV_XPNET_ADDRESS, 5}
};
#endif // INCLUDE_DCC

// feedback decoder
#define MAX_NBR_INPUTS          72
#define NUM_LOCAL_INPUTS        8
#define NUM_INPUTS_PER_EXPANDER 8
#define POLL_DELAY_MILLIS       10

#define LED_BLINK_SLOW_MILLIS 1000
#define LED_BLINK_FAST_MILLIS 200

typedef enum {
  DECODER_FACTORY_RESET,DECODER_INIT,DECODER_RUNNING, DECODER_PROGRAM 
} runState_t;

typedef struct {
  runState_t runState;
  uint8_t nbrExpanders; // hardware configuration from CV
  uint8_t nbrInputs; 
  uint8_t dccAddress;   // the base dcc address, 1 address per 8 inputs
  uint8_t xpnetAddress; // moet dat in de state, enkel nodig bij init?
  struct {
    uint8_t pinValue:1;
    uint8_t feedbackValue:1; // a delayed version of pinValue
    uint8_t activeHigh:1;     // default is active low
  } inputs[MAX_NBR_INPUTS];
  uint8_t feedbackDelayMs; // configurable delay
  uint32_t lastChangeMillis[MAX_NBR_INPUTS];
  bool xpnetConnectionOk; // if no connection we shouldn't send messages, because it will end up blocking the TxBuffer
  bool notifyChange;
  uint8_t curPolledExpander; // reduce i2c reads -> poll 1 expander per 10ms cycle
} decoderState_t;

typedef enum {
  LED_OFF, LED_ON, LED_BLINK_SLOW, LED_BLINK_FAST
} ledState_t;

static decoderState_t decoderState;
static ledState_t ledState;
bool decoderNotifyChange = false;
uint32_t blinkMillis; // for the blinking led
uint32_t pollMillis; // for the i2c i/o expander polling

static const uint8_t localInputs[NUM_LOCAL_INPUTS] = LOCAL_INPUTS;

// keyhandling
#define DEBOUNCE_DELAY 50
#define LONGPRESS_DELAY 3000
typedef enum {UP, DEBOUNCING_DOWN, DOWN, LONG_DOWN, DEBOUNCING_UP} keyState_t;
typedef struct {
  uint8_t Pin;
  keyState_t State;
  uint32_t LastMillis;
} key_t;
key_t keys[NUMBER_OF_KEYS]; // 4 keys, key1 = progkey

/******************************************************************/
/*  EEPROM -> need to save flash for now                          */
/******************************************************************/
#define E2END		(FLASH_DATA_BLOCKS_NUMBER * FLASH_BLOCK_SIZE - 1)
#define EEPROM_read(N)	(*((uint8_t*)((uint16_t)FLASH_DATA_START_PHYSICAL_ADDRESS)+(N)))

static void eeprom_unlock(void)
{
	// it might be easier to just unlock it, independed of the status
	if (!(FLASH->IAPSR & FLASH_FLAG_DUL)) // eeprom is not unlocked
	{
		// EEPROM still locked. Unlock first.
		FLASH->DUKR = FLASH_RASS_KEY2;
		FLASH->DUKR = FLASH_RASS_KEY1;
	}
}

static void eeprom_lock(void)
{
	// re-lock the EEPROM again.
	FLASH->IAPSR &= FLASH_FLAG_DUL;
}

static void EEPROM_update( int idx, uint8_t val )
{
	// make sure not to write data beyond the end of the EEPROM area
	// (this could accidentally hit the option byte area)
	if (idx >= (E2END+1)) return;
	idx += (uint16_t)FLASH_DATA_START_PHYSICAL_ADDRESS;

	if (*((uint8_t*)idx) != val)
	{
		eeprom_unlock();
		if (FLASH->IAPSR & FLASH_FLAG_DUL)
		{
			// write only after a successful unlock.
			*((uint8_t*)idx) = val;
			// re-lock the EEPROM again.
			FLASH->IAPSR &= FLASH_FLAG_DUL;
		}
	}
}
/******************************************************************/
/*  FEEDBACK DECODER                                              */
/******************************************************************/
// read status from i/o expander
static uint8_t ioDigitalRead(uint8_t i2cAddress) {
  uint8_t ioByte = 0xFF;
  Wire_requestFrom(i2cAddress, (uint8_t) 1); // request 1 byte from i/o expander
  while (Wire_available()) { // slave may send less than requested
    ioByte = Wire_read(); // receive a byte as character
  }
  return ioByte;
} // ioDigitalRead

static void decoder_init(void) {
  uint8_t nbrExpanders;
  uint8_t ioByte;
  
  #ifdef INCLUDE_DCC
  nbrExpanders = DCC_getCV(CV_NUMBER_OF_EXPANDERS);
  if ((nbrExpanders > 8) || (nbrExpanders < 1)) { // invalid setting
    nbrExpanders = 1;
    DCC_setCV(CV_NUMBER_OF_EXPANDERS,1);
  }
  #else
  nbrExpanders = 1;
  #endif

  decoderState.nbrExpanders = nbrExpanders;
  decoderState.nbrInputs = NUM_LOCAL_INPUTS + 8*nbrExpanders;

  #ifdef INCLUDE_DCC
  decoderState.feedbackDelayMs = DCC_getCV(CV_FEEDBACK_DELAY_MS); // 50ms default
  #else
  decoderState.feedbackDelayMs = 50; // 50ms default
  #endif

  // local inputs
  for (uint8_t i=0;i<NUM_LOCAL_INPUTS;i++) {
    pinMode(localInputs[i],INPUT_PULLUP);
    decoderState.inputs[i].activeHigh = 0;
    decoderState.inputs[i].pinValue = digitalRead(localInputs[i]);
    decoderState.inputs[i].feedbackValue = decoderState.inputs[i].pinValue;
  }
  // expanded inputs
  for (uint8_t expander=0;expander<decoderState.nbrExpanders;expander++) {
    ioByte = ioDigitalRead(EXPANDER_BASE_I2C_ADDRESS+expander);
    for (uint8_t i=0;i<NUM_INPUTS_PER_EXPANDER;i++) {
      uint8_t inputIdx = NUM_LOCAL_INPUTS+expander*NUM_INPUTS_PER_EXPANDER+i;
      decoderState.inputs[inputIdx].activeHigh = 0;
      decoderState.inputs[inputIdx].pinValue = (ioByte >> i) & 0x1;
      decoderState.inputs[inputIdx].feedbackValue = decoderState.inputs[inputIdx].pinValue;
    }
  }

  #ifdef INCLUDE_DCC
  decoderState.dccAddress = DCC_readDccAddress() & 0xFF; // DCC has 9/11 bit dcc addresses, but xpnet only allows 8 bit!
  decoderState.xpnetAddress = DCC_getCV(CV_XPNET_ADDRESS);
  #else
  decoderState.dccAddress = 10;
  decoderState.xpnetAddress = 5;
  #endif
  decoderState.xpnetConnectionOk = true;
  xpc_Init(decoderState.xpnetAddress);

  decoderState.runState = DECODER_INIT;

  ledState = LED_BLINK_SLOW;
} // decoder_init

// send the proprietary 0x72 message to command station
// we send the local inputs & the last polled io expander
static void decoder_sendNotify(void) {
  uint8_t fbData = 0;
  for (int i=0;i<NUM_LOCAL_INPUTS;i++) {
    uint8_t fbBit = (decoderState.inputs[i].feedbackValue == decoderState.inputs[i].activeHigh);
    fbData |= (fbBit << i);
  }
  xpc_send_AccessoryDecoderInfoNotify(decoderState.dccAddress,fbData);

  fbData = 0;

  for (int i=0;i<NUM_INPUTS_PER_EXPANDER;i++) {
    uint8_t inputIdx = NUM_LOCAL_INPUTS+decoderState.curPolledExpander*NUM_INPUTS_PER_EXPANDER+i;
    uint8_t fbBit = (decoderState.inputs[inputIdx].feedbackValue == decoderState.inputs[inputIdx].activeHigh);
    fbData |= (fbBit << i);
  }
  // io expander X occupies dccAddress (base address) + X + 1!
  xpc_send_AccessoryDecoderInfoNotify(decoderState.dccAddress+decoderState.curPolledExpander+1,fbData);
} // decoder_sendNotify

/******************************************************************/
/*  KEY HANDLING                                                  */
/******************************************************************/
static void keyHandler (uint8_t keyEvent);
static void init_keys (void) {
    keys[0].Pin = PIN_PROGKEY;
    keys[0].State = UP;
} // init_keys

static void detect_keys (void) {
  int i;
  for (i=0;i<NUMBER_OF_KEYS;i++) {
    switch(keys[i].State) {
      case UP : 
        if (digitalRead(keys[i].Pin) == LOW ) {
          keys[i].LastMillis = millis();
          keys[i].State = DEBOUNCING_DOWN;
        }
        break;
      case DEBOUNCING_DOWN :
        if (digitalRead(keys[i].Pin) != LOW )
          keys[i].State = UP;
        else if ( (millis() - keys[i].LastMillis) > DEBOUNCE_DELAY ) {
          keys[i].State = DOWN;
          keyHandler ((i<<2) | KEYEVENT_DOWN); // gebruik index in de keys[] array als keycode
        }
        break;
      case DOWN :
        if (digitalRead(keys[i].Pin) != LOW ) {
          keys[i].State = DEBOUNCING_UP;
          keys[i].LastMillis = millis();
        }
        else if ( (millis() - keys[i].LastMillis) > LONGPRESS_DELAY ) {
          keys[i].State = LONG_DOWN;
          keyHandler ((i<<2) | KEYEVENT_DOWN | KEYEVENT_LONGDOWN );
        }
        break;
      case LONG_DOWN :
        if (digitalRead(keys[i].Pin) != LOW ) {
          keys[i].State = DEBOUNCING_UP;
          keys[i].LastMillis = millis();
        }
        break;
      case DEBOUNCING_UP :
        if (digitalRead(keys[i].Pin) == LOW )
          keys[i].LastMillis = millis();
        else if ( (millis() - keys[i].LastMillis) > DEBOUNCE_DELAY ) {
          keys[i].State = UP;
          keyHandler ((i<<2) | KEYEVENT_UP);
        }
        break;
    }
  }
} // detect_keys

static void keyHandler (uint8_t keyEvent) {
    if (keyEvent & KEYEVENT_LONGDOWN) {
      if (decoderState.runState != DECODER_PROGRAM) {
        decoderState.runState = DECODER_PROGRAM;
        //ledTimer = timer.oscillate(PIN_PROGLED, LED_SLOW_FLASH, HIGH, -1); //forever
      }
    }
    else if (keyEvent & KEYEVENT_DOWN) {
      if (decoderState.runState == DECODER_PROGRAM) {
        decoderState.runState = DECODER_INIT;
      }
    }
} // keyHandler

/******************************************************************/
/*  OTHER LOCAL FUNCTIONS                                         */
/******************************************************************/
// 0 = DONE, 1 = NOT DONE (eeprom not ready)
static uint8_t accessory_FactoryResetCV() {
  #ifdef INCLUDE_DCC
  if (!DCC_isSetCVReady())
    return 1;
  // factory reset de algemene CV's (die niet afhangen van de software mode
  for (int i=0; i < sizeof(FactoryDefaultCVs)/sizeof(CVPair); i++)
    DCC_setCV( FactoryDefaultCVs[i].CV, FactoryDefaultCVs[i].Value);
  #endif

  return 0;
} // accessory_FactoryResetCV

/******************************************************************/
/*  SETUP/LOOP                                                    */
/******************************************************************/

void setup() {

  Wire_begin(); // for the i/o expanders
  init_rs485(RS485_DIRECTION_PIN);

  // Configure the DCC CV Programing ACK pin for an output
  pinMode(PIN_ACKOUT, OUTPUT);
  digitalWrite(PIN_ACKOUT, LOW);

   // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  // + enable the DCC Receiver
  #ifdef INCLUDE_DCC
  DCC_init(FLAGS_DCC_ACCESSORY_DECODER, 0, true);
  #endif

  // progled, progkey
  pinMode (PIN_PROGKEY, INPUT_PULLUP);
  // TODO : merge led & i2c
  // pinMode (PIN_PROGLED, OUTPUT);
  // digitalWrite(PIN_PROGLED, LOW); //LED uit

  #ifdef INCLUDE_DCC
  if (DCC_getCV(CV_MANUFACTURER_ID) != MAN_ID_DIY) {
    decoderState.runState = DECODER_FACTORY_RESET;
  }
  else {
    decoderState.runState = DECODER_INIT;
  }
  #else
  decoderState.runState = DECODER_INIT;
  #endif

  init_keys ();   // init key handling
} // setup

void loop() {
  uint32_t now = millis();

  switch (decoderState.runState) {
    case DECODER_FACTORY_RESET : 
      if (accessory_FactoryResetCV () == 0){ // factory reset gelukt --> DECODER_INIT
        decoderState.runState = DECODER_INIT;
      }
      break;
    case DECODER_INIT :
      //timer.stop (ledTimer); //stop de slow flashing led
      decoder_init();
      decoderState.runState = DECODER_RUNNING;
      break;
    case DECODER_RUNNING :
    case DECODER_PROGRAM : 
      // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function 
      // for correct library operation
      #ifdef INCLUDE_DCC
      DCC_process();
      #endif
      break;
  }

  // update notifies
  if ((now - pollMillis) > POLL_DELAY_MILLIS) {
    uint8_t pinValue;
    uint8_t i;
    // local inputs
    for (i=0;i<NUM_LOCAL_INPUTS;i++) {
      pinValue = digitalRead(localInputs[i]);
      if (pinValue != decoderState.inputs[i].pinValue) {
        decoderState.lastChangeMillis[i] = now;
        decoderState.inputs[i].pinValue = pinValue;
      }
      if ((decoderState.inputs[i].feedbackValue != decoderState.inputs[i].pinValue) &&
          ((now - decoderState.lastChangeMillis[i]) > decoderState.feedbackDelayMs)) {
        decoderState.inputs[i].feedbackValue = pinValue;
        decoderState.notifyChange = true;
      }
    } // local inputs
    // expanded inputs -> poll only 1 expander per iteration
    decoderState.curPolledExpander++;
    if (decoderState.curPolledExpander >= decoderState.nbrExpanders) 
      decoderState.curPolledExpander = 0;
    uint8_t ioByte = ioDigitalRead(EXPANDER_BASE_I2C_ADDRESS+decoderState.curPolledExpander);
    for (i=0;i<NUM_INPUTS_PER_EXPANDER;i++) {
      pinValue = (ioByte >> i) & 0x1;
      uint8_t inputIdx = NUM_LOCAL_INPUTS+decoderState.curPolledExpander*NUM_INPUTS_PER_EXPANDER+i;
      if (pinValue != decoderState.inputs[inputIdx].pinValue) {
        decoderState.lastChangeMillis[inputIdx] = now;
        decoderState.inputs[inputIdx].pinValue = pinValue;
      }
      if ((decoderState.inputs[inputIdx].feedbackValue != decoderState.inputs[inputIdx].pinValue) &&
          ((now - decoderState.lastChangeMillis[inputIdx]) > decoderState.feedbackDelayMs)) {
        decoderState.inputs[inputIdx].feedbackValue = pinValue;
        decoderState.notifyChange = true;
      }
    }

    if (decoderState.xpnetConnectionOk && decoderState.notifyChange) { // send xpnet message
      decoder_sendNotify();
      decoderState.notifyChange = false;
    }
  }
  // run the expressnet
  xpc_Run();

  // update status led
  // TODO : merge led & i2c
  /*
  switch (ledState) {
    case LED_OFF :
      digitalWrite(PIN_PROGLED,LOW);
      break;
    case LED_ON :
      digitalWrite(PIN_PROGLED,HIGH);
      break;
    case LED_BLINK_SLOW :
      if ((now - blinkMillis) > LED_BLINK_SLOW_MILLIS) {
        digitalWrite(PIN_PROGLED,!digitalRead(PIN_PROGLED));
        blinkMillis = now;
      }
      break;
    case LED_BLINK_FAST :
      if ((now - blinkMillis) > LED_BLINK_FAST_MILLIS) {
        digitalWrite(PIN_PROGLED,!digitalRead(PIN_PROGLED));
        blinkMillis = now;
      }
      break;
  }
  */

  detect_keys();  // check key input
  //timer.update();  // run timers

} // loop

/*****************************************************************************/
/*  XPNET NOTIFY FUNCTIONS                                                   */
/*****************************************************************************/
void xpc_FastClockNotify (xpcFastClock_t *newFastClock) {(void)newFastClock;}

void xpc_MessageNotify (uint8_t *msg) {
  (void) msg;
  // any received message reset the connectionOK flag
  decoderState.xpnetConnectionOk = true;
} // xpc_MessageNotify

void xpc_EventNotify (xpcEvent_t xpcEvent) {

  switch (xpcEvent) {
    case XPEVENT_POWER_OFF : // main track disabled
    case XPEVENT_MAIN_SHORT : // main track short circuit
    case XPEVENT_PROG_SHORT : // prog track short circuit
    case XPEVENT_SERVICE_MODE_ON : // service mode entry
      // TODO : decoderState.xpnetConnectionOk = false tijdens service mode??
      // zal de CommandStation de feedback verwerken tijdens service mode??
      ledState = LED_BLINK_FAST;
      break;
    case XPEVENT_CONNECTION_OK :
    case XPEVENT_POWER_ON : // normal operation resumed
      decoderState.xpnetConnectionOk = true; // or we wait until we receive a msg from xpnet?
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


#ifdef INCLUDE_DCC
/******************************************************************************/
/* notify functies uit Nmra layer                                             */
/******************************************************************************/
// SDCC doesn't know weak functions
void notifyDccIdle(void) {}
void notifyDccMsg( DCC_MSG * Msg ){(void)Msg;}

void notifyDccReset(uint8_t hardReset) {
  (void) hardReset;
  // TODO : check, want er worden dcc resets gestuurd bij start programming mode, en dan is DECODER_INIT niet ok!!
  /*
  decoderState = DECODER_INIT; // the loop() function will get the decoder to a reinitialize
  // not doing a real software reset for now
  */
} // notifyDccReset

// This function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read 
void notifyCVAck(void) {
  digitalWrite(PIN_ACKOUT, HIGH);
  delay(6);  
  digitalWrite(PIN_ACKOUT, LOW);
} // notifyCVAck

// This function is called whenever a normal DCC Turnout Packet is received
void notifyDccAccState(uint16_t dccAddress, uint8_t outputId, bool activate) {
  (void) outputId;
  (void) activate;
  if (decoderState.runState == DECODER_PROGRAM) {
    // take the decoderAddress & program it as our own
    DCC_writeDccAddress(dccAddress);
    decoderState.dccAddress = dccAddress; // keep copy in RAM
    //timer.oscillate(PIN_PROGLED, LED_FAST_FLASH, LOW, 3); // 3 led flashes ter bevestiging van een CV write
    return;
  }
  // we don't handle basic turnout packets otherwise
} // notifyDccAccState

// This function is called whenever a DCC Signal Aspect Packet is received
void notifyDccSigState(uint16_t dccAddress, uint8_t signalId, uint8_t signalAspect)
{
  (void) dccAddress;
  (void) signalId;
  (void) signalAspect;
} // notifyDccSigState

uint8_t notifyCVRead(uint16_t cv) {
  return (EEPROM_read((int) cv));
} // readCV


// enkel CV's schrijven als decoder niet 'live' is (INIT, FACTORY_RESET, PROGRAM)
uint8_t notifyCVWrite(uint16_t cv, uint8_t cvValue) {
  if (decoderState.runState == DECODER_RUNNING) {
    return EEPROM_read((int) cv);
  }
  if (cv == CV_MANUFACTURER_ID) { // writing the CV_MANUFACTURER_ID triggers a factory reset
    decoderState.runState = DECODER_FACTORY_RESET;
    EEPROM_update((int) cv, MAN_ID_DIY);
    //Timer_oscillate(PIN_PROGLED, LED_FAST_FLASH, HIGH, 3); // 3 led flashes ter bevestiging van een CV write
    return cvValue; // we pretend to write the value, but only to trigger an ackCV
  }
  else {
    EEPROM_update((int) cv, cvValue);
    //Timer_oscillate(PIN_PROGLED, LED_FAST_FLASH, HIGH, 3); // 3 led flashes ter bevestiging van een CV write
    return EEPROM_read((int) cv);
  }
} // notifyCVWrite

// 0 = ongeldige schrijfactie gevraagd naar CV
// 0 = lezen naar niet-geïmplementeerde CV's : todo SDS!
// 1 = geldige actie
// DECODER_PROGRAM : for now we only allow CV writes while decoder is in DECODER_PROGRAM
bool notifyCVValid(uint16_t cv, uint8_t writable) {
  bool isValid = true;

  // CV read/write only in programming mode
  if( (cv > MAXCV) || (decoderState.runState != DECODER_PROGRAM))
    isValid = false;

  // different from default, because we do allow writing CV_MANUFACTURER_ID to trigger a factory reset
  if (writable && ((cv==CV_VERSION_ID)||(cv==CV_DECODER_CONFIGURATION)||(cv==CV_RAILCOM_CONFIGURATION)))
    isValid = false;

  // TODO : uitbreiden!! (ook afhankelijk van swMode CV33)
  return isValid;
} // notifyCVValid

#endif // INCLUDE_DCC
