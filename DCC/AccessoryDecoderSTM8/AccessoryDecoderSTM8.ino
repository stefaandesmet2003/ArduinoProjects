#include "NmraDcc.h"
#include "AccessoryDecoderSTM8.h"
#include "TurnoutDecoder.h"
#include "Timer.h"
#include <EEPROM.h>


/*
 * dcc.process() verwerkt exact 1 DCC packet. Vanuit dcc.process wordt de notify functie aangeroepen, die het packet verwerkt
 * als de processing langer dan 1 DCC packet duurt, dan gaan er packetten verloren. NmraDcc heeft een buffer van 1 pakket!! (dccRx.PacketCopy)
 * De Rx ISR gaat gewoon door, en gaat op het einde van het pakket de bitbuffer in PacketCopy copiëren, ongeacht dataReady al op 0 stond
 */
// DCC signaal ontvangen op pin D2 (AVR) of PB5 (STM8)
// als je FLAGS_MY_ADDRESS_ONLY gebruikt worden enkel voor dit adres notifyDccAccState callbacks gedaan
// de DCC isr ontvangt nog steeds alle pakketten, en notifyDccMsg dus ook (indien niet NULL)
// zonder FLAGS_MY_ADDRESS_ONLY kan je voor alle accessory decoders  notifyDccAccState ontvangen
// ihb accessory decoder pakketten voor 2 verschillende adressen, om bv een decoder voor 8 wissels te maken
// om een factory reset te doen moet je een CV write naar CV8 doen
// om het adres te wijzigen doe je CV write (direct byte) naar CV1 en/of CV9

#define VersionId 0x1 // versie van deze software

// decoder global
enum {DECODER_FACTORY_RESET,DECODER_INIT,DECODER_RUNNING, DECODER_PROGRAM } decoderState;
uint8_t decoderSoftwareMode;

static uint16_t decoderAddress; // keep this local, other modules will use getter getDecoderAddress;
//factory defaults
static CVPair FactoryDefaultCVs [] = {
  {CV_ACCESSORY_DECODER_ADDRESS_LSB, 1},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, 0},
  {CV_VERSION_ID, VersionId},
  {CV_MANUFACTURER_ID, MAN_ID_DIY},
  {CV_DECODER_CONFIGURATION,0x80},
  {CV_SoftwareMode, 0} 
};

// keyhandling
#define DEBOUNCE_DELAY 50
#define LONGPRESS_DELAY 3000
#define NUMBER_OF_KEYS 4
typedef enum {UP, DEBOUNCING_DOWN, DOWN, LONG_DOWN, DEBOUNCING_UP} keyState_t;
typedef struct {
  uint8_t Pin;
  keyState_t State;
  uint32_t LastMillis;
} key_t;
key_t keys[NUMBER_OF_KEYS]; // 4 keys, key1 = progkey

// timers
int8_t ledTimer; 

/******************************************************************/
/*  KEY HANDLING                                                  */
/******************************************************************/
static void keyHandler (uint8_t keyEvent);

static void init_keys (void) {
    keys[0].Pin = PIN_PROGKEY;
    keys[0].State = UP;
    keys[1].Pin = PIN_KEY2;
    keys[1].State = UP;
    keys[2].Pin = PIN_KEY3;
    keys[2].State = UP;
    keys[3].Pin = PIN_KEY4;
    keys[3].State = UP;
    // keys[0].Lastmillis = 0; //init overbodig
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
    // debug info
    #ifdef DEBUG
      switch (keyEvent & 0x3) {
        case KEYEVENT_UP :
          Serial_print_s("key up");
          break;
        case KEYEVENT_DOWN :
          Serial_print_s("key down");
          break;
        case KEYEVENT_LONGDOWN :
          Serial_print_s("long key down");
          break;
      }
      Serial_print_s (" on key : ");
      Serial_println_u(keyEvent>>2);
    #endif // DEBUG
    
    if (keyEvent & KEYEVENT_LONGDOWN) {
      if (decoderState != DECODER_PROGRAM) {
        decoderState = DECODER_PROGRAM;
        ledTimer = Timer_oscillate(PIN_PROGLED, LED_SLOW_FLASH, HIGH, -1); //forever
      }
    }
    else if (keyEvent & KEYEVENT_DOWN) {
      if (decoderState == DECODER_PROGRAM) {
        decoderState = DECODER_INIT;
      }
      else 
        turnout_ManualToggle (keyEvent >> 2,true); // key down -> send the 'on' command to the turnout/output
    }
    else
      turnout_ManualToggle (keyEvent >> 2,false); // key up -> send the 'off' command
    #ifdef DEBUG
      Serial_print_s ("decoderState = ");
      if (decoderState == DECODER_INIT) Serial_println_s("DECODER_INIT");
      else if (decoderState == DECODER_RUNNING) Serial_println_s("DECODER_RUNNING");
      else if (decoderState == DECODER_PROGRAM) Serial_println_s("DECODER_PROGRAM");
      else Serial_println_s("factory reset");
    #endif
} // keyHandler

/******************************************************************/
/*  OTHER LOCAL FUNCTIONS                                         */
/******************************************************************/

// 0 = DONE, 1 = NOT DONE (eeprom not ready)
static uint8_t accessory_FactoryResetCV() {
  int i;
  uint8_t swMode; // we gebruiken hier een read uit eeprom, niet de global var
  // factory reset de algemene CV's (die niet afhangen van de software mode
  for (i=0; i < sizeof(FactoryDefaultCVs)/sizeof(CVPair); i++)
    DCC_setCV( FactoryDefaultCVs[i].CV, FactoryDefaultCVs[i].Value);
  swMode = DCC_getCV(CV_SoftwareMode);
  switch(swMode) {
    case SOFTWAREMODE_TURNOUT_DECODER :
    case SOFTWAREMODE_OUTPUT_DECODER :
      turnout_FactoryResetCV();
      break;
    default :
      // er staat een unsupported software mode in CV33 --> we gaan die terugzetten naar TURNOUT_DECODER en de TURNOUT_DECODER factory defaults zetten
      DCC_setCV(CV_SoftwareMode,SOFTWAREMODE_TURNOUT_DECODER);
      turnout_FactoryResetCV();
      break;
  }
  return 0;
} // accessory_FactoryResetCV

void setup() {
  decoderState = DECODER_INIT;
  #ifdef DEBUG
    Serial_begin(115200);
    Serial_println_s("AccessoryDecoder STM8!");
  #endif
  
  // Configure the DCC CV Programing ACK pin for an output
  pinMode(PIN_ACKOUT, OUTPUT);
  digitalWrite(PIN_ACKOUT, LOW);

  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  // + enable the DCC Receiver
  DCC_init(FLAGS_DCC_ACCESSORY_DECODER, 0, true);

  // progled, progkey
  pinMode(PIN_PROGKEY, INPUT_PULLUP);
  // key2-4
  pinMode(PIN_KEY2, INPUT_PULLUP);
  pinMode(PIN_KEY3, INPUT_PULLUP);
  pinMode(PIN_KEY4, INPUT_PULLUP);
  // led
  pinMode(PIN_PROGLED, OUTPUT);
  digitalWrite(PIN_PROGLED, HIGH); //LED uit

  // check if factory defaults are written in eeprom
  if (DCC_getCV(CV_MANUFACTURER_ID) != MAN_ID_DIY)
    decoderState = DECODER_FACTORY_RESET;
  else
    decoderState = DECODER_INIT;
  
  init_keys (); // init key handling
  Timer_init(); // init timer lib
} // setup

void loop() {
  switch (decoderState) {
    case DECODER_FACTORY_RESET : 
      if (accessory_FactoryResetCV () == 0) { // factory reset gelukt --> DECODER_INIT
        decoderState = DECODER_INIT;
      }
      break;
    case DECODER_INIT :
      Timer_stop (ledTimer); //stop de slow flashing led
      decoderAddress =  DCC_readDccAddress(); // reread from eeprom
      decoderSoftwareMode = DCC_getCV(CV_SoftwareMode);
      switch (decoderSoftwareMode) {
        case SOFTWAREMODE_TURNOUT_DECODER :
        case SOFTWAREMODE_OUTPUT_DECODER :
          turnout_Init ();
          break;
      }
      decoderState = DECODER_RUNNING;
      #ifdef DEBUG
        Serial_print_s("my address is : ");
        Serial_println_u(getDecoderAddress());
      #endif
      break;
    case DECODER_RUNNING :
    case DECODER_PROGRAM : 
      // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function 
      // for correct library operation
      DCC_process();
      break;
  }
  
  detect_keys(); // check key input
  Timer_update(); // run timers
} // loop

/**********************************************************************************************/
/* external interface                                                                         */
/**********************************************************************************************/
uint16_t getDecoderAddress(void) {
  return decoderAddress;
} // getDecoderAddress

/**********************************************************************************************/
/* notify functies uit Nmra layer                                                             */
/**********************************************************************************************/
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


// for test
/*
uint8_t cntMsg = 0;
void notifyDccMsg( DCC_MSG * Msg ){
  cntMsg++;
  if (cntMsg == 5) {
    Serial_print_s("msg! size = ");
    Serial_println_u(Msg->Size);
    for (int i=0;i<Msg->Size;i++) {
      Serial_print_ub(Msg->Data[i],HEX);
      Serial_print_s(" - ");
    }
    Serial_println_s("");
  }
}
*/

// This function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read 
void notifyCVAck(void) {
  #ifdef DEBUG
    Serial_println_s("ack");
  #endif
  digitalWrite(PIN_ACKOUT, HIGH);
  delay(6);  
  digitalWrite(PIN_ACKOUT, LOW);
} // notifyCVAck

// This function is called whenever a normal DCC Turnout Packet is received
void notifyDccAccState(uint16_t dccAddress, uint8_t outputId, bool activate) {
  if (decoderState == DECODER_PROGRAM) {
    // take the decoderAddress & program it as our own
    DCC_writeDccAddress(dccAddress);
    decoderAddress = dccAddress; // keep copy in RAM
    Timer_oscillate(PIN_PROGLED, LED_FAST_FLASH, HIGH, 3); // 3 led flashes ter bevestiging van een CV write
    #ifdef DEBUG
      Serial_print_s("CV1/CV9 rewritten, my address is now :");
      Serial_println_u(DCC_readDccAddress());
    #endif
    return;
  }

  if (decoderSoftwareMode == SOFTWAREMODE_TURNOUT_DECODER)
    turnout_Handler (dccAddress, outputId, activate);
  else if (decoderSoftwareMode == SOFTWAREMODE_OUTPUT_DECODER) 
    output_Handler (dccAddress, outputId, activate);

  else {
    #ifdef DEBUG
      Serial_println_s("unsupported software mode!");
    #endif
    //goto safe mode??
  }
} // notifyDccAccState

// This function is called whenever a DCC Signal Aspect Packet is received
void notifyDccSigState(uint16_t dccAddress, uint8_t signalId, uint8_t signalAspect) {
  #ifdef DEBUG
    Serial_print_s("notifyDccSigState: unsupported for now :");
    Serial_print_u(dccAddress);
    Serial_print_s(",");
    Serial_print_u(signalId);
    Serial_print_s(",");
    Serial_println_u(signalAspect);
  #endif
  (void) dccAddress;
  (void) signalId;
  (void) signalAspect;
  
} // notifyDccSigState

// enkel CV's schrijven als decoder niet 'live' is (INIT, FACTORY_RESET, PROGRAM)
uint8_t notifyCVWrite(uint16_t cv, uint8_t cvValue) {
  #ifdef DEBUG
    Serial_print_s ("CVWrite ");
    Serial_print_u(cv);
    Serial_print_s(" = ");
    Serial_println_u(cvValue);
    if (decoderState == DECODER_RUNNING)
      Serial_println_s("no CVs written while decoder RUNNING");
  #endif
  if (decoderState == DECODER_RUNNING) {
    return EEPROM_read((int) cv);
  }
  if (cv == CV_MANUFACTURER_ID) { // writing the CV_MANUFACTURER_ID triggers a factory reset
    decoderState = DECODER_FACTORY_RESET;
    EEPROM_update((int) cv, MAN_ID_DIY); // in case of empty eeprom
    Timer_oscillate(PIN_PROGLED, LED_FAST_FLASH, HIGH, 3); // 3 led flashes ter bevestiging van een CV write
    return cvValue; // we pretend to write the value, but only to trigger an ackCV
  }
  else {
    EEPROM_update((int) cv, cvValue);
    Timer_oscillate(PIN_PROGLED, LED_FAST_FLASH, HIGH, 3); // 3 led flashes ter bevestiging van een CV write
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
  if( (cv > MAXCV) || (decoderState != DECODER_PROGRAM))
    isValid = false;

  // different from default, because we do allow writing CV_MANUFACTURER_ID to trigger a factory reset
  if (writable && ((cv==CV_VERSION_ID)||(cv==CV_DECODER_CONFIGURATION)||(cv==CV_RAILCOM_CONFIGURATION)))
    isValid = false;

  // TODO : uitbreiden!! (ook afhankelijk van swMode CV33)
  return isValid;
} // notifyCVValid
