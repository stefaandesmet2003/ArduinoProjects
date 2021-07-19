#include "NmraDcc.h"
#include "AccessoryDecoder.h"
#include "TurnoutDecoder.h"
#include "Timer.h"

/*
 * dcc.process() verwerkt exact 1 DCC packet. Vanuit dcc.process wordt de notify functie aangeroepen, die het packet verwerkt
 * als de processing langer dan 1 DCC packet duurt, dan gaan er packetten verloren. NmraDcc heeft een buffer van 1 pakket!! (dccRx.PacketCopy)
 * De Rx ISR gaat gewoon door, en gaat op het einde van het pakket de bitbuffer in PacketCopy copiëren, ongeacht dataReady al op 0 stond
 * DCC signaal ontvangen op pin D2
 * setFilterAddress : let nmra lib filter on this address
 * no filtering is useful for eg. a accessory decoder spanning 2 addresses, then the filtering has to be done outside nmra lib
 * can't use filtering here, otherwise learning mode won't work
 * broadcast addresses are always callbacked (not filtered)
 * om een factory reset te doen moet je een CV write naar CV8 doen
 * writeDccAddress : use this change decoder address, lib handles writing the correct CVs
 */

#define VersionId 0x1 // versie van deze software

typedef enum {
  DECODER_FACTORY_RESET,DECODER_INIT,DECODER_RUNNING, DECODER_PROGRAM 
} decoderState_t;

typedef struct { // for the decoder state
  decoderState_t state;
  uint16_t address;
  uint8_t softwareMode;
} decoder_t;

static decoder_t decoder;

// dcc receiver
NmraDcc  Dcc;

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
Timer timer;
int8_t ledTimer;

/******************************************************************/
/*  KEY HANDLING                                                  */
/******************************************************************/
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
          Serial.print("key up");
          break;
        case KEYEVENT_DOWN :
          Serial.print("key down");
          break;
        case KEYEVENT_LONGDOWN :
          Serial.print("long key down");
          break;
      }
      Serial.print (" on key : ");
      Serial.println(keyEvent>>2);
    #endif
    
    if (keyEvent & KEYEVENT_LONGDOWN) {
      if (decoder.state != DECODER_PROGRAM) {
        decoder.state = DECODER_PROGRAM;
        ledTimer = timer.oscillate(PIN_PROGLED, LED_SLOW_FLASH, HIGH, -1); //forever
      }
    }
    else if (keyEvent & KEYEVENT_DOWN) {
      if (decoder.state == DECODER_PROGRAM) {
        decoder.state = DECODER_INIT;
      }
      else
        turnout_ManualToggle (keyEvent >> 2,true); // key down -> send the 'on' command to the turnout/output
    }
    else
      turnout_ManualToggle (keyEvent >> 2,false); // key up -> send the 'off' command

    #ifdef DEBUG
      Serial.print ("decoder.state = ");
      if (decoder.state == DECODER_INIT) Serial.println("DECODER_INIT");
      else if (decoder.state == DECODER_RUNNING) Serial.println("DECODER_RUNNING");
      else if (decoder.state == DECODER_PROGRAM) Serial.println("DECODER_PROGRAM");
      else Serial.println("factory reset");
    #endif
} // keyHandler

/******************************************************************/
/*  OTHER LOCAL FUNCTIONS                                         */
/******************************************************************/

// 0 = DONE, 1 = NOT DONE (eeprom not ready)
static uint8_t accessory_FactoryResetCV() {
  int i;
  uint8_t swMode; // we gebruiken hier een read uit eeprom, niet de global var
  
  if (!Dcc.isSetCVReady())
    return 1;
  // factory reset de algemene CV's (die niet afhangen van de software mode
  for (i=0; i < sizeof(FactoryDefaultCVs)/sizeof(CVPair); i++)
    Dcc.setCV( FactoryDefaultCVs[i].CV, FactoryDefaultCVs[i].Value);
  swMode = Dcc.getCV(CV_SoftwareMode);
  switch(swMode) {
    case SOFTWAREMODE_TURNOUT_DECODER :
    case SOFTWAREMODE_OUTPUT_DECODER :
      turnout_FactoryResetCV();
      break;
    default :
      // er staat een unsupported software mode in CV33 --> we gaan die terugzetten naar TURNOUT_DECODER en de TURNOUT_DECODER factory defaults zetten
      Dcc.setCV(CV_SoftwareMode,SOFTWAREMODE_TURNOUT_DECODER);
      turnout_FactoryResetCV();
      break;
  }
  return 0;
} // accessory_FactoryResetCV

void setup() {
  decoder.state = DECODER_INIT;
  #ifdef DEBUG
    Serial.begin(115200);
    Serial.println("AccessoryDecoder AVR!");
  #endif
  
  // Configure the DCC CV Programing ACK pin for an output
  pinMode(PIN_ACKOUT, OUTPUT);
  digitalWrite(PIN_ACKOUT, LOW);
  
  // Call the main DCC Init function to enable the DCC Receiver
  // use default ext int 0 on pin 2
  Dcc.init();

  // progled, progkey
  pinMode (PIN_PROGKEY, INPUT_PULLUP);
  // key2-4
  pinMode (PIN_KEY2, INPUT_PULLUP);
  pinMode (PIN_KEY3, INPUT_PULLUP);
  pinMode (PIN_KEY4, INPUT_PULLUP);
  // led
  pinMode (PIN_PROGLED, OUTPUT);
  digitalWrite(PIN_PROGLED, LOW); //LED uit

  // check if factory defaults are written in eeprom
  if (!Dcc.isSetCVReady()) {
    // we kunnen geen CV's inlezen, dus weten we niet wat te doen!!
    // goto safe mode --> alles outputs uit en wachten op progkey
    return;
  }
  // can't use setFilterAddress, because that breaks the learning mode!
  if (Dcc.getCV(CV_MANUFACTURER_ID) != MAN_ID_DIY)
    decoder.state = DECODER_FACTORY_RESET;
  else
    decoder.state = DECODER_INIT;

  // init key handling
  init_keys ();
} // setup

void loop() {
  switch (decoder.state) {
    case DECODER_FACTORY_RESET : 
      if (accessory_FactoryResetCV () == 0){ // factory reset gelukt --> DECODER_INIT
        decoder.state = DECODER_INIT;
      }
      break;
    case DECODER_INIT :
      timer.stop (ledTimer); //stop de slow flashing led
      decoder.address =  Dcc.readDccAddress(); // reread from eeprom
      decoder.softwareMode = Dcc.getCV(CV_SoftwareMode);
      switch (decoder.softwareMode) {
        case SOFTWAREMODE_TURNOUT_DECODER :
         case SOFTWAREMODE_OUTPUT_DECODER :
          turnout_Init ();
          break;
      }
      decoder.state = DECODER_RUNNING;
      #ifdef DEBUG
        Serial.print("my address is : "); Serial.println(getDecoderAddress());
      #endif
      break;
    case DECODER_RUNNING :
    case DECODER_PROGRAM : 
      Dcc.process();
      break;
  }

  detect_keys();  // check key input
  timer.update();  // run timers
} // loop

/**********************************************************************************************/
/* external interface                                                                         */
/**********************************************************************************************/
uint16_t getDecoderAddress(void) {
  return decoder.address;
} // getDecoderAddress

/**********************************************************************************************/
/* notify functies uit Nmra layer                                                             */
/**********************************************************************************************/
void notifyDccReset(uint8_t hardReset) {
  (void) hardReset;
  // TODO : check, want er worden dcc resets gestuurd bij start programming mode, en dan is DECODER_INIT niet ok!!
  /*
  decoder.state = DECODER_INIT; // the loop() function will get the decoder to a reinitialize
  // avr standard bootloader doesn't like a watchdog reset, so can't do a real reboot
  */
} // notifyDccReset

#ifdef NOTIFY_DCC_MSG
// a debug function to print all received dcc packets
void notifyDccMsg( DCC_MSG * Msg) {
  #ifdef DEBUG
    if ((Msg->Data[0] == 0xFF) || (Msg->Data[0]<128)) return; // don't print idle or loc packets
    Serial.print("notifyDccMsg[ ");
    Serial.print(millis());
    Serial.print("] : ");
    for(uint8_t i = 0; i < Msg->Size; i++) {
      Serial.print(Msg->Data[i], HEX);
      Serial.write(' ');
    }
    Serial.println();
  #endif // DEBUG
} // notifyDccMsg
#endif // NOTIFY_DCC_MSG

// This function is called whenever a normal DCC Turnout Packet is received
void notifyDccAccState(uint16_t decoderAddress, uint8_t outputId, bool activate) {
  if (decoder.state == DECODER_PROGRAM) {
    // take the decoderAddress & program it as our own
    Dcc.writeDccAddress(decoderAddress);
    decoder.address = decoderAddress; // keep copy in RAM
    timer.oscillate(PIN_PROGLED, LED_FAST_FLASH, LOW, 3); // 3 led flashes ter bevestiging van een CV write
    #ifdef DEBUG
      Serial.print("CV1/CV9 rewritten, my address is now :");
      Serial.println(getDecoderAddress());
    #endif
    return;
  }

  if (decoder.softwareMode == SOFTWAREMODE_TURNOUT_DECODER)
    turnout_Handler (decoderAddress, outputId, activate);
  else if (decoder.softwareMode == SOFTWAREMODE_OUTPUT_DECODER)
    output_Handler (decoderAddress, outputId, activate);
  else {
    #ifdef DEBUG
      Serial.println("unsupported software mode!");
    #endif
    //goto safe mode??
  }
} // notifyDccAccState

// This function is called whenever a DCC Signal Aspect Packet is received
void notifyDccSigState(uint16_t decoderAddress, uint8_t signalId, uint8_t signalAspect) {
  #ifdef DEBUG
    Serial.print("notifyDccSigState: unsupported for now:");
    Serial.print(decoderAddress);
    Serial.print(',');
    Serial.print(signalId);
    Serial.print(',');
    Serial.println(signalAspect);
  #endif 
} // notifyDccSigState

// This function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read 
void notifyCVAck() {
  #ifdef DEBUG
    Serial.println("ack");
  #endif
  digitalWrite(PIN_ACKOUT, HIGH);
  delay(6);  
  digitalWrite(PIN_ACKOUT, LOW);
} // notifyCVAck

// enkel CV's schrijven als decoder niet 'live' is (INIT, FACTORY_RESET, PROGRAM)
uint8_t notifyCVWrite( uint16_t cv, uint8_t cvValue) {
  #ifdef DEBUG
    Serial.print ("CVWrite ");
    Serial.print(cv);
    Serial.print(" = ");
    Serial.println(cvValue);
    if (decoder.state == DECODER_RUNNING)
      Serial.println("no CVs written while decoder RUNNING");
  #endif

  if (decoder.state == DECODER_RUNNING) {
    return eeprom_read_byte((uint8_t*) cv);
  }
  if (cv == CV_MANUFACTURER_ID) { // writing the CV_MANUFACTURER_ID triggers a factory reset
    decoder.state = DECODER_FACTORY_RESET;
    eeprom_update_byte((uint8_t*) cv, MAN_ID_DIY); // in case of empty eeprom
    timer.oscillate(PIN_PROGLED, LED_FAST_FLASH, LOW, 3); // 3 led flashes ter bevestiging van een CV write
    return cvValue; // we pretend to write the value, but only to trigger an ackCV
  }
  else {
    eeprom_update_byte((uint8_t*) cv, cvValue);
    timer.oscillate(PIN_PROGLED, LED_FAST_FLASH, LOW, 3); // 3 led flashes ter bevestiging van een CV write
    return eeprom_read_byte((uint8_t*) cv);
  }
} // notifyCVWrite

// 0 = ongeldige schrijfactie gevraagd naar CV
// 0 = lezen naar niet-geïmplementeerde CV's : todo SDS!
// 1 = geldige actie
// DECODER_PROGRAM : for now we only allow CV writes while decoder is in DECODER_PROGRAM
bool notifyCVValid(uint16_t cv, bool writable) {
  bool isValid = true;

  // CV read/write only in programming mode
  if( (cv > MAXCV) || (decoder.state != DECODER_PROGRAM))
    isValid = false;

  // different from default, because we do allow writing CV_MANUFACTURER_ID to trigger a factory reset
  if (writable && ((cv==CV_VERSION_ID)||(cv==CV_DECODER_CONFIGURATION)||(cv==CV_RAILCOM_CONFIGURATION)))
    isValid = false;

  // TODO : uitbreiden!! (ook afhankelijk van swMode CV33)
  return isValid;
} // notifyCVValid

// for now we require the decoder to be in programming mode for both service mode programming & PoM
bool notifyPoMValid(uint16_t decoderAddress, uint16_t cv, bool writable) {
  if (decoder.address == decoderAddress)
    return notifyCVValid(cv,writable);
  else
    return false;
} // notifyPoMValid
