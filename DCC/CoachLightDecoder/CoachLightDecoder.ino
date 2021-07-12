#include "NmraDcc.h"
#include "CoachLightDecoder.h"

#ifdef ARDUINO_AVR_ATTINYX5
  #include <tinyNeoPixel_Static.h>
  #include <avr/eeprom.h> // attiny needs this include explicitly
#else 
  #include <Adafruit_NeoPixel.h>
#endif 

#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// compile for attiny85
// a mobile decoder responding to functions to control the interior light in a coach
// the light is a WS2812B led strip
// TODO : geen programming mode via key zoals accessory decoder
// TODO : is er uberhaupt plaats genoeg voor een ack circuit?

/*
// dcc.process() verwerkt exact 1 DCC packet. Vanuit dcc.process wordt de notify functie aangeroepen, die het packet verwerkt
// als de processing langer dan 1 DCC packet duurt, dan gaan er packetten verloren. NmraDcc heeft een buffer van 1 pakket!! (dccRx.PacketCopy)
De Rx ISR gaat gewoon door, en gaat op het einde van het pakket de bitbuffer in PacketCopy copiëren, ongeacht dataReady al op 0 stond

*/
// DCC signaal ontvangen op pin D2
// als je FLAGS_MY_ADDRESS_ONLY gebruikt worden enkel voor dit adres notifyDccAccState callbacks gedaan
// de DCC isr ontvangt nog steeds alle pakketten, en notifyDccMsg dus ook (indien niet NULL)
// zonder FLAGS_MY_ADDRESS_ONLY kan je voor alle accessory decoders  notifyDccAccState ontvangen
// ihb accessory decoder pakketten voor 2 verschillende adressen, om bv een decoder voor 8 wissels te maken
// om een factory reset te doen moet je een CV write naar CV8 doen
// om het adres te wijzigen doe je CV write (direct byte) naar CV1 en/of CV9

#define VersionId 0x1 // versie van deze software

// led strip stuff
#define LEDPIN        3
#define NUMPIXELS     12
#ifdef ARDUINO_AVR_ATTINYX5
  byte pixBytes[NUMPIXELS * 3];
  tinyNeoPixel pixels = tinyNeoPixel(NUMPIXELS, LEDPIN, NEO_GRB, pixBytes);
#else
  Adafruit_NeoPixel pixels(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);
#endif
uint8_t brightness = 50; // dcc speed command sets brightness
uint8_t funcs = 1;
bool doLedUpdate = true; // for initial update

// decoder global
NmraDcc  Dcc;

typedef enum {
  DECODER_FACTORY_RESET,DECODER_INIT,DECODER_RUNNING, DECODER_PROGRAM 
} decoderState_t;

static decoderState_t decoderState;

static uint16_t decoderAddress; // keep this local, other modules will use getter getDecoderAddress;
//factory defaults
#define CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB 	17
#define CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB 	18

// for a multifunction decoder (mobile decoder)
static CVPair FactoryDefaultCVs [] = {
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, 1},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, 0},
  {CV_VERSION_ID, VersionId},
  {CV_MANUFACTURER_ID, MAN_ID_DIY},
  {CV_DECODER_CONFIGURATION,0x80}
};

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

  return 0;
} // accessory_FactoryResetCV

void setup() {
  decoderState = DECODER_INIT;
  #ifdef DEBUG
    Serial.begin(115200);
    Serial.println("CoachDecoder AVR!");
  #endif

#ifdef ARDUINO_AVR_ATTINYX5
  pinMode(LEDPIN,OUTPUT);
  // pixels.begin() not needed on tinyNeoPixel
  pixels.clear(); // Set all pixel colors to 'off'

#else
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear(); // Set all pixel colors to 'off'
#endif
  
  // Configure the DCC CV Programing ACK pin for an output
  //pinMode(PIN_ACKOUT, OUTPUT);
  //digitalWrite(PIN_ACKOUT, LOW);
  
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  Dcc.pin(0, PIN_DCCIN, 1);
  
  // Call the main DCC Init function to enable the DCC Receiver
  // geen filtering FLAGS_MY_ADDRESS_ONLY !!
  // Dcc.init(FLAGS_DCC_ACCESSORY_DECODER, 0);
  Dcc.init(0, 0); // coach light decoder = mobile decoder, geen accessory decoder. TODO : FLAGS_MY_ADDRESS_ONLY nodig?

  // check if factory defaults are written in eeprom
  if (!Dcc.isSetCVReady()) {
    // we kunnen geen CV's inlezen, dus weten we niet wat te doen!!
    // goto safe mode --> alles outputs uit en wachten op progkey
    return;
  }
  if (Dcc.getCV(CV_MANUFACTURER_ID) != MAN_ID_DIY)
    decoderState = DECODER_FACTORY_RESET;
  else
    decoderState = DECODER_INIT;

} // setup

void loop() {
  switch (decoderState) {
    case DECODER_FACTORY_RESET : 
      if (accessory_FactoryResetCV () == 0){ // factory reset gelukt --> DECODER_INIT
        decoderState = DECODER_INIT;
      }
      break;
    case DECODER_INIT :
      decoderAddress =  Dcc.readDccAddress(); // reread from eeprom
      decoderState = DECODER_RUNNING;
      #ifdef DEBUG
        Serial.print("my address is : "); Serial.println(getDecoderAddress());
      #endif
      break;
    case DECODER_RUNNING :
    case DECODER_PROGRAM : 
      // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function 
      // for correct library operation
      Dcc.process();
      break;
  }
  if (doLedUpdate) {
    // F1 -> R, F2 -> G, F3 -> B
    uint8_t r,g,b;
    r = (funcs & 0x1)*brightness;
    g = ((funcs>>1)&0x1)*brightness;
    b = ((funcs>>2)&0x1)*brightness;
    for(int i=0; i<NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(r,g,b));
    }
    pixels.show();
    doLedUpdate = false; // reset flag
  }
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
void notifyDccReset(uint8_t hardReset) {
  (void) hardReset;
  // TODO : check, want er worden dcc resets gestuurd bij start programming mode, en dan is DECODER_INIT niet ok!!
  /*
  decoderState = DECODER_INIT; // the loop() function will get the decoder to a reinitialize
  // avr standard bootloader doesn't like a watchdog reset, so can't do a real reboot
  */
} // notifyDccReset

//#define NOTIFY_DCC_MSG
#ifdef  NOTIFY_DCC_MSG
void notifyDccMsg( DCC_MSG * Msg)
{
  Serial.print("notifyDccMsg[ ");
  Serial.print(millis());
  Serial.print("] : ");
  for(uint8_t i = 0; i < Msg->Size; i++)
  {
    Serial.print(Msg->Data[i], HEX);
    Serial.write(' ');
  }
  Serial.println();
}
#endif


// callbacks for multifunction (mobile) decoder
// MaxSpeed : 28,127 (waarom niet 14?)
void notifyDccSpeed(uint16_t decoderAddress, uint8_t speed, uint8_t ForwardDir, uint8_t MaxSpeed)
{
  if (speed != brightness) {
    brightness = speed;
    doLedUpdate = true;
    #ifdef DEBUG
      Serial.print("notifyDccSpeed:");Serial.print(decoderAddress);Serial.print(":");
      Serial.print(speed);Serial.print(":");
      Serial.print(ForwardDir);Serial.print(":");Serial.println(MaxSpeed);
    #endif
  }
} // notifyDccSpeed

void notifyDccFunc( uint16_t decoderAddress, FN_GROUP FuncGrp, uint8_t FuncState) {
  if ((FuncGrp == FN_0_4) && (FuncState != funcs)) {
    funcs = FuncState;
    doLedUpdate = true;
    #ifdef DEBUG
      Serial.print("notifyDccFunc:");Serial.print(decoderAddress);Serial.print(":");
      Serial.print(FuncGrp);Serial.print(":");Serial.println(FuncState);
    #endif
  }
} // notifyDccFunc

// This function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read 
void notifyCVAck(void) {
  #ifdef DEBUG
    Serial.println("ack");
  #endif
  /* temp
  digitalWrite(PIN_ACKOUT, HIGH);
  delay(6);  
  digitalWrite(PIN_ACKOUT, LOW);
  */
} // notifyCVAck

// enkel CV's schrijven als decoder niet 'live' is (INIT, FACTORY_RESET, PROGRAM)
uint8_t notifyCVWrite( uint16_t cv, uint8_t cvValue) {
  #ifdef DEBUG
    Serial.print ("CVWrite ");
    Serial.print(cv);
    Serial.print(" = ");
    Serial.println(cvValue);
    if (decoderState == DECODER_RUNNING)
      Serial.println("no CVs written while decoder RUNNING");
  #endif

  if (decoderState == DECODER_RUNNING) {
    return eeprom_read_byte((uint8_t*) cv);
  }
  if (cv == CV_MANUFACTURER_ID) { // writing the CV_MANUFACTURER_ID triggers a factory reset
    decoderState = DECODER_FACTORY_RESET;
    eeprom_update_byte((uint8_t*) cv, MAN_ID_DIY); // in case of empty eeprom
    // timer.oscillate(PIN_PROGLED, LED_FAST_FLASH, LOW, 3); // 3 led flashes ter bevestiging van een CV write
    // misschien coach leds knipperen?
    return cvValue; // we pretend to write the value, but only to trigger an ackCV
  }
  else {
    eeprom_update_byte((uint8_t*) cv, cvValue);
    //timer.oscillate(PIN_PROGLED, LED_FAST_FLASH, LOW, 3); // 3 led flashes ter bevestiging van een CV write
    // misschien coach leds knipperen?
    return eeprom_read_byte((uint8_t*) cv);
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
