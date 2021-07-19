// configure NmraDcc for mobile decoder!
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

/*
 * dcc.process() verwerkt exact 1 DCC packet. Vanuit dcc.process wordt de notify functie aangeroepen, die het packet verwerkt
 * als de processing langer dan 1 DCC packet duurt, dan gaan er packetten verloren. NmraDcc heeft een buffer van 1 pakket!! (dccRx.PacketCopy)
 * De Rx ISR gaat gewoon door, en gaat op het einde van het pakket de bitbuffer in PacketCopy copiëren, ongeacht dataReady al op 0 stond
 * DCC signaal ontvangen op pin D2
 * setFilterAddress : let nmra lib filter on this address
 * no filtering is useful for eg. a accessory decoder spanning 2 addresses, then the filtering has to be done outside nmra lib
 * broadcast addresses are always callbacked (not filtered)
 * om een factory reset te doen moet je een CV write naar CV8 doen
 * writeDccAddress : use this change decoder address, lib handles writing the correct CVs
*/

#define VersionId 0x1 // versie van deze software
#define CV_LIGHTMODE  33
#define CV_BRIGHTNESS 34
#define CV_PRESET     35
#define CV_RGB_R      36
#define CV_RGB_G      37
#define CV_RGB_B      38

#define LIGHTMODE_PRESET 1
#define LIGHTMODE_RGB    2

// led strip stuff
#define LEDPIN        3
#define NUMPIXELS     12
#ifdef ARDUINO_AVR_ATTINYX5
  byte pixBytes[NUMPIXELS * 3];
  tinyNeoPixel pixels = tinyNeoPixel(NUMPIXELS, LEDPIN, NEO_GRB, pixBytes);
#else
  Adafruit_NeoPixel pixels(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);
#endif

// decoder global
NmraDcc  Dcc;

typedef enum {
  DECODER_FACTORY_RESET,DECODER_INIT,DECODER_RUNNING 
} decoderState_t;

static decoderState_t decoderState;

//factory defaults for the mobile decoder
static CVPair FactoryDefaultCVs [] = {
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, 5},
  {CV_VERSION_ID, VersionId},
  {CV_MANUFACTURER_ID, MAN_ID_DIY},
  {CV_LIGHTMODE,LIGHTMODE_PRESET},
  {CV_PRESET,0},
  {CV_BRIGHTNESS,50},
  {CV_DECODER_CONFIGURATION,0x2} // temp, eigenlijk moet nmradcc lib dat doen
};

// params to set the coach lighting
uint8_t lightMode;
uint8_t presetId = 0;
uint8_t rgb[3];
uint8_t brightness;
bool doLedUpdate;
// inputs over dcc:
uint8_t lightValue; // used to set overall brightness (F1), preset (F2), rgb values (F3,F4,F5)
uint8_t lightFuncs = 1; // to force init with lights on in absence of dcc commands to our decoder

// standard lighting colours (copied from fastled)
static uint8_t stdColors[][3] = {
  {255,147, 41},    // Candle
  {255,197,143},    // Tungsten
  {255,241,224},    // Halogen
  {201,226,255},    // overcast sky
  { 64,156,255},    // clear blue sky
  {255,244,229},    // warm fluorescent
  {212,235,255},    // cool white fluorescent
  {255,239,247},    // grow light fluorescent
  {216,247,255},    // mercury vapor
  {255,209,178},    // sodium vapor
  {255,183, 76},    // high pressure sodium
  {255,105,180}     // HotPink just for fun
};

const uint8_t numStdColors = sizeof(stdColors) / 3;

/*****************************************************************************/
/*    LOCAL FUNCTIONS                                                        */
/*****************************************************************************/
// 0 = DONE, 1 = NOT DONE (eeprom not ready)
static uint8_t decoder_FactoryResetCV() {
  if (!Dcc.isSetCVReady())
    return 1;
  // factory reset de algemene CV's (die niet afhangen van de software mode
  for (uint8_t i=0; i < sizeof(FactoryDefaultCVs)/sizeof(CVPair); i++)
    Dcc.setCV( FactoryDefaultCVs[i].CV, FactoryDefaultCVs[i].Value);

  return 0;
} // decoder_FactoryResetCV

static void decoder_writeSettings() {
  if (lightMode == LIGHTMODE_PRESET) {
    Dcc.setCV(CV_LIGHTMODE,LIGHTMODE_PRESET);
    Dcc.setCV(CV_PRESET,presetId);
  }
  else {
    Dcc.setCV(CV_LIGHTMODE,LIGHTMODE_RGB);
    Dcc.setCV(CV_RGB_R,rgb[0]);
    Dcc.setCV(CV_RGB_G,rgb[1]);
    Dcc.setCV(CV_RGB_B,rgb[2]);
  }
  Dcc.setCV(CV_BRIGHTNESS,brightness);
} // decoder_writeSettings

static void decoder_readSettings() {
  lightMode = Dcc.getCV(CV_LIGHTMODE);
  brightness = Dcc.getCV(CV_BRIGHTNESS);
  presetId = Dcc.getCV(CV_PRESET);
  rgb[0] = Dcc.getCV(CV_RGB_R);
  rgb[1] = Dcc.getCV(CV_RGB_G);
  rgb[2] = Dcc.getCV(CV_RGB_B);
  #ifdef DEBUG
    Serial.print("lightMode:");Serial.println(lightMode);
    Serial.print("brightness:");Serial.println(brightness);
    Serial.print("presetId:");Serial.println(presetId);
  #endif
} // decoder_readSettings

// set a preset lighting mode (fixed colors for now)
static void leds_setPreset(uint8_t presetId) {
  uint8_t *preset;
  uint8_t rainbow[][3] = {
    {255,0,0},
    {255,127,0},
    {255,255,0},
    {127,255,0},
    {0,255,0},
    {0,255,127},
    {0,255,255},
    {0,127,255},
    {0,0,255},
    {127,0,255},
    {255,0,255},
    {255,0,127}
  };
  
  if (presetId < numStdColors) {
    preset = stdColors[presetId];
    for (uint8_t i=0;i<NUMPIXELS;i++) {
      pixels.setPixelColor(i,pixels.Color(pixels.gamma8(preset[0]),
                                          pixels.gamma8(preset[1]),
                                          pixels.gamma8(preset[2])));
    }
  }
  else if (presetId < 2*numStdColors) { // a range with half lighting
    preset = stdColors[presetId-numStdColors];
    for (uint8_t i=0;i<NUMPIXELS;i=i+2) {
      pixels.setPixelColor(i,pixels.Color(pixels.gamma8(preset[0]),
                                          pixels.gamma8(preset[1]),
                                          pixels.gamma8(preset[2])));
    }
    for (uint8_t i=1;i<NUMPIXELS;i=i+2) 
      pixels.setPixelColor(i,pixels.Color(0,0,0));
  }
  else {
    switch (presetId) {
      case 30 : // rainbow
        for (uint8_t i=0;i<NUMPIXELS;i++)
          pixels.setPixelColor(i,pixels.Color(pixels.gamma8(rainbow[i][0]),
                                              pixels.gamma8(rainbow[i][1]),
                                              pixels.gamma8(rainbow[i][2])));
        break;
      case 31 : // rainbow red->yellow
        for (uint8_t i=0;i<NUMPIXELS;i++)
          pixels.setPixelColor(i,pixels.Color(255,pixels.gamma8(23*i),0));
        break;
      case 32 : // rainbow yellow->green
        for (uint8_t i=0;i<NUMPIXELS;i++)
          pixels.setPixelColor(i,pixels.Color(pixels.gamma8(255 - 23*i),255,0));
        break;
      case 33 : // rainbow green->cyan
        for (uint8_t i=0;i<NUMPIXELS;i++)
          pixels.setPixelColor(i,pixels.Color(0,255,pixels.gamma8(23*i)));
        break;
      case 34 : // rainbow cyan->blue
        for (uint8_t i=0;i<NUMPIXELS;i++)
          pixels.setPixelColor(i,pixels.Color(0,pixels.gamma8(255 - 23*i),255));
        break;
      case 35 : // rainbow blue->pink
        for (uint8_t i=0;i<NUMPIXELS;i++)
          pixels.setPixelColor(i,pixels.Color(pixels.gamma8(23*i),0,255));
        break;
      case 36 : // rainbow pink->red
        for (uint8_t i=0;i<NUMPIXELS;i++)
          pixels.setPixelColor(i,pixels.Color(255,0,pixels.gamma8(255 - 23*i)));
        break;
      default:
        break;
    }
  }
} // leds_setPreset

/*****************************************************************************/
/*    MAIN                                                                   */
/*****************************************************************************/
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
  pixels.show();
  
  // Call the main DCC Init function to enable the DCC Receiver
  // use default ext int 0 on pin 2
  Dcc.init();

  // check if factory defaults are written in eeprom
  if (!Dcc.isSetCVReady()) { // we kunnen geen CV's inlezen, dus weten we niet wat te doen!!
    return;
  }
  Dcc.setFilterAddress(Dcc.readDccAddress());
  if (Dcc.getCV(CV_MANUFACTURER_ID) != MAN_ID_DIY)
    decoderState = DECODER_FACTORY_RESET;
  else
    decoderState = DECODER_INIT;
} // setup

void loop() {
  switch (decoderState) {
    case DECODER_FACTORY_RESET : 
      if (decoder_FactoryResetCV () == 0){ // factory reset gelukt --> DECODER_INIT
        decoderState = DECODER_INIT;
      }
      break;
    case DECODER_INIT :
      decoderState = DECODER_RUNNING;
      decoder_readSettings();
      doLedUpdate = true;
      #ifdef DEBUG
        Serial.print("my address is : "); Serial.println(Dcc.readDccAddress()); // read address from eeprom
      #endif
      break;
    case DECODER_RUNNING :
      Dcc.process();
      break;
  }
  // the coach lighting function
  if (doLedUpdate) {
    if (lightFuncs & 0x4) { // F2 sets a preset
      lightMode = LIGHTMODE_PRESET;
      presetId = lightValue;
    }
    else if (lightFuncs & 0x38) { // F3-F4-F5 set a arbitrary rgb color
      // we allow to set multiple values simultaneously
      // F2 -> R, F3 -> G, F4 -> B
      lightMode = LIGHTMODE_RGB;
      brightness = 255; // for convenience
      if (lightFuncs & 0x8) { // F3
        rgb[0] = lightValue;
      }
      if (lightFuncs & 0x10) { // F4
        rgb[1] = lightValue;
      }
      if (lightFuncs & 0x20) { // F5
        rgb[2] = lightValue;
      }
    }
    else if (lightFuncs & 0x2) { // F1 sets brightness if higher functions are off
      brightness = lightValue;
    }

    if ((lightFuncs & 0x1)== 0) { // F0 off -> lights off & storing setting
       pixels.clear();
       decoder_writeSettings();
      #ifdef DEBUG
        Serial.println("storing settings!");
        Serial.flush();
      #endif
    }
    else { // now redraw the leds with the new settings
      // note : all pixel values must also be recalculated (setPixelColor) everytime the brightness changes
      #ifdef DEBUG
        Serial.print("brightness:");Serial.println(brightness);
      #endif
      pixels.setBrightness(brightness);
      if (lightMode == LIGHTMODE_PRESET) {
        leds_setPreset(presetId);
        #ifdef DEBUG
          Serial.print("setPreset:");Serial.println(presetId);
        #endif 
      }
      else {
        for(uint8_t i=0; i<NUMPIXELS; i++) {
          pixels.setPixelColor(i, pixels.Color(pixels.gamma8(rgb[0]),pixels.gamma8(rgb[1]),pixels.gamma8(rgb[2])));
        }
        #ifdef DEBUG
          Serial.print("rgb=");Serial.print(rgb[0]);Serial.print(":");
          Serial.print(rgb[1]);Serial.print(":");Serial.println(rgb[2]);
          Serial.flush();
        #endif
        
      }
    }
    pixels.show();
    doLedUpdate = false; // reset flag
  }
} // loop

/*****************************************************************************/
/*    notify functions from Nmra layer                                       */
/*****************************************************************************/
void notifyDccReset(uint8_t hardReset) {
  (void) hardReset;
  // TODO : check, want er worden dcc resets gestuurd bij start programming mode, en dan is DECODER_INIT niet ok!!
  /*
  decoderState = DECODER_INIT; // the loop() function will get the decoder to a reinitialize
  // avr standard bootloader doesn't like a watchdog reset, so can't do a real reboot
  */
} // notifyDccReset

// callbacks for multifunction (mobile) decoder
void notifyDccSpeed(uint16_t decoderAddress, uint8_t speed, bool directionIsForward, uint8_t speedType) {
  // combine dir+speed to get light value in 0..255 range
  // support DCC128 only
  if (directionIsForward) speed += 0x80;
  else if (speed) speed -= 1; // for convenience because we can't easily dial speed=1

  if (speed != lightValue) {
    lightValue = speed;
    if (lightFuncs & 0x1) doLedUpdate = true; // avoid work if leds are off already
    #ifdef DEBUG
      Serial.print("notifyDccSpeed:");Serial.print(decoderAddress);Serial.print(":");
      Serial.print(speed);Serial.print(":");Serial.print(directionIsForward);
      Serial.print(":");Serial.println(speedType);
    #endif
  }

} // notifyDccSpeed

void notifyDccFunc( uint16_t decoderAddress, FN_GROUP FuncGrp, uint8_t FuncState) {
  if ((FuncGrp == FN_0_4) && (FuncState != (lightFuncs & 0x1F))) { // F0..F4
    lightFuncs = (lightFuncs & 0xE0) | FuncState;
    doLedUpdate = true;
  }
  else if ((FuncGrp == FN_5_8) && (FuncState != ((lightFuncs >> 5) & 0x1))) { // F5
    lightFuncs = (lightFuncs & 0x1F) | ((FuncState & 0x1)<<5);
    doLedUpdate = true;
  }
  #ifdef DEBUG
  if (doLedUpdate) {
    Serial.print("notifyDccFunc:");Serial.print(decoderAddress);Serial.print(":");
    Serial.print(FuncGrp);Serial.print(":");Serial.println(FuncState);
  }
  #endif
} // notifyDccFunc

// This function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read 
void notifyCVAck() {
  #ifdef DEBUG
    Serial.println("ack");
  #endif
  // trigger an ack detection by pulsing the leds at max brightness
  pixels.setBrightness(255);
  for(uint8_t i=0; i<NUMPIXELS; i++)
    pixels.setPixelColor(i, pixels.Color(255,255,255));
  pixels.show();
  delay(6);
  pixels.clear();
  pixels.show();
} // notifyCVAck

// Note! CoachLightDecoder version : we always allow writing cv's, independent of decoderState!
uint8_t notifyCVWrite( uint16_t cv, uint8_t cvValue) {
  #ifdef DEBUG
    Serial.print ("CVWrite ");
    Serial.print(cv);
    Serial.print(" = ");
    Serial.println(cvValue);
  #endif

  if (cv == CV_MANUFACTURER_ID) { // writing the CV_MANUFACTURER_ID triggers a factory reset
    decoderState = DECODER_FACTORY_RESET;
    eeprom_update_byte((uint8_t*) cv, MAN_ID_DIY); // in case of empty eeprom
    // misschien coach leds knipperen?
    return cvValue; // we pretend to write the value, but only to trigger an ackCV
  }
  else {
    eeprom_update_byte((uint8_t*) cv, cvValue);
    // misschien coach leds knipperen?
    return eeprom_read_byte((uint8_t*) cv);
  }
} // notifyCVWrite

// 0 = ongeldige schrijfactie gevraagd naar CV
// 0 = lezen naar niet-geïmplementeerde CV's : todo SDS!
// 1 = geldige actie
bool notifyCVValid(uint16_t cv, bool writable) {
  bool isValid = true;

  if(cv > MAXCV)
    isValid = false;

  // different from default, because we do allow writing CV_MANUFACTURER_ID to trigger a factory reset
  if (writable && ((cv==CV_VERSION_ID)||(cv==CV_RAILCOM_CONFIGURATION)))
    isValid = false;

  return isValid;
} // notifyCVValid