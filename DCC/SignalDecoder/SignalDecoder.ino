/*
 * for test : connect signal heads[] to pins 3->8 (8=VCC)
 * bug: CV programming werkt nog niet (DECODER_PROGRAM todo)
 * opgelet CV programming heeft een ack circuit nodig, anders werkt programming niet!
 * 
 */
#include "NmraDcc.h"
#include "SignalDecoder.h"
#include "Wire.h"
#include "Timer.h"

#define VersionId (0x1)

NmraDcc Dcc;

typedef enum {
  DECODER_FACTORY_RESET,DECODER_INIT,DECODER_RUNNING, DECODER_PROGRAM 
} decoderState_t;

static decoderState_t decoderState;
//static uint8_t decoderSoftwareMode;
static uint16_t decoderAddress;

//factory defaults
static CVPair FactoryDefaultCVs [] = {
  {CV_ACCESSORY_DECODER_ADDRESS_LSB, 1},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, 0},
  {CV_VERSION_ID, VersionId},
  {CV_MANUFACTURER_ID, MAN_ID_DIY},
  {CV_DECODER_CONFIGURATION,0x80}
};

// signal decoder
// outputs used : D3..D12,A2..A3 + 8 io expander
#define NUM_HEADS             2
#define NUM_OUTPUTS_PER_HEAD  3
#define NUM_OUTPUTS           20

#define PIN_UNUSED            0xFF

// temp for test
// built-in I/O -> head 0
// deze pins gaan door de pinLUT, dus PIN_0_YELLOW wordt digitalWrite(pinLUT[PIN_0_YELLOW]) = digitalWrite(3)
#define PIN_0_WHITE   4
#define PIN_0_ORANGE  3
#define PIN_0_RED     2
#define PIN_0_GREEN   1
#define PIN_0_YELLOW  0

// expander -> head 1
#define PIN_1_WHITE   16  // lukt niet op breadboard, dus dan maar pin aan de overkant van de chip
#define PIN_1_ORANGE  12   // P0 op de expander volgens pinLUT
#define PIN_1_RED     13
#define PIN_1_GREEN   14
#define PIN_1_YELLOW  15

typedef struct {
  uint8_t pin;
  uint8_t ledOn;
  uint8_t ledOff;
  uint8_t fOnCycle;
  uint8_t fOffCycle;
  uint8_t fTotalCycles;
  uint8_t fCycle; //0->15
} output_t;

typedef struct {
  output_t outputs[NUM_OUTPUTS_PER_HEAD]; // save mem, max [3] outputs active per heads[]
  uint8_t ms;
  uint8_t bCycle; // 0->9
} head_t;
volatile head_t heads[NUM_HEADS]; // a decoder supports up to 4 signal heads

static uint8_t ioByte; // the IO expander bits buffered (1 i2c expander for now)
static bool ioUpdate = false; // minimize i2c accesses for minimal flickering

// list of all output pins
// builtin pins use arduino.h numbering
// expander pins are numbered 0x80 + (A2A1A0 <<3) + pin (0..7)
// for example 8 built-in pins, 8 I/O expander pins on 0x26 (A0=0,A1=1,A2=1)
// headPin (used for the heads definitions) is the index in this pinLUT array
static uint8_t pinLUT[NUM_OUTPUTS] = {3,4,5,6,7,8,9,10,11,12,16,17,0xB0,0xB1,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7};

// keyhandling -> only progkey for signal decoder
#define DEBOUNCE_DELAY 50
#define LONGPRESS_DELAY 3000
#define NUMBER_OF_KEYS 1 // enkel progkey
typedef enum {UP, DEBOUNCING_DOWN, DOWN, LONG_DOWN, DEBOUNCING_UP} keyState_t;
typedef struct {
  uint8_t Pin;
  keyState_t State;
  uint32_t LastMillis;
} key_t;
key_t keys[NUMBER_OF_KEYS];

// timers
Timer timer;
int8_t ledTimer;

/******************************************************************/
/*  KEY HANDLING                                                  */
/******************************************************************/
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
    if (decoderState != DECODER_PROGRAM) {
      decoderState = DECODER_PROGRAM;
      ledTimer = timer.oscillate(PIN_PROGLED, LED_SLOW_FLASH, HIGH, -1); //forever
    }
  }
  else if (keyEvent & KEYEVENT_DOWN) {
    if (decoderState == DECODER_PROGRAM) {
      decoderState = DECODER_INIT;
    }
    else {
      // TODO manual test of signal heads? lamp test?
    }
  }
} // keyHandler

/******************************************************************/
/*  SIGNAL DECODER                                                */
/******************************************************************/
// timer interrupt used for 1ms interrupt
ISR (TIMER2_COMPA_vect) {
  for (uint8_t head=0;head<NUM_HEADS;head++) {
    heads[head].ms++;
    if (heads[head].ms == 250) {
      heads[head].ms = 0;
    }
    // check flashing at every start of a 250ms cycle
    if (heads[head].ms==0) {
      for (uint8_t output=0;output<NUM_OUTPUTS_PER_HEAD;output++) {
        if (heads[head].outputs[output].pin != PIN_UNUSED) {
          heads[head].outputs[output].fCycle++;
          if (heads[head].outputs[output].fCycle == heads[head].outputs[output].fTotalCycles) 
            heads[head].outputs[output].fCycle=0;
        }
      }
    }
    heads[head].bCycle++;
    if (heads[head].bCycle == 10) heads[head].bCycle = 0;
  }
} // ISR

// all io expander bits are outputs, so keep it simple, no need for another library
static void ioDigitalWrite(uint8_t i2cAddress, uint8_t ioByte) {
  Wire.beginTransmission(i2cAddress);
  Wire.write(ioByte);
  Wire.endTransmission();
} // ioDigitalWrite

// TODO : for now we use only 1 expander
// so no need now to separate ioBytes or extract the i2c address from the lutPin
static void myDigitalWrite(uint8_t headPin,uint8_t pinValue) {
  uint8_t lutPin;
  
  if (headPin == PIN_UNUSED)
    return;
  
  lutPin = pinLUT[headPin];
  if (lutPin & 0x80) { // expander pin
    uint8_t curPinValue;
    curPinValue = (ioByte >> (lutPin & 0x7)) & 0x1;
    if (curPinValue != pinValue) {
      ioUpdate = true; // i2c update will happen later
      if (pinValue) { // set bit
        ioByte |= (1<<(lutPin & 0x7));
      }
      else { // clear bit
        ioByte &= ~(1<<(lutPin & 0x7));
      }
    }
  }
  else { // local pin
    digitalWrite(lutPin,pinValue);
  }
} // myDigitalWrite

static void myPinMode(uint8_t headPin,uint8_t pinValue) {
  uint8_t lutPin = pinLUT[headPin];
  if (lutPin & 0x80) { // expander pin --> no need to set pinMode
  }
  else { // local pin
    pinMode(lutPin,pinValue);
  }
} // myPinMode

static void signal_init() {
  Wire.begin(); // for the i2c expander

  // set pins as outputs
  for (uint8_t headPin=0;headPin<NUM_OUTPUTS;headPin++) {
    myPinMode(headPin,OUTPUT);
    myDigitalWrite(headPin, HIGH);
  }

  // start with all head pins unused
  for (uint8_t head=0;head<NUM_HEADS;head++) {
    for (uint8_t output=0;output<NUM_OUTPUTS_PER_HEAD;output++) {
      heads[head].outputs[output].pin = PIN_UNUSED;
    }
  }

  // configure timer 2 for 1ms interrupt
  // 16MHz : prescale /128 = 125kHz -> 8us per tick
  // 8Mhz : prescale /64 = 125kHz ->8us per tich
  // -> gebruik 125x8us als output compare
  TCCR2A = (1 << WGM21);  // CTC mode of operation (tot OCR2A tellen)
  OCR2A = 124;            // 125x 8us per output compare
  TIFR2 = 0x7;            // clear pending interrupts
  TIMSK2 = (1 << OCIE2A); // enable OC channel A interrupt
  TCNT2 = 0;
  #if (F_CPU==8000000L)
    TCCR2B = (1 << CS22); // prescaler /64 voor 8MHz pro Mini
  #else
    TCCR2B = (1 << CS22) | (1 << CS20);   // prescaler /128 (opgelet : niet dezelfde bits voor TIMER0!!! table 19-10 vs. 22-10)
  #endif  

} // signal_init

// initialize a specific aspect on a chosen signal head
// headId : 0..3, aspectId : 0..31 (according DCC)
static void head_init(uint8_t headId, uint8_t aspectId) {
  // reset the outputs used in the previous aspect
  for (uint8_t output=0;output<NUM_OUTPUTS_PER_HEAD;output++) {
    myDigitalWrite(heads[headId].outputs[output].pin,HIGH);
    heads[headId].outputs[output].pin = PIN_UNUSED;
  }

  // test implmementation of various heads
  // 2 identieke heads, maar op verschillende output sets
  // head 0 : pins 1..5
  // head 1 : pins 9..13 (I/O expander)
  
  cli();
  // defaults voor JMRI infrabel-2013
  if (aspectId == 0) { // stop - rood
    if (headId == 0) heads[headId].outputs[0].pin = PIN_0_RED; // rood
    else if (headId == 1) heads[headId].outputs[0].pin = PIN_1_RED; // rood
    heads[headId].outputs[0].ledOn = 0;
    heads[headId].outputs[0].ledOff = 8;
    heads[headId].outputs[0].fOnCycle = 0;
    heads[headId].outputs[0].fOffCycle = 1;
    heads[headId].outputs[0].fTotalCycles = 1;
    heads[headId].outputs[0].fCycle = 0;
  }
  else if (aspectId == 1) { // approach - dubbel geel
    if (headId == 0) heads[headId].outputs[0].pin = PIN_0_ORANGE; // geel
    else if (headId == 1) heads[headId].outputs[0].pin = PIN_1_ORANGE; // geel
    heads[headId].outputs[0].ledOn = 0;
    heads[headId].outputs[0].ledOff = 8;
    heads[headId].outputs[0].fOnCycle = 0;
    heads[headId].outputs[0].fOffCycle = 1;
    heads[headId].outputs[0].fTotalCycles = 1;
    heads[headId].outputs[0].fCycle = 0;
    if (headId == 0) heads[headId].outputs[1].pin = PIN_0_YELLOW; // geel
    else if (headId == 1) heads[headId].outputs[1].pin = PIN_1_YELLOW; // geel
    heads[headId].outputs[1].ledOn = 0;
    heads[headId].outputs[1].ledOff = 8;
    heads[headId].outputs[1].fOnCycle = 0;
    heads[headId].outputs[1].fOffCycle = 1;
    heads[headId].outputs[1].fTotalCycles = 1;
    heads[headId].outputs[1].fCycle = 0;
  }
  else if (aspectId == 2) { // clear - groen
    if (headId == 0) heads[headId].outputs[0].pin = PIN_0_GREEN; // groen
    else if (headId == 1) heads[headId].outputs[0].pin = PIN_1_GREEN; // groen
    heads[headId].outputs[0].ledOn = 0;
    heads[headId].outputs[0].ledOff = 8;
    heads[headId].outputs[0].fOnCycle = 0;
    heads[headId].outputs[0].fOffCycle = 1;
    heads[headId].outputs[0].fTotalCycles = 1;
    heads[headId].outputs[0].fCycle = 0;
  }
  else if (aspectId == 3) { // restricting - rood wit
    if (headId == 0) heads[headId].outputs[0].pin = PIN_0_WHITE; // wit
    else if (headId == 1) heads[headId].outputs[0].pin = PIN_1_WHITE; // wit
    heads[headId].outputs[0].ledOn = 0;
    heads[headId].outputs[0].ledOff = 1;
    heads[headId].outputs[0].fOnCycle = 0;
    heads[headId].outputs[0].fOffCycle = 1;
    heads[headId].outputs[0].fTotalCycles = 1;
    heads[headId].outputs[0].fCycle = 0;
    if (headId == 0) heads[headId].outputs[1].pin = PIN_0_RED; // rood
    else if (headId == 1) heads[headId].outputs[1].pin = PIN_1_RED; // rood
    heads[headId].outputs[1].ledOn = 2;
    heads[headId].outputs[1].ledOff = 9;
    heads[headId].outputs[1].fOnCycle = 0;
    heads[headId].outputs[1].fOffCycle = 1;
    heads[headId].outputs[1].fTotalCycles = 1;
    heads[headId].outputs[1].fCycle = 0;  
  }
  else if (aspectId == 4) { // advanced approach - geel-groen vertikaal
    if (headId == 0) heads[headId].outputs[0].pin = PIN_0_ORANGE; // geel
    else if (headId == 1) heads[headId].outputs[0].pin = PIN_1_ORANGE; // geel
    heads[headId].outputs[0].ledOn = 0;
    heads[headId].outputs[0].ledOff = 8;
    heads[headId].outputs[0].fOnCycle = 0;
    heads[headId].outputs[0].fOffCycle = 1;
    heads[headId].outputs[0].fTotalCycles = 1;
    heads[headId].outputs[0].fCycle = 0;
    if (headId == 0) heads[headId].outputs[1].pin = PIN_0_GREEN; // groen
    else if (headId == 1) heads[headId].outputs[1].pin = PIN_1_GREEN; // groen
    heads[headId].outputs[1].ledOn = 0;
    heads[headId].outputs[1].ledOff = 8;
    heads[headId].outputs[1].fOnCycle = 0;
    heads[headId].outputs[1].fOffCycle = 1;
    heads[headId].outputs[1].fTotalCycles = 1;
    heads[headId].outputs[1].fCycle = 0;  
  }
  else if (aspectId == 5) { // slow approach - geel-groen horizontaal
    if (headId == 0) heads[headId].outputs[0].pin = PIN_0_YELLOW; // geel
    else if (headId == 1) heads[headId].outputs[0].pin = PIN_1_YELLOW; // geel
    heads[headId].outputs[0].ledOn = 0;
    heads[headId].outputs[0].ledOff = 8;
    heads[headId].outputs[0].fOnCycle = 0;
    heads[headId].outputs[0].fOffCycle = 1;
    heads[headId].outputs[0].fTotalCycles = 1;
    heads[headId].outputs[0].fCycle = 0;
    if (headId == 0) heads[headId].outputs[1].pin = PIN_0_GREEN; // groen
    else if (headId == 1) heads[headId].outputs[1].pin = PIN_1_GREEN; // groen
    heads[headId].outputs[1].ledOn = 0;
    heads[headId].outputs[1].ledOff = 8;
    heads[headId].outputs[1].fOnCycle = 0;
    heads[headId].outputs[1].fOffCycle = 1;
    heads[headId].outputs[1].fTotalCycles = 1;
    heads[headId].outputs[1].fCycle = 0;  
  }
  else if (aspectId == 8) { // unlit - alles uit
  }
  // nog wat fantasietjes
  else if (aspectId == 6) { // dubbel geel knipperend
    if (headId == 0) heads[headId].outputs[0].pin = PIN_0_ORANGE; // geel midden
    else if (headId == 1) heads[headId].outputs[0].pin = PIN_1_ORANGE; // geel midden
    heads[headId].outputs[0].ledOn = 1;
    heads[headId].outputs[0].ledOff = 9;
    heads[headId].outputs[0].fOnCycle = 0;
    heads[headId].outputs[0].fOffCycle = 2;
    heads[headId].outputs[0].fTotalCycles = 4;
    heads[headId].outputs[0].fCycle = 0;
    if (headId == 0) heads[headId].outputs[1].pin = PIN_0_YELLOW; // geel boven
    else if (headId == 1) heads[headId].outputs[1].pin = PIN_1_YELLOW; // geel boven
    heads[headId].outputs[1].ledOn = 1;
    heads[headId].outputs[1].ledOff = 9;
    heads[headId].outputs[1].fOnCycle = 0;
    heads[headId].outputs[1].fOffCycle = 2;
    heads[headId].outputs[1].fTotalCycles = 4;
    heads[headId].outputs[1].fCycle = 0;  
  }
  else if (aspectId == 7) { // rood wit knipperend
    if (headId == 0) heads[headId].outputs[0].pin = PIN_0_WHITE; // wit
    else if (headId == 1) heads[headId].outputs[0].pin = PIN_1_WHITE; // wit
    heads[headId].outputs[0].ledOn = 0;
    heads[headId].outputs[0].ledOff = 1;
    heads[headId].outputs[0].fOnCycle = 0;
    heads[headId].outputs[0].fOffCycle = 4;
    heads[headId].outputs[0].fTotalCycles = 8;
    heads[headId].outputs[0].fCycle = 0;
    if (headId == 0) heads[headId].outputs[1].pin = PIN_0_RED; // rood
    else if (headId == 1) heads[headId].outputs[1].pin = PIN_1_RED; // rood
    heads[headId].outputs[1].ledOn = 2;
    heads[headId].outputs[1].ledOff = 10;
    heads[headId].outputs[1].fOnCycle = 0;
    heads[headId].outputs[1].fOffCycle = 4;
    heads[headId].outputs[1].fTotalCycles = 8;
    heads[headId].outputs[1].fCycle = 0;  
  }
  else  {// alles uit
  }
  heads[headId].ms = 0;
  heads[headId].bCycle = 0;

  sei();
} // head_init

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
    // TODO for signal decoder
  }
  return 0;
} // accessory_FactoryResetCV

void setup() {
  decoderState = DECODER_INIT;
  #ifdef DEBUG
    Serial.begin(115200);
    Serial.println("SignalDecoder AVR!");
  #endif

  // Configure the DCC CV Programing ACK pin for an output
  pinMode(PIN_ACKOUT, OUTPUT);
  digitalWrite(PIN_ACKOUT, LOW);
  
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  Dcc.pin(0, PIN_DCCIN, 1);
  
  // Call the main DCC Init function to enable the DCC Receiver
  // geen filtering FLAGS_MY_ADDRESS_ONLY !!
  Dcc.init(FLAGS_DCC_ACCESSORY_DECODER, 0);

  // progled, progkey
  pinMode (PIN_PROGKEY, INPUT_PULLUP);
  pinMode (PIN_PROGLED, OUTPUT);
  digitalWrite(PIN_PROGLED, LOW); //LED uit

  if (Dcc.getCV(CV_MANUFACTURER_ID) != MAN_ID_DIY) {
    decoderState = DECODER_FACTORY_RESET;
  }
  else {
    decoderState = DECODER_INIT;
  }

  // init key handling
  init_keys ();
} // setup

void loop() {
  switch (decoderState) {
    case DECODER_FACTORY_RESET : 
      if (accessory_FactoryResetCV () == 0) { // factory reset gelukt --> DECODER_INIT
        decoderState = DECODER_INIT;
      }
      break;
    case DECODER_INIT :
      timer.stop (ledTimer); //stop de slow flashing led
      decoderAddress =  Dcc.readDccAddress(); // reread from eeprom
      // TODO : no software modes for now
      // decoderSoftwareMode = Dcc.getCV(CV_SoftwareMode);
      signal_init();
      // set 2 heads for test
      head_init(0,6); // (head,aspect) 0 = rood, 1=dubbel-geel
      head_init(1,6); // (head,aspect) 0 = rood, 1=dubbel-geel
      decoderState = DECODER_RUNNING;
      #ifdef DEBUG
        Serial.print("my address is : "); Serial.println(Dcc.readDccAddress());
      #endif
      break;
    case DECODER_RUNNING :
    case DECODER_PROGRAM : 
      // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function 
      // for correct library operation
      Dcc.process();
      break;
  }

  // update heads
  for (uint8_t head=0;head<NUM_HEADS;head++) {
    for (uint8_t output=0;output<NUM_OUTPUTS_PER_HEAD;output++) {
      if (heads[head].outputs[output].pin != PIN_UNUSED) {
        if ((heads[head].outputs[output].fCycle < heads[head].outputs[output].fOnCycle) || 
          (heads[head].outputs[output].fCycle >= heads[head].outputs[output].fOffCycle))
            myDigitalWrite(heads[head].outputs[output].pin,HIGH); // led off (flashing off cycle)
        else { // flashing on cycle
          // brightness control
          if (heads[head].bCycle < heads[head].outputs[output].ledOn) {
            myDigitalWrite(heads[head].outputs[output].pin,HIGH); // led off
          }
          else if (heads[head].bCycle < heads[head].outputs[output].ledOff) {
            myDigitalWrite(heads[head].outputs[output].pin,LOW); // led on
          }
          else {
            myDigitalWrite(heads[head].outputs[output].pin,HIGH); // led off
          }
        }
      } // is pin used on head
    } // loop outputs on head
  } // head loop
  
  if (ioUpdate) { // i2c update needed
    ioDigitalWrite(EXPANDER_I2C_ADDRESS,ioByte);
    ioUpdate = false;
  }
  detect_keys();  // check key input
  timer.update();  // run timers
  
} // loop

/**********************************************************************************************/
/* notify functies uit Nmra layer                                                             */
/**********************************************************************************************/
void notifyDccReset(uint8_t hardReset) {
  (void) hardReset;
  decoderState = DECODER_INIT; // the loop() function will get the decoder to a reinitialize
  // avr standard bootloader doesn't like a watchdog reset, so can't do a real reboot
} // notifyDccReset

// This function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read 
void notifyCVAck(void) {
  #ifdef DEBUG
    Serial.println("ack");
  #endif
  digitalWrite(PIN_ACKOUT, HIGH);
  delay(6);  
  digitalWrite(PIN_ACKOUT, LOW);
} // notifyCVAck

// This function is called whenever a normal DCC Turnout Packet is received
void notifyDccAccState(uint16_t dccAddress, uint8_t outputId, bool activate) {
  if (decoderState == DECODER_PROGRAM) {
    // take the decoderAddress & program it as our own
    Dcc.writeDccAddress(dccAddress);
    decoderAddress = dccAddress; // keep copy in RAM
    timer.oscillate(PIN_PROGLED, LED_FAST_FLASH, LOW, 3); // 3 led flashes ter bevestiging van een CV write
    #ifdef DEBUG
      Serial.print("CV1/CV9 rewritten, my address is now :");
      Serial.println(Dcc.readDccAddress());
    #endif
    return;
  }
  // we don't handle basic turnout packets otherwise
} // notifyDccAccState

// This function is called whenever a DCC Signal Aspect Packet is received
void notifyDccSigState(uint16_t dccAddress, uint8_t signalId, uint8_t signalAspect) {
  #ifdef DEBUG
    Serial.print("notifyDccSigState: ");
    Serial.print(dccAddress);
    Serial.print(',');
    Serial.print(signalId);
    Serial.print(',');
    Serial.println(signalAspect);
  #endif

  // not using FLAGS_MY_ADDRESS_ONLY in nmradcc lib
  // so we get all accessory packets here, 
  // and need to filter the dccAddress we are interested in
  if ((dccAddress != decoderAddress) && (dccAddress != 511)) { // 511 = broadcast
    return;
  }

  // particular case of emergency off (RCN-213) -> set all signal heads to aspect=0
  if ((dccAddress == 511) && (signalId == 3) && (signalAspect == 0)) {
    for (uint8_t head=0;head<NUM_HEADS;head++) {
      head_init(head,0);
    }
    timer.oscillate(PIN_PROGLED, LED_FAST_FLASH, LOW, 1); // 1 led flashes ter bevestiging van een turnout commando
    return;
  }
  head_init(signalId,signalAspect);
} // notifyDccSigState

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
