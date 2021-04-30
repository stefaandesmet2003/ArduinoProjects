/*
 * for test : connect signal head to pins 3->8 (8=VCC) AVR
 * for test : connect signal head to pins D3 -> C5 (D3=VCC) STM8
 * bug: CV programming werkt nog niet (DECODER_PROGRAM todo)
 * opgelet CV programming heeft een ack circuit nodig, anders werkt programming niet!
 * 
 */
#include "NmraDcc.h"
#include "SignalDecoderSTM8.h"
#include <EEPROM.h>
#include "Wire_tx.h"
#include "Timer.h"

#define VersionId (0x1)

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
#define NUM_OUTPUTS           19

#define PIN_UNUSED            0xFF

// temp for test
// built-in I/O -> head 0
// deze pins gaan door de pinLUT, dus PIN_0_YELLOW wordt digitalWrite(pinLUT[PIN_0_YELLOW]) = digitalWrite(3)
#define PIN_0_WHITE   0
#define PIN_0_ORANGE  1
#define PIN_0_RED     2
#define PIN_0_GREEN   3
#define PIN_0_YELLOW  4

// expander -> head 1
#define PIN_1_WHITE   10  
#define PIN_1_ORANGE  11   // P0 op de expander volgens pinLUT
#define PIN_1_RED     12
#define PIN_1_GREEN   13
#define PIN_1_YELLOW  14

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
static uint8_t pinLUT[NUM_OUTPUTS] = {PD3,PD2,PD1,PC7,PC6,PC5,PC4,PC3,PD4,PD5,PD6,0xB0,0xB1,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7};

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
int8_t ledTimer; 

/******************************************************************/
/*  SIGNAL DECODER                                                */
/******************************************************************/
// timer interrupt used for 1ms interrupt

INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_BRK_IRQHandler, ITC_IRQ_TIM1_OVF) {
  TIM1->SR1 = 0;

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
} // TIM1_UPD_OVF_TRG_BRK_IRQHandler

// all io expander bits are outputs, so keep it simple, no need for another library
static void ioDigitalWrite(uint8_t i2cAddress, uint8_t ioByte) {
  Wire_beginTransmission(i2cAddress);
  Wire_write(ioByte);
  Wire_endTransmission();
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
  pinMode (PIN_PROGLED, INPUT); // vreemd genoeg is dit niet strikt noodzakelijk

  // het lijkt dat dit nodig is om te kunnen togglen ts GPIO & I2C op PB5
  I2C->CR2 = 0x80;
  delay(1);
  I2C->CR2 = 0x0;
  
  Wire_begin(); // for the i2c expander

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
  
  // configure TIM1 for 1ms interrupt
  // assume F_CPU is always 16MHz
  // careful PSC : different than TIM2!!
  // 16MHz : prescale /128 = 125kHz -> 8us per tick
  // -> gebruik 125x8us als output compare
  TIM1->CR1 = TIM1_CR1_URS; // =4, counter disable & URS=1
  TIM1->PSCRH = 0; // prescale 128 = 16Mhz/128 = 125kHz=8us
  TIM1->PSCRL = 128; // prescale 128 = 16Mhz/128 = 125kHz=8us
  TIM1->ARRH= 0; // write MSB first
  TIM1->ARRL= 124; // overflow after 1ms
  TIM1->CNTRH = 0;
  TIM1->CNTRL = 0;
  TIM1->EGR = 1; // UG, forced update of ARRH & PSCR, not waiting for UEV, URS=1, so no update interrupt after this manual update
  TIM1->SR1 = 0; // clear all int flags
  TIM1->IER = 1; // update interrupt enable, IRQ triggered on UIF flag in SR1
  TIM1->CR1 = 1; // counter enable met URS=0 (CR1=5 met URS=1)  
} // signal_init

// needed for sharing PB5 between i2c & progled
static void signal_end () {
  Wire_end();
  TIM1->CR1 = 0; // stop the signal heads updates
  ioUpdate = false; // just to be sure we are not trying to do i2c from now on
  pinMode (PIN_PROGLED, OUTPUT);
  digitalWrite(PIN_PROGLED,LOW);
}

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
  
  disableInterrupts();
  // defaults voor JMRI infrabel-2013
  if (aspectId == 0) { // stop - rood
    if (headId == 0) heads[headId].outputs[0].pin = PIN_0_RED; // rood
    else if (headId == 1) heads[headId].outputs[0].pin = PIN_1_RED; // rood
    heads[headId].outputs[0].ledOn = 0;
    heads[headId].outputs[0].ledOff = 10;
    heads[headId].outputs[0].fOnCycle = 0;
    heads[headId].outputs[0].fOffCycle = 1;
    heads[headId].outputs[0].fTotalCycles = 1;
    heads[headId].outputs[0].fCycle = 0;
  }
  else if (aspectId == 1) { // approach - dubbel geel
    if (headId == 0) heads[headId].outputs[0].pin = PIN_0_ORANGE; // geel
    else if (headId == 1) heads[headId].outputs[0].pin = PIN_1_ORANGE; // geel
    heads[headId].outputs[0].ledOn = 0;
    heads[headId].outputs[0].ledOff = 10;
    heads[headId].outputs[0].fOnCycle = 0;
    heads[headId].outputs[0].fOffCycle = 1;
    heads[headId].outputs[0].fTotalCycles = 1;
    heads[headId].outputs[0].fCycle = 0;
    if (headId == 0) heads[headId].outputs[1].pin = PIN_0_YELLOW; // geel
    else if (headId == 1) heads[headId].outputs[1].pin = PIN_1_YELLOW; // geel
    heads[headId].outputs[1].ledOn = 0;
    heads[headId].outputs[1].ledOff = 10;
    heads[headId].outputs[1].fOnCycle = 0;
    heads[headId].outputs[1].fOffCycle = 1;
    heads[headId].outputs[1].fTotalCycles = 1;
    heads[headId].outputs[1].fCycle = 0;
  }
  else if (aspectId == 2) { // clear - groen
    if (headId == 0) heads[headId].outputs[0].pin = PIN_0_GREEN; // groen
    else if (headId == 1) heads[headId].outputs[0].pin = PIN_1_GREEN; // groen
    heads[headId].outputs[0].ledOn = 0;
    heads[headId].outputs[0].ledOff = 10;
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
    heads[headId].outputs[0].ledOff = 10;
    heads[headId].outputs[0].fOnCycle = 0;
    heads[headId].outputs[0].fOffCycle = 1;
    heads[headId].outputs[0].fTotalCycles = 1;
    heads[headId].outputs[0].fCycle = 0;
    if (headId == 0) heads[headId].outputs[1].pin = PIN_0_GREEN; // groen
    else if (headId == 1) heads[headId].outputs[1].pin = PIN_1_GREEN; // groen
    heads[headId].outputs[1].ledOn = 0;
    heads[headId].outputs[1].ledOff = 10;
    heads[headId].outputs[1].fOnCycle = 0;
    heads[headId].outputs[1].fOffCycle = 1;
    heads[headId].outputs[1].fTotalCycles = 1;
    heads[headId].outputs[1].fCycle = 0;  
  }
  else if (aspectId == 5) { // slow approach - geel-groen horizontaal
    if (headId == 0) heads[headId].outputs[0].pin = PIN_0_YELLOW; // geel
    else if (headId == 1) heads[headId].outputs[0].pin = PIN_1_YELLOW; // geel
    heads[headId].outputs[0].ledOn = 0;
    heads[headId].outputs[0].ledOff = 10;
    heads[headId].outputs[0].fOnCycle = 0;
    heads[headId].outputs[0].fOffCycle = 1;
    heads[headId].outputs[0].fTotalCycles = 1;
    heads[headId].outputs[0].fCycle = 0;
    if (headId == 0) heads[headId].outputs[1].pin = PIN_0_GREEN; // groen
    else if (headId == 1) heads[headId].outputs[1].pin = PIN_1_GREEN; // groen
    heads[headId].outputs[1].ledOn = 0;
    heads[headId].outputs[1].ledOff = 10;
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
/*  
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
*/  
  else  {// alles uit
  }
  heads[headId].ms = 0;
  heads[headId].bCycle = 0;

  enableInterrupts();
} // head_init

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
    if (decoderState != DECODER_PROGRAM) {
      decoderState = DECODER_PROGRAM;
      signal_end();
      ledTimer = Timer_oscillate(PIN_PROGLED, LED_SLOW_FLASH, HIGH, -1); //forever
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
/*  OTHER LOCAL FUNCTIONS                                         */
/******************************************************************/
// 0 = DONE, 1 = NOT DONE (eeprom not ready)
static uint8_t accessory_FactoryResetCV() {
  uint8_t swMode; // we gebruiken hier een read uit eeprom, niet de global var
  // factory reset de algemene CV's (die niet afhangen van de software mode)
  for (int i=0; i < sizeof(FactoryDefaultCVs)/sizeof(CVPair); i++) {
    DCC_setCV(FactoryDefaultCVs[i].CV, FactoryDefaultCVs[i].Value);
  }

  swMode = DCC_getCV(CV_SoftwareMode);
  switch(swMode)
  {
    // TODO for signal decoder
  }
  return 0;
} // accessory_FactoryResetCV

void setup() {
  decoderState = DECODER_INIT;
  #ifdef DEBUG
    Serial_begin(115200);
    Serial_println_s("SignalDecoder STM8!");
  #endif

  // Configure the DCC CV Programing ACK pin for an output
  pinMode(PIN_ACKOUT, OUTPUT);
  digitalWrite(PIN_ACKOUT, LOW);

   // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  // + enable the DCC Receiver
  DCC_init(FLAGS_DCC_ACCESSORY_DECODER, 0, true);

  // progled, progkey
  pinMode (PIN_PROGKEY, INPUT_PULLUP);
  //pinMode (PIN_PROGLED, OUTPUT);
  //digitalWrite(PIN_PROGLED, HIGH); //LED uit

  if (DCC_getCV(CV_MANUFACTURER_ID) != MAN_ID_DIY) {
    decoderState = DECODER_FACTORY_RESET;
  }
  else {
    decoderState = DECODER_INIT;
  }

  init_keys (); // init key handling
  Timer_init(); // init timer lib
} // setup

void loop() {
  switch (decoderState) {
    case DECODER_FACTORY_RESET : 
      if (accessory_FactoryResetCV () == 0) {
        // factory reset gelukt --> DECODER_INIT
        decoderState = DECODER_INIT;
      }
      break;
    case DECODER_INIT :
      Timer_stop (ledTimer); //stop de slow flashing led
      decoderAddress =  DCC_readDccAddress(); // reread from eeprom
      // TODO : no software modes for now
      // decoderSoftwareMode = Dcc.getCV(CV_SoftwareMode);
      signal_init();
      // set 2 heads for test
      head_init(0,6); // (head,aspect) 0 = rood, 1=dubbel-geel
      head_init(1,6); // (head,aspect) 0 = rood, 1=dubbel-geel
      decoderState = DECODER_RUNNING;
      #ifdef DEBUG
        Serial_print_s("my address is : "); 
        Serial_println_u(Dcc_readDccAddress());
      #endif
      break;
    case DECODER_RUNNING :
    case DECODER_PROGRAM : 
      // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function 
      // for correct library operation
      DCC_process();
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
  Timer_update();  // run timers  
} // loop

/**********************************************************************************************/
/* notify functies uit Nmra layer */
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
    Timer_oscillate(PIN_PROGLED, LED_FAST_FLASH, LOW, 3); // 3 led flashes ter bevestiging van een CV write
    #ifdef DEBUG
      Serial_print_s("CV1/CV9 rewritten, my address is now :");
      Serial_println_u(DCC_readDccAddress());
    #endif
    return;
  }
  // we don't handle basic turnout packets otherwise
} // notifyDccAccState

// This function is called whenever a DCC Signal Aspect Packet is received
void notifyDccSigState(uint16_t dccAddress, uint8_t signalId, uint8_t signalAspect)
{
  #ifdef DEBUG
    Serial_print_s("notifyDccSigState: ");
    Serial_print_u(dccAddress);
    Serial_print_s(",");
    Serial_print_u(signalId);
    Serial_print_s(",");
    Serial_println_u(signalAspect);
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
    // dit gaat vermoedelijk problemen geven omdat PROGLED==SDA!!
    //Timer_oscillate(PIN_PROGLED, LED_FAST_FLASH, LOW, 1); // 1 led flashes ter bevestiging van een commando
    return;
  }
  head_init(signalId,signalAspect);
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
