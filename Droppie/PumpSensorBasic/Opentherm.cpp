#include "Arduino.h"
#include "Opentherm.h"

#define STATE_RX_WAIT_STARTBIT    1
#define STATE_RX_DO_FRAME         2
#define STATE_RX_WAIT_STOPBIT     3
#define STATE_RX_FRAME_ERROR      4
#define STATE_TX_DO_STARTBIT      10
#define STATE_TX_DO_FRAME         11
#define STATE_TX_DO_STOPBIT       12
#define STATE_TX_DO_IDLE          13

struct otState_t {
  uint32_t lastEdgeMicros;
  uint8_t state;
  uint32_t frameData;
  uint8_t frameBitIdx;
};

static volatile otState_t otState = {0};
static uint32_t frameDataBuffer; // a 1 item frame buffer for now
static bool frameDataAvailable = false;

/**********************************************************/
/* helper functions timeout timer                         */
/**********************************************************/
static void timeoutTimerStart() {
  // configure timer 2 for 1.5ms interrupt :
  // prescale /128 = 125kHz -> 8us per tick x256 = 2048us overflow
  // -> gebruik 188x8us als output compare
  TCCR2A = (1 << WGM21);  // CTC mode of operation (tot OCR2A tellen)
  OCR2A = 187; // 188x 8us per output compare
  TIFR2 = 0x7; // clear pending interrupts
  TIMSK2 = (1 << OCIE2A); // enable OC channel A interrupt
  TCNT2 = 0;
#if (F_CPU==8000000L)
  TCCR2B = (1 << CS22); // prescaler /64 voor 8MHz pro Mini
#else
  TCCR2B = (1 << CS22) | (1 << CS20);   // prescaler /128 (opgelet : niet dezelfde bits voor TIMER0!!! table 19-10 vs. 22-10)
#endif  

} // timeoutTimerInit

static void timeoutTimerStop() {
  // disable clock source; this effectively stops the counter
  TCCR2B = 0;
}
static void timeoutTimerReset() {
  TCNT2 = 0;
}

static void txTimerStart() {
  // configure timer 2 for 0.5ms interrupt :
  // prescale /64 = 250kHz -> 4us per tick
  // -> use 125x4us als output compare
  TCCR2A = (1 << WGM21);  // CTC mode of operation (tot OCR2A tellen)
  OCR2A = 124; // 125x 4us per output compare
  TIFR2 = 0x7; // clear pending interrupts
  TIMSK2 = (1 << OCIE2A); // enable OC channel A interrupt
  TCNT2 = 0;
#if (F_CPU==8000000L) 
  TCCR2B = (1 << CS20) | (1 << CS21); // prescaler /32 voor 8MHz pro Mini 
#else 
  TCCR2B = (1 << CS22); // prescaler /64 voor 16MHz (opgelet : niet dezelfde bits voor TIMER0!!! table 19-10 vs. 22-10)
#endif
} // txTimerStart

static void txTimerStop() {
  // disable clock source; this effectively stops the counter
  TCCR2B = 0;
} // txTimerStop

/**********************************************************/
/* helper functions TX bits                               */
/**********************************************************/


// busy wait version of manchester coding of 1 bit @ 1000Hz
static void sendBit(bool high) {
  OT_setTxState(high);
  delayMicroseconds(500);
  OT_setTxState(!high);
  delayMicroseconds(500);
}

/**********************************************************/
/* ISR                                                    */
/**********************************************************/
// timer interrupt used for 1.5ms timeout & 0.5ms edge timing for bitbanging TX
// so it's either TX or RX in this driver
ISR (TIMER2_COMPA_vect) {
  if (otState.state < STATE_TX_DO_STARTBIT) { // timer configured for Rx timeouts
    cli();
    otState.state = STATE_RX_FRAME_ERROR;
    sei();
  }
  else { // timer for Tx bit banging
    // for some strange reason a switch statement didn't work here
    // maybe because state is volatile?? couldn't get it to work -> if-then-else
    if (otState.state == STATE_TX_DO_STARTBIT) {
      OT_setTxState(!(otState.lastEdgeMicros & 0x1));
      if (otState.lastEdgeMicros & 0x1) {
        otState.state = STATE_TX_DO_FRAME;
        otState.frameBitIdx = 0;
      }
    }
    else if (otState.state == STATE_TX_DO_FRAME) {
      uint8_t txBit = (otState.frameData >> (31-otState.frameBitIdx)) & 0x1;
      OT_setTxState(!(otState.lastEdgeMicros & 0x1) != (!txBit));
      if (otState.lastEdgeMicros & 0x1) {
        otState.frameBitIdx += 1;
      }
      if (otState.frameBitIdx >= 32) {
        otState.state = STATE_TX_DO_STOPBIT;
      }
    }
    else if (otState.state == STATE_TX_DO_STOPBIT) {
      OT_setTxState(!(otState.lastEdgeMicros & 0x1));
      if (otState.lastEdgeMicros & 0x1) {
        otState.state = STATE_TX_DO_IDLE;
      }
    }
    else if (otState.state == STATE_TX_DO_IDLE) {
      OT_setTxState(false);
      otState.state = STATE_RX_WAIT_STARTBIT;
      txTimerStop();
    }
    otState.lastEdgeMicros += 1; // abusing this field for tracking pos/neg edges
  }
} // TIMER2_COMPA_vect

#ifdef USE_ANALOG_COMPARE
ISR (ANALOG_COMP_vect) {
  uint8_t rxBit = (ACSR >> 5) & 0x1;

#else
ISR (PCINT2_vect) {
  uint8_t rxBit = digitalRead(T_RX_PIN) & 0x1;
#endif
  
  uint32_t now = micros();
  switch (otState.state) {
    case STATE_RX_WAIT_STARTBIT :
      if (rxBit != RX_PIN_LEVEL_ACTIVE) { // startbit received
        timeoutTimerStart();
        otState.state = STATE_RX_DO_FRAME;
        otState.lastEdgeMicros = now;
        otState.frameData = 0;
        otState.frameBitIdx = 0;
      }
      break;
    case STATE_RX_DO_FRAME : 
      if (now - otState.lastEdgeMicros > 800) {
        otState.frameData = (otState.frameData << 1) | (rxBit^RX_PIN_LEVEL_ACTIVE & 0x1); // idle->active = O-bit, active->idle = 1-bit
        otState.frameBitIdx ++;
        otState.lastEdgeMicros = now;
        timeoutTimerReset();
      }
      if (otState.frameBitIdx == 32) {
        otState.state = STATE_RX_WAIT_STOPBIT;
      }
      break;
    case STATE_RX_WAIT_STOPBIT : 
      if ((now - otState.lastEdgeMicros > 800) && (rxBit != RX_PIN_LEVEL_ACTIVE)){ // valid stop bit received
        timeoutTimerStop();
        frameDataAvailable = true;
        frameDataBuffer = otState.frameData;
        otState.state = STATE_RX_WAIT_STARTBIT;
      }
      break;
    case STATE_RX_FRAME_ERROR :
      timeoutTimerStop();
      otState.state = STATE_RX_WAIT_STARTBIT;
      break;
  }
} // ISR



/**********************************************************/
/* public interface                                       */
/**********************************************************/

void OT_init(){
  pinMode(T_TX_PIN, OUTPUT);

#ifdef USE_ANALOG_COMPARE
  ADCSRB = 0;           // (Disable) ACME: Analog Comparator Multiplexer Enable
  ACSR =  bit (ACI)     // (Clear) Analog Comparator Interrupt Flag
        | bit (ACBG)    // select bandgap reference ipv. AIN0
        | bit (ACIE);    // Analog Comparator Interrupt Enable
        // geen ACIS bits -> interrupt on output toggle (= falling+rising)
        //| bit (ACIS1);  // ACIS1, ACIS0: Analog Comparator Interrupt Mode Select (trigger on falling edge)
  
  DIDR1 = bit(AIN1D); // digital input disable -> omdat we die nu niet nodig hebben op AIN1

#else // pin change interrupt
  pinMode(T_RX_PIN, INPUT);
  // manual configure pin change interrupt for T_RX_PIN = D7
  PCICR = 1 << PCIE2; // enable pin change interrupt 2
  PCIFR = 0x7; // clear pending interrupts
  PCMSK2 = 1 << PCINT23; // enable pin change interrupt on pin PCINT23 = PD7
  PCMSK1 = 0;
  PCMSK0 = 0;
#endif

  otState.state = STATE_RX_WAIT_STARTBIT;
  OT_setTxState(false); // send idle level on TX  
}

bool OT_available() {
  return frameDataAvailable;
}

uint32_t OT_readFrame() {
  frameDataAvailable = false;
  return frameDataBuffer;
}

// busy waiting version
void OT_writeFrame(uint32_t frameData) {
  sendBit(HIGH); //start bit
  for (int i = 31; i >= 0; i--) {
    sendBit(bitRead(frameData, i));
  }
  sendBit(HIGH); //stop bit  
  OT_setTxState(false); // idle state
}

// interrupt version
void OT_writeFrameInt(uint32_t frameData) {
  otState.frameData = frameData;
  otState.state = STATE_TX_DO_STARTBIT;
  otState.lastEdgeMicros = 0; // used to identify before/after edge in each bit
  txTimerStart();
}

// set active or idle pin level
void OT_setTxState(bool active) {
  digitalWrite(T_TX_PIN,active == TX_PIN_LEVEL_ACTIVE); // beetje vies, gaat ervan uit dat true==1==HIGH
}
