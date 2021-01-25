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

typedef struct  {
  uint32_t lastEdgeMicros;
  uint8_t state;
  uint32_t frameData;
  uint8_t frameBitIdx;
} otState_t;

static volatile otState_t otState;
static uint32_t frameDataBuffer; // a 1 item frame buffer for now
static bool frameDataAvailable = false;

/**********************************************************/
/* helper functions timeout timer                         */
/**********************************************************/
static void timeoutTimerStart() {
  // configure timer 2 for 1.5ms interrupt :
  // prescale /128 = 125kHz -> 8us per tick
  // -> gebruik 188x8us als output compare
  TIM2->CR1 = TIM2_CR1_URS; // =4, counter disable & URS=1
  TIM2->PSCR = 7; // prescale 2^7 = 128
  TIM2->ARRH = 0; // write MSB first
  TIM2->ARRL = 187; 
  TIM2->CNTRH = 0;
  TIM2->CNTRL = 0;
  TIM2->EGR = 1; // UG, forced update of ARRH & PSCR, not waiting for UEV, URS=1, so no update interrupt after this manual update
  TIM2->SR1 = 0; // clear interrupts, just to be sure there are no outstanding ints
  TIM2->IER = TIM2_IER_UIE; // =1, update interrupt enable
  TIM2->CR1 |= TIM2_CR1_CEN; // counter enable (we could disable URS again, don't need it anymore)

} // timeoutTimerInit

static void timeoutTimerStop() {
  TIM2->CR1 = 0; // disable counter
}
static void timeoutTimerReset() {
  TIM2->CNTRH = 0;
  TIM2->CNTRL = 0;
}

static void txTimerStart() {
  // configure timer 2 for 0.5ms interrupt :
  // prescale /64 = 250kHz -> 4us per tick
  // -> use 125x4us als output compare
  TIM2->CR1 = TIM2_CR1_URS; // =4, counter disable & URS=1
  TIM2->PSCR = 6; // prescale 2^6 = 64
  TIM2->ARRH = 0; // write MSB first
  TIM2->ARRL= 124;
  TIM2->CNTRH = 0;
  TIM2->CNTRL = 0;
  TIM2->EGR = 1; // UG, forced update of ARRH & PSCR, not waiting for UEV, URS=1, so no update interrupt after this manual update
  TIM2->SR1 = 0; // clear interrupts, just to be sure there are no outstanding ints
  TIM2->IER = TIM2_IER_UIE; // =1, update interrupt enable
  TIM2->CR1 |= TIM2_CR1_CEN; // counter enable (we could disable URS again, don't need it anymore)
} // txTimerStart

static void txTimerStop() {
  TIM2->CR1 = 0; // disable counter
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
INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, ITC_IRQ_TIM2_OVF) {
  TIM2->SR1 = 0;

  if (otState.state < STATE_TX_DO_STARTBIT) { // timer configured for Rx timeouts
    disableInterrupts();
    otState.state = STATE_RX_FRAME_ERROR;
    enableInterrupts();
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
} // TIM2_UPD_OVF_BRK_IRQHandler

static void portd_irq (void) {
  uint8_t rxBit;
  rxBit = digitalRead(T_RX_PIN);
  
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
  pinMode(T_RX_PIN, INPUT);
  // manual configure pin change interrupt for T_RX_PIN = PD2
  attachInterrupt(3,portd_irq, 0); // mode is not implemented in sduino, but we do the config manually

  disableInterrupts(); // EXTI->CR1 kan je maar schrijven onder disabled interrupts (CCR=3); manual nog eens goed lezen, maar zo lijkt het wel te werken
  EXTI->CR1 = 0xC0; // set falling+rising interrupt for all pins on port D
  GPIOD->CR2 = 0x04; // enable ext interrupt on pin PD2
  // there seems to be no way to clear pending interrupts here
  enableInterrupts();

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
  digitalWrite(T_TX_PIN,active == TX_PIN_LEVEL_ACTIVE); // beetje vies, gaat ervan uit dat true==1==HIGH (Arduino.h)
}
