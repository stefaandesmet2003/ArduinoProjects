/*
 * TODO : de xpc mag in z'n slot maar 1 msg sturen!
 * de TxBuffer is in principe groot genoeg voor meerdere msgs
 * de xpc kan dus meerdere msgs pushen zonder de tx af te wachten
 * voorlopig in xpc 1 msg tegelijk en wachten tot tx complete
 * TODO : eventueel auto-retransmit als er een data error terugkomt van de command station
 * die retransmit wordt wel moeilijk als er meerdere msgs in de tx buffer zitten
 * en xpc zal dan ook niet weten welke msg een error heeft gegeven
 * dus : misschien beter max 1 msg tegelijk in TxBuffer ? 
 * 
*/

/* TODO : the sreg code is commented out 
 * -> need to test or remove
 * -> for now code is not needed because aligned with xpc.c
 * (xpc.c calls send_byte under disabled int)
*/

#include "Arduino.h"
#include "rs485c.h"

#define RS485Transmit    HIGH
#define RS485Receive     LOW

// move some xpnet logic here to handle fast interrupt response
#define CALL_TYPE         0x60
#define CALL_TYPE_INQUIRY 0x40

#define XPNET_BROADCAST_ADDRESS (0)

// the slave address we will be listening on
static uint8_t X_slaveAddress = 0; // 1..31 xpnet slave, 0 = broadcast
static uint8_t rs485DirectionPin;

static uint8_t sreg; // TODO : need a better solution


// FIFO objects for RX and TX
// max. Size: 255
// careful types! client TX is 8-bit (well, bit8=0 always), RX=9-bit (need bit8!)
#define X_RxBuffer_Size  32           // >= 16
#define X_TxBuffer_Size  32
static uint16_t X_RxBuffer[X_RxBuffer_Size]; // client tx 8bit, rx 9bit
static uint8_t X_TxBuffer[X_TxBuffer_Size];  // client tx 8bit, rx 9bit

static uint8_t X_rx_read_ptr = 0;            // point to next read
static volatile uint8_t X_rx_write_ptr = 0;  // point to next write
static volatile uint8_t X_tx_read_ptr = 0;
static uint8_t X_tx_write_ptr = 0;
static uint8_t X_tx_fill = 0;
static volatile uint8_t X_slotAddress;       // active slot on the xpnet, discard all data if X_slotAddress != X_slaveAddress or broadcast

static inline void set_XP_to_receive(void)
{
  digitalWrite(rs485DirectionPin,RS485Receive);
}

static inline void set_XP_to_transmit(void)
{
  digitalWrite(rs485DirectionPin,RS485Transmit);
}

static void XP_flush_rx(void) // Flush Receive-Buffer
{
  //uint8_t sreg;
  //sreg = SREG;
  //__asm__("push cc\npop _sreg");

  //disableInterrupts();
  do {
    UART1->DR;
  }
  while (UART1->SR & UART1_SR_RXNE);

  // toggle receiver enable off & back on, nodig voor STM8?
  UART1->CR2 &= ~UART1_CR2_REN;
  UART1->CR2 |= UART1_CR2_REN;
  X_rx_read_ptr = 0;      
  X_rx_write_ptr = 0;

  // SREG = sreg; // this will renable global ints (sei) if they were enabled before
  //__asm__("push _sreg\npop cc"); // TODO test -> volgens refman §6.2, table9
  //enableInterrupts();
  
} // XP_flush_rx

void init_rs485(uint8_t enablePin)
{
  uint16_t ubrr;
  //uint8_t sreg;

  rs485DirectionPin = enablePin;
  digitalWrite(rs485DirectionPin,RS485Receive); // low = default anyway
  pinMode (enablePin, OUTPUT);

  //sreg = SREG;
  //__asm__("push cc\npop _sreg");

  UART1->CR2 = 0; // stop everything
  UART1->SR = 0; // clear TX/RXNE

  ubrr = (uint16_t) (F_CPU/62500L);
  UART1->BRR2 = (uint8_t) ((ubrr & 0x0F) + ((ubrr & 0xF000) >> 8)); // 4LSB & 4MSB together, see refman
  UART1->BRR1 = (uint8_t) ((ubrr & 0x0FF0) >> 4); // bits 4..11 here

  UART1->CR1 = UART1_CR1_M;         // 9-bit data mode, other bits 0 = ok default
                                    // UART1_CR1_T8 = 0, slave always sends 0 for bit8
  UART1->CR3 = 0;                    // defaults ok (1stop bit)

  // enable receiver & transmitter without interrupts
  UART1->CR2 = UART1_CR2_TEN | UART1_CR2_REN;
  delay(1); // needed to avoid TXE/TC flags being set, without ever sending data! with this delay the next statement works correctly.
  UART1->SR = 0; // and now clearing TXE/TC flags works ok
  XP_flush_rx();
  // enable interrupts (the TIEN interrupt is activated only when we are sending data)
  UART1->CR2 |= UART1_CR2_TCIEN | UART1_CR2_RIEN;

  // FIFOs fur Ein- und Ausgabe initialisieren
  X_tx_read_ptr = 0;
  X_tx_write_ptr = 0;
  X_tx_fill = 0;

  // SREG = sreg; // this will renable global ints (sei) if they were enabled before
  //__asm__("push _sreg\npop cc"); // TODO test -> volgens refman §6.2, table9
} // init_rs485

//---------------------------------------------------------------------------
// Halbduplex: auf TXC Complete schalten wir sofort die Richtung um
// Das als NAKED um es wirklich schnell zu bekommen.
// Wenn Optimize wegfallt, dann darf NAKED nicht mehr benutzt werden - Flags!
// Vermutlich wird alles andere als ISR_NOBLOCK laufen mussen!
//

// this will override the sduino default ISRs
INTERRUPT_HANDLER(UART1_TX_IRQHandler, ITC_IRQ_UART1_TX) {
  // 1 isr for TXE & TC
  if (UART1->SR & UART1_SR_TC) {
    set_XP_to_receive();
    UART1->SR &= ~UART1_SR_TC; // reset TC flag
  }

  // We transmit 9 bits --> sds : client tx altijd 9de bit = 0 (9de bit wordt in de RX introutine nooit gelezen door de master)
  // transmit register empty
  if (UART1->SR & UART1_SR_TXE) {
    if (X_tx_read_ptr != X_tx_write_ptr)
    {
      // moeten we hier telkens T8 op 0 zetten in CR1?
      UART1->DR = X_TxBuffer[X_tx_read_ptr++];
      if (X_tx_read_ptr == X_TxBuffer_Size) X_tx_read_ptr=0;
      X_tx_fill--;
    }
    else
      UART1->CR2 &= ~UART1_CR2_TIEN;  // disable further TxINT
  }
} // UART-TX ISR

// we filteren hier al op call bytes voor ons, en starten TX bij inquiry call
// zodat we enkel transmitten in ons timeslot
// RxBuffer is 16-bit (bit8 also stored, so xpc can further handle the different call types)
INTERRUPT_HANDLER(UART1_RX_IRQHandler, ITC_IRQ_UART1_RX) {
  uint16_t aData = 0;
  if (UART1->SR & UART1_SR_FE) { // frame error
    UART1->DR; // read data anyway to stop the int
    return;
  }

  if (UART1->SR & UART1_SR_OR){ // data overrun
    // sds : we zijn data kwijt, xpc zal het wel oplossen, er zal een timeout of xor error volgen
  }

  if (UART1->CR1 & UART1_CR1_R8) { // bit 8 is 1
    aData = 0x100;
  }
  aData |= UART1->DR;

  // NOTE : this code doesn't detect that another device is erroneously sending data on our slot
  // for now these bytes will be stored in X_RxBuffer and further discarded by xpc (data without preceding call byte in XPC_WAIT_FOR_CALL)
  if (aData & 0x100) { // a  call-byte
    X_slotAddress = aData & 0x1F; // store the active slot address, subsequent data bytes are then associated with this slot
    if ((X_slotAddress == X_slaveAddress) && 
        ((aData & CALL_TYPE) == CALL_TYPE_INQUIRY)) { // a normal inquiry for us
      if (X_tx_write_ptr != X_tx_read_ptr) { /// we have something to send
        set_XP_to_transmit();         // start data transmission!
        UART1->SR &= ~UART1_SR_TC;    // clear any pending tx complete flag
        UART1->CR2 |= UART1_CR2_TIEN; // enable TxINT
        // allicht overbodig, want die stonden al aan, en worden niet uitgezet
        UART1->CR2 | UART1_CR2_TCIEN; // enable TX complete int
        //UART1->CR2 | UART1_CR2_TEN;   // enable transmitter
      }
      // TODO : we could return here and not store the normal inquiry in RxBuffer, but 
      // we need to track connection errors, and that's done in xpc for now
      // return; 
    }
  }

  // any data on broadcast slot or our slot are put in RxBuffer
  if ((X_slotAddress == XPNET_BROADCAST_ADDRESS) || (X_slotAddress == X_slaveAddress))
    X_RxBuffer[X_rx_write_ptr++] = aData;

  if (X_rx_write_ptr == X_RxBuffer_Size) X_rx_write_ptr=0; // wrap-around the buffer pointer
}  // UART-RX ISR

//=============================================================================
// Upstream Interface
//-----------------------------------------------------------------------------

void XP_setAddress (uint8_t slaveAddress) {
  X_slaveAddress = slaveAddress;
}

// TX:
bool XP_tx_ready (void) {
  if (X_tx_fill < (X_TxBuffer_Size-18)) { // keep space for one complete message (16)
    return(true);
  }
  else return(false);
} // XP_tx_ready

// true if TxBuffer empty & all bytes sent
bool XP_tx_empty(void) {
  // not enough to just look at driver bit, need to check the TxBuffer pointers too
  // we want to prevent multiple xpc_SendMessage() between the last completed transmission and the next inquiry call byte
  return (X_tx_write_ptr == X_tx_read_ptr) && (digitalRead(rs485DirectionPin) != RS485Transmit);

} // XP_tx_empty

void XP_tx_clear (void) {
  //uint8_t sreg;
  //sreg = SREG;
  //__asm__("push cc\npop _sreg");

  disableInterrupts();
  X_tx_write_ptr = X_tx_read_ptr;
  X_tx_fill = 0;

  // SREG = sreg; // this will renable global ints (sei) if they were enabled before
  //__asm__("push _sreg\npop cc"); // TODO test -> volgens refman §6.2, table9
  enableInterrupts();
  
}

// ret 1 if full
// This goes with fifo (up to 32), bit 8 is 0.
bool XP_send_byte (const uint8_t c)
{
  //uint8_t sreg;

  X_TxBuffer[X_tx_write_ptr] = c;

  X_tx_write_ptr++;
  if (X_tx_write_ptr == X_TxBuffer_Size) X_tx_write_ptr=0;

  //sreg = SREG;
  //__asm__("push cc\npop _sreg");
  X_tx_fill++;
  // SREG = sreg; // this will renable global ints (sei) if they were enabled before
  //__asm__("push _sreg\npop cc"); // TODO test -> volgens refman §6.2, table9

  if (X_tx_fill > (X_TxBuffer_Size-18)) return(true);
  return(false);
} // XP_send_byte

//------------------------------------------------------------------------------
// RX:

// SDS TODO 2021 : rename isAvailable() of zoiets
bool XP_rx_ready (void)
{
  if (X_rx_read_ptr != X_rx_write_ptr)
    return(true);     // there is something
  else return(false);  
} // XP_rx_ready


//-------------------------------------------------------------------
// XP_rx_peek : lees een 9-bit char uit de buffer, maar laat het in de buffer staan (!= XP_rx_read)
// there is no check whether a char is ready, this must be
// done before calling with a call to rx_fifo_ready();

//-------------------------------------------------------------------
// rx_fifo_read gets one char from the input fifo
//
// there is no check whether a char is ready, this must be
// done before calling with a call to rx_fifo_ready();
uint16_t XP_rx_read (void)
{
  unsigned int retval;

  retval = X_RxBuffer[X_rx_read_ptr]; //sds 9 bit RX
  X_rx_read_ptr++;
  if (X_rx_read_ptr == X_RxBuffer_Size) X_rx_read_ptr=0;

  return(retval);
} // XP_rx_read
