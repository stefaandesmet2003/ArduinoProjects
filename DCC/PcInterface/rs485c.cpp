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


#include "Arduino.h"
#include "rs485c.h"

// move some xpnet logic here to handle fast interrupt response
#define CALL_TYPE         0x60
#define CALL_TYPE_INQUIRY 0x40

#define XPNET_BROADCAST_ADDRESS (0)

// the slave address we will be listening on
static uint8_t X_slaveAddress = 0; // 1..31 xpnet slave, 0 = broadcast

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
  HARDWARE_SET_XP_RECEIVE;
}

static inline void set_XP_to_transmit(void)
{
  HARDWARE_SET_XP_TRANSMIT;
}

static void XP_flush_rx(void) // Flush Receive-Buffer
{
  uint8_t sreg = SREG;
  cli();
  do {
    UDR0;
  }
  while (UCSR0A & (1 << RXC0));

  UCSR0B &= ~(1 << RXEN0);
  UCSR0B |= (1 << RXEN0);
  X_rx_read_ptr = 0;      
  X_rx_write_ptr = 0;
  SREG = sreg; // this will renable global ints (sei) if they were enabled before
} // XP_flush_rx

void init_rs485(void)
{
  uint16_t ubrr;
  uint8_t sreg = SREG;
  
  pinMode (RS485_DERE, OUTPUT); // voor zover dit in ino-setup nog niet is gebeurd..
  cli();
  UCSR0B = 0;                  // stop everything

  // we calculate 100* and add 50 to get correct integer cast;
  // normal speed - pre divider 16 (high speed mode - pre divider 8)
  // ubrr = (uint16_t) ((uint32_t) F_CPU/(16*62500L) - 1);    
  ubrr = (uint16_t) ((uint32_t) (F_CPU/(16*625L) - 100L + 50L) / 100);
  UBRR0H = (uint8_t) (ubrr>>8);
  UBRR0L = (uint8_t) (ubrr);
  UCSR0A = (1 << RXC0) | (1 << TXC0) | (0 << U2X0);  // U2X1: Speed Mode
  
  // FIFOs fur Ein- und Ausgabe initialisieren
  X_tx_read_ptr = 0;
  X_tx_write_ptr = 0;
  X_tx_fill = 0;

  // UART Receiver und Transmitter anschalten, Receive-Interrupt aktivieren
  // Data mode 9N1, asynchron
  UCSR0B = (1 << RXCIE0)          // enable receive Int
          | (1 << TXCIE0)         // enable TX complete Interrupt
          | (0 << UDRIE0)         // deze int wordt maar aangezet als we effectief data willen sturen
          | (1 << RXEN0)          // enable receiver
          | (1 << TXEN0)          // enable transmitter
          | (1 << UCSZ02)         // set frame length to 9bit;
          | (0 << RXB80)	
          | (0 << TXB80);          // niet 1 zoals master, we sturen enkel dataframes als xp-slave

  UCSR0C = (0 << UMSEL01)         // 00 = asyn mode
         | (0 << UMSEL00)         // 
         | (0 << UPM01)           // 00 = parity disabled
         | (0 << UPM00)           // 
         | (0 << USBS0)           // 0 = tx with 1 stop bit
         | (1 << UCSZ01)          // 11 = 8 or 9 bits
         | (1 << UCSZ00)
         | (0 << UCPOL0);

  XP_flush_rx();
  UCSR0A |= (1 << TXC0);         // clear tx complete flag
  SREG = sreg; // this will renable global ints (sei) if they were enabled before
} // init_rs485

//---------------------------------------------------------------------------
// Halbduplex: auf TXC Complete schalten wir sofort die Richtung um
// Das als NAKED um es wirklich schnell zu bekommen.
// Wenn Optimize wegfallt, dann darf NAKED nicht mehr benutzt werden - Flags!
// Vermutlich wird alles andere als ISR_NOBLOCK laufen mussen!
//

ISR(USART_TX_vect)
{    
  set_XP_to_receive();
} // USART_TX_vect

//----------------------------------------------------------------------------
// Ein Zeichen aus der Ausgabe-FIFO lesen und ausgeben
// Ist das Zeichen fertig ausgegeben, wird ein neuer SIG_UART_DATA-IRQ getriggert
// Ist das FIFO leer, deaktiviert die ISR ihren eigenen IRQ.

// We transmit 9 bits --> sds : client tx altijd 9de bit = 0 (9de bit wordt in de RX introutine nooit gelezen door de master)
ISR(USART_UDRE_vect) 
{
  if (X_tx_read_ptr != X_tx_write_ptr)
  {
    // sds : niet gecomment in rs232.cpp, 
    // allicht omdat we hier de TXC int systematisch gebruiken, 
    // en dan wordt TXC flag automatisch gereset
    // UCSR0A |= (1 << TXC0);   
    UCSR0B &= ~(1<<TXB80);              // clear bit 8, moet dat eigenlijk bij elk frame?
    UDR0 = X_TxBuffer[X_tx_read_ptr++];
    if (X_tx_read_ptr == X_TxBuffer_Size) X_tx_read_ptr=0;
    X_tx_fill--;
  }
  else
    UCSR0B &= ~(1 << UDRIE0);           // disable further TxINT
} // USART_UDRE_vect

// sds : code anders dan RS485 master!! 
// we filteren hier al op call bytes voor ons, en starten TX bij inquiry call
// zodat we enkel transmitten in ons timeslot
// RxBuffer is 16-bit (bit8 also stored, so xpc can further handle the different call types)
ISR(USART_RX_vect)
{
  uint16_t aData = 0;
  if (UCSR0A & (1<< FE0)) { 
    // Frame Error
    UDR0; // read data anyway to stop the int
    return;
	}

  if (UCSR0A & (1<< DOR0)) { // DATA Overrun
    // sds : we zijn data kwijt, xpc zal het wel oplossen, er zal een timeout of xor error volgen
  }

  if (UCSR0B & (1<< RXB80)) { // bit 8 is 1
    aData = 0x100;
  }
  aData |= UDR0;

  // TODO : test if this can be more easily handled by using the MPCM bit in the usart
  // NOTE : this code doesn't detect that another device is erroneously sending data on our slot
  // for now these bytes will be stored in X_RxBuffer and further discarded by xpc (data without preceding call byte in XPC_WAIT_FOR_CALL)
  if (aData & 0x100) { // a  call-byte
    X_slotAddress = aData & 0x1F; // store the active slot address, subsequent data bytes are then associated with this slot
    if ((X_slotAddress == X_slaveAddress) && 
        ((aData & CALL_TYPE) == CALL_TYPE_INQUIRY)) { // a normal inquiry for us
      if (X_tx_write_ptr != X_tx_read_ptr) { /// we have something to send
        set_XP_to_transmit();       // start data transmission!
        UCSR0A |= (1 << TXC0);      // clear any pending tx complete flag
        UCSR0B |= (1 << TXEN0);     // enable TX
        UCSR0B |= (1 << UDRIE0);    // enable TxINT
        UCSR0B |= (1 << TXCIE0);    // enable TX complete --> sds : this will feed ISR(USART_TX_vect)
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
} // USART_RX_vect

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
  return (X_tx_write_ptr == X_tx_read_ptr) && (digitalRead(RS485_DERE) != RS485Transmit);

} // XP_tx_empty

void XP_tx_clear (void) {
  uint8_t sreg = SREG;
  cli();
  X_tx_write_ptr = X_tx_read_ptr;
  X_tx_fill = 0;
  SREG = sreg; // this will renable global ints (sei) if they were enabled before
}

// ret 1 if full
// This goes with fifo (up to 32), bit 8 is 0.
bool XP_send_byte (const uint8_t c)
{
  uint8_t sreg;

  X_TxBuffer[X_tx_write_ptr] = c;

  X_tx_write_ptr++;
  if (X_tx_write_ptr == X_TxBuffer_Size) X_tx_write_ptr=0;

  sreg = SREG;
  cli();
  X_tx_fill++;
  SREG = sreg; // this will renable global ints (sei) if they were enabled before

  // niet hier maar enkel als ons slot geopend is!
  /*
  set_XP_to_transmit();
  UCSR0A |= (1 << TXC0);      // clear any pending tx complete flag
  UCSR0B |= (1 << TXEN0);     // enable TX
  UCSR0B |= (1 << UDRIE0);    // enable TxINT
  UCSR0B |= (1 << TXCIE0);    // enable TX complete --> sds : dit gaat de ISR(USART_TX_vect) voeden
  */

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
unsigned int XP_rx_peek (void)
{
  return (X_RxBuffer[X_rx_read_ptr]); //sds 9 bit RX
} // XP_rx_peek

//-------------------------------------------------------------------
// rx_fifo_read gets one char from the input fifo
//
// there is no check whether a char is ready, this must be
// done before calling with a call to rx_fifo_ready();
unsigned int XP_rx_read (void)
{
  unsigned int retval;

  retval = X_RxBuffer[X_rx_read_ptr]; //sds 9 bit RX
  X_rx_read_ptr++;
  if (X_rx_read_ptr == X_RxBuffer_Size) X_rx_read_ptr=0;

  return(retval);
} // XP_rx_read
