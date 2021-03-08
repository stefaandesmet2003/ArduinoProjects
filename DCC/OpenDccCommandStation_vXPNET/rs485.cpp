//------------------------------------------------------------------------
//
// OpenDCC
//
// Copyright (c) 2006-2008 Wolfgang Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//------------------------------------------------------------------------
//
// file:      rs485.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2008-07-08 V0.1 copied from DCCsniffer, modified for
//                            Xpressnet
//            2008-08-17 V0.2 changed TxBuff to int.;
//                            Broadcasts could be buffered
//
//------------------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   routines for RS485
//            see also:
//            xpressnet definition
//            here: RX and TX Fifos
//                  Control of line direction 
//
//------------------------------------------------------------------------


#include <stdlib.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>         // put var to program memory
#include <avr/eeprom.h>
#include <util/parity.h> 
#include <string.h>

#include "Arduino.h"
#include "hardware.h"
#include "rs485.h"

//
//=====================================================================
//
// RS485 --> gebruikt UART0 van de arduino
// RS485_DERE pin gebruikt voor tx/rx control
//
// purpose:   send and receive messages from xpressnet
//
// how:       uart acts with interrupt on fifos.
//            ohter programs access only the fifos.
//            hardware flowcontrol (direction pin)
//
// interface: see rs485.h
//
//-----------------------------------------------------------------


// FIFO objects for RX and TX
// max. Size: 255

#define X_RxBuffer_Size  32              // mind. 16
unsigned char X_RxBuffer[X_RxBuffer_Size];


#define X_TxBuffer_Size  32
unsigned int X_TxBuffer[X_TxBuffer_Size];       // this is int, we have 9 bits

unsigned char X_rx_read_ptr = 0;                // point to next read
unsigned char X_rx_write_ptr = 0;               // point to next write
unsigned char X_rx_fill = 0;
unsigned char X_tx_read_ptr = 0;
unsigned char X_tx_write_ptr = 0;
unsigned char X_tx_fill = 0;

// support

void XP_flush_rx(void)                 // Flush Receive-Buffer
  {
    uint8_t sreg = SREG;
    cli();
    do
      {
        UDR0;
      }
    while (UCSR0A & (1 << RXC0));

    // R�cksetzen von Receive Complete-Flags
    // UCSR1A |= (1 << RXC0);  //sds : waarom gecomment in orig? niet het geval in rs232.cpp
    UCSR0B &= ~(1 << RXEN0);
    UCSR0B |= (1 << RXEN0);
    X_rx_read_ptr = 0;      
    X_rx_write_ptr = 0;
    X_rx_fill = 0; 
    SREG = sreg;     
  }

inline void set_XP_to_receive(void) __attribute__((always_inline));
void set_XP_to_receive(void)
{
    HARDWARE_SET_XP_RECEIVE;        // see rs485.h
}

inline void set_XP_to_transmit(void) __attribute__((always_inline));
void set_XP_to_transmit(void)
  {
    HARDWARE_SET_XP_TRANSMIT;        // see rs485.h
  }

//sds : aangepast uart1 -> uart0 voor atmega328p
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
   
    // FIFOs f�r Ein- und Ausgabe initialisieren

    X_tx_read_ptr = 0;
    X_tx_write_ptr = 0;
    X_tx_fill = 0;

    // UART Receiver und Transmitter anschalten, Receive-Interrupt aktivieren
    // Data mode 9N1, asynchron
    
    UCSR0B = (1 << RXCIE0)          // enable receive Int
           | (1 << TXCIE0)          // enable TX complete Interrupt
           | (0 << UDRIE0)
           | (1 << RXEN0) 
           | (1 << TXEN0) 
           | (1 << UCSZ02)          // set frame length to 9bit;
           | (0 << RXB80)	
           | (1 << TXB80);          // set 9th bit to 1


    UCSR0C = (0 << UMSEL01)         // 00 = asyn mode
           | (0 << UMSEL00)         // 
           | (0 << UPM01)           // 00 = parity disabled
           | (0 << UPM00)           // 
           | (0 << USBS0)           // 0 = tx with 2 stop bits
           | (1 << UCSZ01)          // 11 = 8 or 9 bits
           | (1 << UCSZ00)
           | (0 << UCPOL0);


    XP_flush_rx();
    
    UCSR0A |= (1 << TXC0);         // clear tx complete flag
    
    SREG = sreg;
}

//---------------------------------------------------------------------------
// Halbduplex: auf TXC Complete schalten wir sofort die Richtung um
// Das als NAKED um es wirklich schnell zu bekommen.
// Wenn Optimize wegf�llt, dann darf NAKED nicht mehr benutzt werden - Flags!
// Vermutlich wird alles andere als ISR_NOBLOCK laufen m�ssen!
//
// ISR(USART1_TX_vect, ISR_NAKED)
//  {    
//    set_XP_to_receive();
//    reti();
//  }

ISR(USART_TX_vect)
  {    
    set_XP_to_receive();

    // UCSR1B |= (1 << RXEN1);
    // UCSR1B |= (1 << RXCIE1);
    // UCSR1B &= ~(1 << TXCIE1);   // stop TX Complete Interrupts

    //X_rx_read_ptr = 0;      
    //X_rx_write_ptr = 0;
    //X_rx_fill = 0;
  }

//----------------------------------------------------------------------------
// Ein Zeichen aus der Ausgabe-FIFO lesen und ausgeben
// Ist das Zeichen fertig ausgegeben, wird ein neuer SIG_UART_DATA-IRQ getriggert
// Ist das FIFO leer, deaktiviert die ISR ihren eigenen IRQ.

// We transmit 9 bits

//sds aangepast voor uart0 ipv uart1
ISR(USART_UDRE_vect) 
{
    union
    {
        unsigned int w;
        unsigned char b[sizeof(unsigned int)];
    } tdat;

    if (X_tx_read_ptr != X_tx_write_ptr)
    {
        // UCSR1A |= (1 << TXC0);              // sds : waarom gecomment?? niet het geval in rs232.cpp

        tdat.w = X_TxBuffer[X_tx_read_ptr];
        if (tdat.b[1]) UCSR0B |= (1<<TXB80);               // set bit 8
        else           UCSR0B &= ~(1<<TXB80);              // clear bit 8
        
        UDR0 = tdat.b[0];
        X_tx_read_ptr++;
        if (X_tx_read_ptr == X_TxBuffer_Size) X_tx_read_ptr=0;
        X_tx_fill--;
    }
    else
        UCSR0B &= ~(1 << UDRIE0);           // disable further TxINT
}


//---------------------------------------------------------------------------
// Empfangene Zeichen werden in die Eingabgs-FIFO gespeichert und warten dort
// keine �berlaufsicherung, da ja wir Master sind.
//
// sds aangepast voor uart0 ipv uart1
ISR(USART_RX_vect)
{
    if (UCSR0A & (1<< FE0))
	{ // Frame Error
        UDR0;                   // zumindest lesen, damit der INT stirbt
	}
    else
    {
        if (UCSR0A & (1<< DOR0))
        { // DATA Overrun -> Fatal
            // !!! 
        }

       	X_RxBuffer[X_rx_write_ptr] = UDR0;

        X_rx_write_ptr++;
        if (X_rx_write_ptr == X_RxBuffer_Size) X_rx_write_ptr=0;
        X_rx_fill++;          // no check for full, we just do an overrun!
    }
}


//=============================================================================
// Upstream Interface
//-----------------------------------------------------------------------------
// TX:
bool XP_tx_ready (void)
{
    if (X_tx_fill < (X_TxBuffer_Size-18))     // keep space for one complete message (16)
    {
        return(1);                        // true if enough room
    }
    else
    {
        return(0);
    }
}


// ret 1 if full
// This goes with fifo (up to 32), bit 8 is 0.

//sds : aangepast voor uart0 ipv uart1!
bool XP_send_word (const unsigned int c)
{
    X_TxBuffer[X_tx_write_ptr] = c;

    X_tx_write_ptr++;
    if (X_tx_write_ptr == X_TxBuffer_Size) X_tx_write_ptr=0;

    unsigned char mysreg = SREG;
    cli();
    X_tx_fill++;
    SREG = mysreg;

    set_XP_to_transmit();
    UCSR0A |= (1 << TXC0);      // clear any pending tx complete flag
    UCSR0B |= (1 << TXEN0);     // enable TX
    UCSR0B |= (1 << UDRIE0);    // enable TxINT
    UCSR0B |= (1 << TXCIE0);    // enable TX complete --> sds : dit gaat de ISR(USART_TX_vect) voeden

//    if (X_tx_fill < (X_TxBuffer_Size-18)) // sds : is dit juist???
    if (X_tx_fill > (X_TxBuffer_Size-18))
      {
        return(1);
      }
    return(0);
}

bool XP_send_byte (const unsigned char c)
{
    return(XP_send_word(c));
}

bool XP_send_call_byte (const unsigned char c)
{
    unsigned char my_c, temp;

    my_c = c & 0x7F;
    temp = my_c ^ (my_c << 4);
    temp = temp ^ (temp << 2);
    temp = temp ^ (temp << 1);
    my_c |= temp & 0x80;
   
    // alternative: my_c = c & 0x7F;
    // alternative: if (parity_even_bit(my_c)) my_c |= 0x80;

    return(XP_send_word(my_c | 0x100)); //bit 9 op 1 zetten in de call byte
}



// ret 1 if all is sent
// commented here an move to header
/* SDS - inline werkt niet als deze code niet in de .h file staat
bool XP_is_all_sent (void)
  {
    return (!(PORTD & (1 << PD4)));      //SDS aangepast aan onze CS config (was PB5)               // just look at driver bit
  }
SDS */

void X_uart_puts (const char *s)
{
    do
    {
        XP_send_byte (*s);   // no check, only short strings
    }
    while (*s++);
}


//------------------------------------------------------------------------------
// RX:
bool XP_rx_ready (void)
{
    if (X_rx_read_ptr != X_rx_write_ptr)
    {
        return(1);     // there is something
    }
    else
    {
        return(0);  
    }
}

//-------------------------------------------------------------------
// rx_fifo_read gets one char from the input fifo
//
// there is no check whether a char is ready, this must be
// done before calling with a call to rx_fifo_ready();

unsigned char XP_rx_read (void)
{
    unsigned char retval;

    retval = X_RxBuffer[X_rx_read_ptr];
    X_rx_read_ptr++;
    if (X_rx_read_ptr == X_RxBuffer_Size) X_rx_read_ptr=0;

    unsigned char mysreg = SREG;
    cli();
    X_rx_fill--;
    SREG = mysreg;
        
    return(retval);
}


//-----------------------------------------------------------------------------------
// testroutinen zu Parsertest
// Ersatzroutine, um ein Zeichen in den Empfangsbuffer abzulegen
// Sendepuffer kann im Simulator mit einen Databreakpoint auf UDR �berwacht werden.
void X_push_to_rx(unsigned char d)
{
	X_RxBuffer[X_rx_write_ptr] = d;

    X_rx_write_ptr++;
    if (X_rx_write_ptr == X_RxBuffer_Size) X_rx_write_ptr=0;
    X_rx_fill++;
}
