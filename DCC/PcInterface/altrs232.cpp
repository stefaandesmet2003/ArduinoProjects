// sds : (beperkte) implementatie van rs232.h interface (dcc) obv AltSoftSerial

#include <AltSoftSerial.h>
#include "altrs232.h"
AltSoftSerial pcSerial;

void init_rs232()
{
  // set the data rate for the SoftwareSerial port
  pcSerial.end(); // sds : werkt dit? zodat je ook reinits kan doen via de lenz intf commando??
  pcSerial.begin(19200); // voorlopig doen we enkel 19200 (lenz intf default)
} // init_rs232

bool tx_fifo_ready (void)
{
  return (1);
  // met AltSoftSerial hebben we geen zicht hoeveel de TxBuffer gevuld is -> we gaan ervan uit dat er altijd plaats is .. 
  // AltSoftSerial.write wacht indien nodig met het schrijven tot er plaats is, dus overwrites in de TxBuffer kunnen in principe niet voorvallen
}

// ret 1 if full
bool tx_fifo_write (const unsigned char c)
{
  pcSerial.write(c);
  return (0);
}

void tx_flush (void)
{
  pcSerial.flush(); // Waits for the transmission of outgoing serial data to complete.
}

bool rx_fifo_ready (void)
{
  return (pcSerial.available() > 0);
}

uint8_t rx_fifo_read (void)
{
  int retval;
  retval = pcSerial.read();
  return ((unsigned char) retval);
}