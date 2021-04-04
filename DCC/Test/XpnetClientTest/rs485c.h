#ifndef __RS485C_H__
#define __RS485C_H__

void init_rs485(uint8_t dirPin);            // the DE/RE direction control pin
void XP_setAddress (uint8_t slaveAddress);  // set the slave address we will be listening on
bool XP_send_byte (const uint8_t c);        // sends one byte with isr and fifo, bit 8 = 0

bool XP_tx_ready (void);    // true if byte can be sent
bool XP_tx_empty(void);     // true if fifo is empty and all data are sent
void XP_tx_clear (void);    // clear TX buffer
uint16_t XP_rx_peek (void); // sds added, voor het gemak van xpc
uint16_t XP_rx_read (void); // reads one byte, sds : 9bits voor de client, check callbit!
bool XP_rx_ready (void);    // true if one byte can be read

#endif  // __RS485C_H__
