#ifndef __RS485C_H__
#define __RS485C_H__

void rs485_Init (uint8_t dirPin);            // the DE/RE direction control pin
void rs485_SetSlaveAddress (uint8_t slaveAddress);  // set the slave address we will be listening on
bool rs485_send_byte (const uint8_t c);        // sends one byte with isr and fifo, bit 8 = 0

bool rs485_tx_ready();    // true if byte can be sent
bool rs485_tx_empty();     // true if fifo is empty and all data are sent
void rs485_tx_clear();    // clear TX buffer
uint16_t rs485_rx_peek(); // sds added, voor het gemak van xpc
uint16_t rs485_rx_read(); // reads one byte, sds : 9bits voor de client, check callbit!
bool rs485_rx_ready();    // true if one byte can be read

#endif  // __RS485C_H__
