#ifndef __ALTRS232_H__
#define __ALTRS232_H__

void init_rs232(void);
bool tx_fifo_write (const uint8_t c);
bool tx_fifo_ready (void);
void tx_flush (void); // sds : vervangt de busy loop met tx_all_sent
bool rx_fifo_ready (void);
uint8_t rx_fifo_read (void);

#endif  // __RS232_H__

