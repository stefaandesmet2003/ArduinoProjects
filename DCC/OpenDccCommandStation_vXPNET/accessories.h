#ifndef _ACCESSORIES_H_
#define _ACCESSORIES_H_

// store input data from a feedback decoder
// return : previous contents for the decoderAddress, if changed xpnet knows it needs to broadcast the changes
uint8_t feedback_update(uint8_t decoderAddress, uint8_t data);


// store output data from turnouts
void turnout_update(uint16_t turnoutAddress, uint8_t coil);

// generate the xpnet message bytes
void accessory_getInfo (uint8_t decoderAddress, uint8_t nibble, uint8_t *msg);
void turnout_getInfo (uint16_t turnoutAddress, uint8_t *msg);

#endif // _ACCESSORIES_H_
