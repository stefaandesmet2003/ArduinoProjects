#ifndef _OPENTHERM_H_
#define _OPENTHERM_H_

/*
implementation uses either a pin change interrupt or analog compare interrupt on rxPin
Therefire rxPin is fixed for the moment : AIN1 = PD7 = PCINT23
timer2 is used for bitbang write timing and rx timeout handling
*/

#define T_RX_PIN  7
#define T_TX_PIN  8

//#define LOOPTEST
#define BOILERSIDE
//#define THERMOSTATSIDE


#ifdef LOOPTEST // direct connection between boiler-side & thermostat-side for testing
  #define RX_PIN_LEVEL_ACTIVE HIGH
  #define TX_PIN_LEVEL_ACTIVE HIGH
#endif
#ifdef BOILERSIDE
  // use analog compare instead of pin change interrupt, because the voltage at R5/R6 is too low for a digital input
  #define USE_ANALOG_COMPARE
  #define RX_PIN_LEVEL_ACTIVE LOW // ACO=0 if AIN1 > 1.1V = active level from thermostat
  #define TX_PIN_LEVEL_ACTIVE LOW // TX pin low activates Q5 and the high current source
#endif
#ifdef THERMOSTATSIDE
  #define RX_PIN_LEVEL_ACTIVE HIGH // active level sent from boiler results in digital HIGH on Rx pin on thermostat side
  #define TX_PIN_LEVEL_ACTIVE LOW // TX pin low disables OK1A and forces active voltage via D10
#endif 

void OT_init();
bool OT_available();
uint32_t OT_readFrame();
void OT_writeFrame(uint32_t frameData); // write 32-bit frame with busy wait
void OT_writeFrameInt(uint32_t frameData); // write 32-bit frame in interrupt mode (loop continues to work)
// normally the Opentherm driver will set the TX state to idle after communication is completed
// this function forces another state, for instance to force high current from boiler->thermostat for charging
void OT_setTxState(bool active);

#endif // _OPENTHERM_H_
