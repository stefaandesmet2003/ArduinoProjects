
#ifndef TurnoutDecoder_h
#define TurnoutDecoder_h

// turnout-decoder config specific definitions here

#define CV_TimeOnOutput1  3
#define CV_TimeOnOutput2  4
#define CV_TimeOnOutput3  5
#define CV_TimeOnOutput4  6

#define DEFAULT_TIMEONOUTPUT 5 // = 5*50ms = 250ms, voor factory defaults


void turnout_Init();
void turnout_Handler(uint16_t decoderAddress, uint8_t outputId, bool activate);
void turnout_ManualToggle (uint8_t turnoutId); // turnoutId = 0..3, manuele bediening met de knoppen
uint8_t turnout_FactoryResetCV ( void );

#endif //TurnoutDecoder_h
