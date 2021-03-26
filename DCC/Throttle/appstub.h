#ifndef _appstub_h_
#define _appstub_h_
#include "Arduino.h"
#include "status.h"

#define APP_OK              0
#define APP_BUSY            1 // command is niet op xpnet doorgegeven aangezien een vorige transactie nog bezig is
#define APP_INTERNALERR0R   2

#define LOCSTATE_UNKNOWN 0 // init stage; no info received from xpnet
#define LOCSTATE_FREE 1 // this xpnet device can take or has already taken control
#define LOCSTATE_OWNED 2 // another xpnet device has taken control

void app_Init(void);
uint8_t app_GetTime (t_fast_clock *curTime);
// returns 1 of the LOCSTATE_xxx values
uint8_t app_GetLocState(uint16_t locAddress);

char * app_GetCSState (void);
uint16_t app_GetPrevLoc (uint16_t curLoc);
uint16_t app_GetNextLoc (uint16_t curLoc);

// returns DCC128 speed, or 0 if locAddress is LOCSTATE_UNKNOWN
uint8_t app_GetSpeed (uint16_t locAddress);
// returnt een APP_xxx 
uint8_t app_SetSpeed (uint16_t locAddress, uint8_t dccSpeed);
uint32_t app_GetFuncs (uint16_t locAddress);
void app_SetFunc (uint16_t locAddress, uint8_t func, uint8_t onOff); // 1 bit per func, 0=uit, 1=aan
void app_GetLocName (uint16_t locAddress, char *locName);

bool app_ToggleAccessory (uint16_t turnoutAddress);

void app_TestCVRead();
uint8_t app_GetProgResults (uint16_t &cv, uint8_t &cvdata);

// functions as macros
#define app_SetCSState(__csState) set_opendcc_state(__csState)
#define app_GetCSState() (opendcc_state)

#endif // _appstub_h_
