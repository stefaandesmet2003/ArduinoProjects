#ifndef _appstub_h_
#define _appstub_h_
#include "Arduino.h"

#define LOCSTATE_UNKNOWN 0 // init stage; no info received from xpnet
#define LOCSTATE_FREE 1 // this xpnet device can take or has already taken control
#define LOCSTATE_OWNED 2 // another xpnet device has taken control

void app_Init();
void app_GetTime();
char app_GetCSState (void); // 2021 : ??

void app_GetLocState(uint16_t locAddress, uint8_t *locState);
uint16_t app_GetPrevLoc (uint16_t curLoc);
uint16_t app_GetNextLoc (uint16_t curLoc);

// returns DCC128 speed, or 0 if locAddress is LOCSTATE_UNKNOWN
void app_GetLocSpeed (uint16_t locAddress, uint8_t *locSpeed);
void app_SetLocSpeed (uint16_t locAddress, uint8_t locSpeed);

void app_GetLocFuncs (uint16_t locAddress, uint32_t *locFuncs);
void app_SetLocFunc (uint16_t locAddress, uint8_t func, uint8_t onOff);
void app_GetLocName (uint16_t locAddress, char *locName);

void app_ToggleAccessory (uint16_t turnoutAddress);
void app_TestCVRead();
void app_GetProgResults (uint16_t &cv, uint8_t &cvdata);

#endif // _appstub_h_
