#ifndef _appstub_h_
#define _appstub_h_
#include "Arduino.h"
#include "config.h"
#include "status.h"

#define APP_OK              0
#define APP_INTERNALERR0R   1

void app_Init(void);
char * app_GetTime (void);
char * app_GetCSState (void);
uint16_t app_GetPrevLoc (uint16_t curLoc);
uint16_t app_GetNextLoc (uint16_t curLoc);

uint8_t app_GetSpeed (uint16_t locAddr);
void app_SetSpeed (uint16_t locAddr, uint8_t dccSpeed);
uint32_t app_GetFuncs (uint16_t locAddr);
void app_SetFunc (uint16_t locAddr, uint8_t func, uint8_t onOff); // 1 bit per func, 0=uit, 1=aan
void app_GetLocName (uint16_t locAddr, char *locName);

bool app_ToggleAccessory (uint16_t turnoutAddr);

void app_TestCVRead();
uint8_t app_GetProgResults (uint16_t &cv, uint8_t &cvdata);

// functions as macros
#define app_SetCSState(__csState) set_opendcc_state(__csState)
#define app_GetCSState() (opendcc_state)

#endif // _appstub_h_
