// SDS TODO 2021 : cleanup + check!

#include "Arduino.h"
#include "appstub.h"

#include "config.h"
#include "organizer.h"
#include "status.h"
#include "programmer.h"
#include "database.h" // loc names

// xpnet broadcast na een wisselbediening vanaf local ui
#include "xpnet.h"
#include "accessories.h"

// the organizer keeps track of each xpnet device
// 0 = invalid xpnet slot (broadcast), so we can use this here to identify the local UI
// original opendcc used slot 0 = PC (LENZ intf)
#define LOCAL_UI_SLOT (0)

char app_CurTime[] = "12:07";

// parameters van de huidig bestuurde loc
uint8_t locSpeed; //DCC128
uint16_t locAddr;
uint32_t locFuncs;
char app_DefaultLocName[] = "LOC?";

/****************************************************************************************************/
/****************************************************************************************************/
/****************************************************************************************************/
// app stub
void app_Init()
{
} // app_Init

char * app_GetTime (void)
{
	return app_CurTime;
}

uint16_t app_GetPrevLoc (uint16_t curLoc)
{
  return (curLoc - 1);
}

uint16_t app_GetNextLoc (uint16_t curLoc)
{
  return (curLoc + 1);
}

/*
 * do_loco_speed gebruikt altijd DCC128! (intern organizer formaat)
 * het formaat dat op de rail wordt gezet hangt af van de loco database (eeprom) -> 06/2021 : is nu ook default DCC128
 * en dcc_default_format == DCC128 (in config.h) voor nieuwe loc addressen
 * met do_loco_speed_f kan je toch nog aansturen met een ander formaat, maar dan moet je eerst 'convert_speed_from_rail' :
 * speed128 = convert_speed_from_rail(speed,format)
 * en dan do_loco_speed_f (speed128,format)
 * wat op rail komt moet door de decoder ondersteund worden, maar is ok voor de mijne (doen zowel DCC28 & DCC128) 
 * dus momenteel geen nood om vanuit UI een ander formaat kunnen instellen, en do_loco_speed_f te gebruiken
 */

/*
The optional NMRA 128 speed step mode is available for both decoders and command stations that support it. 
Both the decoder and the command station controlling it must support this feature. 
A decoder switches to 128 speed step mode automatically at the track level when a it receives a 128 speed step command from the command station. 
The decoder will switch back to 14 or 28 speed step mode automatically when it receives a command in that speed command format. 
Unlike 14 and 28 speed step commands, the decoder does not need to pre-programmed to enable 128 speed step mode operation. 
It is always on, ready to be used at any time.
If a decoder capable of only 28 speed steps is on the layout, 
it will ignore the 128 mode and function at the 28 step mode.
That is why the 28 step mode is often referred to as 28/128.
*/

// #define DCC_SHORT_ADDR_LIMIT   112  (in config.h)
// als locAddr > DCC_SHORT_ADDR_LIMIT, gaat de organiser DCC msgs voor long addr gebruiken
// dus : locAddr < 112 -> short addr naar loc-decoder, > 112 --> long addr naar loc-decoder
// dus : locSpeed in het locobuffer format (128 steps), dwz 0 = stop, 1= noodstop, 2..127 = speedsteps, msb = richting, 1=voorwaarts, 0=achterwaarts
void app_SetSpeed (uint16_t locAddr, uint8_t locSpeed)
{
  unsigned char retval;
  if (organizer_ready()) {
    retval = do_loco_speed (LOCAL_UI_SLOT,locAddr, locSpeed);
    if (retval & ORGZ_STOLEN)
      xp_send_lok_stolen(orgz_old_lok_owner,locAddr);
  }
} // app_SetSpeed

uint8_t app_GetSpeed (uint16_t locAddr)
{
  uint8_t lb_index;
  lb_index = scan_locobuffer(locAddr);  
  if (lb_index != SIZE_LOCOBUFFER)
    return (locobuffer[lb_index].speed);
  else
    return (0); // dit is geen invalid speed, maar stop met reverse bit, we gaan ervan uit dat dit normaal niet wordt gebruikt, stop speed = 0x0
} // app_GetSpeed

// support voor 32 functies, 1 bit per functie, 29 max gebruikt : fl + f1..f28
// zijn die funcs allemaal geinitialiseerd in locobuffer? bv bij startup uit de database? of moet de commandstation/ui dit doen?
// retval 0xFFFF = locAddr niet in loco buffer
uint32_t app_GetFuncs (uint16_t locAddr)
{
  uint8_t lb_index;
  uint32_t retval = 0;
  lb_index = scan_locobuffer(locAddr);  
  if (lb_index != SIZE_LOCOBUFFER) {
    retval = locobuffer[lb_index].fl; //FL = F0
    retval = retval | ((locobuffer[lb_index].f4_f1 & 0x0F) << 1);
    retval = retval | ((locobuffer[lb_index].f8_f5 & 0x0F) << 5);
    retval = retval | ((locobuffer[lb_index].f12_f9 & 0x0F) << 9);
#if (DCC_F13_F28 == 1)
    retval = retval | (locobuffer[lb_index].f20_f13 << 13);
    retval = retval | ((uint32_t)locobuffer[lb_index].f28_f21 << 21); // TODO 2021: moeten dan niet alle lijnen cast naar uint32_t hebben?
#endif 
    return (retval);
  }
  else
    return (0xFFFF); // dit is een invalid code, want er zijn maar 28 funcs gedefinieerd in dcc
} // app_GetFuncs

// 1 bit tegelijk, we kunnen toch maar 1 bit tegelijk togglen in de UI
// func 0 = light
// f1 ->f28 
// on = 1, off = 0
void app_SetFunc (uint16_t locAddr, uint8_t func, uint8_t onOff) 
{
  uint8_t retval;
  if (organizer_ready()) {
    if (func ==0)
      retval= do_loco_func_grp0 (LOCAL_UI_SLOT,locAddr, onOff);
    else if ((func >=1) && (func <= 4))
      retval= do_loco_func_grp1 (LOCAL_UI_SLOT,locAddr, (onOff & 0x1) << (func-1));
    else if ((func >=5) && (func <= 8))
      retval= do_loco_func_grp2 (LOCAL_UI_SLOT,locAddr, (onOff & 0x1) << (func-5));
    else if ((func >=9) && (func <= 12))
      retval= do_loco_func_grp3 (LOCAL_UI_SLOT,locAddr, (onOff & 0x1) << (func-9));
#if (DCC_F13_F28 == 1)        
    else if ((func >=13) && (func <= 20))
      retval= do_loco_func_grp4 (LOCAL_UI_SLOT,locAddr, (onOff & 0x1) << (func-13));
    else if ((func >=21) && (func <= 28))
      retval= do_loco_func_grp5 (LOCAL_UI_SLOT,locAddr, (onOff & 0x1) << (func-21));
#endif
    if (retval & ORGZ_STOLEN)
      xp_send_lok_stolen(orgz_old_lok_owner,locAddr);

  }
} // app_SetFunc

// caller to provide memory for *name
// database.cpp : store_loco_name wordt voorlopig nergens gebruikt
void app_GetLocName( uint16_t locAddr, char *locName) {
  uint8_t retval;
  retval = get_loco_name(locAddr, (uint8_t*)locName);
  if (!retval)
    strcpy(locName,app_DefaultLocName);
} // app_GetLocName

// voorlopig hier een geheugen van de wissels
uint32_t app_Turnouts = 0xFFFFFFFF; // alle wissels rechtdoor, 1 bit per turnoutAddress 0..31
// turnoutAddress 0-4095, coil red/green 0/1
// enkel het activate commando wordt gestuurd, de turnout decoder doet zelf de deactivate
// voorlopig turnoutAddress 0..31

bool app_ToggleAccessory (uint16_t turnoutAddress, bool activate) {
  bool retval;
  // een 1-bit in app_Turnouts betekent dat de wissel rechtdoor staat
  // om te togglen moeten we dan coil=1 aansturen (throw)
  // bij succes, komt dan een 0-bit in app_Turnouts = wissel gebogen op display
  uint8_t coil = ((app_Turnouts >> turnoutAddress) & 0x1);
  if (!organizer_ready())
    return (APP_INTERNALERR0R);
  
  if (activate) {
    retval = do_accessory(turnoutAddress,coil,activate); // retval==0 means OK
    if (retval==0) { // only notify the 'on' command, not the 'off'
      app_Turnouts ^= (1 << turnoutAddress);
      // notify andere xp clients dat we de wissel verzet hebben!
      unsigned char tx_message[3];
      tx_message[0] = 0x42;
      turnout_getInfo(turnoutAddress,&tx_message[1]);
      xp_send_message(FUTURE_ID, tx_message); // feedback broadcast
    }
  }
  else {
    retval = do_accessory(turnoutAddress,coil^0x1,activate); // retval==0 means OK
  }
  return (retval);
} // app_ToggleAccessory

bool app_DoExtendedAccessory (uint16_t decoderAddress, uint8_t signalId, uint8_t signalAspect) {
  bool retval;
  if (!organizer_ready())
    return (APP_INTERNALERR0R);
  retval = do_signal_accessory(decoderAddress,signalId, signalAspect); // retval==0 means OK
  return retval;
} // app_DoExtendedAccessory

void app_TestCVRead()
{
  // read CV1
  //Serial.println("test CV read");
  my_XPT_DCCRD(1);
}

uint8_t app_GetProgResults (uint16_t &cv, uint8_t &cvdata)
{
  uint8_t retval;
  
  if (prog_event.busy)
    return 1;
  retval = (uint8_t) prog_result;
  cv = prog_cv;
  cvdata = prog_data;
  return (retval);
}
