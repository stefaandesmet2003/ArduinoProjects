#include "appstub.h"
#include "Arduino.h"

// opendcc includes
#include "config.h"
#include "organizer.h"
#include "status.h"
#include "programmer.h"

char app_CurTime[] = "12:07";

// parameters van de huidig bestuurde loc
uint8_t locSpeed; //DCC128
uint16_t locAddr;
uint32_t locFuncs;
char app_LocName[] = "DIESEL"; // temp

// voor de analoge appstub
#define PWM_MINSPEED 90  // daaronder beweegt de loc niet (analoge modus)
#define PWM_MAXSPEED 255 // wanneer gaat de rijrichtingsschakelaar tussenkomen?


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

// slot 1 = local UI, slot 0 = PC (via lenz intf)
// welk format gebruikt do_loco_speed ?? hangt af van de eeprom database (DCC14, DCC28, DCC128)
// dcc_default_format == DCC28 (in config.h), tenzij anders opgeslagen in eeprom database
// de intf gebruikt DCC128, maar wordt vertaald door organizer met 'convert_speed_to_rail'
// moet dit overeenkomen met een decoder CV, of zal de decoder automatisch de verschillende speed formaten ondersteunen in de verschillende dcc msgs?
// --> checken!! 
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
// dus : locSpeed moet DCC128 formaat volgen, dwz 0 = stop, 1= noodstop, 2..127 = speedsteps, msb = richting, 1=voorwaarts, 0=achterwaarts
void app_SetSpeed (uint16_t locAddr, uint8_t locSpeed)
{
    if (organizer_ready())
    {
        do_loco_speed (1,locAddr, locSpeed);
    }
} // app_SetSpeed

uint8_t app_GetSpeed (uint16_t locAddr)
{
    uint8_t lb_index;
    lb_index = scan_locobuffer(locAddr);  
    if (lb_index != SIZE_LOCOBUFFER)
    {
        return (locobuffer[lb_index].speed);
    }
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
    if (lb_index != SIZE_LOCOBUFFER)
    {
        retval = locobuffer[lb_index].fl; //FL = F0
        retval = retval | ((locobuffer[lb_index].f4_f1 & 0x0F) << 1);
        retval = retval | ((locobuffer[lb_index].f8_f5 & 0x0F) << 5);
        retval = retval | ((locobuffer[lb_index].f12_f9 & 0x0F) << 9);
#if (DCC_F13_F28 == 1)
        retval = retval | (locobuffer[lb_index].f20_f13 << 13);
        retval = retval | ((uint32_t)locobuffer[lb_index].f28_f21 << 21);  // TODO 2021: moeten dan niet alle lijnen cast naar uint32_t hebben?
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
    if (organizer_ready())
    {
        if (func ==0)
        {
            do_loco_func_grp0 (1,locAddr, onOff);
        }
        else if ((func >=1) && (func <= 4))
        {
            do_loco_func_grp1 (1,locAddr, (onOff & 0x1) << (func-1));
        }
        else if ((func >=5) && (func <= 8))
        {
            do_loco_func_grp2 (1,locAddr, (onOff & 0x1) << (func-5));
        }
        else if ((func >=9) && (func <= 12))
        {
            do_loco_func_grp3 (1,locAddr, (onOff & 0x1) << (func-9));
        }
#if (DCC_F13_F28 == 1)        
        else if ((func >=13) && (func <= 20))
        {
            do_loco_func_grp4 (1,locAddr, (onOff & 0x1) << (func-13));
        }
        else if ((func >=21) && (func <= 28))
        {
            do_loco_func_grp5 (1,locAddr, (onOff & 0x1) << (func-21));
        }
#endif        
    }
} // app_SetFunc

// caller to provide memory for *name
// database.cpp : store_loco_name wordt nergens gebruikt
// waar wordt de locname bewaard? in database.cpp, en elders?
void app_GetLocName( uint16_t locAddr, char *locName )
{
    //todo
    strcpy(locName,app_LocName);
    
} // app_GetLocName

// voorlopig hier een geheugen voor 32 wissels
uint32_t app_Turnouts = 0xFFFFFFFF; // alle wissels rechtdoor, 1 bit per turnoutAddr 0..31
// turnoutAddr 0-4095, coil red/green 0/1
// enkel het activate commando wordt gestuurd, de turnout decoder doet zelf de deactivate
// voorlopig turnoutAddr 0..31, organizer mapt die op dcc accessory addresses 1->8 (4 wissels per decoder)

bool app_ToggleAccessory (uint16_t turnoutAddr)
{
    bool retval;
    uint8_t coil = ((app_Turnouts >> turnoutAddr) & 0x1) ^0x1;
    
    //Serial.print("toggle accessory : ");
    //Serial.print(turnoutAddr);
    //Serial.print(" coil : ");
    //Serial.print(coil);
    if (!organizer_ready())
        return (APP_INTERNALERR0R);
    
    retval = do_accessory(1,turnoutAddr,coil,1);
    //Serial.print("retval = ");
    //Serial.println(retval);
    if (!retval)
        app_Turnouts ^= (1 << turnoutAddr);
    //Serial.println(app_Turnouts,HEX);
    return (retval);
    
} // app_ToggleAccessory


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



/***********************************************************************************************************/
#ifdef ANALOG_OLD_STUFF
uint8_t app_GetSpeed (uint16_t locAddr)
{
    // uitbreiden naar de andere locs in de databank
    // voorlopig enkel de huidige loc, zonder locAddr te checken
    return locSpeed;
}
uint32_t cnt;
void app_SetSpeed (uint16_t locAddr, uint8_t dccSpeed)
{
    uint32_t pwm;
    locSpeed = dccSpeed; // eventueel sanity check
    
    //dccSpeed = dccSpeed & 0x7F; // for analog pwm only! forget direction bit
    pwm = PWM_MINSPEED + (PWM_MAXSPEED - PWM_MINSPEED)* (dccSpeed - 2) / 125;
    // TODO : de pwm doen
    
    // zet de speed in dcc of analog
    Serial.print(cnt);
    Serial.print(" : app_SetSpeed : ");
    Serial.println(dccSpeed);
    cnt++;
}
uint32_t app_GetFuncs (uint16_t locAddr)
{
    // uitbreiden naar de andere locs in de databank
    // voorlopig enkel de huidige loc, zonder locAddr te checken
    return locFuncs;
}

// 1 bit per func, 0=uit, 1=aan
void app_SetFunc (uint16_t locAddr, uint8_t func, uint8_t onOff)
{
    uint32_t temp;
    
    if (onOff)
        locFuncs = locFuncs | ((onOff & 0x1) << func);
    else
        locFuncs = locFuncs & (~(1 << func));
    
    Serial.print("app_SetFuncs : ");
    Serial.println(locFuncs,HEX);
}

void app_GetLocName( uint16_t locAddr, char *locName )
{
    //todo
    strcpy(locName,app_LocName);
    
} // app_GetLocName



#endif // ANALOG_OLD_STUFF

