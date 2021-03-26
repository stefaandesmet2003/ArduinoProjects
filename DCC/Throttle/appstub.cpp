// TODO 2021 : dit is ouwe rommel xpc_IsBusy bestaat niet, 
// is ook niet meer nodig, omdat xpc_sendXXX nu blocking is zolang er nog een eerder msg in de TxBuffer staat

#include "appstub.h"
#include "Arduino.h"

#include "status.h"

// opendcc includes -> replace with xpc intf here
#include "xpc.h"

// #include "config.h"
// #include "organizer.h"
// #include "programmer.h"

// TODO : de F13..F28 correct initialiseren (extra interne locstate, die xpc_send_LocGetFuncStatus_F13_F18_Request aanroept)
// voorlopig enkel F0..F12
// TODO : de callbacks van xpc opvangen voor response data
// TODO : app_ToggleAccessory, app_GetProgResults


// parameters van de huidig bestuurde loc (throttle kan maar 1 loc tegelijk besturen)
static uint8_t app_LocSpeed; //DCC128
static uint16_t app_LocAddress;
static uint32_t app_LocFuncs;
static uint8_t app_LocState; 

// locName not supported by xpnet
char app_LocName[] = "DIESEL"; // temp


/****************************************************************************************************/
/****************************************************************************************************/
/****************************************************************************************************/
// app stub
void app_Init()
{

} // app_Init

uint8_t app_GetTime (t_fast_clock *curTime)
{
    curTime = &fast_clock; // we gaan de pointer doorgeven naar de global, ipv de hele struct te copieren
    
    return (APP_OK);
} // app_GetTime

// te pollen totdat locState != LOCSTATE_UNKNOWN
uint8_t app_GetLocState(uint16_t locAddress, uint8_t *locState) // of GetLocOwner ?
{
    if ((locAddress == app_LocAddress) && (app_LocState != LOCSTATE_UNKNOWN))
    {
        *locState = app_LocState;
        return (APP_OK);
    }
    
    // loc wijzigen in de applayer
    app_LocAddress = locAddress;
    app_LocState = LOCSTATE_UNKNOWN;
    app_LocSpeed = 0;
    app_LocFuncs = 0;
    
    *locState = LOCSTATE_UNKNOWN;
    if (!xpc_IsBusy())
    {
        xpc_send_LocGetInfoRequest(locAddress);
        return (APP_OK);
    }
    else return (APP_BUSY);
    
} // app_GetLocState

//callback!
void xpc_LocGetInfoResponse(uint8_t dccSpeed128, uint16_t locFuncs_f0_f12, bool isLocFree)
{
    app_LocSpeed = dccSpeed128;
    app_LocFuncs = locFuncs_f0_f12; // daarmee worden wel de funcs 13..28 op 0 gezet
    if(isLocFree) app_LocState = LOCSTATE_FREE;
    else app_LocState = LOCSTATE_OWNED;
} // xpc_LocGetInfoResponse

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

// locSpeed moet DCC128 formaat volgen, dwz 0 = stop, 1= noodstop, 2..127 = speedsteps, msb = richting, 1=voorwaarts, 0=achterwaarts
// Command Station gaat over xpnet altijd DCC128 formaat overnemen, en bij gevolg de locdecoder ook
// locdecoder switcht (indien nodig) zonder CV-wijziging automatisch over tussen DCC28/DCC128
// als retval != APP_OK, is de locSpeed niet correct doorgegeven --> de app/ui moet opnieuw proberen
uint8_t app_SetLocSpeed (uint16_t locAddress, uint8_t locSpeed)
{
    if (locAddress != app_LocAddress)
    {
        // dit hoort de UI eigenlijk niet te doen, die moet eerst de locState opvragen, maar goed
        // je kan dan de speed zetten, maar app_LocState blijft LOCSTATE_UNKNOWN tot je een Getxxx aanroept
        app_LocAddress = locAddress;
        app_LocState = LOCSTATE_UNKNOWN;
        app_LocSpeed = 0; // dit is geen invalid speed, maar stop met reverse bit
        app_LocFuncs = 0;
    }
    if (locSpeed != app_LocSpeed)
    {
        if (!xpc_IsBusy())
        {
            xpc_send_LocSetSpeedRequest(app_LocAddress,locSpeed);
            // we krijgen geen feedback van xpnet, dus we gaan ervan uit dat dit is gelukt
            app_LocSpeed = locSpeed;
            return (APP_OK);
        }
        else return (APP_BUSY); // we vangen dit niet hier op, anders moeten we requests kunnen queuen
    }
    return (APP_OK); // we sturen geen speed request voor dezelfde speed, xpnet heeft dit niet graag
    
} // app_SetLocSpeed

// *locSpeed is de huidig gekende speed, niet persé de huidige speed
// als retval != APP_OK moet de app/ui opnieuw requesten
uint8_t app_GetLocSpeed (uint16_t locAddress, uint8_t *locSpeed)
{
    if (locAddress != app_LocAddress)
    {
        app_LocAddress = locAddress;
        app_LocState = LOCSTATE_UNKNOWN;
        app_LocSpeed = 0; // dit is geen invalid speed, maar stop met reverse bit
        app_LocFuncs = 0;
    }
    // we gaan sowieso een nieuwe info request sturen, en in afwachting returnen we de momenteel gekende locSpeed
    *locSpeed = app_LocSpeed;
    if (!xpc_IsBusy())
    {
        xpc_send_LocGetInfoRequest(locAddress);
        return (APP_OK);
    }
    else return (APP_BUSY);
    
} // app_GetLocSpeed

// support voor 32 functies, 1 bit per functie, 29 max gebruikt : fl + f1..f28
// zijn die funcs allemaal geinitialiseerd in locobuffer? bv bij startup uit de database? of moet de commandstation/ui dit doen?
// retval 0xFFFF = locAddr niet in loco buffer
uint8_t app_GetLocFuncs (uint16_t locAddress, uint32_t *locFuncs)
{
    if (locAddress != app_LocAddress)
    {
        app_LocAddress = locAddress;
        app_LocState = LOCSTATE_UNKNOWN;
        app_LocSpeed = 0; // dit is geen invalid speed, maar stop met reverse bit
        app_LocFuncs = 0;
    }
    // we gaan sowieso een nieuwe info request sturen, en in afwachting returnen we de momenteel gekende locFuncs
    *locFuncs = app_LocFuncs;
    if (!xpc_IsBusy())
    {
        xpc_send_LocGetInfoRequest(locAddress);
        return (APP_OK);
    }
    else return (APP_BUSY);

} // app_GetLocFuncs

// 1 bit tegelijk, we kunnen toch maar 1 bit tegelijk togglen in de UI
// func 0 = light
// f1 ->f28 
// on = 1, off = 0
// retval = APP_xxx constant
uint8_t app_SetLocFunc (uint16_t locAddress, uint8_t func, uint8_t onOff) 
{
    uint8_t funcByte, funcGroup;

    onOff = onOff & 0x1;
    if (locAddress != app_LocAddress)
    {
        // dit hoort de UI eigenlijk niet te doen, die moet eerst de locState opvragen, maar goed
        // je kan dan de funcs zetten, maar app_LocState blijft LOCSTATE_UNKNOWN tot je een Getxxx aanroept
        app_LocAddress = locAddress;
        app_LocState = LOCSTATE_UNKNOWN;
        app_LocSpeed = 0; // dit is geen invalid speed, maar stop met reverse bit
        app_LocFuncs = 0;
    }
    if (func > 28) return (APP_INTERNALERR0R);
    
    if ((app_LocFuncs >> func) != onOff) // enkel verschillen naar xpnet doorgeven
    {
        if (!xpc_IsBusy())
        {
            // in afwachting van een betere implementatie van de bitmanipulatie..
            // we krijgen geen feedback van xpnet, dus we gaan ervan uit dat het zal lukken
            if (onOff)
                app_LocFuncs = app_LocFuncs | (1 << func);
            else
                app_LocFuncs = app_LocFuncs & (~(1 << func));
            
            if ((func >= 0) && (func <=4))
            {
                funcGroup = 1;
                funcByte = (uint8_t) (app_LocFuncs & 0x1F);
            }
            else if ((func >= 5) && (func <=8)) 
            {
                funcGroup = 2;
                funcByte = (uint8_t) ((app_LocFuncs >> 5) & 0x0F);
            }
            else if ((func >= 9) && (func <=12))
            {
                funcGroup = 3;
                funcByte = (uint8_t) ((app_LocFuncs >> 9) & 0x0F);
            }
            else if ((func >= 13) && (func <=20))
            {
                funcGroup = 4;
                funcByte = (uint8_t) ((app_LocFuncs >> 13) & 0xFF);
            }
            else if ((func >= 21) && (func <=28))
            {
                funcGroup = 5;
                funcByte = (uint8_t) ((app_LocFuncs >> 21) & 0xFF);
            }               
            xpc_send_LocSetFuncRequest(app_LocAddress,funcGroup, funcByte);
            return (APP_OK);
        }
        else return (APP_BUSY); // we vangen dit niet hier op, anders moeten we requests kunnen queuen
    }
    return (APP_OK); // we sturen geen speed request voor dezelfde speed, xpnet heeft dit niet graag

    } // app_SetLocFunc

// caller to provide memory for *name
// database.cpp : store_loco_name wordt nergens gebruikt
// waar wordt de locname bewaard? in database.cpp, en elders?
void app_GetLocName( uint16_t locAddr, char *locName )
{
    //todo
    strcpy(locName,app_LocName);
    
} // app_GetLocName

// voorlopig hier een geheugen van de wissels
uint32_t app_Turnouts = 0xFFFFFFFF; // alle wissels rechtdoor, 1 bit per turnoutAddr 0..31
// turnoutAddr 0-4095, coil red/green 0/1
// enkel het activate commando wordt gestuurd, de turnout decoder doet zelf de deactivate
// voorlopig turnoutAddr 0..31


bool app_ToggleAccessory (uint16_t turnoutAddr)
{
    bool retval;
    uint8_t coil = ((app_Turnouts >> turnoutAddr) & 0x1) ^0x1;
    
    //Serial.print("toggle accessory : ");
    //Serial.print(turnoutAddr);
    //Serial.print(" coil : ");
    //Serial.print(coil);
/*    
    if (!organizer_ready())
        return (APP_INTERNALERR0R);
    
    retval = do_accessory(1,turnoutAddr,coil,1);
    //Serial.print("retval = ");
    //Serial.println(retval);
    if (!retval)
        app_Turnouts ^= (1 << turnoutAddr);
    //Serial.println(app_Turnouts,HEX);
    return (retval);
*/
    // TODO!!
    return (APP_OK);    
    
} // app_ToggleAccessory


uint8_t app_GetProgResults (uint16_t &cv, uint8_t &cvdata)
{
/*     
    uint8_t retval;
    
    if (prog_event.busy)
        return 1;
    retval = (uint8_t) prog_result;
    cv = prog_cv;
    cvdata = prog_data;
    return (retval);
*/
    // TODO !!!
    return (APP_OK);    
}
