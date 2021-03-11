//----------------------------------------------------------------
//
// OpenDCC
//
// Copyright (c) 2008,2009 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//-----------------------------------------------------------------
//
// file:      xpnet.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
//
// differences to Xpressnet - docu:
// a) There is no request for acknowledge - we act different:
//    if an client creates an error:
//    - it will get an data error message
//    - it will be put back to nearly faded out.
// b) if a client does not answer during 255 calls,
//    it will dynamically fade out to unused, 
//    so there is no need to put it on a watch list  
//      
//-----------------------------------------------------------------
//
// content:   XPressnet Interface (client)
//            Only Xpressnet V3 is supported.
//            no support for Multiheader
// used sw:   
//            rs485.c:      low level rs485 io
//            

//-----------------------------------------------------------------

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>                 // using sscanf and sprintf increases prog. by 4k!
#include <inttypes.h>
#include <avr/pgmspace.h>          // put var to program memory
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#include <string.h>
#include "Arduino.h"
#include "rs485c.h"                 // rx and tx serial if, made for xpressnet
#include "status.h" // de status van de client die door xpnet rx msg wordt geüpdate (bv fast clock, track status, feedback status)
#include "xpc.h"

// zie xpnet spec 2.1.15 : adres 0..99 in 1 byte (AH=0), adres 100..9999 in 2-byte
#define XP_SHORT_ADDR_LIMIT (100)
// SDS voorlopig fixed address!!
#define XPNET_MY_ADDRESS (5)
#define XPNET_BROADCAST_ADDRESS (0)
uint8_t xpc_MyAddress = XPNET_MY_ADDRESS;
bool xpc_Busy = false;


// generell fixed messages
unsigned char xp_datenfehler[] = {0x61, 0x80};             // xor wrong
unsigned char xp_busy[] = {0x61, 0x81};                    // busy
unsigned char xp_unknown[] = {0x61, 0x82};                 // unknown command

// fixed messages client -> Command Station
static unsigned char xpc_AcknowledgementResponse[] = {0x20};                // 2.2.1
static unsigned char xpc_PowerOnRequest[] = {0x21,0x81};                    // 2.2.2 Resume operations request
static unsigned char xpc_PowerOffRequest[] = {0x21,0x80};                   // 2.2.3 (emergency off)
static unsigned char xpc_EmergencyStopRequest[] = {0x80};                   // 2.2.4
static unsigned char xpc_ServiceModeResultsRequest[] = {0x21,0x10};         // 2.2.x
static unsigned char xpc_CommandStationSwVersionRequest[] = {0x21,0x21};    // 2.2.x
static unsigned char xpc_CommandStationStatusRequest[] = {0x21,0x24};       // 2.2.x
static unsigned char xpc_GetFastClockRequest[] = {0x01,0xF2};               // undocumented xpnet

//temp debug serial
#include <SoftwareSerial.h>
extern SoftwareSerial mySerial;


static bool xpc_isConnectionError = false; // als we geen callbytes ontvangen van de commandStation
static uint32_t rxLastMillis, txLastMillis;
// hebben we ook een 'busy' timeout nodig, die bewaakt dat we op tijd een antwoord krijgen op een request?

//===============================================================================
//
// 4. Xpressnet Parser
//
//===============================================================================
//
// 4.a) Xpressnet variables
//
//-------------------------------------------------------------------------------

#define ACK_ID      0x00
#define FUTURE_ID   0x20        // A message with future ID to slot is Feedback broadcast (see also s88.c)
#define CALL_ID     0x40
#define MESSAGE_ID  0x60


unsigned char rx_callbyte;      // current slot (SDS2021)
unsigned char rx_message[17];             // current message from master
unsigned char rx_index;
unsigned char rx_size;

unsigned char tx_message[17];             // current message from client ready to transmit (queue voor 1 msg)

static void xpc_parser();


// ret 0 = identical, 1 = not identical
static uint8_t xpc_compareMessage (uint8_t *msg1, uint8_t *msg2)
{
    uint8_t msgSize; 
    msgSize = (msg1[0] & 0x0F) + 1;
    for (int i=0; i<msgSize; i++)
    {
        if (msg1[i] != msg2[i]) return 1;
    }
    return 0;
} // xpc_compareMessage

static void xpc_QueueMessage (uint8_t *msg)
{
    uint8_t msgSize; 
    msgSize = (msg[0] & 0x0F) + 1;
    strncpy(tx_message,msg,msgSize);
    xpc_Busy = true; // hiermee weet run_xpclient dat er valid data in de queue staan
} // xpc_QueueMessage

// deze functie mag enkel door run_xpclient gecalled worden na ontvangen van een callbyte
// want deze functie gaat onmiddellijk in de UART buffer schrijven en TX enablen
// dat mag enkel als het ons beurt is!
static void xpc_sendMessage(unsigned char *str)
{
    unsigned char n, total, my_xor;
 
    n = 0;
    my_xor = str[0];
    total = str[0] & 0x0F;
    
    while (!XP_tx_ready()) ;                 // busy waiting! (but shouldn't happen)

    XP_send_byte(str[0]);                    // send header

    while (n != total)
    {
         n++;
         my_xor ^= str[n];
         XP_send_byte(str[n]);              // send data
    }    
    XP_send_byte(my_xor);                   // send xor

    txLastMillis = millis(); // tx timeout manager
    // SDS2021 - om te vermijden dat de xpnet-tx buiten het slot valt doen we de serial.print op het eind!
    // altserial is allicht ook int gestuurd, want de eerste xpc_sendMessage viel mooi binnen slot
    // maar sommige niet ->TODO !!
    mySerial.println("xpc_sendMessage");
} // xpc_sendMessage


//===============================================================================
//
// 4. Xpressnet CLIENT 
//
//===============================================================================
// Timeouts: SLOT_TIMEOUT: We wait this time after an INQUIRY for the first response
//           RX_TIMEOUT:   We wait this time for the complete message (error timeout)

#define RX_TIMEOUT          500
#define TX_TIMEOUT          5 
#define CONNECTION_TIMEOUT  500

#define ACK_ID      0x00
#define FUTURE_ID   0x20        // A message with future ID to slot is Feedback broadcast (see also s88.c)
#define CALL_ID     0x40
#define MESSAGE_ID  0x60
#define CALLBYTE_TYPE_FILTER 0x60 //SDS
// internal and static

enum xpc_states
{                                   // actual state for the Xpressnet Task
    XPC_INIT,
    XPC_WAIT_FOR_CALL,              // wait for call byte (my_addr or broadcast addr)
    XPC_WAIT_FOR_MESSAGE,           // msg from command station, wait for 1st byte to know the msg length
    XPC_WAIT_FOR_MESSAGE_COMPLETE,  // read all bytes in msg
    XPC_WAIT_FOR_TX_COMPLETE,       // Our request complete sent?
    XPC_WAIT_FOR_REPLY              // reply from CS or new call window?
} xpc_state;

void xpc_send_PowerOnRequest ()
{
    xpc_QueueMessage(xpc_PowerOnRequest);
}
void xpc_send_PowerOffRequest ()
{
    xpc_QueueMessage(xpc_PowerOffRequest);
}
void xpc_send_EmergencyStopRequest ()
{
    xpc_QueueMessage(xpc_EmergencyStopRequest);
}
void xpc_send_ServiceModeResultsRequest ()
{
    xpc_QueueMessage(xpc_ServiceModeResultsRequest);
}
void xpc_send_CommandStationStatusRequest ()
{
    xpc_QueueMessage(xpc_CommandStationStatusRequest);
} // xpc_send_CommandStationStatusRequest

void xpc_send_CommandStationSwVersionRequest ()
{
    xpc_QueueMessage(xpc_CommandStationSwVersionRequest);
} // xpc_send_CommandStationSwVersionRequest

// cv : 1..1024, 1024 wordt als 0 getransmit
void xpc_send_DirectModeCVReadRequest(uint16_t cvAddr)
{
    // 0x22 0x15 CV X-Or --> oud formaat, voor CV1..256
    // 0x22 0x18 en verder voor CV1..1024
    cvAddr = cvAddr & 0x3FF; // houd 10 bits over van de cv, en 1024 wordt 0)
    tx_message[0] = 0x22;
    tx_message[1] = 0x18 | ((cvAddr >> 8) & 0x3); // hoogste 2 bits van de CV
    tx_message[2] = cvAddr & 0xFF; // laagste 8 bits van de CV
    xpc_Busy = true;
    
} // xpc_send_DirectModeCVReadRequest

// cv : 1..1024, 1024 wordt als 0 getransmit
void xpc_send_DirectModeCVWriteRequest(uint16_t cvAddr, uint8_t cvData)
{
    // 0x23 0x16 CV X-Or --> oud formaat, voor CV1..256
    // 0x23 0x1C en verder voor CV1..1024
    cvAddr = cvAddr & 0x3FF; // houd 10 bits over van de cv, en 1024 wordt 0)
    tx_message[0] = 0x23;
    tx_message[1] = 0x1C | ((cvAddr >> 8) & 0x3); // hoogste 2 bits van de CV
    tx_message[2] = cvAddr & 0xFF; // laagste 8 bits van de CV
    tx_message[3] = cvData;
    xpc_Busy = true;
    
} // xpc_send_DirectModeCVWriteRequest

// cv : 1..1024, (wordt hier op xpnet als cv-1 doorgestuurd)
void xpc_send_PomCVWriteRequest(uint16_t pomAddress, uint16_t cvAddress, uint8_t cvData, uint8_t pomWriteMode)
{
    // 0xE6 0x30 AH AL C CV D
    tx_message[0] = 0xE6;
    tx_message[1] = 0x30;
    tx_message[2] = (uint8_t) ((pomAddress & 0xFF00)>>8); // hoogste 8-bits van  decoder address
    tx_message[3] = (uint8_t) (pomAddress & 0xFF); // laagste 8-bits van decoder address
    if (pomAddress > XP_SHORT_ADDR_LIMIT)
    {
        tx_message[2] |= 0xC0;
    }
    cvAddress = cvAddress-1;

    tx_message[4] = pomWriteMode + ((cvAddress>>8) & 0x3); // hoogste 2 bits van cvAddress
    tx_message[5] = cvAddress & 0xFF; // laagste 8 bits van cvAddress
    tx_message[6] = cvData;
    xpc_Busy = true;
    
} // xpc_send_PomCVWriteRequest

// decAddr : 0..255, 
// schakeldecoder :  elk decAddr heeft 8 outputs (4wissels)
// feedbackdecoder : elk decAddr heeft 8 inputs
// met InfoRequest kan je maar 4 bits tegelijk opvragen, nibble : 0=laagste 4 bits, 1=hoogste 4 bits
// de applayer moet de requests organiseren om alle nodige info te bekomen dmv verschillende func calls
void xpc_send_AccessoryDecoderInfoRequest(uint8_t decAddr, uint8_t nibble)
{
    // 0x42 Addr Nibble X-Or
    tx_message[0] = 0x42;
    tx_message[1] = decAddr;
    tx_message[2] = 0x80 + (nibble & 0x1);
    xpc_Busy = true;
    
} // xpc_send_AccessoryDecoderInfoRequest

// dit commando is enkel voor schakeldecoders!
// turnoutAddr : 0..1023 (10bits) 
// -> decAddr = hoogste 8-bits van het turnoutAddr
// -> turnout = laagste 2-bits van het turnoutAddr (elke switching decoder controleert 4 wissels)
// position : 0 = rechtdoor (output 1), 1 = afslaan (output 2)
// de sds switching decoder gaat ervoor zorgen dat de 2 outputs van een paar nooit tegelijk aan staan
// een 'off' commando op de gepairde output hoeft dus niet (activate bit altijd 1)
void xpc_send_SetTurnoutRequest(uint8_t turnoutAddr, uint8_t turnoutPosition)
{
    // 0x52 Addr 1000DBBD X-Or
    tx_message[0] = 0x52;
    tx_message[1] = (uint8_t)((turnoutAddr >> 2) & 0xFF);
    tx_message[2] = 0x88 + ((turnoutAddr & 0x3) < 1) + (turnoutPosition & 0x1); // activate bit = 1 altijd
    xpc_Busy = true;
    
} // xpc_send_SetTurnoutRequest

// undocumented xpnet, maar aanwezig in opendcc
// geen command station response
// signalAdress : 11 bits
// signalAspect : 5 bits code
void xpc_send_SetSignalAspectRequest(uint16_t signalAddress, uint8_t signalAspect)
{
    // 0x13 0x01 B+AH AL X-Or
    tx_message[0] = 0x13;
    tx_message[1] = 0x01;
    signalAspect &= 0x1F; // houd 5 bits over just in case
    signalAddress &= 0x7FF;
    tx_message[2] = (signalAspect << 3) | (uint8_t)(signalAddress>>8); // signal aspect & higher 3bits of signalAddress
    tx_message[3] = signalAddress & 0xFF; // lower 8bits of signalAddress
    xpc_Busy = true;
    
} // xpc_send_SetSignalAspectRequest

// locAddr : 0 -> 9999 (xpnet range)
// de CS antwoordt met speed info, en wie de loc bestuurt
void xpc_send_LocGetInfoRequest(uint16_t locAddr)
{
    // 0xE3 0x00 Addr-H Addr-L X-Or
    tx_message[0] = 0xE3;
    tx_message[1] = 0x00;
    tx_message[2] = (uint8_t) ((locAddr & 0xFF00)>>8); // hoogste 8-bits van locAddr
    tx_message[3] = (uint8_t) (locAddr & 0xFF); // laagste 8-bits van locAddr
    if (locAddr > XP_SHORT_ADDR_LIMIT)
    {
        tx_message[2] |= 0xC0;
    }
    xpc_Busy = true;
    
} // xpc_send_LocGetInfoRequest

// locAddr : 0 -> 9999 (xpnet range)
// voor functies F13 -> F28
void xpc_send_LocGetFuncStatus_F13_F28_Request(uint16_t locAddr)
{
    // 0xE3 0x08 Addr-H Addr-L X-Or
    tx_message[0] = 0xE3;
    tx_message[1] = 0x09;
    tx_message[2] = (uint8_t) ((locAddr & 0xFF00)>>8); // hoogste 8-bits van locAddr
    tx_message[3] = (uint8_t) (locAddr & 0xFF); // laagste 8-bits van locAddr
    if (locAddr > XP_SHORT_ADDR_LIMIT)
    {
        tx_message[2] |= 0xC0;
    }
    xpc_Busy = true;
    
} // xpc_send_LocGetFuncStatus_F13_F28_Request

// locAddr : 0 -> 9999 (xpnet range)
// voor functies F0 -> F12
void xpc_send_LocGetFuncModeRequest(uint16_t locAddr)
{
    // 0xE3 0x07 Addr-H Addr-L X-Or
    tx_message[0] = 0xE3;
    tx_message[1] = 0x07;
    tx_message[2] = (uint8_t) ((locAddr & 0xFF00)>>8); // hoogste 8-bits van locAddr
    tx_message[3] = (uint8_t) (locAddr & 0xFF); // laagste 8-bits van locAddr
    if (locAddr > XP_SHORT_ADDR_LIMIT)
    {
        tx_message[2] |= 0xC0;
    }
    xpc_Busy = true;
    
} // xpc_send_LocGetFuncModeRequest

// locAddr : 0 -> 9999 (xpnet range)
// voor functies F13 -> F28
void xpc_send_LocGetFuncMode_F13_F28_Request(uint16_t locAddr)
{
    // 0xE3 0x08 Addr-H Addr-L X-Or
    tx_message[0] = 0xE3;
    tx_message[1] = 0x08;
    tx_message[2] = (uint8_t) ((locAddr & 0xFF00)>>8); // hoogste 8-bits van locAddr
    tx_message[3] = (uint8_t) (locAddr & 0xFF); // laagste 8-bits van locAddr
    if (locAddr > XP_SHORT_ADDR_LIMIT)
    {
        tx_message[2] |= 0xC0;
    }
    xpc_Busy = true;
    
} // xpc_send_LocGetFuncMode_F13_F28_Request

// locAddr : 0 -> 9999 (xpnet range)
// we gaan 128speed steps gebruiken in de client (DCC128)
void xpc_send_LocSetSpeedRequest(uint16_t locAddr, uint8_t locSpeed)
{
    // 0xE4 0x13 Addr-H Addr-L  RV X-Or
    tx_message[0] = 0xE4;
    tx_message[1] = 0x13;
    tx_message[2] = (uint8_t) ((locAddr & 0xFF00)>>8); // hoogste 8-bits van locAddr
    tx_message[3] = (uint8_t) (locAddr & 0xFF); // laagste 8-bits van locAddr
    if (locAddr > XP_SHORT_ADDR_LIMIT)
    {
        tx_message[2] |= 0xC0;
    }
    tx_message[4] = locSpeed;
    xpc_Busy = true;
    
} // xpc_send_LocSetSpeedRequest

// locAddr : 0 -> 9999 (xpnet range)
// per func grp, de applayer moet zorgen dat alle bits in een grp correct worden doorgegeven!!
// grp=1 (F0..F4), 2 (F5..F8), 3 (F9..F12), 4 (F13..F20), 5 (F21..F28)
void xpc_send_LocSetFuncRequest(uint16_t locAddr, uint8_t grp, uint8_t locFuncs)
{
    if ((grp > 5) || (grp ==0))
        return;
    // 0xE4 0x20 Addr-H Addr-L  RV X-Or
    tx_message[0] = 0xE4;
    
    tx_message[1] = 0x20 + (grp - 1);
    if (grp == 5) tx_message[1] = 0x28;
    else tx_message[1] = 0x20 + (grp - 1); // cases grp=1..4

    tx_message[2] = (uint8_t) ((locAddr & 0xFF00)>>8); // hoogste 8-bits van locAddr
    tx_message[3] = (uint8_t) (locAddr & 0xFF); // laagste 8-bits van locAddr
    if (locAddr > XP_SHORT_ADDR_LIMIT)
    {
        tx_message[2] |= 0xC0;
    }
    tx_message[4] = locFuncs;
    xpc_Busy = true;
    
} // xpc_send_LocSetFuncRequest

void xpc_send_FindNextLocAddress (uint16_t locAddr, uint8_t searchDirection)
{
    searchDirection &= 0x1; // 1-bit
    tx_message[0] = 0xE3;
    tx_message[1] = 0x05 + searchDirection;
    tx_message[2] = (uint8_t) ((locAddr & 0xFF00)>>8); // hoogste 8-bits van locAddr
    tx_message[3] = (uint8_t) (locAddr & 0xFF); // laagste 8-bits van locAddr
    if (locAddr > XP_SHORT_ADDR_LIMIT)
    {
        tx_message[2] |= 0xC0;
    }
    xpc_Busy = true;
} // xpc_send_FindNextLocAddress

void xpc_send_DeleteLocAddress (uint16_t locAddr)
{
    tx_message[0] = 0xE3;
    tx_message[1] = 0x44;
    tx_message[2] = (uint8_t) ((locAddr & 0xFF00)>>8); // hoogste 8-bits van locAddr
    tx_message[3] = (uint8_t) (locAddr & 0xFF); // laagste 8-bits van locAddr
    if (locAddr > XP_SHORT_ADDR_LIMIT)
    {
        tx_message[2] |= 0xC0;
    }
    xpc_Busy = true;
} // xpc_send_DeleteLocAddress

void xpc_send_GetFastClock ()
{
    xpc_QueueMessage(xpc_GetFastClockRequest);
} // xpc_send_GetFastClock

// sds : dit is om vanuit de client de fast clock te kunnen wijzigen, eventueel houden
void xpc_send_SetFastClock(t_fast_clock* newFastClock)
{
    // 0x05 0x01 TCODE0 {TCODE1 TCODE2 TCODE3} 
    tx_message[0] = 0x05;
    tx_message[1] = 0xF1;
    tx_message[2] = 0x00 | newFastClock->minute;
    tx_message[3] = 0x80 | newFastClock->hour;
    tx_message[4] = 0x40 | newFastClock->day_of_week;
    tx_message[5] = 0xC0 | newFastClock->ratio;

    xpc_Busy = true;
} // xpc_send_SetFastClock

static void xpc_parser()
{
    mySerial.print("parser msg ! ");
    mySerial.print("rx_size = ");
    mySerial.print(rx_size);
    mySerial.print(" - rx_index = ");
    mySerial.println(rx_index);

    for (int i=0; i <= (rx_message[0]& 0x0F);i++)
    {
        mySerial.print(rx_message[i],HEX);mySerial.print(" - ");
    }
    mySerial.println();
    
    // rx_message[0] = header byte (de callbyte is al gestript door xpc_run)
    switch(rx_message[0] >> 4) 
    {
        case 0x0:
            switch(rx_message[1])
            {
                case 0xF1:
                    // set clock
                    for(uint8_t field = 2; field <= (rx_message[0] & 0x0F); field++ )  
                    {
                        uint8_t val = 99;
                        switch(rx_message[field] & 0xC0)
                        {
                            case 0x00:  val = rx_message[field] & 0x3F;
                                        if (val < 60) fast_clock.minute = val;
                                        break;
                            case 0x80:  val = rx_message[field] & 0x3F;
                                        if (val < 24) fast_clock.hour = val;
                                        break;
                            case 0x40:  val = rx_message[field] & 0x3F;
                                        if (val < 7) fast_clock.day_of_week = val;
                                        break;
                            case 0xC0:  val = rx_message[field] & 0x3F;
                                        if (val < 32) fast_clock.ratio = val;
                                        break;
                        }
                    } 
                    if (xpc_FastClockUpdated) xpc_FastClockUpdated();
                    break;
                case 0xF2:
                    // command station response komt als 0x05 0xF1 ...
                    break;
              }
            break;
        case 0x01 :
            break;
        case 0x04 : //accessory decoder info response & feedback broadcast!!
            if ((rx_message[0] & 0xF) == 2)
            {
                // voor het speciaal geval van 1 nibble hebben we voorlopig een aparte callback implementatie
                // dit is ook de response op een accdecoderInfoRequest
                uint8_t tmp = rx_message[2];
                if (xpc_AccessoryDecoderInfoResponse)
                    xpc_AccessoryDecoderInfoResponse (rx_message[1], ((tmp>>4) & 0x1), ((tmp>>5) & 0x3), (tmp & 0xF));
            }
            else
            {
                // TODO : betere functie definitie van deze callback!
                if (xpc_AccessoryDecoderInfoBC)
                    xpc_AccessoryDecoderInfoBC(rx_message[0] & 0xF, rx_message+1);
            }
            break;
        
        case 0x06 :
            if (rx_message[1] == 0x00)
            {
                if (xpc_EventNotify) xpc_EventNotify (XPEVENT_POWER_OFF);
                mySerial.println("track power off!"); // main track disabled
            }
            else if (rx_message[1] == 0x01)
            {
                if (xpc_EventNotify) xpc_EventNotify (XPEVENT_POWER_ON);
                mySerial.println("normal operation resumed!"); // normal operation resumed 
            }
            else if (rx_message[1] == 0x02)
            {
                if (xpc_EventNotify) xpc_EventNotify (XPEVENT_SERVICE_MODE_ON);
                mySerial.println("service mode entry!");
            }
            else if (rx_message[1] == 0x10) // register mode programming results (eventueel implementeren voor pcintf module)
            {
            }
            else if (rx_message[1] == 0x12)
            {
                if (xpc_ProgStatusResponse) xpc_ProgStatusResponse (PROG_SHORT);
                mySerial.println("proginfo: short!");
            }
            else if (rx_message[1] == 0x13)
            {
                if (xpc_ProgStatusResponse) xpc_ProgStatusResponse (PROG_NOACK);
                mySerial.println("proginfo: no ack!");
            }
            else if (rx_message[1] == 0x1F)
            {
                if (xpc_ProgStatusResponse) xpc_ProgStatusResponse (PROG_BUSY);
                mySerial.println("proginfo: busy!");
            }
            else if (rx_message[1] == 0x11)
            {
                if (xpc_ProgStatusResponse) xpc_ProgStatusResponse (PROG_READY);
                mySerial.println("proginfo: ready!");
            }
            else if (rx_message[1] == 0x14)
            {
                uint16_t cvAddr;
                cvAddr = ((rx_message[1] - 0x14) & 0x3) << 8;
                cvAddr |= rx_message[2];
                if (cvAddr == 0) cvAddr = 1024;
                if (xpc_ProgResultResponse) xpc_ProgResultResponse (cvAddr,rx_message[3]);
                mySerial.println("progresults!");
            }
            else if (rx_message[1] == 0x21) // command station sw version response
            {
                if (xpc_CommandStationSwVersionResponse) xpc_CommandStationSwVersionResponse (rx_message[2]);
                mySerial.println("command station sw version response!");
            }
            else if (rx_message[1] == 0x22) // command station status response
            {
                if (xpc_CommandStationStatusResponse) xpc_CommandStationStatusResponse (rx_message[2]);
                mySerial.println("command station status response!");
            }
            break;
        case 0x07 :
            // TODO : hier heeft opendcc een aantal bidi functies
            break;
        case 0x0A :
        case 0x0B :
            // dit zijn V1 & V2 commando's, doen we niet
            break;
        case 0x0C :
            // TODO : double header functies, eventueel voor pcintf module, maar dit gaan we toch niet gebruiken allicht
            break;
        case 0x0E :
            if ((rx_message[1] & 0xF0) == 0x30) // address retrieval response, KKKK gebruiken we voorlopig niet volledig
            {
                uint16_t locAddr;
                if ((rx_message[1] & 0x0F) == 0x4) // KKKK=4 : not found
                    locAddr = 10000; // use an invalid address as return value
                else
                    locAddr = ((rx_message[2] & 0x3F) << 8) + rx_message[3];
                if (xpc_FindNextLocAddressResponse)
                    xpc_FindNextLocAddressResponse(locAddr);
                
            }
            else if (rx_message[1] == 0x40)
            {
                uint16_t locStolenAddr;
                locStolenAddr = ((rx_message[2] & 0x3F) << 8) + rx_message[3];
                if ((locStolenAddr >=0) && (locStolenAddr < 10000) &&  (xpc_LocStolenNotify)) 
                    xpc_LocStolenNotify(locStolenAddr);
            }
            else if (rx_message[1] == 0x52) // function status F13..F28 response
            {
                uint16_t f13_f28;
                f13_f28 = (uint16_t) rx_message[3];
                f13_f28 = (f13_f28 << 8) | rx_message[2];
                if (xpc_LocGetFuncStatus_F13_F28_Response)
                    xpc_LocGetFuncStatus_F13_F28_Response(f13_f28);
            }
            else if ((rx_message[1] & 0xF0) == 0x00) // loc information normal locomotive 2.1.14.1
            {
                uint8_t dccSpeed128;
                uint16_t locFuncs;
                dccSpeed128 = rx_message[2]; // todo : uitbreiden als de speedstep != 128, herrekenen naar DCC128
                locFuncs = ((uint16_t)rx_message[4]) << 8;
                locFuncs |= (rx_message[3] << 1) | ((rx_message[3] & 0x10) >> 4); // func0 op bit0 zetten
                if (xpc_LocGetInfoResponse)
                    xpc_LocGetInfoResponse(dccSpeed128, locFuncs, ((rx_message[1] & 0x8)== 0) );
            }
            else if ((rx_message[1] & 0xF0) == 0x01) // loc information for a loc in multi-unit 2.1.14.2 --> TODO
            {
            }
            else if ((rx_message[1] & 0xF0) == 0x02) // loc information for multi-unit address 2.1.14.3 --> TODO
            {
            }
            else if ((rx_message[1] & 0xF0) == 0x06) // loc information for a loc in double header 2.1.14.4 --> TODO
            {
            }
            break;
        default :
            break;
    }
  
} // xpc_parser

void init_xpclient(void)
{
    xpc_state = XPC_INIT;
} // init_xpclient

void run_xpclient(void)
{
    if ((!xpc_isConnectionError) & (millis() - rxLastMillis > CONNECTION_TIMEOUT))
    {
        xpc_Busy = false; // cancel ongoing transaction
        xpc_isConnectionError = true;
        xpc_state = XPC_WAIT_FOR_CALL;
        if (xpc_EventNotify) xpc_EventNotify(XPEVENT_CONNECTION_ERROR);
    }
    
    switch (xpc_state)
    {
        case XPC_INIT:
            xpc_state = XPC_WAIT_FOR_CALL;
            break;
        case XPC_WAIT_FOR_CALL:
            if (XP_rx_ready())
            {
                digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN)); // SDS2021 toggle led for test
                unsigned int tmp = XP_rx_read(); // the call byte ?
                rxLastMillis = millis(); // rx timeout manager
                if (tmp & 0x100) // het is een callbyte
                {
                    rx_callbyte = (uint8_t) (tmp & 0xFF);
                }
                else return; // we hebben een byte in het midden van een conversatie opgepikt; we gaan hersynchroniseren en wachten op een call byte
                
                xpc_isConnectionError = false; // we hebben contact met CommandStation
                if ((rx_callbyte & 0x1F) == XPNET_BROADCAST_ADDRESS)
                {
                    xpc_state = XPC_WAIT_FOR_MESSAGE; // deze state gaat de call byte overschrijven in rx_message[0]
                }
                // filteren op eigen address
                else if ((rx_callbyte & 0x1F) == xpc_MyAddress)
                {
                    if ((rx_callbyte & CALLBYTE_TYPE_FILTER) == CALL_ID)
                    {
                        if (xpc_Busy)
                        {
                            // er staat een msg in de queue klaar, we kunnen nu versturen!
                            xpc_sendMessage(tx_message);
                            xpc_state = XPC_WAIT_FOR_TX_COMPLETE;
                        }
                    }
                    else if ((rx_callbyte & CALLBYTE_TYPE_FILTER) == ACK_ID)
                    {
                        // de ack request kunnen we direct hier afhandelen (zonder parser)
                        // command station verwacht een acknowledge
                        // TODO : er is dus allicht iets fout gegaan bij onze vorige request : opletten dat de UI hier niet op blokkeert
                        // want de uitstaande request gaat dus geen normale reply krijgen!!
                        // we zouden ook een transfer errors boodschap moeten gekregen hebben vd command station (§2.1.8)
                        xpc_sendMessage(xpc_AcknowledgementResponse);
                        xpc_state = XPC_WAIT_FOR_TX_COMPLETE;
                    }
                    else  if ((rx_callbyte & CALLBYTE_TYPE_FILTER) == MESSAGE_ID)
                    {
                        // we moeten eerst het volledig msg ontvangen, en dan parsen
                        xpc_state = XPC_WAIT_FOR_MESSAGE;
                    }
                }
            }
            break;

        case XPC_WAIT_FOR_MESSAGE: 
            if (XP_rx_ready())
            {
                rx_message[0] = (unsigned char) (XP_rx_read() & 0xFF);           // save header
                rx_size = rx_message[0] & 0x0F;         // length is without xor 
                rx_size++;                              // now including xor; (er volgen nog rx_size bytes na de header : rx_message[1] .. rx_message[rx_size];  rx_message[rx_size] = XOR
                rx_index = 1;        
                xpc_state = XPC_WAIT_FOR_MESSAGE_COMPLETE;
            }
            break;

        case XPC_WAIT_FOR_MESSAGE_COMPLETE:
            if ((millis() - rxLastMillis) >= RX_TIMEOUT)
            {
                // discard msg in rx_message and go back to  XPC_WAIT_FOR_CALL
                if (xpc_EventNotify) xpc_EventNotify(XPEVENT_RX_TIMEOUT);
                xpc_state = XPC_WAIT_FOR_CALL;
                xpc_Busy = false; // applayer moet request opnieuw doen
            }
            else
            {
                if (XP_rx_ready())
                {
                    rx_message[rx_index] = (unsigned char) (XP_rx_read() & 0xFF);
                    if (rx_index == rx_size)
                    {
                        unsigned char i, my_check = 0;
                        // all data and xor read, now check xor
                        for (i=0; i<=rx_size; i++) my_check ^= rx_message[i];   
            
                        if (my_check == 0)
                        {
                            // packet is received and okay, now parse it
                            // sds : eerst checken of we het antwoord in xpc kunnen afhandelen
                            // 1. data error ?
                            if (!xpc_compareMessage(rx_message,xp_datenfehler))
                            {
                                // je hebt een data error gestuurd
                                // wacht op de request acknowledge van CommandStation alvorens opnieuw te sturen
                                // busy flag blijft true; de xpc zal de retransmit doen
                                if (xpc_EventNotify) xpc_EventNotify(XPEVENT_TX_ERROR);
                                xpc_state = XPC_WAIT_FOR_CALL;
                                return;
                            }
                            // 2. unknown command ? (commando niet ondersteund door command station)
                            if (!xpc_compareMessage(rx_message,xp_unknown))
                            {
                                if (xpc_EventNotify) xpc_EventNotify(XPEVENT_UNKNOWN_COMMAND);
                                xpc_state = XPC_WAIT_FOR_CALL;
                                xpc_Busy = false;
                                return;
                            }
                            // 3. busy ? Command station kan efkes geen nieuwe commando's verdragen --> wacht met retries?
                            if (!xpc_compareMessage(rx_message,xp_busy))
                            {
                                // we gaan gewoon de volgende call byte afwachten en dan opnieuw proberen (busy blijft true)
                                xpc_state = XPC_WAIT_FOR_CALL;
                                return;
                            }
                            // else
                            xpc_parser();
                            xpc_state = XPC_WAIT_FOR_CALL; 
                            // xpc_Busy niet wijzigen hier! (dit kan een broadcast zijn terwijl de caller ondertussen een xpc request heeft gequeued )
                            // SDS2021 -> we houden callbyte bij, zodat we hier wel xpc_Busy kunnen resetten
                            // we hebben een geldige message gekregen, dus gaan ervan uit dat dat het antwoord is op de request die we hebben gestuurd
                            if ((rx_callbyte & 0x1F) == xpc_MyAddress) xpc_Busy = false;
                        }
                        else
                        {
                            // XOR is wrong! --> discard (volgens xpnet specification)
                            if (xpc_EventNotify) xpc_EventNotify(XPEVENT_RX_ERROR);
                            xpc_Busy = false;
                            xpc_state = XPC_WAIT_FOR_CALL;
                        }
                    }
                    else
                    {
                        rx_index++;
                        if (rx_index == 17-1)
                        {
                            rx_index = 17-1;   // overrun!
                            if (xpc_EventNotify) xpc_EventNotify(XPEVENT_MSG_ERROR);
                            xpc_Busy = false;
                            xpc_state = XPC_WAIT_FOR_CALL;
                        }
                    }
                }
            }           
            break;

        case XPC_WAIT_FOR_TX_COMPLETE :
            if (XP_is_all_sent())
            {
                xpc_state = XPC_WAIT_FOR_REPLY;
            }
            else if ((millis() - txLastMillis) > TX_TIMEOUT)
            {
                //reset
                 if (xpc_EventNotify) xpc_EventNotify(XPEVENT_TX_TIMEOUT);
                 xpc_state = XPC_WAIT_FOR_CALL;
                 xpc_Busy = false;
            }
            break;
            
        case XPC_WAIT_FOR_REPLY : 
            /* SDS 2021 : reply komt in een msg met callbyte naar ons slot
                dus kunnen we gewoon afhandelen via XPC_WAIT_FOR_CALL
                TODO : nog eens goed checken!
            */
            xpc_state = XPC_WAIT_FOR_CALL;
           /*
            if (XP_rx_ready())
            {
                unsigned int tmp = XP_rx_peek(); // the call byte ?
                rxLastMillis = millis(); // rx timeout manager
                if (tmp & 0x100) // het is een callbyte --> er is een nieuw slot begonnen
                {
                    xpc_state = XPC_WAIT_FOR_CALL;
                }
                else // we krijgen een reply --> lezen!
                {
                    xpc_state = XPC_WAIT_FOR_MESSAGE;
                    xpc_Busy = false; // we kunnen een nieuwe request opnemen terwijl het antwoord op de vorige request binnenkomt
                }
            }
            */
            break;
    }
} // run_xpclient
