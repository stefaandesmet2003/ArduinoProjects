//----------------------------------------------------------------
//
// Xpressnet client
//
//-----------------------------------------------------------------
// TODO : voorlopig doen we maar 1 msg tegelijk naar de TxBuffer
// TODO : je kan zo snel naar de TxBuffer schrijven als de rs485 toelaat, zonder tijd te laten aan xpc_Run om de RxBuffer te lezen
// --> dat geeft wel RX_ERRORS in xpc_Run, waardoor je broadcasts of replies kan missen

//#define XPC_DEBUG // do serial prints on myserial software serial interface (to be initiated by app)
#undef XPC_DEBUG

#include "Arduino.h"
#include "rs485c.h" // rx and tx serial interface, made for xpressnet
#include "xpc.h"

#ifdef XPC_DEBUG
#include <SoftwareSerial.h>
extern SoftwareSerial mySerial;
#endif

// xpnet definitions
#define XPNET_BROADCAST_ADDRESS (0)

#define CALL_TYPE_ACK     0x00
#define CALL_TYPE_FUTURE  0x20 // A message with future ID to slot is Feedback broadcast (see also s88.c)
#define CALL_TYPE_INQUIRY 0x40
#define CALL_TYPE_MESSAGE 0x60
#define CALL_TYPE         0x60

#define RX_TIMEOUT          500
#define CONNECTION_TIMEOUT  500

typedef enum {                                   // actual state for the Xpressnet Task
    XPC_INIT,
    XPC_WAIT_FOR_CALL,              // wait for call byte (my_addr or broadcast addr)
    XPC_WAIT_FOR_MESSAGE,           // msg from command station, wait for 1st byte to know the msg length
    XPC_WAIT_FOR_MESSAGE_COMPLETE,  // read all bytes in msg
} xpcState_t;

// zie xpnet spec 2.1.15 : adres 0..99 in 1 byte (AH=0), adres 100..9999 in 2-byte
#define XP_SHORT_DCC_ADDR_LIMIT (100)

static uint8_t xpc_MyAddress = 0; // 0 = broadcast address, so is safe
static xpcState_t xpc_state;

// fixed messages
// TODO : hoe zorgen we ervoor dat die in ROM zitten, ipv RAM ?

// fixed messages Command Station -> client
uint8_t xp_datenfehler[] = {0x61,0x80};             // xor wrong
uint8_t xp_busy[] = {0x61,0x81};                    // busy
uint8_t xp_unknown[] = {0x61,0x82};                 // unknown command

// fixed messages client -> Command Station
static uint8_t xpc_AcknowledgementResponse[] = {0x20};                // 2.2.1
static uint8_t xpc_PowerOnRequest[] = {0x21,0x81};                    // 2.2.2 Resume operations request
static uint8_t xpc_PowerOffRequest[] = {0x21,0x80};                   // 2.2.3 (emergency off)
static uint8_t xpc_EmergencyStopRequest[] = {0x80};                   // 2.2.4
static uint8_t xpc_ServiceModeResultsRequest[] = {0x21,0x10};         // 2.2.x
static uint8_t xpc_CommandStationSwVersionRequest[] = {0x21,0x21};    // 2.2.x
static uint8_t xpc_CommandStationStatusRequest[] = {0x21,0x24};       // 2.2.x
static uint8_t xpc_GetFastClockRequest[] = {0x01,0xF2};               // undocumented xpnet

static bool xpc_isConnectionError = false; // als we geen callbytes ontvangen van de commandStation
static uint32_t rxLastMillis;

uint8_t tx_message[17]; // current message from client ready to transmit (queue voor 1 msg)
uint8_t rx_message[17]; // current message from master
uint8_t rx_index;
uint8_t rx_size;

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

// deze functie copieert de msg naar de rs485 transmit buffer
// rs485c zal de transmit starten na een callbyte
void xpc_SendMessage(uint8_t *msg)
{
  uint8_t n, total, my_xor;

  n = 0;
  my_xor = msg[0];
  total = msg[0] & 0x0F;

  // TODO : voorlopig doen we maar 1 msg tegelijk naar de TxBuffer
  // we doen busy wait totdat de vorige msg is verstuurd
  while (!XP_tx_empty());

  // alternatief : hiermee kunnen we meerdere msg in de TxBuffer steken:
  // while (!XP_tx_ready());

  // we copy all msg bytes to the TxBuffer under disabled ints because
  // we had occasional data errors when the first byte was in transmission (UDRE) before all bytes of the msg were copied to rs485c
  // UDRE int comes immediately after the 1st byte, and UDRIE=0.
  disableInterrupts(); 
  XP_send_byte(msg[0]); // send header
  while (n != total) {
    n++;
    my_xor ^= msg[n];
    XP_send_byte(msg[n]); // send data
  }    
  XP_send_byte(my_xor); // send xor
  enableInterrupts();
  #ifdef XPC_DEBUG
    mySerial.println("xpc_SendMessage");
  #endif
} // xpc_SendMessage

void xpc_send_PowerOnRequest ()
{
  xpc_SendMessage(xpc_PowerOnRequest);
}
void xpc_send_PowerOffRequest ()
{
  xpc_SendMessage(xpc_PowerOffRequest);
}
void xpc_send_EmergencyStopRequest ()
{
  xpc_SendMessage(xpc_EmergencyStopRequest);
}
void xpc_send_ServiceModeResultsRequest ()
{
  xpc_SendMessage(xpc_ServiceModeResultsRequest);
}
void xpc_send_CommandStationStatusRequest ()
{
  xpc_SendMessage(xpc_CommandStationStatusRequest);
} // xpc_send_CommandStationStatusRequest

void xpc_send_CommandStationSwVersionRequest ()
{
  xpc_SendMessage(xpc_CommandStationSwVersionRequest);
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
  xpc_SendMessage(tx_message);
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
  xpc_SendMessage(tx_message);
} // xpc_send_DirectModeCVWriteRequest

// cv : 1..1024, (wordt hier op xpnet als cv-1 doorgestuurd)
void xpc_send_PomCVWriteRequest(uint16_t pomAddress, uint16_t cvAddress, uint8_t cvData, uint8_t pomWriteMode)
{
  // 0xE6 0x30 AH AL C CV D
  tx_message[0] = 0xE6;
  tx_message[1] = 0x30;
  tx_message[2] = (uint8_t) ((pomAddress & 0xFF00)>>8); // hoogste 8-bits van  decoder address
  tx_message[3] = (uint8_t) (pomAddress & 0xFF); // laagste 8-bits van decoder address
  if (pomAddress > XP_SHORT_DCC_ADDR_LIMIT)
    tx_message[2] |= 0xC0;
  cvAddress = cvAddress-1;

  tx_message[4] = pomWriteMode + ((cvAddress>>8) & 0x3); // hoogste 2 bits van cvAddress
  tx_message[5] = cvAddress & 0xFF; // laagste 8 bits van cvAddress
  tx_message[6] = cvData;
  xpc_SendMessage(tx_message);
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
  xpc_SendMessage(tx_message);
} // xpc_send_AccessoryDecoderInfoRequest

// our own xpnet extension
void xpc_send_AccessoryDecoderInfoNotify(uint8_t decAddr, uint8_t data) {
  tx_message[0] = 0x72; 
  tx_message[1] = decAddr;
  tx_message[2] = data;
  xpc_SendMessage(tx_message);
} // xpc_send_AccessoryDecoderInfoNotify

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
  xpc_SendMessage(tx_message);
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
  xpc_SendMessage(tx_message);
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
  if (locAddr > XP_SHORT_DCC_ADDR_LIMIT)
    tx_message[2] |= 0xC0;
  xpc_SendMessage(tx_message);
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
  if (locAddr > XP_SHORT_DCC_ADDR_LIMIT)
    tx_message[2] |= 0xC0;
  xpc_SendMessage(tx_message);
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
  if (locAddr > XP_SHORT_DCC_ADDR_LIMIT)
    tx_message[2] |= 0xC0;
  xpc_SendMessage(tx_message);
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
  if (locAddr > XP_SHORT_DCC_ADDR_LIMIT)
    tx_message[2] |= 0xC0;
  xpc_SendMessage(tx_message);
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
  if (locAddr > XP_SHORT_DCC_ADDR_LIMIT)
    tx_message[2] |= 0xC0;
  tx_message[4] = locSpeed;
  xpc_SendMessage(tx_message);
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
  if (locAddr > XP_SHORT_DCC_ADDR_LIMIT)
    tx_message[2] |= 0xC0;
  tx_message[4] = locFuncs;
  xpc_SendMessage(tx_message);
} // xpc_send_LocSetFuncRequest

void xpc_send_FindNextLocAddress (uint16_t locAddr, uint8_t searchDirection)
{
  searchDirection &= 0x1; // 1-bit
  tx_message[0] = 0xE3;
  tx_message[1] = 0x05 + searchDirection;
  tx_message[2] = (uint8_t) ((locAddr & 0xFF00)>>8); // hoogste 8-bits van locAddr
  tx_message[3] = (uint8_t) (locAddr & 0xFF); // laagste 8-bits van locAddr
  if (locAddr > XP_SHORT_DCC_ADDR_LIMIT)
    tx_message[2] |= 0xC0;
  xpc_SendMessage(tx_message);
} // xpc_send_FindNextLocAddress

void xpc_send_DeleteLocAddress (uint16_t locAddr)
{
  tx_message[0] = 0xE3;
  tx_message[1] = 0x44;
  tx_message[2] = (uint8_t) ((locAddr & 0xFF00)>>8); // hoogste 8-bits van locAddr
  tx_message[3] = (uint8_t) (locAddr & 0xFF); // laagste 8-bits van locAddr
  if (locAddr > XP_SHORT_DCC_ADDR_LIMIT)
    tx_message[2] |= 0xC0;
  xpc_SendMessage(tx_message);
} // xpc_send_DeleteLocAddress

void xpc_send_GetFastClock ()
{
  xpc_SendMessage(xpc_GetFastClockRequest);
} // xpc_send_GetFastClock

// sds : dit is om vanuit de client de fast clock te kunnen wijzigen, eventueel houden
void xpc_send_SetFastClock(xpcFastClock_t *newFastClock)
{
  // 0x05 0x01 TCODE0 {TCODE1 TCODE2 TCODE3} 
  tx_message[0] = 0x05;
  tx_message[1] = 0xF1;
  tx_message[2] = 0x00 | newFastClock->minute;
  tx_message[3] = 0x80 | newFastClock->hour;
  tx_message[4] = 0x40 | newFastClock->day_of_week;
  tx_message[5] = 0xC0 | newFastClock->ratio;
  xpc_SendMessage(tx_message);
} // xpc_send_SetFastClock

static void xpc_parser()
{
  xpcFastClock_t newFastClock;

  #ifdef XPC_DEBUG
  mySerial.print("parser msg ! ");
  mySerial.print("rx_size = ");
  mySerial.print(rx_size);
  mySerial.print(" - rx_index = ");
  mySerial.println(rx_index);
  for (int i=0; i <= (rx_message[0]& 0x0F);i++) {
    mySerial.print(rx_message[i],HEX);mySerial.print(" - ");
  }
  mySerial.println();
  #endif

  // rx_message[0] = header byte (de callbyte is al gestript door xpc_run)
  switch(rx_message[0] >> 4) {
    case 0x0:
      switch(rx_message[1]) {
        case 0xF1:
          // set clock
          for (uint8_t field = 2; field <= (rx_message[0] & 0x0F); field++ ) {
            uint8_t val;
            switch(rx_message[field] & 0xC0) {
              case 0x00:  
                val = rx_message[field] & 0x3F;
                if (val < 60) newFastClock.minute = val;
                break;
              case 0x80:  
                val = rx_message[field] & 0x3F;
                if (val < 24) newFastClock.hour = val;
                break;
              case 0x40:  
                val = rx_message[field] & 0x3F;
                if (val < 7) newFastClock.day_of_week = val;
                break;
              case 0xC0:  
                val = rx_message[field] & 0x3F;
                if (val < 32) newFastClock.ratio = val;
                break;
            }
          }
          xpc_FastClockNotify(&newFastClock);
          break;
        case 0xF2:
          // command station response komt als 0x05 0xF1 ...
          break;
      }
      break;
    case 0x01 :
      break;
    case 0x04 : //accessory decoder info response & feedback broadcast!!
      // [0]   [1]  [2]   [3]   [4] ...
      // 0x4? ADR1 DATA1 ADR2 DATA2 ...
      // the callback is called for every addr/data pair
      //not STM8 
      /*
      if (xpc_AccessoryDecoderInfoResponse) {
        uint8_t nbrBytes = rx_message[0] & 0x0F;
        for (int i=1;i<nbrBytes;i+=2) { // handle all addr/data pairs
          uint8_t addr = rx_message[i]; // accessory decoder address 0..127
          uint8_t data = rx_message[i+1];
          if (data & 0x10) // high nibble
            xpc_AccessoryDecoderInfoResponse (addr, (data&0x0F)<<4, 0xF0, ((data>>5)&0x3));
          else // low nibble
            xpc_AccessoryDecoderInfoResponse (addr, (data&0x0F), 0x0F, ((data>>5)&0x3));
        }
      }
      */
      break;
    case 0x06 :
      if (rx_message[1] == 0x00) {
        xpc_EventNotify (XPEVENT_POWER_OFF);
      }
      else if (rx_message[1] == 0x01) {
        xpc_EventNotify (XPEVENT_POWER_ON);
      }
      else if (rx_message[1] == 0x02) {
        xpc_EventNotify (XPEVENT_SERVICE_MODE_ON);
      }
      else if (rx_message[1] == 0x10) // register mode programming results (eventueel implementeren voor pcintf module)
      {
      }
      else if (rx_message[1] == 0x12) {
        //not STM8 if (xpc_ProgStatusResponse) xpc_ProgStatusResponse (PROG_SHORT);
        xpc_EventNotify (XPEVENT_PROG_SHORT);
      }
      else if (rx_message[1] == 0x13) {
        //not STM8 if (xpc_ProgStatusResponse) xpc_ProgStatusResponse (PROG_NOACK);
      }
      else if (rx_message[1] == 0x1F) {
        //not STM8 if (xpc_ProgStatusResponse) xpc_ProgStatusResponse (PROG_BUSY);
      }
      else if (rx_message[1] == 0x11) {
        //not STM8 if (xpc_ProgStatusResponse) xpc_ProgStatusResponse (PROG_READY);
      }
      else if (rx_message[1] == 0x14) {
        uint16_t cvAddr;
        cvAddr = ((rx_message[1] - 0x14) & 0x3) << 8;
        cvAddr |= rx_message[2];
        if (cvAddr == 0) cvAddr = 1024;
        //not STM8 if (xpc_ProgResultResponse) xpc_ProgResultResponse (cvAddr,rx_message[3]);
      }
      else if (rx_message[1] == 0x21) { // command station sw version response
        //not STM8 if (xpc_CommandStationSwVersionResponse) xpc_CommandStationSwVersionResponse (rx_message[2]);
      }
      else if (rx_message[1] == 0x22) { // command station status response
        //not STM8 if (xpc_CommandStationStatusResponse) xpc_CommandStationStatusResponse (rx_message[2]);
      }
      break;
    case 0x0A :
    case 0x0B :
      // dit zijn V1 & V2 commando's, doen we niet
      break;
    case 0x0C :
      // TODO : double header functies, eventueel voor pcintf module, maar dit gaan we toch niet gebruiken allicht
      break;
    case 0x0E :
      if ((rx_message[1] & 0xF0) == 0x30) { // address retrieval response, KKKK gebruiken we voorlopig niet volledig
        uint16_t locAddr;
        if ((rx_message[1] & 0x0F) == 0x4) // KKKK=4 : not found
          locAddr = 10000; // use an invalid address as return value
        else
          locAddr = ((rx_message[2] & 0x3F) << 8) + rx_message[3];
        //not STM8 
        //if (xpc_FindNextLocAddressResponse)
        //  xpc_FindNextLocAddressResponse(locAddr);
      }
      else if (rx_message[1] == 0x40) {
        uint16_t locStolenAddr;
        locStolenAddr = ((rx_message[2] & 0x3F) << 8) + rx_message[3];
        //not STM8 
        //if ((locStolenAddr >=0) && (locStolenAddr < 10000) && (xpc_LocStolenNotify)) 
        //  xpc_LocStolenNotify(locStolenAddr);
      }
      else if (rx_message[1] == 0x52) { // function status F13..F28 response
        uint16_t f13_f28;
        f13_f28 = (uint16_t) rx_message[3];
        f13_f28 = (f13_f28 << 8) | rx_message[2];
        //not STM8 
        //if (xpc_LocGetFuncStatus_F13_F28_Response)
        //  xpc_LocGetFuncStatus_F13_F28_Response(f13_f28);
      }
      else if ((rx_message[1] & 0xF0) == 0x00) { // loc information normal locomotive 2.1.14.1
        uint8_t dccSpeed128;
        uint16_t locFuncs;
        dccSpeed128 = rx_message[2]; // todo : uitbreiden als de speedstep != 128, herrekenen naar DCC128
        locFuncs = ((uint16_t)rx_message[4]) << 8;
        locFuncs |= (rx_message[3] << 1) | ((rx_message[3] & 0x10) >> 4); // func0 op bit0 zetten
        //not STM8 
        //if (xpc_LocGetInfoResponse)
        //  xpc_LocGetInfoResponse(dccSpeed128, locFuncs, ((rx_message[1] & 0x8)== 0) );
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

void xpc_Init(uint8_t slaveAddress)
{
  xpc_state = XPC_INIT;
  xpc_MyAddress = slaveAddress;
  XP_setAddress (slaveAddress);
} // xpc_Init

void xpc_Run(void)
{
  if ((!xpc_isConnectionError) & (millis() - rxLastMillis > CONNECTION_TIMEOUT)) {
    XP_tx_clear(); // clear TX buffer, can't send anyway
    xpc_isConnectionError = true;
    xpc_state = XPC_WAIT_FOR_CALL;
    xpc_EventNotify(XPEVENT_CONNECTION_ERROR);
  }
  
  switch (xpc_state) {
    case XPC_INIT:
      xpc_state = XPC_WAIT_FOR_CALL;
      break;

    case XPC_WAIT_FOR_CALL:
      if (XP_rx_ready()) {
        digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN)); //toggle pin13 for test
        uint16_t callbyte = XP_rx_read(); // the call byte ?
        rxLastMillis = millis(); // rx timeout manager
        if (!(callbyte & 0x100)) return; // we heben een byte in het midden van een conversatie opgepikt; we gaan hersynchroniseren en wachten op een call byte

        if (xpc_isConnectionError) {
          // we had a connection error, 
          // now notify pc that we are back in contact with the command station!
          xpc_EventNotify(XPEVENT_CONNECTION_OK);
        }
        
        xpc_isConnectionError = false; // we hebben contact met CommandStation
        if ((callbyte & 0x1F) == XPNET_BROADCAST_ADDRESS)
          xpc_state = XPC_WAIT_FOR_MESSAGE;
        else if ((callbyte & 0x1F) == xpc_MyAddress) { // filteren op eigen address
          if ((callbyte & CALL_TYPE) == CALL_TYPE_INQUIRY) {
            // komt niet meer hier,is nu in rs485 afgehandeld -> TODO mag weg
          }
          else if ((callbyte & CALL_TYPE) == CALL_TYPE_ACK) {
            // command station verwacht een acknowledge
            // TODO : werkt dit? door de ack niet in rs485 af te handelen is het mogelijk
            // dat command station meerdere calls moet sturen voor we deze ack terugsturen!
            // TODO : moeten we ook niet de TxBuffer clearen zodat er niets anders wordt gestuurd,
            // zoals door xpnet spec wordt gevraagd?
            // TODO : er is dus allicht iets fout gegaan bij onze vorige request : opletten dat de UI hier niet op blokkeert
            // want de uitstaande request gaat dus geen normale reply krijgen!!
            // we zouden ook een transfer errors boodschap moeten gekregen hebben vd command station (§2.1.8)
            xpc_SendMessage(xpc_AcknowledgementResponse);
            xpc_EventNotify(XPEVENT_ACK_REQUEST); // xpnet spec demands we inform the PC about this

            // xpc_state = XPC_WAIT_FOR_CALL; -> we blijven in deze state
          }
          else if ((callbyte & CALL_TYPE) == CALL_TYPE_MESSAGE) {
            // we moeten eerst het volledig msg ontvangen, en dan parsen
            xpc_state = XPC_WAIT_FOR_MESSAGE;
          }
        }
      }
      break;

    case XPC_WAIT_FOR_MESSAGE: 
      if (XP_rx_ready()) {
        rx_message[0] = (uint8_t) (XP_rx_read() & 0xFF); // header
        rx_size = rx_message[0] & 0x0F;  // length is without xor 
        rx_size++;                       // now including xor; (er volgen nog rx_size bytes na de header : rx_message[1] .. rx_message[rx_size];  rx_message[rx_size] = XOR
        rx_index = 1;        
        xpc_state = XPC_WAIT_FOR_MESSAGE_COMPLETE;
      }
      break;

    case XPC_WAIT_FOR_MESSAGE_COMPLETE:
      if ((millis() - rxLastMillis) >= RX_TIMEOUT) {
        // discard msg in rx_message and go back to  XPC_WAIT_FOR_CALL
        xpc_EventNotify(XPEVENT_RX_TIMEOUT);
        xpc_state = XPC_WAIT_FOR_CALL;
      }
      else if (XP_rx_ready()) {
        rx_message[rx_index] = (uint8_t) (XP_rx_read() & 0xFF);
        if (rx_index == rx_size) {
          uint8_t i, my_check = 0;
          xpc_state = XPC_WAIT_FOR_CALL; 
          // all data and xor read, now check xor
          for (i=0; i<=rx_size; i++) my_check ^= rx_message[i];   
          if (my_check == 0) {
            // packet is received and okay, now parse it
            // sds : eerst checken of we het antwoord in xpc kunnen afhandelen
            // 1. data error ?
            if (!xpc_compareMessage(rx_message,xp_datenfehler)) {
              // je hebt een data error gestuurd
              // wacht op de request acknowledge van CommandStation alvorens opnieuw te sturen
              // busy flag blijft true; de xpc zal de retransmit doen
              xpc_EventNotify(XPEVENT_TX_ERROR);
              return;
            }
            // 2. unknown command ? (commando niet ondersteund door command station)
            if (!xpc_compareMessage(rx_message,xp_unknown)) {
              xpc_EventNotify(XPEVENT_UNKNOWN_COMMAND);
              return;
            }
            // 3. busy ? Command station kan efkes geen nieuwe commando's verdragen
            if (!xpc_compareMessage(rx_message,xp_busy)) {
              xpc_EventNotify(XPEVENT_BUSY);
              return;
            }
            // else
            xpc_MessageNotify(rx_message);
            // the parser produces dedicated notifies to simplify the work for an application
            xpc_parser();
          }
          else {
            // XOR is wrong! --> discard (volgens xpnet specification)
            xpc_EventNotify(XPEVENT_RX_ERROR);
          }
        }
        else {
          rx_index++;
          if (rx_index == 17-1) {
            rx_index = 17-1;   // overrun!
            xpc_EventNotify(XPEVENT_MSG_ERROR);
            xpc_state = XPC_WAIT_FOR_CALL;
          }
        }
      }
      break;
  }
} // xpc_Run
