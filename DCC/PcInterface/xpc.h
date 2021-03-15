#ifndef _xpc_h_
#define _xpc_h_

typedef enum { 
  XPEVENT_POWER_OFF, XPEVENT_POWER_ON, XPEVENT_SERVICE_MODE_ON, 
  XPEVENT_MAIN_SHORT, XPEVENT_PROG_SHORT, 
  XPEVENT_LOC_STOLEN, 
  XPEVENT_CONNECTION_ERROR, XPEVENT_CONNECTION_OK, 
  XPEVENT_RX_TIMEOUT, XPEVENT_RX_ERROR, XPEVENT_MSG_ERROR, XPEVENT_TX_ERROR, 
  XPEVENT_ACK_REQUEST, XPEVENT_UNKNOWN_COMMAND, XPEVENT_BUSY 
} xpcEvent_t;

typedef struct {
  uint8_t hour;
  uint8_t minute; 
  uint8_t day_of_week;
  uint8_t ratio;
} xpcFastClock_t;
               
typedef enum {PROG_READY, PROG_SHORT, PROG_NOACK, PROG_BUSY} progStatus_t;               

// deze waarden komen overeen met xpnet spec 2.1.7 (v3.6; opgelet bit0 en bit1 zijn gewisseld in v3.6!! correct geimplementeerd in opendcc)
#define COMMANDSTATION_STATUS_POWER_ON          0x0
#define COMMANDSTATION_STATUS_EMERGENCY_STOP    0x1          
#define COMMANDSTATION_STATUS_POWER_OFF         0x2
#define COMMANDSTATION_STATUS_SERVICE_MODE_ON   0x8

extern uint8_t xpc_MyAddress; // om eventueel on-the-fly je xpnet addres te wijzigen

void xpc_Init(uint8_t slaveAddress);
void xpc_Run(void);

// functionele xpnet intf
void xpc_SendMessage(uint8_t *msg); // send a raw message, correctly formatted by caller (pcintf), xor will be appended

void xpc_send_PowerOnRequest ();
void xpc_send_PowerOffRequest ();
void xpc_send_EmergencyStopRequest ();
void xpc_send_ServiceModeResultsRequest (); // prog status/results request
void xpc_send_CommandStationStatusRequest ();
void xpc_send_CommandStationSwVersionRequest ();

// cv : 1..1024, 1024 wordt als 0 getransmit
void xpc_send_DirectModeCVReadRequest(uint16_t cvAddress);
void xpc_send_DirectModeCVWriteRequest(uint16_t cvAddress, uint8_t cvData);

#define POM_WRITE_LOC                   (0xEC)
#define POM_WRITE_ACCESSORY             (0xF0) // undocumented xpnet in opendcc
#define POM_WRITE_EXTENDED_ACCESSORY    (0xF8) // undocumented xpnet in opendcc
#define POM_READ_LOC                    (0xE4)
#define POM_READ_ACCESSORY              (0xF4) // undocumented xpnet in opendcc
#define POM_READ_EXTENDED_ACCESSORY     (0xFC) // undocumented xpnet in opendcc

// cv : 1..1024, (wordt hier op xpnet als cv-1 doorgestuurd)
// geen xpnet response op deze functie
void xpc_send_PomCVWriteRequest(uint16_t pomAddress, uint16_t cvAddress, uint8_t cvData, uint8_t pomWriteMode);

// decAddr : 0..255, 
// schakeldecoder :  elk decAddr heeft 8 outputs (4wissels)
// feedbackdecoder : elk decAddr heeft 8 inputs
// met InfoRequest kan je maar 4 bits tegelijk opvragen, nibble : 0=laagste 4 bits, 1=hoogste 4 bits
// de applayer moet de requests organiseren om alle nodige info te bekomen dmv verschillende func calls
void xpc_send_AccessoryDecoderInfoRequest(uint8_t decAddr, uint8_t nibble);

// send accessory decoder feedback to the command station
// decAddr : 0..255
// data : 8-bits, status of 8 inputs, signal aspect, 8outputs van een wisseldecoder?
void xpc_send_AccessoryDecoderInfoNotify(uint8_t decAddr, uint8_t data);

// dit commando is enkel voor schakeldecoders!
// turnoutAddr : 0..1023 (10bits) 
// -> decAddr = hoogste 8-bits van het turnoutAddr
// -> turnout = laagste 2-bits van het turnoutAddr (elke switching decoder controleert 4 wissels)
// position : 0 = rechtdoor (output 1), 1 = afslaan (output 2)
// de sds switching decoder gaat ervoor zorgen dat de 2 outputs van een paar nooit tegelijk aan staan
// een 'off' commando op de gepairde output hoeft dus niet (activate bit altijd 1)
void xpc_send_SetTurnoutRequest(uint8_t turnoutAddr, uint8_t turnoutPosition);
void xpc_send_SetSignalAspectRequest(uint16_t signalAddress, uint8_t signalAspect);

// locAddr : 0 -> 9999 (xpnet range)
// de CS antwoordt met speed info, en wie de loc bestuurt
void xpc_send_LocGetInfoRequest(uint16_t locAddr);
void xpc_send_LocGetFuncStatus_F13_F28_Request(uint16_t locAddr); // GET voor functies F13 -> F28

// locAddr : 0 -> 9999 (xpnet range)
// funcmode (tastend/nicht tastend) is niet ondersteund door opendcc 
void xpc_send_LocGetFuncModeRequest(uint16_t locAddr); // GET voor functies F0 -> F12
void xpc_send_LocGetFuncMode_F13_F28_Request(uint16_t locAddr); // GET voor functies F13 -> F28

// locAddr : 0 -> 9999 (xpnet range)
// we gaan 128speed steps gebruiken in de client (DCC128)
void xpc_send_LocSetSpeedRequest(uint16_t locAddr, uint8_t locSpeed);

// locAddr : 0 -> 9999 (xpnet range)
// per func grp, de applayer moet zorgen dat alle bits in een grp correct worden doorgegeven!!
// grp=1 (F0..F4), 2 (F5..F8), 3 (F9..F12), 4 (F13..F20), 5 (F21..F28)
void xpc_send_LocSetFuncRequest(uint16_t locAddr, uint8_t grp, uint8_t locFuncs);

// loc stack functions
#define SEARCHDIRECTION_FORWARD     (0)
#define SEARCHDIRECTION_BACKWARD    (1)
// locAddr : 0 -> 9999 (xpnet range)
// search the loc stack for the next loc address, in forward/backward direction
void xpc_send_FindNextLocAddress (uint16_t locAddress, uint8_t searchDirection);
void xpc_send_DeleteLocAddress (uint16_t locAddr);

// fast clock functies
void xpc_send_GetFastClock ();
void xpc_send_SetFastClock (xpcFastClock_t *newFastClock);

/***************************************************************************************************************/
/*** notify intf   *********************************************************************************************/
/***************************************************************************************************************/

extern void xpc_EventNotify( xpcEvent_t xpcEvent ) __attribute__ ((weak)); // de xpnet broadcasts
extern void xpc_MessageNotify ( uint8_t *msg) __attribute__ ((weak)); // to notify a received message unparsed (for pcintf)
extern void xpc_LocStolenNotify(uint16_t stolenLocAddress) __attribute__ ((weak));
extern void xpc_FastClockNotify (xpcFastClock_t *newFastClock) __attribute__ ((weak));

// dccSpeed128 : de speed wordt steeds in DCC128 teruggemeld, ook al wordt de loc momenteel volgens andere speedsteps gestuurd
// de xp client gaat zelf altijd DCC128 gebruiken
// locFuncs_f0_f12 : 1 bit per functie, bit0=functie0 (licht), bit1=functie1, ... tot f12
// isLocFree : true = niet in gebruik door een andere xpclient
extern void xpc_LocGetInfoResponse(uint8_t dccSpeed128, uint16_t locFuncs_f0_f12, bool isLocFree) __attribute__ ((weak));
extern void xpc_LocGetFuncStatus_Response (uint16_t f0_f12) __attribute__ ((weak));
extern void xpc_LocGetFuncStatus_F13_F28_Response (uint16_t f13_f28) __attribute__ ((weak));
extern void xpc_CommandStationStatusResponse (uint8_t CsStatus) __attribute__ ((weak));
extern void xpc_CommandStationSwVersionResponse(uint8_t CsVersion) __attribute__ ((weak));
extern void xpc_ProgStatusResponse (progStatus_t progResponse) __attribute__ ((weak));
// cvAddress : 1..1024
extern void xpc_ProgResultResponse (uint16_t cvAddress, uint8_t cvData) __attribute__ ((weak));

// xpnet specification 2.1.11
#define DECODERTYPE_SWITCHING_DECODER_WITHOUT_FEEDBACK  (0)
#define DECODERTYPE_SWITCHING_DECODER_WITH_FEEDBACK     (1)
#define DECODERTYPE_FEEDBACK_DECODER                    (2)
#define DECODERTYPE_FUTURE_USE                          (3)
#define TURNOUTPOSITION_UNKNOWN (0) // wissel nog niet bediend tijdens deze sessie
#define TURNOUTPOSITION_CLOSED  (1) // rechtdoor (output 1 'groen')
#define TURNOUTPOSITION_THROWN  (2) // afslaan (output 2 'rood')
#define TURNOUTPOSITION_INVALID (3)
// decAddr : 0..255, 
// met InfoRequest kan je maar 4 bits tegelijk opvragen, nibble : 0=laagste 4 bits, 1=hoogste 4 bits
// nibble 0:inputs/outputs 0..3 van de decoder, 1:inputs/outputs 4..7 van de decoder
// decType : 1 vd defines DECODERTYPE_xxx
// decBits :
// schakeldecoder :  bits0..1 : wissel 1, bits2..3 : wissel 2
// feedbackdecoder : 4 inputs van de feedback decoder
// de applayer moet de requests organiseren om alle nodige info te bekomen dmv verschillende func calls
// decBitsValid : a mask with the 4 bits that are valid from the received nibble
void xpc_AccessoryDecoderInfoResponse (uint8_t decAddress, uint8_t decBits, uint8_t decBitsValid, uint8_t decType) __attribute__ ((weak));

// nextLocAddress : 0..9999 = valid, 10000 = invalid
extern void xpc_FindNextLocAddressResponse(uint16_t nextLocAddress) __attribute__ ((weak));

#endif // _xpc_h_
