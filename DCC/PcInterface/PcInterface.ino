// altserial naar pc : LENZ interface @ 19200 baud
// xpnet client op uart1
#include "altrs232.h"
#include "rs485c.h"
#include "xpc.h"

#define XPNET_MY_ADDRESS (1)
uint8_t xpc_Address = XPNET_MY_ADDRESS; // kan aangepast worden door de pc interface

typedef enum { // actual state
  IDLE,
  WF_MESSAGE,
  WF_XOR,
} pc_parser_state_t;
pc_parser_state_t pc_parser_state;

uint8_t pcc[16];          // pc_message speicher
uint8_t pcc_size, pcc_index;

#define PC_RX_TIMEOUT   250 // in ms, klopta met de spec, of eerder 500ms? dat is voor de complete msg
uint32_t pcRxMillis;

// PC<>PCintf communication
uint8_t pcm_timeout[] = {0x01, 0x01};                 // Timeout between PC & PCIntf
uint8_t pcm_timeout_xp[] = {0x01, 0x02};              // timeout between PCIntf & Command Station
uint8_t pcm_comms_error_xp[] = {0x01, 0x03};          // communication error PCIntf & Command Station
uint8_t pcm_ack[] = {0x01, 0x04};                     // ack
uint8_t pcm_timeslot_error[] = {0x01, 0x05};          // Command Station is no longer providing us a timeslot for communication
uint8_t pcm_overrun[] = {0x01, 0x06};                 // buffer overrun (voorlopig enkel aan de rs485c kant, geen idee hoe overruns aan de altSerial te detecteren)

// general fixed messages
uint8_t pcm_datenfehler[] = {0x61, 0x80};             // xor wrong
uint8_t pcm_busy[] = {0x61, 0x81};                    // busy
uint8_t pcm_unknown[] = {0x61, 0x82};                 // unknown command
uint8_t pcm_version[] = {0x63, 0x21, 0x36, 0x00};     // LZ100 Zentrale in Version 3.6
uint8_t pcm_liversion[] = {0x02, 0x10, 0x01};         // LI101F Version 1.0 Code 01
                       // {0x02, 0x30, 0x01};         // LIUSB 3.0

static void pc_SendMessage(unsigned char *str)
{
  unsigned char n, total, my_xor;

  n = 0;
  my_xor = str[0];
  total = str[0] & 0x0F;
  while (!tx_fifo_ready()) ;             // busy waiting!

  tx_fifo_write(str[0]);                   // send header
  while (n != total) {
    n++;
    my_xor ^= str[n];
    tx_fifo_write(str[n]);              // send data
  }    
  tx_fifo_write(my_xor);                // send xor
} // pc_SendMessage

static void pc_ParseMessage(void) {
  // de meeste messages kan je rechtstreeks op xpc zetten
  // sommige moet de pc intf zelf afhandelen
  // TODO : de msgs die geen respons van het command station krijgen, moeten we pcm_ack naar de PC
  // hoe doen we dat zonder alle msgs te parsen??
  // of mogen we gewoon alles acken? -> voorlopig zo
  switch(pcc[0] >> 4) { // this is opcode
    case 0xF:
      if (pcc[0] == 0xF0) // ask LI-Version
      {
        pc_SendMessage(pcm_liversion);
        return;
      }
      switch(pcc[1]) {
        default:
          break;
        case 0x01:  // ask / set xpnet address
          if ((pcc[2] >= 1) && (pcc[2] < 32)) { // set new xpnet address
            xpc_Address = pcc[2];
            xpc_Init(xpc_Address);
          }
          // respond with xpnet address currently in use
          pcc[2] = xpc_Address; // respond with xpnet address currently in use
          pc_SendMessage(&pcc[0]);
          break;
        case 0x02: 
          // doen we niet, altSerial werkt toch niet deftig boven 19200
          pc_SendMessage(pcc); // send back as reply
          break;
      }
      return;
    default :
      break; // and continue sending the message onto xpnet
  }
  xpc_SendMessage(pcc);

  // some commands don't generate a response from CS -> we have to send ack to PC instead. 
  // and apparently we can't just ack any command!
  switch(pcc[0] >> 4) { // this is opcode
    case 0x5: // accessory command
    case 0x9: // loc stop
      pc_SendMessage(pcm_ack);
      break;
    case 0x2: // CS power-up mode
      if (pcc[1] == 0x22)
        pc_SendMessage(pcm_ack);
      break;
    case 0xE:
      if (((pcc[1] & 0xF0) == 0x10) || // 0x10..0x13 commands
          ((pcc[1] & 0xF0) == 0x20) || // 0x20..0x26 commands
          (pcc[1] == 0x30) || (pcc[1] == 0x44)) {
        pc_SendMessage(pcm_ack);
      }
      break;
    default:
      break;
  }
} // pc_ParseMessage

// vgl lenz_parser run, maar zonder parsing
static void pc_Run() {
  uint8_t i, my_check;
      
  switch (pc_parser_state) {
    case IDLE:
      if (!rx_fifo_ready()) return;
      pcc[0] = rx_fifo_read();      // read header
      pcc_size = pcc[0] & 0x0F;
      pcc_index = 0;                // message counter
      pcRxMillis = millis();        // timeout monitoring
      pc_parser_state = WF_MESSAGE;
      break;
    case WF_MESSAGE:
      if (pcc_index == pcc_size) {
        pc_parser_state = WF_XOR;
        return;
      }
      if (!rx_fifo_ready()) {
        if ((millis() - pcRxMillis) < PC_RX_TIMEOUT) return;
        else {
          pc_parser_state = IDLE;
          pc_SendMessage(pcm_timeout); // timeout PC<>PCintf
          return;
        }
      }
      pcc_index++;
      pcc[pcc_index] = rx_fifo_read();
      break;
    case WF_XOR:
      if (!rx_fifo_ready()) {
        if ((millis() - pcRxMillis) < PC_RX_TIMEOUT) return;
        else {
          pc_parser_state = IDLE;
          pc_SendMessage(pcm_timeout); // timeout PC<>PCintf
          return;
        }
      }
      my_check = 0;  
      for (i=0; i<=pcc_size; i++) my_check ^= pcc[i];   
      if (my_check != rx_fifo_read()) {
        // XOR is wrong!
        // we onderscheppen hier al de fout en antwoorden alsof we een CS zijn
        pc_SendMessage(pcm_datenfehler);
        pc_parser_state = IDLE;
        return;
      }
      
      pc_ParseMessage();
      pc_parser_state = IDLE;
      break;
  }
} // pc_Run

// als we een msg van xpc krijgen sturen we gewoon naar de pc
void xpc_MessageNotify ( uint8_t *msg) {
  // pc_SendMessage is potentieel blocking, en ondertussen kan de xpc X_RxBuffer vollopen
  // dus als de pc niet kan volgen, krijgen we overruns in rs485c
  pc_SendMessage(msg);
} // xpc_MessageNotify

void xpc_EventNotify( xpcEvent_t xpcEvent ) {
  switch (xpcEvent) {
    case XPEVENT_BUSY : 
      pc_SendMessage(pcm_busy);
      break;
    case XPEVENT_UNKNOWN_COMMAND : 
      pc_SendMessage(pcm_unknown);
      break;
    case XPEVENT_RX_TIMEOUT : // msg timeout op xpnet
    case XPEVENT_ACK_REQUEST : 
    case XPEVENT_TX_ERROR : // CS stuurt ons een data error notify, dus we hebben iets verkeerd gestuurd naar CS
      // xpnet spec verwacht dat we pcm_comms_error_xp sturen als CS ons een ack request stuurt, maar dat handelt xpc voorlopig autonoom af
      // geen idee of JMRI hier nu wacht op een req ack
      // we gaan ervan uit dat de PC een retransmit zal doen, en dat we dit dus niet in de pcintf moeten implementeren
      pc_SendMessage(pcm_comms_error_xp);
      break;
    case XPEVENT_RX_ERROR : // antwoord van CS bevat een XOR error
      pc_SendMessage(pcm_timeout_xp);
      break;
    case XPEVENT_MSG_ERROR : // buffer overrun in X_RxBuffer
      // TODO : how to detect overruns on the altSerial interface?
      pc_SendMessage(pcm_overrun);
      break;
    case XPEVENT_CONNECTION_ERROR : // not receiving call bytes from command station
      pc_SendMessage(pcm_timeslot_error);
      break;
    case XPEVENT_CONNECTION_OK : 
      pc_SendMessage(pcm_ack);
      break;
  }
} // xpc_EventNotify

void setup() 
{
  init_rs232(); // pc interface
  init_rs485();
  xpc_Init(xpc_Address);
} // setup

void loop() 
{
  xpc_Run();
  pc_Run();
} // loop
