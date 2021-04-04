#include "rs485c.h"
#include "xpc.h"

//temp voor debugprint
#include <AltSoftSerial.h>
AltSoftSerial mySerial;

//#define SSerialRX        8 //Serial Receive pin --> is fixed in AltSoftSerial
//#define SSerialTX        9  //Serial Transmit pin --> is fixed in AltSoftSerial


// command line input via mySerial for test

#define XPNET_MY_ADDRESS (5)
#define RS485_DIRECTION_PIN   4 // hw specific


uint16_t curLocAddress = 3;
uint8_t curLocSpeed = 0;

bool do_a = false;

static void commandLine_Run()
{
  char inByte;
  
  if (mySerial.available() > 0)
  {
    inByte = mySerial.read();
    mySerial.print("got:");mySerial.println(inByte);
    switch (inByte) {
      case '+' : // increase speed
        break;
      case '-' : // decrease speed
        break;
      case '?' : // loc info request
        xpc_send_LocGetInfoRequest(curLocAddress); // xpnet gaat antwoorden met call naar xpc_LocGetInfoResponse
        break;
      case 'a': // send a dummy accessory decoder notify (my xpnet extension)
        //xpc_send_AccessoryDecoderInfoNotify(1,0xAA); // decAddr - data
        do_a = !do_a;
        break;
    }
  }
} // commandLine_Run

// sds info : xpclient heeft geen timer2 nodig (xpmaster wel)

void setup() 
{
  // sds temp debug :
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);

  init_rs485(RS485_DIRECTION_PIN);
  xpc_Init(XPNET_MY_ADDRESS);
  
  // set the data rate for the SoftwareSerial port
  mySerial.begin(19200);
  mySerial.println("Xpnet client!");
} // setup

uint32_t lastCmdMillis;
bool powerOn = true;
void loop() 
{
    xpc_Run();
    commandLine_Run();
    if (do_a) {
      xpc_send_AccessoryDecoderInfoNotify(1,0x5A); // decAddr - data
    }
    /*
    // todo : UI
    if ((millis() - lastCmdMillis) > 2000)
    {
        lastCmdMillis = millis();
        if (powerOn)
        {
            xpc_send_PowerOffRequest();
            powerOn = false;
        }
        else
        {
            xpc_send_PowerOnRequest();
            powerOn = true;
        }
    }
    */

} // loop

// temp : xpc callback implementaties hier
void xpc_EventNotify( xpcEvent_t xpcEvent )
{
  mySerial.print("xpc_Event : ");
  switch (xpcEvent) {
    case XPEVENT_POWER_OFF : 
      mySerial.println("XPEVENT_POWER_OFF"); // main track disabled
      break;
    case XPEVENT_POWER_ON :
      mySerial.println("XPEVENT_POWER_ON"); // normal operation resumed
      break;
    case XPEVENT_SERVICE_MODE_ON :
      mySerial.println("XPEVENT_SERVICE_MODE_ON"); // service mode entry
      break;
    case XPEVENT_MAIN_SHORT :
      mySerial.println("XPEVENT_MAIN_SHORT");
      break;
    case XPEVENT_PROG_SHORT :
      mySerial.println("XPEVENT_PROG_SHORT");
      break;
    case XPEVENT_LOC_STOLEN :
      mySerial.println("XPEVENT_LOC_STOLEN");
      break;
    case XPEVENT_CONNECTION_ERROR :
      mySerial.println("XPEVENT_CONNECTION_ERROR"); // haven't received a call byte during CONNECTION_TIMEOUT; this could be due to programming mode?
      break;
    case XPEVENT_RX_TIMEOUT :
      mySerial.println("XPEVENT_RX_TIMEOUT"); // not all msg bytes received within RX_TIMEOUT
      break;
    case XPEVENT_RX_ERROR :
      mySerial.println("XPEVENT_RX_ERROR"); // xor error in received message, msg discarded. application needs to resend the last request
      break;
    case XPEVENT_MSG_ERROR :
      mySerial.println("XPEVENT_MSG_ERROR"); // receive buffer overrun
      break;
    case XPEVENT_TX_ERROR :
      mySerial.println("XPEVENT_TX_ERROR"); // xp-master signalled a data error, current command stays in the txBuffer
      break;
    case XPEVENT_UNKNOWN_COMMAND :
      mySerial.println("XPEVENT_UNKNOWN_COMMAND"); // xp-master signalled an unknown command
      break;
    case XPEVENT_BUSY :
      mySerial.println("XPEVENT_BUSY"); // xp-master signalled "command station busy"
      break;
  }
} // xpc_EventNotify

void xpc_LocStolenNotify(uint16_t stolenLocAddress)
{
  mySerial.print("xpc_LocStolenNotify : ");
  mySerial.println(stolenLocAddress);
}
xpcFastClock_t theClock;
void xpc_FastClockNotify (xpcFastClock_t *newClock)
{
  theClock = *newClock; // kopieert dat alle fields?
  mySerial.print("fast clock updated : day = ");
  mySerial.print(theClock.day_of_week);
  mySerial.print(" - ");
  mySerial.print(theClock.hour);
  mySerial.print(":");
  mySerial.println(theClock.minute);
}
void xpc_LocGetInfoResponse(uint8_t dccSpeed128, uint16_t locFuncs_f0_f12, bool isLocFree)
{
  mySerial.println("xpc_LocGetInfoResponse received!");
}
void xpc_AccessoryDecoderInfoResponse (uint8_t decAddress, uint8_t decBits, uint8_t decBitsValid, uint8_t decType){
  mySerial.print("update decoder @addr : "); mySerial.print(decAddress);
  mySerial.print(" : "); mySerial.print(decBits,BIN);
  mySerial.print(" & mask : "); mySerial.println(decBitsValid,BIN);
}
