#include "status.h" //voor de fast clock waarde
#include "rs485c.h"

#include "xpc.h"

//temp voor debugprint
#include <AltSoftSerial.h>
AltSoftSerial mySerial;

//#define SSerialRX        8 //Serial Receive pin --> is fixed in AltSoftSerial
//#define SSerialTX        9  //Serial Transmit pin --> is fixed in AltSoftSerial


// temp : ui via serial, om alle xpnet intf funcs te testen
// check command line input via mySerial

uint16_t curLocAddress = 3;
uint8_t curLocSpeed = 0;

static void run_commandLine()
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
    }
  }
} // run_commandLine

// sds info : xpclient heeft geen timer2 nodig (xpmaster wel)

void setup() 
{
  init_rs485();
  init_xpclient();
  
  // set the data rate for the SoftwareSerial port
  mySerial.begin(19200);
  mySerial.println("Xpnet client!");
} // setup

uint32_t lastCmdMillis;
bool powerOn = true;
void loop() 
{
    run_xpclient();
    run_commandLine();
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
  mySerial.println(xpcEvent);
  if (xpcEvent == 9)
  {
    for (int i=0;i<17;i++)
    {
      mySerial.print(rx_message[i],HEX);
      mySerial.print(":");
    }
    mySerial.println();
  }
}

void xpc_LocStolenNotify(uint16_t stolenLocAddress)
{
  mySerial.print("xpc_LocStolenNotify : ");
  mySerial.println(stolenLocAddress);
}

void xpc_FastClockUpdated ()
{
  mySerial.print("fast clock updated : day = ");
  mySerial.print(fast_clock.day_of_week);
  mySerial.print(" - ");
  mySerial.print(fast_clock.hour);
  mySerial.print(":");
  mySerial.println(fast_clock.minute);
}
void xpc_LocGetInfoResponse(uint8_t dccSpeed128, uint16_t locFuncs_f0_f12, bool isLocFree)
{
  mySerial.println("xpc_LocGetInfoResponse received!");
}
