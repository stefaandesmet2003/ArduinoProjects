#include "rs485c.h"
#include "xpc.h"

// #include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include "Ucglib.h"

#include "keys.h"
#include "UI.h"

//temp voor debugprint
#include <AltSoftSerial.h>
AltSoftSerial mySerial;
//#define SSerialRX        8 //Serial Receive pin --> is fixed in AltSoftSerial
//#define SSerialTX        9  //Serial Transmit pin --> is fixed in AltSoftSerial

// TODO : graphics lcd object
Ucglib_ILI9341_18x240x320_HWSPI ucg(/*cd=*/ 14 , /*cs=*/ 16, /*reset=*/ 15);

// sds info : xpclient heeft geen timer2 nodig (xpmaster wel)

static void setup_lcd()
{
    int charBitmapSize = (sizeof(charBitmap ) / sizeof (charBitmap[0]));
    

    ui_Init ();
    keys_Init ();
    
    // setup graphics display
    ucg.begin(UCG_FONT_MODE_TRANSPARENT);
    ucg.setFont(ucg_font_helvB18_hr);
    ucg.setColor(0,255,255,255);
    ucg.clearScreen();
    ucg.setFontPosTop(); // dit heeft impact op de kaderkes in widgets.cpp
    ucg.setRotate270();
    
    // todo : char-lcd stub
    // special chars : todo for graphical display!!
    // init special characters
    for ( int i = 0; i < charBitmapSize; i++ )
    {
        lcd.createChar ( i, (uint8_t *)charBitmap[i] );
    }
    lcd.begin(20,4); // initialize the lcd 
    // Switch on the backlight
    //lcd.setBacklight(1); // overbodig, want by default al aan
    lcd.home (); 
    
} // setup_lcd

void setup() 
{
    init_rs485();
    init_xpclient();
    
    // set the data rate for the SoftwareSerial port
    mySerial.begin(19200);
    mySerial.println("Xpnet client!");
    
    setup_lcd();
} // setup

uint32_t lastCmdMillis;
bool powerOn = true;
void loop() 
{
    run_xpclient();
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





