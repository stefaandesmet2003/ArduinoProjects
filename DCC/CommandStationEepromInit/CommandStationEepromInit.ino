#include <EEPROM.h>

// start reading from the first byte (address 0) of the EEPROM
int address = 0;
byte value;
//[25] = railcom enabled 
byte CommandStationConfig[] = {0x17,0x01,0x30,0xF8,0x07,0x17,0xFF,0x01,0x06,0x02,0x02,0x02,0x01,0x02,0x64,0x00,
                               0x00,0x02,0x03,0x03,0x03,0x03,0x00,0x00,0x02,0x01,0x08,0x00,0x00,0x00,0x02,0x00,
                               0x00,0x00,0x08,0x28,0x01,0x1E,0x00,0x01,0x62,0xC0,0x01,0x10,0x42,0x42,0x49,0x49,
                               0x00,0x00,0x00,0x00,0x00,0x00,0x0E,0x00,0x01,0x30,0x56,0x32,0x30,0x30,0x00,0x00,
                               0x00,0x00,0x00,0x00,0x1C,0x80,0x02,0x40,0x45,0x39,0x34,0x00,0x00,0x00,0x00,0x00,
                               0x00,0x00,0x7B,0x80,0x06,0x40,0x54,0x61,0x75,0x72,0x75,0x73,0x00,0x00,0x00,0x00};                               

void setup()
{
  uint16_t addr = 0;

  // initialize the LED pin as an output.
  pinMode(13, OUTPUT);

  // initialize serial and wait for port to open:
  Serial.begin(115200);
  Serial.print("sizeof CommandStationConfig = ");
  Serial.println(sizeof(CommandStationConfig));
  for (addr=0;addr<sizeof(CommandStationConfig);addr++)
  {
    Serial.print("eeprom write addr ");
    Serial.print(addr);
    Serial.print(" : val = ");
    Serial.println(CommandStationConfig[addr]);
    
    EEPROM.update(addr,CommandStationConfig[addr]);
  }
  // schrijf 0 in de andere eeprom locaties tot aan addr 256
  for (addr=sizeof(CommandStationConfig);addr<256;addr++)
  {
    EEPROM.update(addr,0);
  }

  // turn the LED on when we're done
  digitalWrite(13, HIGH);
}

void loop()
{
  // all done!
}
