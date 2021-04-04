/*
 * om deze sketch in een atmega168 te krijgen moet je u8g2 optimalisaties doen : in u8g2.h :
 * //sds #define U8G2_WITH_CLIP_WINDOW_SUPPORT
 * //sds #define U8G2_WITH_FONT_ROTATION
 * 
 * de defines zijn default gezet, en includen meer code, door te commenten wordt de flash size kleiner
 * net klein genoeg 14278/14336 bytes
 * voorlopig geen andere lib dan adafruit gevonden voor ina219, die lijkt ook zeer groot voor niets te doen
 * TODO 03/2021 verkleinen, want fit niet meer, wat een vooruitgang
 */

#include <U8g2lib.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);   // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED


void setup(void) 
{
  //Serial.begin(115200);
  //Serial.println("Hello!");
  
  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  ina219.begin();
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  ina219.setCalibration_16V_400mA();

  u8g2.begin();  
  //u8g2.setFont(u8g2_font_ncenB12_tr); // NOK 03/2021
  //u8g2.setFont(u8g2_font_tenfatguys_tu); // use font without lowercase to fit 16k flash, OK 03/2021
  u8g2.setFont(u8g2_font_VCR_OSD_tu); // use font without lowercase to fit 16k flash, OK 03/2021
  
}

void loop(void) 
{
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  /*
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.println("");
  */

  u8g2.firstPage();
  do {
    u8g2.setCursor(0,15);
    u8g2.print(busvoltage);u8g2.print(" V");
    u8g2.setCursor(0,31);
    u8g2.print(current_mA);u8g2.print(" MA"); // use font without lowercase to fit 16k flash
  } while ( u8g2.nextPage() );
  delay(200);

}
