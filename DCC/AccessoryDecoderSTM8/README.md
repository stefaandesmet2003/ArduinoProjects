# Accessory Decoder STM8
- fixed DCC_PIN : PB4, because sduino has not yet implemented external interrupts on any pin, so we do it manually for this pin only.
- using timer2 for the 70us delay between edge interrupt and probing dcc level. could be done similarly as avr version with output compare on timer4 (millis timer), but timer2 is unused anyway, so keep it simple.  
- sdcc has problem with the if (ntf) ntf(...) construct for weak functions. Not only weak functions are not supported but if (ntf) returns false, even if ntf() is implemented! So these constructs had to be removed from nmradcc.c because callbacks were never called.
- PB4 doesn't have an internal weak pull-up -> pinMode(INPUT_PULLUP) has no effect. confirmed in datasheet.  
--> weirdly enough datasheet says there is no external interrupt on PB4. But it works, fortunately.  
- OUTPUT_ADDRESS_MODE : removed from code (not used)  

## tested
- basic accessory decoder packets : OK  
- programming track : reading & writing CV's : OK with decoder powered from DC  
- programming fails (PROG SHORT at power-up) when decoder is powered from DCC
--> inrush current on 470uF smoothing capacitor is detected as a short circuit on the programming track  
--> TODO : modify short detection on Command Station  
- breadboard circuit : occasionally reading CV fails, not sure why. maybe the ack current is too low or too short?  
--> seems ok on soldered print

## TODO
- reprogramming own decoder address on main track  
--> activate 'learning' mode, any accessory decoder command is then used to set the decoder address  
--> needs progkey  

Alternatively remove progkey : 
- programming on programming track (service mode packets)
- programming on main is supported in nmradcc lib, but compiled out for now (NMRA_DCC_PROCESS_MULTIFUNCTION undefined)

- use flag MY_ADDRESS_ONLY ? For now decoder address is filtered by turnout_decoder.c (to be copied in avr version!)
- getMyAddr -> simplify to avoid reading eeprom on every DCC packet  

## other topics
see Accessory Decoder (AVR version)

