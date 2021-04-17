# Accessory Decoder STM8
- fixed DCC_PIN : PB4, because sduino has not yet implemented external interrupts on any pin, so we do it manually for this pin only.
- using timer2 for the 70us delay between edge interrupt and probing dcc level. could be done similarly as avr version with output compare on timer4 (millis timer), but timer2 is unused anyway, so keep it simple.  
- sdcc has problem with the if (ntf) ntf(...) construct for weak functions. Not only weak functions are not supported but if (ntf) returns false, even if ntf() is implemented! So these constructs had to be removed from nmradcc.c because callbacks were never called.
- PB4 doesn't have an internal weak pull-up -> pinMode(INPUT_PULLUP) has no effect. confirmed in datasheet.  
--> weirdly enough datasheet says there is no external interrupt on PB4. But it works, fortunately.  
- OUTPUT_ADDRESS_MODE : removed from code (not used)  
- reprogramming own decoder address on main track (learning mode):  
--> activate programming mode (long key press) and send a DCC turnout command.  
--> The DCC decoderAddress is taken as the new accessory decoder address  
--> flag MY_ADDRESS_ONLY in nmradcc lib is not used to permit receiving DCC packets for all accessory addresses. In 'normal' mode, the decoder address is filtered by turnout_decoder.c, and only own address packets are executed.

## tested
- basic accessory decoder packets : OK  
- reprogramming decoder address on main track
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
- test other decoder types included in lib (mobile decoder etc)
- replace the timer/event bullshit code

## other topics
see Accessory Decoder (AVR version)

