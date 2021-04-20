# Accessory Decoder STM8
- 8 physical outputs paired into 4 logical turnouts
- decoder accepts basic accessory decoder commands
- decoder accepts basic accessory broadcast packets
- decoder accepts emergency off (RCN-213) packet -> all outputs off
- led flashes after every executed command
- reprogramming own decoder address on main track (learning mode):  
--> activate programming mode (long key press) and send a DCC turnout command.  
--> The decoderAddress in the DCC packet is taken as the new accessory decoder address  
--> 3 fast led flashes as acknowledge of CV write operation
- programming mode : 3-second longpress the progkey until the led starts blinking slowly
- CV programming : reading/writing CV only in programming mode
--> 3 fast led flashes after a CV write operation

## Implementation notes
- fixed DCC_PIN : PB4, because sduino has not yet implemented external interrupts on any pin, so we do it manually for this pin only.
- using timer2 for the 70us delay between edge interrupt and probing dcc level. could be done similarly as avr version with output compare on timer4 (millis timer), but timer2 is unused anyway, so keep it simple.  
- sdcc has problem with the if (ntf) ntf(...) construct for weak functions. Not only weak functions are not supported but if (ntf) returns false, even if ntf() is implemented! So these constructs had to be removed from nmradcc.c because callbacks were never called.
- PB4 doesn't have an internal weak pull-up -> pinMode(INPUT_PULLUP) has no effect. confirmed in datasheet.  
--> weirdly enough datasheet says there is no external interrupt on PB4. But it works, fortunately.  
- OUTPUT_ADDRESS_MODE : removed from code (not used)  
- flag MY_ADDRESS_ONLY in nmradcc lib is not used to permit receiving DCC packets for all accessory addresses, including broadcast (accessory address 511). In 'normal' mode, the decoder address is filtered by turnout_decoder.c, and only own address packets are executed.
- added NOP packet (RCN-213) to nmradcc lib, not sure yet what to do with it
--> CommandStation is not sending this packet anyway for now  


## tested
- basic accessory decoder packets : OK  
- learning mode : reprogramming decoder address on main track  
- programming track : reading & writing CV's : OK with decoder powered from DC  
- programming fails (PROG SHORT at power-up) when decoder is powered from DCC
--> inrush current on 470uF smoothing capacitor is detected as a short circuit on the programming track  
--> TODO : modify short detection on Command Station  
- breadboard circuit : occasionally reading CV fails, not sure why. maybe the ack current is too low or too short?  
--> seems ok on soldered print  


## TODO
- bug : led keeps blinking after factory reset  
- programming on main is supported in nmradcc lib, but compiled out for now (NMRA_DCC_PROCESS_MULTIFUNCTION undefined)
- getMyAddr -> simplify to avoid reading eeprom on every DCC packet  
- test other decoder types included in nmradcc lib (mobile decoder etc)
- replace the timer/event bullshit code

## other topics
see Accessory Decoder (AVR version)

