# Accessory Decoder STM8
- fixed DCC_PIN : PB4, because sduino has not yet implemented external interrupts on any pin, so we do it manually for this pin only.
- using timer2 for the 70us delay between edge interrupt and probing dcc level. could be done similarly as avr version with output compare on timer4 (millis timer), but timer2 is unused anyway, so keep it simple.  
- sdcc has problem with the if (ntf) ntf(...) construct for weak functions. Not only weak functions are not supported but if (ntf) returns false, even if ntf() is implemented! So these constructs had to be removed from nmradcc.c because callbacks were never called.
- todo : it seems as if PB4 doesn't have a pullup? external pullup had to be added even with INPUT_PULLUP pinMode (??)
## tested
- on breadboard without ULN output buffer  
- basic accessory decoder packets : OK  
- programming track : reading & writing CV's : OK (don't forget to hook up the ACK output)  
Occasionally reading CV fails, not sure why. maybe the ack current is too low or too short?  
## other topics
see Accessory Decoder (AVR version)

