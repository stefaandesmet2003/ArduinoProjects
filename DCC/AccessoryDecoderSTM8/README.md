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
- writing any value to CV8 (manufacturer ID) triggers a factory reset
--> the actual value is not stored, CV8 is read-only  
--> Note! after a factory reset, the decoder reinitializes and is no longer in programming mode!  


## Implementation notes
- fixed DCC_PIN : PA2 (update), because sduino has not yet implemented external interrupts on any pin, so we do it manually for this pin only.
- using timer2 for the 70us delay between edge interrupt and probing dcc level. could be done similarly as avr version with output compare on timer4 (millis timer), but timer2 is unused anyway, so keep it simple.  
- sdcc has problem with the if (ntf) ntf(...) construct for weak functions. Not only weak functions are not supported but if (ntf) returns false, even if ntf() is implemented! So these constructs had to be removed from nmradcc.c because callbacks were never called.
- PB4 doesn't have an internal weak pull-up -> pinMode(INPUT_PULLUP) has no effect. confirmed in datasheet.  
--> weirdly enough datasheet says there is no external interrupt on PB4. But it works, fortunately.  
- OUTPUT_ADDRESS_MODE : removed from code (not used)  
- flag MY_ADDRESS_ONLY in nmradcc lib is not used to permit receiving DCC packets for all accessory addresses, including broadcast (accessory address 511). In 'normal' mode, the decoder address is filtered by turnout_decoder.c, and only own address packets are executed.
- added NOP packet (RCN-213) to nmradcc lib, not sure yet what to do with it
--> CommandStation is not sending this packet anyway for now  
## CV's
CVs correspond with NMRA spec S-9.2.2  
| CV#  | description     | default value |  remark  
| :--- | :---: | :---: | ---
|1  | address LSB | 1  | |
|9  | address MSB | 0  | |
|29 | configuration | 0x80 | read-only for now |
  bit3 : railcom enabled  
  bit5 : extended accessory decoder  
  bit7 : accessory decoder  
|28 | railcom config | 0  | read-only for now  |
|8  | manufacturerID | 13 | read-only, write any value for factory reset |
|7  | versionNumber  | 1  | read-only |
|33 | software mode  | 0  | |
| 0 : turnout decoder  |
| 1 : output decoder
In software mode = 0 :   
| 3 | timeOnOutput1 | 5 | x50ms, =250ms default coil activation time  |
| 4 | timeOnOutput2 | 5 | x50ms, =250ms default coil activation time  |
| 5 | timeOnOutput3 | 5 | x50ms, =250ms default coil activation time  |
| 6 | timeOnOutput4 | 5 | x50ms, =250ms default coil activation time  |

turnout decoder mode (factory default) : the decoder occupies 1 dcc accessory address with 4 turnouts  
output decoder mode : the decoder occupies 2 dcc accessory addresses with 8 on/off outputs  


## tested
- basic accessory decoder packets : OK  
- learning mode : reprogramming decoder address on main track  
- programming track : reading & writing CV's : OK with decoder powered from DC  
- programming fails (PROG SHORT at power-up) when decoder is powered from DCC
--> inrush current on 470uF smoothing capacitor is detected as a short circuit on the programming track  
--> TODO : modify short detection on Command Station  
- breadboard circuit : occasionally reading CV fails, not sure why. maybe the ack current is too low or too short?  
--> seems ok on soldered print  
- factory reset by writing CV8


## TODO
- programming on main is supported in nmradcc lib, but compiled out for now (NMRA_DCC_PROCESS_MULTIFUNCTION undefined)
- test other decoder types included in nmradcc lib (mobile decoder etc)
- replace the timer/event bullshit code

## other topics
see Accessory Decoder (AVR version)

