# Signal Decoder STM8
- implemented as extended accessory decoder, address stored in CV1/CV9
- decoder implements 4 signal heads with configurable aspects (via CV programming, see below), with up to 5 lamps per aspect
- decoder accepts extended accessory decoder commands
- decoder accepts extended accessory broadcast packets
- decoder accepts emergency off (RCN-213) packet -> all outputs off
- reprogramming own decoder address on main track (learning mode):  
--> activate programming mode (long key press) and send a DCC turnout command.  
--> The decoderAddress in the DCC packet is taken as the new accessory decoder address  
--> 3 fast led flashes as acknowledge of CV write operation
- programming mode : 3-second longpress the progkey until the led starts blinking slowly
- CV programming : reading/writing CV only in programming mode  
--> 3 fast led flashes after a CV write operation
- factory reset : writing any value to CV8 (manufacturer ID) triggers a factory reset  
--> the actual value is not stored, CV8 is read-only  
--> Note! after a factory reset, the decoder reinitializes and is no longer in programming mode!  
- no dedicated output for indication led, the builtin led (PB5) is used, and shared with i2c expander. The led will dimly light during i2c operation    

## compatible signals
on the prototype print 4 headers are foreseen, 1 header per signal head  
Each header has 6 pins, VCC + 5 led outputs  
led outputs are active low, ie. signal leds must have their anodes to ground.  
The signals need only 1 common current-limiting resistor for all leds (ie. all leds can be in parallel), because the led outputs are multiplexed by the decoder. This allows leds with different forward voltages to light 'together'.

## configuring signal heads
hardware has 19 physical outputs, 11 directly on STM8, 8 additional over i2c expander.  
every signal head uses max 5 outputs = individually controllable lamps  
Each header has 6 pins, VCC + 5 outputs :  
header 1 : outputs 1,2,3,4,5  
header 2 : outputs 6,7,8,9,10  
header 3 : outputs 11,12,13,14,15  
header 4 : outputs 16,17,18,19,(20=not available)  

The factory defaults sets signal head 0 on header 1, signal head 1 on header 2, etc. The defauls aspects are the infrabel 2013 main signal aspects on the 4 heads, using JMRI convention:  
aspect 0 : stop (rood)  
aspect 1 : approach (dubbel geel)  
aspect 2 : clear (groen)  
aspect 3 : restricting (rood-wit)  
aspect 4 : advanced approach (geel-groen vertikaal)  
aspect 5 : slow approach (geel-groen horizontaal)  
aspect 8 : unlit (lampen uit)  

For this to work the signal has to be wired as follows:  
VCC-White-YellowV-Red-Green-YellowH  
For example the default head 0, aspect 0 will activate output 3, so the signal's red led must be wired to output 3 for this to work correctly

## CV for configuring the signal aspects
CVs consist of an index table, and an aspect definition table
### Aspect Index Table
The aspect index table contains all the used heads/aspects combinations, and starts at CV 33  
1 byte per aspect, with this bit layout :  
A-H1-H0-A4-A3-A2-A1-A0  
A : active, the index entry refers to a valid aspect definition in the Aspect Definition Table  
H1..H0 : 2-bit head ID (0..3), as per NMRA  
A4..A0 : 5-bit aspect ID (0..31), as per NMRA  
There are up to INDEX_TABLE_SIZE (40) entries (CV33->CV72)
--> the INDEX_TABLE_SIZE #define can be modified to increase the number of entries  
--> the only firm limit is the eeprom size (640 bytes for STM8F103 used here)  

### Aspect Definition Table
The aspect definition table starts immediately after the aspect index table, and takes 10 bytes per aspect (ASPECT_BLOCK_SIZE), ie. 5 lamp definitions with 2 bytes per lamp.  
The position in the aspect definition table is defined by the index!  

Each lamp is defined with 2 bytes with the following layout :  
byte 1 : B2-B1-B0-O4-O3-O2-O1-O0  
byte 2 : x-I-Off2-Off1-Off0-0n2-On1-On0  
byte 1 sets the lamp output and the brightness
O4..O0 : output used for this lamp (1..19, 0 for 'not used')  
B2..B0 : brightness setting for the lamp  
Brightness is set by PWM in 10 steps. The B2..B0 bits correspond to following brightness settings :  
0 : 100%  
1 : 80% second phase  
2 : 60% second phase  
3 : 40% second phase  
4 : 20% second phase  
5 : 30% first phase  
6 : 20% first phase  
7 : 10% first phase  
white leds have a higher forward voltage than red/green/yellow, and will not lit when parallel with red/green/yellow. Therefore use setting 5/6/7 for white + 1/2/3/4 for the red/green/yellow leds. White leds are very bright, so 10% brightness is more than enough

byte 2 sets the blinking behaviour of the leds. Blinking starts with on-phase, then off-phase, but can be inverted.
On2..On0 : on-period in multiples of 250ms
Off2..Off0 : off-period in multiples of 250ms
I : invert-bit : to start with off-phase. this allows to blink leds alternatively
--> for instance lamp 1 with I=0, lamp 2 with I=1, and same on/off periods
x : not used

### example 
the factory defaults have aspect 6 & 7 on head 0 as demo for the blinking :

0x22,0x12,0x25,0x12,0,0,0,0,0,0,  // dubbel geel 80%, knipperend 0.5s
0xE1,0x24,0x23,0x64,0,0,0,0,0,0,  // rood-wit, afwisselend knipperend 1s

-> first line defines aspect 6, 10 bytes  
-> lamp 1 : 80% output 2, with blinking 2x250ms on/off, not inverted  
-> lamp 2 : 80% output 5, with blinking 2x250ms on/off, not inverted  
-> so both lamps blink together  
-> other lamps not used (6 zero-bytes)  

-> second line defines aspect 7  
-> lamp 1 : 10% output 1, with blinking 4x250ms on/off, not inverted  
-> lamp 2 : 80% output 3, with blinking 4x250ms off/on, inverted  
-> so both lamps blink alternatively  
-> other lamps not used (6 zero-bytes)  

## CV List
CVs correspond with NMRA spec S-9.2.2  
| CV#  | description     | default value |  remark  
| :--- | :---: | :---: | ---
|1  | address LSB | 1  | |
|9  | address MSB | 0  | |
|29 | configuration | 0xA0 | read-only for now |
  bit3 : railcom enabled  
  bit5 : extended accessory decoder  
  bit7 : accessory decoder
|28 | railcom config | 0  | read-only for now  |
|8  | manufacturerID | 13 | read-only, write any value for factory reset |
|7  | versionNumber  | 1  | read-only |
|33->639 | Aspect Definitions  | -  | see above |

## tested
- extended accessory decoder packets : OK  
- learning mode : reprogramming decoder address on main track : OK
- programming track : reading & writing CV's : OK, 
--> need to power STM8 over the USB and activate programming mode (because command station doesn't power programming track all the time!)  
- factory reset by writing CV8 : OK

## Notes on (JMRI) accessory addressing
- the dcc extended accessory 11-bit address consists of 9-bit address (same as basic accessory) + 2 bits head  
--> e.g. dcc address = 1 -> extended accessory addresses are 4..7  
--> for some unknown reason dcc accessory address = 0 is not used ? (and thus extended accessory addresses 0..3 are not used either)
- JMRI addresses signal masts by the extended address, but counts from 1 instead of 0!  
- learning mode only reprograms the basic accessory address (9 MSB)

### Example
dcc address = 2, head 1  
--> extended accessory address = 2*4+1 = 9
--> in JMRI use : DCC accessory address = 10 for signal mast

## TODO
- some flickering on the white leds when dimming is used over i2c. White leds are extremely bright and used at 10% brightness. Probably with i2c delays the timing for the 1ms pwm is not always met and white led occasionally remains off for >10ms?
- programming on main is supported in nmradcc lib, but compiled out for now (NMRA_DCC_PROCESS_MULTIFUNCTION undefined)
- replace the timer/event bullshit code

## Implementation notes
- fixed DCC_PIN : PA2, because sduino has not yet implemented external interrupts on any pin, so we do it manually for this pin only.
- DCC decoding using timer2 for the 70us delay between edge interrupt and probing dcc level. could be done similarly as avr version with output compare on timer4 (millis timer), but timer2 is unused anyway, so keep it simple.  
- 11 outputs directly on STM8, 8 additional over i2c expander. 
--> i2c/sda is shared with builtin led. 
--> during normal operation, i2c is used, and the builtin led is dimly lit during i2c operations  
--> During programming, the i2c bus is disabled and PB5 is used as normal output to blink the builtin led   
- NUM_OUTPUTS_PER_HEAD : 3 for now, can be up to 5
- timer1 used as 1ms interrupt for brightness/blinking timing of signal leds
- needed some optimisations to fit the 8kB flash  
--> used a reduced implementation of i2c (only TX required for the expander)  
--> used a simplified implementation of eeprom  
- sdcc has problem with the if (ntf) ntf(...) construct for weak functions. Not only weak functions are not supported but if (ntf) returns false, even if ntf() is implemented! So these constructs had to be removed from nmradcc.c because callbacks were never called.
- OUTPUT_ADDRESS_MODE : removed from nmradcc lib code (not used)  
- flag MY_ADDRESS_ONLY in nmradcc lib is not used to permit receiving DCC packets for all accessory addresses, including broadcast (accessory address 511).
- added NOP packet (RCN-213) to nmradcc lib, not sure yet what to do with it
--> CommandStation is not sending this packet anyway for now  