# DCC CommandStation
- built from the original opendcc code
- comes in 2 main configurations :  with RS485 xpressnet, or direct RS232 PC interface. 
--> Atmega328 has only 1 uart interface. The original opendcc cpu had 2 uarts.  
--> Here,both xpressnet & pc-interface are implemented on uart1, and are therefore mutually exclusive
--> the PC interface is modelled following the Lenz specification (quasi identical to xpressnet, except for the missing call byte), and can be used to interface with e.g. JMRI.

## Building xpressnet configuration
- define XPRESSNET_ENABLED=1 and PARSER=NONE in config.h  
- uses RS485 PHY for xpnet  (see hardware.h for details)
- PcInterface sketch (see below) provides code for a separate pc interface. It acts as bridge between xpressnet and PC, similar to Lenz LI101, and requires only basic hardware (atmega+rs485PHY+USBserial-ttl converter)    

## Building direct PC interface configuration
- define XPRESSNET_ENABLED=0 and PARSER=LENZ in config.h
- uart1 talks Lenz protocol at default 19200 baud. The atmega328 connects directly to the PC with e.g. JMRI

## xpressnet addresses  
- using static addresses for the moment, no dynamic addresses. Fixed addressing via defines or CV variable.
- overview of used addresses :  
-> 0 : CommandStation, using 'slot 0' for the local UI, this is a not-valid XPNET address  
-> 1 : PcInterface, fixed define for now  
-> 3 : Throttle, fixed define for now  
-> 5 : FeedbackDecoder, configurable over CV35, for potentially multiple decoders  
-> other addresses in range 1..31 not yet used

## Tested functionality
- loco control via local UI & JMRI, JMRI receives loc stolen message
- turnout control via local UI & JMRI
- turnout feedback, possibility to assign feedback decoders to provide turnout feedback  
--> use a feedback decoder with the same address as the corresponding accessory decoder. Since the feedback decoder is not connected to DCC (except during programming), there is no problem with this approach  
--> or use an accessory decoder with additional xpnet interface providing feedback directly, not considered for now.
- track power on/off, on/off/short/external-stop notifications  
- not all possible commands in the parser have been tested!!  
- turnout control from local UI broadcasted over xpnet
- local UI syncs with commands over xpnet

## Command Station CVs
- for the moment it's not possible to read/write CommandStation CVs  
-> code is gecomment, nog iets te voorzien via de UI (key_go_at_start_pressed, dat is opendcc, dat werkt hier niet)  
-> fixed config via 'CommandStationEepromInit

| CV#  | description     | default value |  remark  
| :--- | :---: | :---: | ---
0 | eadr_OpenDCC_Version | ? | wordt in de code gebruikt, en moet dus in eeprom staan, maar kan je niet uitlezen, daarom CV5  
1  | eadr_baudrate | 1 | DEFAULT_BAUD 19200  
2  | eadr_OpenDCC_Mode  | ? | is read-only, maar reflecteert niet de compile flags, omdat eeprom niet systematisch wordt mee geprogrammeerd in arduino IDE  
5  | eadr_VersionMirror  | ?| == CV0  
13 | eadr_dcc_acc_repeat  | 2 | Accessory Commands are repeated this number of times  
18 | eadr_extend_prog_resets | 3 | add this number the number of resets command during programming  
19 | eadr_extend_prog_command | 3 | add this number the number of prog command to releave timing (??)  
20 | eadr_dcc_pom_repeat | 3 | Program on the main are repeated this number  
21 | eadr_dcc_speed_repeat | 3 | Speed commands are repeated this number   
22 | eadr_dcc_func_repeat  | 0 | Function Commands are repeated this number     
24 | eadr_dcc_default_format | DCC28 | default format: 0 =DCC14, 1=DCC27, 2=DCC28, 3=DCC128  
25 | eadr_railcom_enabled | 1 | dccout genereert de railcom cutout
26 | eadr_fast_clock_ratio | 8 | 1..31  
34 | eadr_short_turnoff_time | 8 | wait Xms before turning off power after  -> TODO, was in ticks, we gebruiken nu millis()  
35 | eadr_prog_short_toff_time | 40 | idem TODO check!  
36 | eadr_ext_stop_enabled  | 1 | 0=default, 1=enable external Stop Input  
37 | eadr_ext_stop_deadtime  |  30 | in ms  

## turnout numbering
- not sure why opendcc translates DCC accessory address = XPNET accessory address + 1  (build_nmra_basic_accessory function in organizer.cpp)  
-> according to the NMRA spec, DCC accessory address 0 is valid  
-> 0..511 are valid, with 511= broadcast, so 511 individual accessory decoders = 511*4=2044 turnouts supported  
-> the xpnet range is 0..255, smaller than DCC range, so no problem  
- JMRI counts turnouts from 1->MAX, turnout 1 is on XPNET accessory address = 0, and thus DCC accessory address = 1  
- CommandStation UI counts from 1, with same translation from XPNET to DCC accessory address, so aligned with JMRI turnout numbering.  

## feedback decoder numbering
- for the moment CommandStation supports feedback decoder addresses 0..31 (accessories.cpp)
-> these are not real dcc accessory addresses, because feedback decoders are not connected to DCC. But in xpnet feedback decoders use the same address range as accessory decoders
- feedback decoder below has 16 inputs, and hence occupies 2 successive accessory addresses
-> default accessory addresses : 10 + 11
-> in JMRI this translates to sensors 81 -> 96 (address X has sensors 8X+1 -> 8X+8), because JMRI counts from 1

## loc database
- in eeprom from address 0x40, 
- a store for a number of locs with their default DCC format, and a name (shown on the UI)
- functions to modify the loc database are implemented, but not hooked up to the UI for now. loc database is initialized using the default eeprom init sketch.

## note on DCC fast clock
- is a DCC extension proposed by W. Kufer, CommandStation has a builtin fast clock, and sends updates as DCC packets
- but fast clock is not part of standard xpnet, and not implemented in JRMI
- JMRI can be fast clock generator (or master), but doesn't send fast clock updates to CommandStation
- similarly, JMRI can synchronize with eg. loconet fast clock, but not xpnet : JMRI doesn't handle the xpnet broadcast of fast clock by the CommandStation (proprietary xpnet extension by Kufer not implemented in JMRI)
- as a result, CommandStation & JRMI cannot synchronize fast clocks over xpnet

## note on stealing locos between xpnet devices
- multiple xpnet devices can control the same loc decoder. A 'loc_stolen' event is sent to the device that lost control of a loc.
- JMRI implements a polling of the loc status (speed & functions) and maintains these when taking over control  
- local UI can steal a loc from another xpnet device, and takes over the current settings (speed & functions)
--> TODO for the throttle
- when a new loc is chosen from the CommandStation local UI, the previous loc is 'released' (i.e. will appear as 'free' to other xpnet devices)    
- release loco from xpnet device : 
--> no specific xpnet command available for this function
-->'Remove Loc from Command Station Stack' command is implemented this way in the CommandStation. On 'release' the loc is set inactive (it will not receive repeated speed/function instructions), and will appear 'free' to other xpnet devices. Another device can take over the loc with its current settings.  
- When JMRI receives the loc stolen notification, it continues to poll the CS for the loc status, and can steal the loc back at any time  
--> initially there was an issue with this, but solved after correctly implementing the F13_F28 Status Response (0xE4-0x51)

## todo
- modify timing for short detection on programming track at startup  
--> when the accessory decoder is powered from DCC, the inrush current on the 470uF smoothing capacitor is seen as a short on the programming track, and the programming is aborted  
--> timing for short detection is too tight  
--> same for programming a loc with smoothing capacitor
--> prog track could stay powered on as well? (but this doesn't solve the short detection by the inrush currents)
- CS blijft fast clock msgs sturen tijdens programming, is dat ok?
- accessory screen (for decoupler rails & lights)  
--> keys have toggle on/off function on the decoder outputs  
- turnout screen : key-up event should send 'off' turnout command (for test with accessory decoder with pulseDuration==0)
--> test various activation times for the turnouts based on how long key is pressed
- JMRI 0xE?-0x30 : juist implementeren in xpnet_parser  
-> dit is een algemeen xpnet msg die DCC packet encapsuleert
-> ok, nog checken of de POM commands nu nog goed worden uitgevoerd
- fast clock lijkt iets te snel te lopen
- CVs van CommandStation schrijven via programming?
- add checks on feedback addresses to avoid writing beyond allocated array memory!!!
- fast clock configuration in CommandStation (is now fixed, starts at 8:00 and speed via CV, but this can be more flexible over UI config)

# PcInterface
- a sketch that implements a LI101 PC Interface  
-> PC side : softSerial (pin 8=RX,9=TX) to usb-serial adapter, running default 19200 baud  
-> XPNET side : atmega uart1  
- works with JMRI connected to the softSerial  
- only basic hardware is required : atmega328 + MAX485 + usb-to-serial adapter
- notes:  
--> JMRI doesn't accept ACK for every command => parseCommand function updated to only ack requests that don't generate a command station response!  
--> rs485c - xpc : adapted so message bytes for other slots are discarded by the ISR, 
i.e. never make it to the RxBuffer = less work for xpc_Run()

## tested
- see DCC CommandStation

## stability
keeps working under constant polling from JMRI, especially during programming  
issue with occasional RX_ERRORS solved; xpnet command is now copied to rs485c with global ints disabled to garantee transmission in 1 slot.

## todo
- version STM8, requires AltSoftSerial  
-> port to STM8 TODO, not considered for now

# Accessory decoder (AVR + STM8)
- 2 modes : turnout decoder mode & output decoder mode (see below)  
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
- added NOP packet (RCN-213) to nmradcc lib, not sure yet what to do with it
--> CommandStation is not sending this packet anyway for now  

## Turnout Decoder Mode
- 8 physical outputs paired into 4 logical turnouts
- the decoder occupies 1 dcc accessory address with 4 turnouts  
- = 'factory default'

## Output Decoder Mode
- the decoder occupies 2 dcc accessory addresses with 8 on/off outputs  
- for compatibility with JMRI:
--> JMRI addresses lights similarly as turnouts, using 2 'coils'
--> output decoder : coil 0 activates output, coil 1 deactivates output

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

- code vereenvoudigen (timer.cpp en event.cpp nodig??)  
- dcc clock in dcc-rx ?
- Serial prints opkuisen
- nmra code opkuisen
- safe mode : needed?  
--> original idea was for a safe mode with all outputs 0 and fast blinking led if decoder has wrong CV settings, or a watchdog reset happens, or ...


## Implementation notes AVR

- momenteel nog geen POM, daarvoor moet NMRA_DCC_PROCESS_MULTIFUNCTION gedefined zijn.  
--> Maar daar is iets mis met ackCV() in POM, die stuurt gewoon een 60mA puls, maar dat werkt niet op main track, want de ack detector hw zit aan de prog sense resistor  
--> de dcc spec voorziet operations mode acknowledge via railcom, en dat hebben we nog niet  
--> wel service mode programming via NMRA_DCC_PROCESS_SERVICE_MODE
- OUTPUT_ADDRESS_MODE : removed from code (not used)  
- flag MY_ADDRESS_ONLY in nmradcc lib is not used to permit receiving DCC packets for all accessory addresses, including broadcast (accessory address 511). In 'normal' mode, the decoder address is filtered by turnout_decoder.c, and only own address packets are executed.
- added NOP packet (RCN-213) to nmradcc lib, not sure yet what to do with it
--> CommandStation is not sending this packet anyway for now  
- nmra lib also uses avr timer0 without disturbing millis()  
--> because timer scale/counter aren't touched, only output compare added!

## Implementation notes STM8

- fixed DCC_PIN : PA2 (update), because sduino has not yet implemented external interrupts on any pin, so we do it manually for this pin only.
- using timer2 for the 70us delay between edge interrupt and probing dcc level. could be done similarly as avr version with output compare on timer4 (millis timer), but timer2 is unused anyway, so keep it simple.  
- sdcc has problem with the if (ntf) ntf(...) construct for weak functions. Not only weak functions are not supported but if (ntf) returns false, even if ntf() is implemented! So these constructs had to be removed from nmradcc.c because callbacks were never called.
- outdated (older use of PB4 as DCC input, not compatible with I2C use for signaldecoder)
- PB4 doesn't have an internal weak pull-up -> pinMode(INPUT_PULLUP) has no effect. confirmed in datasheet.  
--> weirdly enough datasheet says there is no external interrupt on PB4. But it works, fortunately.  
- OUTPUT_ADDRESS_MODE : removed from code (not used)  
- flag MY_ADDRESS_ONLY in nmradcc lib is not used to permit receiving DCC packets for all accessory addresses, including broadcast (accessory address 511). In 'normal' mode, the decoder address is filtered by turnout_decoder.c, and only own address packets are executed.
- added NOP packet (RCN-213) to nmradcc lib, not sure yet what to do with it
--> CommandStation is not sending this packet anyway for now  


# Feedback Decoder (AVR + STM8)
- feedback decoder has 8 inputs local to the microcontroller, and 8 additional inputs over i2c expander (PCF8574)
- xpnet uses the same dcc accessory address range for feedback inputs
--> with 16 inputs the feedback decoder occupies 2 consecutive dcc addresses
--> feedback decoder has a header for connecting additional I/O expanders (ie. additional dcc addresses)
- feedback decoder is implemented as XPNET slave, and sends changes over XPNET using a personal extension (0x7X)
- the CommandStation XPNET stores the feedback for 32 dcc addresses, and broadcasts changes to all other xpnet slaves (JMRI in particular)
- inputs are default active low  
  -> avr/stm8 local inputs with internal pullup  
  -> expanded inputs are pulled up internally with 100uA  
  -> the feedback bit is then '1' if the pin value is LOW  
  .activeHigh = foresee option to define inputs as active high (not used for now)  
- 1 common feedback delay for all inputs
- configuration over CV variables, programmable over DCC. In normal operation the feedback decoder is not connected to DCC however. DCC only used for programming, esp. easy config of decoder address.  
--> well, easy, it doesn't fit in STM8 8kB yet...

## CVs
| CV#  | description     | default value |  remark  
| :--- | :---: | :---: | ---
|1  | address LSB | 10  | |
|9  | address MSB | 0  | |
|29 | configuration | 0x80 | read-only for now |
|8  | manufacturerID | 13 | read-only, write any value for factory reset |
|7  | versionNumber  | 1  | read-only |
|33  | # expanders | 1  | number of expanders determines the number of consecutive dcc accessory addresses occupied by the feedback decoder : 1 + CV33 |
| 34 | feedback delay | 50 | in ms, default = 50 |
| 35 | xpnet address | 5 | address of this xpnet slave |
 
## Implementation notes
- max inputs = 8 local + 8x8 expanded inputs = 72
--> the feedback decoder occupies multiple dcc addresses : 1 + #expanders (CV33)  
--> 1 dcc address for the local inputs (the address in CV1/9)  
--> 1 extra dcc address per expander (8 inputs)
- expanders must have consecutive i2c addresses (A0.A2); the first expander is on the feedback decoder print with A0.A2=0 (ie. i2C address = 0x20).  
--> the next expander has to have A0.A2 = 1 (i2c address 0x21)  
--> 8 expanders possible on the same i2c bus

## TODO
- check dat DCC address < 256 bij cv programming, want dat is de limiet voor xpnet
-  decoderState.xpnetConnectionOk = false tijdens service mode??
 -->    zal de CommandStation de feedback verwerken tijdens service mode??
- for STM8 blinking led & i2c are mutually exclusive! (same as signalDecoder todo)
- STM8 version : has DCC code removed for now to fit in 8kB
--> no possibility to program CV's over DCC (must be done by programming eeprom directly)

# Throttle
initial version with static xpnet address = 3.  
throttle controls loc speed on dcc loc address = 3, and shows dcc fast clock  

## todo
- UI
- encoder doesn't work well, probably linked to slow display refresh (compared to char-lcd)

# SignalDecoder (AVR+STM8)
an extended accessory decoder implementing various signal heads  
working on AVR & STM8
- AVR version has 20 outputs (12 on AVR + 8 over i/o expander)
- STM8 version has 19 outputs (11 on STM8 + 8 over i/o expander)
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

## Building compatible signals
on the prototype print 4 headers are foreseen, 1 header per signal head  
Each header has 6 pins, VCC + 5 led outputs  
led outputs are active low, ie. signal leds must have their anodes to ground.  
The signals need only 1 common current-limiting resistor for all leds (ie. all leds can be in parallel), because the led outputs are multiplexed by the decoder. This allows leds with different forward voltages to light 'together'.

## Configuring signal heads
STM8 version has 19 physical outputs, 11 directly on STM8, 8 additional over i2c expander.  
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

## CVs for configuring the signal aspects
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

## Implementation notes STM8
- fixed DCC_PIN : PA2, because sduino has not yet implemented external interrupts on any pin, so we do it manually for this pin only.
- DCC decoding using timer2 for the 70us delay between edge interrupt and probing dcc level. could be done similarly as avr version with output compare on timer4 (millis timer), but timer2 is unused anyway, so keep it simple.  
- 11 outputs directly on STM8, 8 additional over i2c expander.  
--> i2c/sda is shared with builtin led. 
--> during normal operation, i2c is used, and the builtin led is dimly lit during i2c operations  
--> During programming, the i2c bus is disabled (signal heads are paused) and PB5 is used as normal output to blink the builtin led   
--> i2c peripheral needs a SWRST before Wire_begin, otherwise i2c doesn't work anymore. Maybe the SWRST should be part of Wire_begin()??  
- NUM_OUTPUTS_PER_HEAD : 3 for now, can be up to 5
- timer1 used as 1ms interrupt for brightness/blinking timing of signal leds
- needed some optimisations to fit the 8kB flash  
--> used a reduced implementation of i2c (only TX required for the expander)  
--> used a simplified implementation of eeprom  
- STM8 version needs a light version of the Wire library with TX only in order to fit in 8kB flash  
- sdcc has problem with the if (ntf) ntf(...) construct for weak functions. Not only weak functions are not supported but if (ntf) returns false, even if ntf() is implemented! So these constructs had to be removed from nmradcc.c because callbacks were never called.
- OUTPUT_ADDRESS_MODE : removed from nmradcc lib code (not used)  
- flag MY_ADDRESS_ONLY in nmradcc lib is not used to permit receiving DCC packets for all accessory addresses, including broadcast (accessory address 511).
- added NOP packet (RCN-213) to nmradcc lib, not sure yet what to do with it
--> CommandStation is not sending this packet anyway for now  

# RAILCOM
- todo!

# TODO
- bedienpaneel wissels op xpnet (luistert naar feedback messages voor led-indicatie updates & knoppen voor bediening)
- esp als XnTcp interface of volledig CS  
--> looks problematic because esp hw-uart can't do 9-bit data
- dcc clock receiver  
--> dcc packet format is based on multifunction decoder feature expansion (CCC=110), with case DDDDD=00001 : not implemented in nmra lib
--> multifunction packets are not used yet in the decoders
--> CommandStation does send the fast clock dcc packets (organizer.cpp, build_dcc_fast_clock)



