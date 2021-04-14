# CommandStation LENZ
configuration for command station with LENZ PC interface on uart1

## CV's
voorlopig niet mogelijk om CV's van de command station te lezen/schrijven
-> code is gecomment, nog iets te voorzien via de UI (key_go_at_start_pressed, dat is opendcc, dat werkt hier niet)

#0  : eadr_OpenDCC_Version      : wordt in de code gebruikt, en moet dus in eeprom staan, maar kan je niet uitlezen, daarom CV5  
#1  : eadr_baudrate             : 1, DEFAULT_BAUD 19200  
#2  : eadr_OpenDCC_Mode         : is read-only, maar reflecteert niet de compile flags, omdat eeprom niet systematisch wordt mee geprogrammeerd in arduino IDE  
#5  : eadr_VersionMirror        : == CV0  
#13 : eadr_dcc_acc_repeat       : =2, Accessory Commands are repeated this number of times  
#18 : eadr_extend_prog_resets   : =3,  add this number the number of resets command during programming  
#19 : eadr_extend_prog_command  : =3, add this number the number of prog command to releave timing (??)  
#20 : eadr_dcc_pom_repeat       : =3, Program on the main are repeated this number  
#21 : eadr_dcc_speed_repeat     : =3, Speed commands are repeated this number   
#22 : eadr_dcc_func_repeat      : =0, Function Commands are repeated this number     
#24 : eadr_dcc_default_format   : =DCC28, default format: 0 =DCC14, 1=DCC27, 2=DCC28, 3=DCC128  
#25 : eadr_railcom_enabled      : =1, dccout genereert de railcom cutout
#26 : eadr_fast_clock_ratio     : =8, 1..31  
#34 : eadr_short_turnoff_time   : =8, wait Xms before turning off power after  
-> TODO, was in ticks, we gebruiken nu millis()  
#35 : eadr_prog_short_toff_time : =40, idem TODO check!  
#36 : eadr_ext_stop_enabled     : =1, 0=default, 1=enable external Stop Input  
#37 : eadr_ext_stop_deadtime    : =30, in ms  

## turnout numbering
not sure why opendcc translates DCC accessory address = XPNET accessory address + 1  (build_nmra_basic_accessory function in organizer.cpp)
according to the NMRA spec, DCC accessory address 0 is valid  
0..511 are valid, with 511= broadcast, so 511 individual accessory decoders = 511*4=2044 turnouts supported
the xpnet range is 0..255, smaller than DCC range, so no problem
JMRI counts turnouts from 1->MAX, turnout 1 is on XPNET accessory address = 0, and thus DCC accessory address = 1
CommandStation UI counts from 0, with same translation from XPNET to DCC accessory address --> todo align with JMRI ao.

## todo
- JMRI 0xE?-0x30 : juist implementeren in lenz_parser  
-> dit is een algemeen xpnet msg die DCC packet encapsuleert
-> ok, nog checken of de POM commands nu nog goed worden uitgevoerd
- locobuffer uitmesten (ook startup checken, DCC default format etc. nodig?)
- lenz parser cleanup
- fast clock lijkt iets te snel te lopen
- CVs van CommandStation schrijven via programming?

# CommandStation XPNET
configuration for command station with XPNET interface on uart1  
needs RS485 PHY for xpnet  
works with PcInterface (connection with JMRI)  
## tested :
- loco control via local UI & JMRI, JMRI receives loc stolen message
- turnout control via local UI & JMRI
- turnout feedback, possibility to assign feedback decoders to provide turnout feedback  
--> use a feedback decoder with the same address as the corresponding accessory decoder. Since the feedback decoder is not connected to DCC (except during programming), there is no problem with this approach  
--> or use an accessory decoder with additional xpnet interface providing feedback directly.
- track power on/off, on/off/short/external-stop notifications  
- not all possible commands in the parser have been tested!!  
- turnout control from local UI broadcasted over xpnet

## todo
- lok_stolen_by_pc etc -> rework locobuffer & organizer (orgz_old_lok_owner=dirty, ".manual_operated" field : can be removed? )  
- release loco from local UI :  
  --> not sure how JMRI can steal a loc, it keeps polling the lost loco but stops after a while  
  --> and no way to steal it back ??  
- loc stolen notification on local UI
- appstub clean-up
- modify timing for short detection on programming track at startup  
--> when the accessory decoder is powered from DCC, the inrush current on the 470uF smoothing capacitor is seen as a short on the programming track, and the programming is aborted  
--> timing for short detection is too tight  
- CS blijft fast clock msgs sturen tijdens programming, is dat ok?

# PcInterface
a sketch that implements a LI101 PC Interface  
-> PC side : softSerial (pin 8=RX,9=TX) to usb-serial adapter, running default 19200 baud  
-> XPNET side : atmega uart1  
works with JMRI connected to the softSerial  
JMRI doesn't accept ACK for every command => parseCommand function updated to only ack requests that don't generate a command station response!  
rs485c - xpc : adapted so message bytes for other slots are discarded by the ISR, 
i.e. never make it to the RxBuffer = less work for xpc_Run()
## tested
- see CommandStationXPNET  

## stability
keeps working under constant polling from JMRI, especially during programming  
issue with occasional RX_ERRORS solved; xpnet command is now copied to rs485c with global ints disabled to garantee transmission in 1 slot.

# Accessory Decoder
working version of a turnout decoder  
2 versions : atmega328 & STM8  
see readme there  

# Feedback Decoder
working version with xpressnet as feedback bus  
the basic version uses 8 inputs on atmega328 and sends the status over xpressnet

## todo
- configuration using variables stored in eeprom
Lenz uses a DCC interface to program the configuration as CV variables.  
Feedback decoder is not connected to DCC on the track, but probably the easiest way, the DCC input only requires a fast opto & few resistors

# Throttle
initial version with static xpnet address = 3.  
throttle controls loc speed on loc address = 3, and shows dcc fast clock  

## todo
- UI
- encoder doesn't work well, probably linked to slow display refresh (compared to char-lcd)

# SignalDecoder
an extended accessory decoder implementing various signal heads  
a working prototype on atmega328 for now  
signal heads can be controlled via JMRI over PcInterface (command station UI does not implement extended accessories)

## TODO
- configuration over CV
- use STM8

# RAILCOM
- todo!

# TODO
- bedienpaneel wissels op xpnet (luistert naar feedback messages voor led-indicatie updates & knoppen voor bediening)
- esp als XnTcp interface of volledig CS  
--> looks problematic because esp hw-uart can't do 9-bit data
- dcc clock



