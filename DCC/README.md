# CommandStation LENZ
configuration for command station with LENZ PC interface on uart1

## CV's
voorlopig niet mogelijk om CV's van de command station te lezen/schrijven
-> code is gecomment, nog iets te voorzien via de UI (key_go_at_start_pressed, dat is opendcc, dat werkt hier niet)

// TODO 2021 : to delete  
// eadr_CTS_usage  
// s88*  
// eadr_dcc_acc_time  
// eadr_startmode_ibox  
// eadr_feedback_s88_offset  
// eadr_feedback_s88_type  
// eadr_s88_clk_timing  
// eadr_feedback_s88_size  
// eadr_s88_total_from_pc  
// eadr_I2C_present  
// eadr_serial_id  

#0  : eadr_OpenDCC_Version      : wordt in de code gebruikt, en moet dus in eeprom staan, maar kan je niet uitlezen, daarom CV5  
#1  : eadr_baudrate             : 1, DEFAULT_BAUD 19200  
#2  : eadr_OpenDCC_Mode         : is read-only, maar reflecteert niet de compile flags, omdat eeprom niet systematisch wordt mee geprogrammeerd in arduino IDE  
#3  : eadr_virtual_decoder_l    : TODO wadisda?  
#4  : eadr_virtual_decoder_h    : TODO wadisda?  
#5  : eadr_VersionMirror        : == CV0  
#12 : eadr_invert_accessory     : 0x1 TODO : check of dit gebruikt wordt, en juist, en nodig?  
#13 : eadr_dcc_acc_repeat       : =2, Accessory Commands are repeated this number of times  
#18 : eadr_extend_prog_resets   : =3,  add this number the number of resets command during programming  
#19 : eadr_extend_prog_command  : =3, add this number the number of prog command to releave timing (??)  
#20 : eadr_dcc_pom_repeat       : =3, Program on the main are repeated this number  
#21 : eadr_dcc_speed_repeat     : =3, Speed commands are repeated this number   
#22 : eadr_dcc_func_repeat      : =0, Function Commands are repeated this number     
#24 : eadr_dcc_default_format   : =DCC28, default format: 0 =DCC14, 1=DCC27, 2=DCC28, 3=DCC128  
#25 : eadr_railcom_enabled      : =0, TODO wat doet dit?   
#26 : eadr_fast_clock_ratio     : =8, 1..31  
#29 : eadr_xpressnet_feedback   : =0 -> moet weg, is opendcc specifiek   
#33 : eadr_short_turnoff_time   : =8, wait Xms before turning off power after  
-> TODO, was in ticks, we gebruiken nu millis()  
#33 : eadr_prog_short_toff_time : =40, idem TODO check!  
#33 : eadr_ext_stop_enabled     : =1, 0=default, 1=enable external Stop Input  
#33 : eadr_ext_stop_deadtime    : =30, in ms  

## turnout numbering
not sure why opendcc translates DCC accessory address = XPNET accessory address + 1  (build_nmra_basic_accessory function in organizer.cpp)
according to the NMRA spec, DCC accessory address 0 is valid  
0..511 are valid, with 511= broadcast, so 511 individual accessory decoders = 511*4=2044 turnouts supported
the xpnet range is 0..255, smaller than DCC range, so no problem
JMRI counts turnouts from 1->MAX, turnout 1 is on XPNET accessory address = 0, and thus DCC accessory address = 1
CommandStation UI counts from 0, with same translation from XPNET to DCC accessory address --> todo align with JMRI ao.

## todo
- turnout feedback herimplementeren zonder s88  
- JMRI 0xE?-0x30 : juist implementeren in lenz_parser  
-> dit is een algemeen xpnet msg die DCC packet encapsuleert  
-> code die do_extended_accessory callt in de parsers mag weg  
- locobuffer uitmesten (ook startup checken, DCC default format etc. nodig?)
- lenz parser cleanup
- dccout uitmesten (opendcc feedback stuff eruit)
- UI : count turnouts from 1 instead of 0

# CommandStation XPNET
configuration for command station with XPNET interface on uart1  
needs RS485 PHY for xpnet  
works with PcInterface (connection with JMRI)  
## tested :
- loco control via local UI & JMRI, JMRI receives loc stolen message
- turnout control via local UI & JMRI, turnout feedback doesn't work yet (s88 functionality to be reimplemented)
- track power on/off, on/off/short notifications
- not all possible commands in the parser have been tested!!  

## todo
- lok_stolen_by_pc etc -> rework locobuffer & organizer (orgz_old_lok_owner=dirty, ".manual_operated" field : can be removed? )  
- release loco from local UI :  
  --> not sure how JMRI can steal a loc, it keeps polling the lost loco but stops after a while  
  --> and no way to steal it back ??  
- loc stolen notification on local UI
- appstub clean-up
- there is an issue in programming mode : JMRI polls too fast, and results in communication errors (rs485c RxBuffer overflow?), and it stops the programming
- modify timing for short detection on programming track at startup  
--> when the accessory decoder is powered from DCC, the inrush current on the 470uF smoothing capacitor is seen as a short on the programming track, and the programming is aborted  
--> timing for short detection is too tight  

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
keeps working under constant polling from JMRI, but with RX_ERRORS (buffer overrun probably) when JMRI uses every slot  
--> JMRI was polling very fast on a few wrongly implemented command responses; because it wasn't receiving the right answer  
--> JMRI logs showed occasional 0x1 - 0x3 - 0x2 = "comms error"  
no crashes so far

# Accessory Decoder
working version of a turnout decoder  
2 versions : atmega328 & STM8  
see readme there  
## todo
- extended accessory (signals)

# Feedback Decoder
working version with xpressnet as feedback bus  
the basic version uses 8 inputs on atmega328 and sends the status over xpressnet

## todo
- configuration using variables stored in eeprom
Lenz uses a DCC interface to program the configuration as CV variables.  
Feedback decoder is not connected to DCC on the track, but probably the easiest way, the DCC input only requires an opto & few resistors

# RAILCOM
- todo!

# TODO
- xpnet throttle
- bedienpaneel wissels op xpnet (luistert naar feedback messages voor led-indicatie updates & knoppen voor bediening)
- esp als XnTcp interface of volledig CS
- dcc clock



