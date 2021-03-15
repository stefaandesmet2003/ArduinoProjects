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

## todo
- turnout feedback herimplementeren zonder s88  
- JMRI 0xE?-0x30 : juist implementeren in lenz_parser  
-> dit is een algemeen xpnet msg die DCC packet encapsuleert  

