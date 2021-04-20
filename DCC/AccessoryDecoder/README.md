# Accessory decoder
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
- added NOP packet (RCN-213) to nmradcc lib, not sure yet what to do with it
--> CommandStation is not sending this packet anyway for now  

## Implementation notes

- momenteel nog geen POM, daarvoor moet NMRA_DCC_PROCESS_MULTIFUNCTION gedefined zijn.  
--> Maar daar is iets mis met ackCV() in POM, die stuurt gewoon een 60mA puls, maar dat werkt niet op main track, want de ack detector hw zit aan de prog sense resistor  
--> de dcc spec voorziet operations mode acknowledge via railcom, en dat hebben we nog niet  
--> wel service mode programming via NMRA_DCC_PROCESS_SERVICE_MODE
- OUTPUT_ADDRESS_MODE : removed from code (not used)  
- flag MY_ADDRESS_ONLY in nmradcc lib is not used to permit receiving DCC packets for all accessory addresses, including broadcast (accessory address 511). In 'normal' mode, the decoder address is filtered by turnout_decoder.c, and only own address packets are executed.
- added NOP packet (RCN-213) to nmradcc lib, not sure yet what to do with it
--> CommandStation is not sending this packet anyway for now  
- hoe komt het dat millis() werkt als nmra ook de timer0 gebruikt???
--> timer scale verandert niet, enkel output compare toegevoegd  

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
| 1 : light decoder
In software mode = 0 :   
| 3 | timeOnOutput1 | 5 | x50ms, =250ms default coil activation time  |
| 4 | timeOnOutput2 | 5 | x50ms, =250ms default coil activation time  |
| 5 | timeOnOutput3 | 5 | x50ms, =250ms default coil activation time  |
| 6 | timeOnOutput4 | 5 | x50ms, =250ms default coil activation time  |

## TODO
- bug : led keeps blinking after factory reset  
- programming on main is supported in nmradcc lib, but compiled out for now (NMRA_DCC_PROCESS_MULTIFUNCTION undefined)
- getMyAddr -> simplify to avoid reading eeprom on every DCC packet  
- test other decoder types included in nmradcc lib (mobile decoder etc)
- replace the timer/event bullshit code


- configuraties voor wisseldecoder, seindecoder etc maken  
- code vereenvoudigen (timer.cpp en event.cpp nodig??)  
- dcc clock in dcc-rx ?
- Serial prints opkuisen
- nmra code opkuisen
