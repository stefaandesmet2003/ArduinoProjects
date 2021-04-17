# Accessory decoder
- momenteel nog geen POM, daarvoor moet NMRA_DCC_PROCESS_MULTIFUNCTION gedefined zijn.  
--> Maar daar is iets mis met ackCV() in POM, die stuurt gewoon een 60mA puls, maar dat werkt niet op main track, want de ack detector hw zit aan de prog sense resistor  
--> de dcc spec voorziet operations mode acknowledge via railcom, en dat hebben we nog niet  
--> wel service mode programming via NMRA_DCC_PROCESS_SERVICE_MODE

- reprogramming own decoder address on main track (learning mode):  
--> activate programming mode (long key press) and send a DCC turnout command.  
--> The DCC decoderAddress is taken as the new accessory decoder address  
--> flag MY_ADDRESS_ONLY in nmradcc lib is not used to permit receiving DCC packets for all accessory addresses. In 'normal' mode, the decoder address is filtered by turnout_decoder.c, and only own address packets are executed.

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
- hoe komt het dat millis() werkt als nmra ook de timer0 gebruikt???
--> timer scale verandert niet, enkel output compare toegevoegd  
- configuraties voor wisseldecoder, seindecoder etc maken  
- code vereenvoudigen (timer.cpp en event.cpp nodig??)  
- dcc clock in dcc-rx ?
- Serial prints opkuisen
- nmra code opkuisen
