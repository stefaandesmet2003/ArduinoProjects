"C:\Program Files (x86)\Arduino\hardware\tools\avr\bin\avrdude" -C"C:\Program Files (x86)\Arduino\hardware\tools\avr\etc\avrdude.conf" -v -patmega328p -cstk500v1 -PCOM4 -b19200 -D -Ueeprom:w:OpenDccCommandStation.cpp.eep:i 