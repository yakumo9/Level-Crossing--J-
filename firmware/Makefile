# Fuse extended byte:
# 0x07 = 0 0 0 0   0 1 1 1 <-- BOOTRST (boot reset vector at 0x0000)
#        ^ ^ ^ ^   ^ ^ ^------ BOOTSZ0
#        | | | |   | +-------- BOOTSZ1
# Fuse high byte:
# 0xdf = 1 1 0 1   1 1 1 1 <-- BODLEVEL0
#        ^ ^ ^ ^   ^ ^ ^------ BODLEVEL1
#        | | | |   | +-------- BODLEVEL2
#        | | | |   +---------- EESAVE (set to 0 to preserve EEPROM over chip erase)
#        | | | +-------------- WDTON (if set to 0, watchdog is always on)
#        | | +---------------- SPIEN (if set to 1, serial programming is disabled)
#        | +------------------ DWEN 
#        +-------------------- RSTDISBL (if set to 0, RESET pin is disabled)
