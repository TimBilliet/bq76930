# This is currently a Work In Progress
Many thanks to https://github.com/mikethezipper for the original Arduino code.

## This does currently not work, there are some problems with writing the registers on the BMS IC.
## Info
This version only supports 10 cells, no temperature readings, no discharge protection.\
This version has only been tested on the IC with CRC.\

Things that do work:
- Cell voltage reading
- Overvoltage protection
- Undervoltage protection
- Balancing
- Charge current measurement

# Fixes/changes of the original Arduino code.
- Charge current wasn't calculated properly, the shunt resistor value wasn't being used. A hardcoded number was used.
- current reading was not using the CC_HI and CC_LOW registers from header file
- Added ability do disable charge FET
