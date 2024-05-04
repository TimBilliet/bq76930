# This is currently a Work In Progress
Many thanks to https://github.com/mikethezipper for the original Arduino code.
## Info
This version only supports 10 cells, no temperature readings, no discharge protection.\
This version has only been tested on the IC with CRC.\
Currently the current measurement does not work yet.\
Things that do work:
- Cell voltage reading
- Overvoltage protection
- Undervoltage protection
- Balancing

# Fixes/changes of the original Arduino code.
- Charge current wasn't calculated properly, the shunt resistor value wasn't being used. A hardcoded number was used.
- current reading was not using the CC_HI and CC_LOW registers from header file
- Added ability do disable charge FET
