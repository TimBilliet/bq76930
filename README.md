# ESP-IDF BQ76930 driver
Many thanks to https://github.com/mikethezipper for the original Arduino code.

## Info
This version only supports 10 cells, no temperature readings, no discharge protection.\
This version has only been tested on the IC with CRC.

Things that currently work:
- Cell voltage reading
- Pack voltage reading
- Charge current reading
- Overvoltage protection
- Undervoltage detection (protection hasn't been tested since i have a charge only bms)
- Balancing

# Fixes/changes of the original Arduino code.
- Charge current wasn't calculated properly, the shunt resistor value wasn't being used. A hardcoded number was used.
- current reading was not using the CC_HI and CC_LOW registers from header file
- Added ability do disable charge FET
