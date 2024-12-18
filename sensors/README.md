# Sensors

## General

It seems to be like [tpms_ford.c](https://github.com/merbanan/rtl_433/blob/master/src/devices/tpms_ford.c). 

The total transmission is 160 bits. The transmission is OOK and then FSK modulated while car is standing or when using the 125kHz trigger tool (BC2T-1A203-AB). It changes to only FSK modulation while driving. The car however seems to only receive the FSK messages.

Preamble 16 Bits: `0101010101010101` = `0x5555`.  
Sync Word 16 Bits: `0101010101010110` = `0x5556`.

Eight Manchester encoded data bytes `II II II II PP TT FF CC`.

## Other (probably EV6T) type

`IIIIIIII`: ID, mine started with `4` in the first nibble.

`PP`: Pressure, as PSI * 4.

`TT`: Sequence increased after every transmission, if MSB is set.

`TT`: Temperature, in °C + 56, if MSB is not set.

`FF`: Flags:  
`00001010`(0x0A) or `10001010`(0x8A): while training with trigger tool.  
`00000110`(0x06): while car standing (once every few hours).  
`01000110`(0x46) or `11000110`(0xC6): while driving.  
Bit6 (0x40) seems to indicate driving, Bit3 (0x08) indicates learning while Bit2 (0x40) is off and Bit1 (0x02) is always on.
Maybe Bit8 (0x80) indicates a near empty battery.

`CC`: Sum of byte 0 to 6 = byte 7

Sequence counting in `TT` is transmitted every 15-16 seconds for pretty accurately for 10 minutes while driving.
The sequence starts with 0xC0 and increases each transmission.  
After that the telegram with temperature at `TT` is transmitted every minute.

Each message is repeated four times with 100ms distance with identical data.

## F2GT type

`IIIIIIII`: ID, mine started with `6` in the first nibble.

`PP`: Pressure, as PSI * 4.

`TT`: Unknown sequence, while Bit2(0x04) of flags is not set.

`TT`: Temperature, in °C + 56, while Bit2(0x04) of flags is set.

`FF`: Flags:  
`00010100`(0x14): while training with trigger tool.  
`00000101`(0x05) or `00000110`(0x06): while car standing (once every few hours).  
`01001011`(0x4B): while driving and transmitting unknown sequence.  
`01000110`(0x46): while driving and transmitting temperature.  
Bit7 (0x40) seems to indicate driving, Bit4 (0x10) indicates learning.
Bit2 (0x04) most likely is on while transmitting temperature and Bit3 (0x08) is on while transmitting sequence.
Bit0 (0x01) and Bit1 (0x02) is unknown.

`CC`: Sum of byte 0 to 6 = byte 7

Unknown sequence `TT` is transmitted every 17-18 seconds for pretty accurately for 7 minutes while driving.  
After that the telegram with temperature at `TT` is transmitted every minute.

Each message is repeated four times with 100ms distance with increased sequence counter or repeated while transmitting temperature.

![F2GT Sensor](../pic/f2gt-sensor.jpg)