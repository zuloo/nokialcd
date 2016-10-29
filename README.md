# nokialcd

### Python Class to use a Nokia 6100  Display with Epson Controller on a Raspberry Pi

There is also some stuff for the Phillips Controller, but I lack a
Display with that Controller to test it.

Have a look at test.py to see an usage example

I use native SPI to communicate with the Display - no Bitbanging -
but I have to use 2 byte for every 9-bit-word the display expects.

Performance is ok, but do not expect more than 1/2 to 1 FPS.

### Additional material

I also added some Eagel Files from Sparkfun in the PCB folder
so you can produce a Breakout PCB,since it no longer available on Sparkfun.

The Datasheets can be found in DATASHEET as well, as an excellant Documentation from
James P. Lynch, which helped me a lot getting this thing to work properly.

### Wiring

If you use the Sparkfun Board connect the Pins as Follow

|RPi|PCB|
|-|-|
|MOSI|DIO|
|SCLK|SCK|
|CS1|CS|
|GPIO #21|RESET|
|GND|GND|
|3.3V|3.3V|
|5V|VBATT|

that should do it

### License
License is MIT or something.
