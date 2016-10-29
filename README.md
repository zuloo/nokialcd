# nokialcd

### Python Class to use a Nokia 6100  Display with Epson Controller on a Raspberry Pi

There is also some stuff for the Phillips Controller, but I lack a
Display with that Controller to test it.

Have a look at test.py to see an usage example

I use native SPI to communicate with the Display - no Bitbanging -
but I have to use 2 byte for every 9-bit-word the display expects.

Performance is ok, but do not expect more than 1/2 to 1 FPS.

License is MIT or something.
