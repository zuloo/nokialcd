#!/usr/bin/env python
# -*- coding: utf8 -*-

import RPi.GPIO as GPIO
from PIL import Image
import spidev
import time

class PCF: # philips controller functions
    MODID   = 'PCF'

    NOP     = 0x00  # no operation

    SWRESET = 0x01  # software reset

    BSTROFF = 0x02  # booster voltage off
    BSTRON  = 0x03  # booster voltage on

    RDDIDIF = 0x04  # read display identification
    RDDST   = 0x09  # read display status

    SLPIN   = 0x0A  # sleep in
    SLPOUT  = 0x0B  # sleep out

    PTLON   = 0x0C  # partial display mode on
    NORON   = 0x0D  # normal display mode on

    INVOFF  = 0x14  # display inversion off
    INVON   = 0x15  # display inverion on

    DALO    = 0x16  # all pixel off
    DAL     = 0x17  # all pixel on

    SETCON  = 0x19  # set contrast 

    DISPOFF = 0x28 # display OFF
    DISPON  = 0x29 # display ON

    CASET   = 0x2A # column address set
    PASET   = 0x2B # page address set

    RAMWR   = 0x2C # memory write
    RGBSET  = 0x2D # colour set
    PTLAR   = 0x30 # partial area

    VSCRDEF = 0x33 # vertical scrolling definition

    TEOFF   = 0x34 # test mode off
    TEON    = 0x35 # test mode on

    MADCTL  = 0x36 # memory access control

    SEP     = 0x37 # vertical scrolling start address

    IDMOFF  = 0x38 # idle mode OFF
    IDMON   = 0x39 # idle mode ON

    COLMOD  = 0x3A # interface pixel format

    SETVOP  = 0xB0 # set Vop

    BRS     = 0xB4 # bottom row swap
    TRS     = 0xB6 # top row swap

    DISCTL = 0xB9 # display control

    DOR     = 0xBA # data order

    TCDFE   = 0xBD # enable/disable DF temperature compensation
    TCVOPE  = 0xBF # enable/disable Vop temp comp

    EC      = 0xC0 # internal or external oscillator

    SETMUL  = 0xC2 # set multiplication factor

    TCVOPAB = 0xC3 # set TCVOP slopes A and B
    TCVOPCD = 0xC4 # set TCVOP slopes c and d
    
    TCDF    = 0xC5 # set divider frequency

    DF8COLOR = 0xC6 # set divider frequency 8-color mode

    SETBS   = 0xC7 # set bias system

    RDTEMP  = 0xC8 # temperature read back

    NLI     = 0xC9 # n-line inversion

class EPS: # epson controller functions
    MODID   = 'EPS'

    NOP     = 0x25 # NOP instruction

    DISPON  = 0xAF # Display on
    DISPOFF = 0xAE # Display off
    
    DISNOR  = 0xA6 # Normal display
    DISINV  = 0xA7 # Inverse display

    COMSCN  = 0xBB # Common scan direction
    
    DISCTL  = 0xCA # Display control

    SLPIN   = 0x95 # Sleep in
    SLPOUT  = 0x94 # Sleep out

    PASET   = 0x75 # Page address set
    CASET   = 0x15 # Column address set
    
    DATCTL  = 0xBC # Data scan direction, etc.

    RGBSET8 = 0xCE # 256-color position set

    RAMWR   = 0x5C # Writing to memory
    RAMRD   = 0x5D # Reading from memory

    PTLIN   = 0xA8 # Partial display in
    PTLOUT  = 0xA9 # Partial display out

    RMWIN   = 0xE0 # Read and modify write
    RMWOUT  = 0xEE # End

    ASCSET  = 0xAA # Area scroll set
    SCSTART = 0xAB # Scroll start set

    OSCON   = 0xD1 # Internal oscillation on
    OSCOFF  = 0xD2 # Internal oscillation off

    PWRCTL  = 0x20 # Power control

    VOLCTL  = 0x81 # Electronic volume control
    VOLUP   = 0xD6 # Increment electronic control by 1
    VOLDOWN = 0xD7 # Decrement electronic control by 1

    TMPGRD  = 0x82 # Temperature gradient set

    EPCTIN  = 0xCD # Control EEPROM
    EPCOUT  = 0xCC # Cancel EEPROM control
    EPMWR   = 0xFC # Write into EEPROM
    EPMRD   = 0xFD # Read from EEPROM

    EPSRRD1 = 0x7C # Read register 1
    EPSRRD2 = 0x7D # Read register 2

class LCD:

    def __init__(self, dev='/dev/spidev0.0', spd=1000, reset_pin=16, model='EPS'):
        self.__model = model

        if self.__model == EPS.MODID:
            self.NOOP = EPS.NOP
        elif self.__model == PCF.MODID:
            self.NOOP = PCF.NOP
        else:
            self.NOOP = 0x00

        self.__bit_buffer = []
        self.__prev_byte = None
        self.__ncur_byte = 0

        self.__reset_pin = reset_pin

        self.__prev_color = None

        self.v_res = 131
        self.h_res = 131

        self.spi = spidev.SpiDev()
        self.spi.open(0,1)
        #self.spi.bits_per_word = 9
        #SPI(dev, 3, spd, bits_per_word=8)

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.__reset_pin, GPIO.OUT)
        GPIO.output(self.__reset_pin, 1)

        if self.__model == EPS.MODID:
            self.__init_epson()
        elif self.__model == PCF.MODID:
            self.__init_philips()

    def __write_spi(self, byte, cmd=False):
        out1 =  (byte >> 1)
        out2 = 0x00 | (byte << 7)
        
        if cmd:
            out1 = out1 & ~(1 << 7)
        else:
            out1 = out1 | (1 << 7)

        self.spi.xfer([out1,out2])
        return

    def __write_spi_long(self, byte, cmd=False):
        out = 0x00
        
        # shift byte ncur_byte positions to the right
        out = (byte >> (self.__ncur_byte + 1))

        if self.__ncur_byte > 0:
            # and combine with last byte send if not the first byte
            out = (byte >> (self.__ncur_byte + 1)) | (self.__prev_byte << (8 - self.__ncur_byte))

        # set first bit of 9bit-block to 0 if cmd, else to 1 for data
        if cmd:
            out = out & ~(1 << (7 - self.__ncur_byte))
        else:
            out = out | (1 << (7 - self.__ncur_byte))

        # store for next invokation
        self.__bit_buffer.append(out)
        self.__prev_byte = byte

        # fix the last byte and flush to spi if self.__ncur_byte >= 7:
        if len(self.__bit_buffer) >= 7:
            # last byte fits fully into the final 9th bute of the buffer
            self.__bit_buffer.append(byte)
            # write to spi
            self.spi.xfer(self.__bit_buffer)

            # reset
            self.__bit_buffer = []
            self.__prev_byte = 0x00
            self.__ncur_byte = 0
        else:
            self.__ncur_byte = self.__ncur_byte + 1

    def __flush_spi(self, noop=None ):
        return
        nop = self.NOOP
        if noop is not None:
            nop = noop

        while self.__ncur_byte > 0:
            self.__write_spi(noop, cmd=True)

    def cmd(self, byte):
        self.__write_spi(byte, cmd=True)

    def data(self, byte):
        self.__write_spi(byte)

    def flush(self):
        self.__flush_spi(self.NOOP)

    def __reset_hard(self):
        GPIO.output(self.__reset_pin, 0)
        time.sleep(2)
        GPIO.output(self.__reset_pin, 1)
        time.sleep(2)
    
    def __init_epson(self):
        self.__reset_hard()

        # Display Control, set Display timing, duty setting and inversily highlighting
        self.cmd(EPS.DISCTL)
        self.data(0x02) # P1: 0x00= 2 divisions, switching period 8
        self.data(0x20) # P2: 0x20= nlines/4-1 = 132/4-1 = 32
        self.data(0x00) # P3: 0x00= no inversely highlighted lines

        # common output scan direction - experimental value of the internet
        self.cmd(EPS.COMSCN)
        self.data(0x01) # P1: 0x01= Scan 1->80,160<-81

        # Oscillator on
        self.cmd(EPS.OSCON)

        # Sleep out
        self.cmd(EPS.SLPOUT)
        self.flush()

        # turn on volatage regulatore
        self.cmd(EPS.PWRCTL)
        self.data(0x0f) # reference voltage reg on, circuit voltage follower on, BOOST on
        
        # invert Display so colors are shown correctly
        #self.cmd(EPS.DISINV)

        # setup Display mode, rgb sequence and addressing
        self.cmd(EPS.DATCTL)
        self.data(0x00) # P1: 0x01= page addr inverted, col addr normal, addr scan in col direction
        self.data(0x00) # P2: 0x00= RGB sequence (default)
        self.data(0x02) # P3: 0x02= Grayscale -> 16 (selects 12-bit color, type A)

        # set contrast
        self.cmd(EPS.VOLCTL)
        self.data(0x1A) # P1: 32 Volume value [0-63]
        self.data(0x03) # P2: 3 resistance ratio
        self.flush()

        # allow power supply to stabilize
        time.sleep(1)

        # turn on the display
        self.cmd(EPS.DISPON)

    def __init_philips(self):
        # sleep out
        self.cmd(PCF.SLPOUT)

        # inversion on
        self.cmd(PCF.INVON)

        # set color interface pixel format
        self.cmd(PCF.COLMOD)
        self.data(0x03) # P1: 0x03= 12 bits per pixel

        # memory access controler
        self.cmd(PCF.MADCTL)
        self.data(0xC8) # P1: 0xC8= mirror x and y, reverse rgb

        # write contrast
        self.cmd(PCF.SETCON)
        self.data(0x30)

        # allow to stabilize
        time.sleep(0.2)

        # turn Display on
        self.cmd(PCF.DISPON)

    def reset(self):
        if self.__model == PCF.MODID:
            self.__init_philips()
        elif self.__model == EPS.MODID:
            self.__init_epson()

    def display_off(self):
        if self.__model == PCF.MODID:
            self.cmd(PCF.DISPOFF)
        elif self.__model == EPS.MODID:
            self.cmd(EPS.DISPOFF)

    def display_on(self):
        if self.__model == PCF.MODID:
            self.cmd(PCF.DISPON)
        elif self.__model == EPS.MODID:
            self.cmd(EPS.DISPON)

    def column_address_set(self, x0, x1):
        """set column address from a specified to a specified x-coord
        """
        if self.__model == PCF.MODID:
            self.cmd(PCF.CASET)
        elif self.__model == EPS.MODID:
            self.cmd(EPS.CASET)

        self.data(x0)
        self.data(x1)

    def page_address_set(self, y0, y1):
        """set page address from a specified to a specified y-coord
        """
        if self.__model == PCF.MODID:
            self.cmd(PCF.PASET)
        elif self.__model == EPS.MODID:
            self.cmd(EPS.PASET)

        self.data(y0)
        self.data(y1)

    def set_area(self, start_point, end_point):
        """set up area to from specified point (x,y) to a specified point (x,y) 
           setup mirror mode accordingly and enter RAMWR mode
        """
        self.column_address_set(start_point[0],end_point[0])
        self.page_address_set(start_point[1],end_point[1])

        if self.__model == PCF.MODID:
            self.cmd(PCF.RAMWR)
        elif self.__model == EPS.MODID:
            self.cmd(EPS.RAMWR)

    def set_pixel(self, color):
        """Set a pixel to a specific color
           This method will store each odd pixel collor and will write in every other bit
               Data format: 2-pixes-per-3-bytes
                     BBBB GGGG | RRRR BBBB | GGGG RRRR
        """
        r = (color[2]>>4) ^ 0xFF
        g = (color[1]>>4) ^ 0xFF
        b = (color[0]>>4) ^ 0xFF
        if self.__prev_color is None:
            self.__prev_color = (r, g, b)
        else:
            byte1=((self.__prev_color[2]<<4) | (self.__prev_color[1] & 0x0F)) & 0xFF
            byte2=((self.__prev_color[0]<<4) | (b & 0x0F)) & 0xFF
            byte3=((g<<4) | (r & 0x0F)) & 0xFF
            self.data(byte1)
            self.data(byte2)
            self.data(byte3)
            self.__prev_color = None

    def clear_screen(self, color):
        self.set_area((0,0),(self.v_res, self.h_res))
        for i in range(0,self.v_res*self.h_res):
            self.set_pixel(color)

    def display_image(self, path):
        # TODO: find out if this needs to be (0,0)
        self.set_area((0,0),(self.v_res, self.h_res))
        bitmap = Image.open(path)
        pixels = list(bitmap.getdata())
        pixels.reverse()
        for p in pixels:
            self.set_pixel(p)


    def cleanup(self):
        GPIO.cleanup()
