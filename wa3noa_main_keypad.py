#####################################################################################
# FM Deviation generator - Raspberry Pi Pico										#
# 	and AD9580 DDS module with SSD1306 OLED											#
#																					#
# Pico:		Pin# / Function ... AD9850:	Pin # / Function ... SSD1306:	Function	#
# 			36		3V3					1,20	Vcc						Vcc			#
#			13,18,28	GND				6,11	Gnd						Gnd			#
#			16		GP12				2		W_CLK								#
#			17		GP13				3		FU_UD								#
#			19		GP14				5		RESET								#
#			20		GP15				4		SER_DATA							#
#			31		GP26(SDA1)											SDA			#
#			32		GP27(SCL1)											SCL			#
#										18		10K pull-up to 3V3					#
#										19		10K pull-up to 3V3					#
#										10		Sine wave out to SMA				#
#																					#
#		Keypad connections:															#
#				Row 1 = GP0(1), Row 2 = GP1(2), Row 3 = GP2(4), Row 4 = GP3(5)		#
#				Col 1 = GP4(6), Col 2 = GP5(7), Col 3 = GP6(9), Col 4 = GP7(10)		#
#																					#
#	Designed by: K3JSE - Andy														#
#	Keypad addition by: WA3NOA - Jim												#
#####################################################################################

#################################################################################
#
# High level concept of operation
#
# The application uses an AD9850 DDS chip module to output sine waves that create an
# FM modulated signal.  Each set of sine waves comprisinmg the modulated signal is
# composed of 64 samples which the pio block outputs to the AD9850 to create
# one full modulated audio cycle.  The sine wave table is centered around a
# carrier frequency and the 64 samples vary + / - arround that carrier frequency.
# The amount of variance determines the deviatiion.  E.G. varying the modulated 
# sine wave frequency by +/- 2500Hz will create an FM deviation of 2500Hz.
#
# The modulated audio frequency is determined by how rapidly the sine wave table
# is output to the AD9850.  The AD9850 pio block clock is configured to vary
# the rate and therefore the audio frequency.
#
# The AD9850 module has two outputs.  One is unfiltered and passes the fundamental plus all
# images and the other has a 70MHz low pass filter which supppresses the images. The
# unfilterd output has an image at a frequency of fundamental + 125MHz so a 20 Mhz
# fundamental will have an image at 145MHz in the VHF band.  This image is
# potentially useful providing the receiving radio has adequate front end filteering.
# never feed th eoutput directly into the receivers antenna hack without 80dB or more
# of attenuation.  Even though the 145MHz image has a low amplitude the 20MHz fundamental
# could be "deadly"

# The filtered output with suitable attenuation may be connected to a transverter to create
# a potentially cleaner VHF signal.  Note that a transverter will likely provide low pass
# filtering so the unfiltered output may also be adequate.
#
#  The default operation when powered on is a 1500Hz audio tone, 2500Hz deviation with a 19,600,00Hz carrier / center
# frequency.
#
#
######################################################################################


import select
import sys, framebuf
from machine import Pin, mem32, freq, I2C
from ssd1306 import SSD1306_I2C
import time
import utime

import rp2
import onewire

###############################################################################
# Initialize the SSD1306 OLED display if present.
#
# The variable have_oled is set to false if there is no display present and the
# display routines will not attempt to update (a non-existant display)
#
pix_res_x  = 128 # SSD1306 horizontal resolution
pix_res_y = 64   # SSD1306 vertical resolution

i2c_dev = I2C(1,scl=Pin(27),sda=Pin(26),freq=200000)  # start I2C on I2C1 (GPIO 26/27)
i2c_addr = [hex(ii) for ii in i2c_dev.scan()]         # get I2C address in hex format
if i2c_addr==[]:
    print('No I2C Display Found') 
    have_oled = False
else:
    print("I2C Address      : {}".format(i2c_addr[0])) # I2C device address
    print("I2C Configuration: {}".format(i2c_dev))     # print I2C params
    oled = SSD1306_I2C(pix_res_x, pix_res_y, i2c_dev)  # oled controller
    have_oled = True

# Default (power up) configuration: Carrier - 19.6 MHz., Audio - 1,500 Hz., Deviation - 2,500 Hz.
base = 19600000
dev = 2500
audio = 1500

# Configure PIO block on RPI Pico
@rp2.asm_pio(
    out_shiftdir=1,
    out_init=rp2.PIO.OUT_LOW,
    set_init=rp2.PIO.OUT_LOW,
    sideset_init=[rp2.PIO.OUT_LOW] * 2,
    autopull=False,
    fifo_join=rp2.PIO.JOIN_TX,
)

# pio block to output sine table to AD9850
#   pull command gets data from DMA
def ad9850():
    wrap_target()
    pull(block).side(0b00)
    set(x, 31)
    label("loop32")
    out(pins, 1).side(0b00)
    jmp(x_dec, "loop32").side(0b01)
    set(x, 0x00).side(0b00)
    mov(osr, x)
    set(x, 7)
    label("loop8")
    out(pins, 1).side(0b00)
    jmp(x_dec, "loop8").side(0b01)
    set(pins, 0).side(0b10)
    wrap()


#pio block to reset AD9850
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW, autopull=False)
def ad9850_reset():
    wrap_target()
    pull(block)
    set(pins, 1)[31]
    nop()[31]
    set(pins, 0)[31]
    wrap()

sm_reset = rp2.StateMachine(1, ad9850_reset, freq=200_000, set_base=Pin(14))

print("Resetting AD9850")
sm_reset.active(1)
# put kicks off the pio block pull command
sm_reset.put(1)

def sm_div_calc(target_f):
    # calculate the pio clock divider, I plagiarized this :-)
    # Note that the pio clock speed determines the audio frequency
    if target_f < 0:
        div = 256
    elif target_f == 0:
        # Special case: set clkdiv to 0.
        div = 0
    else:
        div = freq() * 256 // target_f          # // is floor division
        if div <= 256 or div >= 16777216:
            print("out of range")
            raise ValueError("freq out of range")
    return div << 8

# AD9850 frequency word values for [half of] a 1 KHz deviation sine wave.
# The table is scaled according to the required deviation when populating the DMA buffer
sines1000_64 = [
    0, 3368, 6703, 9974, 13149, 16197, 19089, 21798,
    24296, 26560, 28569, 30303, 31744, 32880, 33700, 34194,
    34360, 34194, 33700, 32880, 31744, 30303, 28569, 26560,
    24296, 21798, 19089, 16197, 13149, 9974, 6703, 3368,
]

# DMA buffer containg AD9850 frequency word values for sine wave with the desired deviation
src_data = bytearray(64 * 4)

# DMA interrupt fucntion restart each DMA buffer after chaining to the next one
# dma0 and dma1 ping / pong for continuity
def dma_handler(dm):
    dm.read = src_data

# Initialize the DMA buffer with values from the sine table scaled by required deviation (in Hz)
def init_deviation(carrier, deviation):
    global src_data
    print("initialize Sine Table (deviation)")
    # adjust for TXCO error on AD9850 board
    ad9850_txco_calib = 20
    base_freq = int(carrier / ((125_000_000 +ad9850_txco_calib) / pow(2, 32)))
    dev1000 = deviation / 1000
    for j in range(32):
        for i in range(4):
            src_data[j * 4 + i] = (
                (base_freq + int(sines1000_64[j] * dev1000)) >> i * 8
            ) & 0xFF
            src_data[(j + 32) * 4 + i] = (
                (base_freq - int(sines1000_64[j] * dev1000)) >> i * 8
            ) & 0xFF

# This value is th enumber of pio clocks in a complete 64 AD9850 frequency word cycle
# The value is scaled by the desired audio frequency and used to calculate the pio clock speed
PIO_CYCLE_COUNT = 5504

def start_modulation(carrier, audio, deviation):
    global src_data
    global sm_freq
    global dma_0
    global dma_1

    dma_0 = rp2.DMA()
    dma_0.irq(handler=dma_handler)
    dma_1 = rp2.DMA()
    dma_1.irq(handler=dma_handler)

    pio_freq = int(PIO_CYCLE_COUNT * audio)
    
    init_deviation(carrier, deviation)
    # Instantiate a state machine with the AD9850 serial load program, at 125 mHz
    #   GP12 W_CLK
    #   GP13 Update pin
    #   GP14 reset
    #   GP15 data pin
    # sm_freq = rp2.StateMachine(0, ad9850, freq=pio_freq, out_base=Pin(15), set_base=Pin(15), sideset_base=Pin(12))
    sm_freq = rp2.StateMachine(
        0,
        ad9850,
        freq=pio_freq,
        out_base=Pin(15),
        set_base=Pin(15),
        sideset_base=Pin(12),
    )

    DATA_REQUEST_INDEX = 0  # value for pio0, sm 0, tx fifo empty interrupt
    c_0 = dma_0.pack_ctrl(
        size=2,
        inc_write=False,
        treq_sel=DATA_REQUEST_INDEX,
        irq_quiet=False,
        chain_to=dma_1.channel
    )
    c_1 = dma_1.pack_ctrl(
        size=2,
        inc_write=False,
        treq_sel=DATA_REQUEST_INDEX,
        irq_quiet=False,
        chain_to=dma_0.channel
    )

    dma_0.config(read=src_data, write=sm_freq, count=64, ctrl=c_0, trigger=False)

    dma_1.config(read=src_data, write=sm_freq, count=64, ctrl=c_1, trigger=False)

    print("Starting State Machine")
    sm_freq.active(1)
    sm_freq.put(1)               # kick-start the state machine
    print("Starting dma_0")
    dma_0.active(1)              # dma_0 will chain to dma_1

def update_audio(new_audio):
    SM0_CLKDIV = 0x50200000 + 0xC8
    mem32[SM0_CLKDIV] = sm_div_calc(int(PIO_CYCLE_COUNT * new_audio))


def update_deviation(carrier, new_dev):
    init_deviation(carrier, new_dev)

def update_display(audio,dev, freq):
    if have_oled:
        oled.fill(0)
        oled.text("Audio: "+audio+" Hz.",5,5)
        oled.text("  Dev: "+ dev+" Hz.",5,25)
        oled.text("f: "+ freq+" Hz.",5,45)
        oled.hline(0,0,127,1)
        oled.hline(0,63,127,1)
        oled.vline(0,0,63,1)
        oled.vline(127,0,63,1)
        oled.show() # show the new text

print("attempting to start OLED display")
update_display(str(audio),str(dev),str(base))

#################################################################
#																#
#	Code to implement 4 x 4 matrix keypad and use it to choose	#
#		functions on deviation generator.						#
#																#
#	Enter the numeric value for the desired function one digit	#
#		at a time. When done, depress the desired function key	#
#		to enter it into the appropriate 'pseudo-register'.		#
#				Function Keys:									#
#					A = Audio frequency							#
#					C = Carrier frequency						#
#					D = Deviation frequency						#
#					B = Bessel setting							#
#						using previous carrier frequency and	#
#						desired deviation as entered data with	#
#						Bessel function key						#
#																#
#	Once the audio and carrier values are entered as above, 	#
#		press the "#" key to load these values into the 		#
#		AD9850 module to generate the desired waveform signal.	#
#																#
#	Note: the Bessel funcion is immediate and does not require	#
#		using the "#" key.										#
#																#
#	The "*" can be used to completely reset the system to the	#
#		defaults of 19.6 MHz. carrier, 1,500 Hz. audio, and		#
#		2,500 Hz. deviation.									#
#																#
#################################################################


# Define key layout with assigned values to each key
keyMatrix = [
    [ 1,	2,	3,	"A" ],
    [ 4,	5,	6,	"B" ],
    [ 7,	8,	9,	"C" ],
    [ "*",	0,	"#","D" ]
]

#############################################################
# Define pin connections from keypad to Pico				#
#	Row 1 = GP0, Row 2 = GP1, Row 3 = GP2, Row 4 = GP3		#
#	Col. 1 = GP4, Col. 2 = GP5, Col. 3 = GP6, Col. 4 = GP7	#
#############################################################
rowPins = [0,1,2,3]
colPins = [4,5,6,7]

row = []
column = []

# Set up pin definitions for pins connected to keypad
for item in rowPins:
    row.append(machine.Pin(item,machine.Pin.OUT))
for item in colPins:
    column.append(machine.Pin(item,machine.Pin.IN,machine.Pin.PULL_DOWN))

# Variable for depressed key value
key = '0'

# Variable used to build value entered string
digits = [0]

#####################################################################
# Function to build a number from a list of single-digit integers.	#
#	uses 'digits' variable and returns an integer that is a number	#
#	formed by the digits.											#
#####################################################################
def build_number(digits):
    num_str = "".join(map(str, digits))
    return int(num_str)

#########################################################################
# Function to scan the keypad. Shifts a '1' to each row sequentially	#
#	and looks for a '1' returning on on a column. The depressed key		#
# value is derived by the index into the keyMatrix[].					#
#########################################################################
def scanKeypad():
    global key
    for rowKey in range(4):
        row[rowKey].value(1)
        for colKey in range(4):
            if column[colKey].value() == 1:
                key = keyMatrix[rowKey][colKey]
                row[rowKey].value(0)
                return(key)
        row[rowKey].value(0)
        


#jch# Create a polling object instance
#jchpoll_obj = select.poll()

# Register sys.stdin (standard input) for monitoring read events with priority 1
#jchpoll_obj.register(sys.stdin, select.POLLIN)

#jchtry:
print("starting outer loop")
start_modulation(base,audio,dev)
print("modulation started")


debounce = 0.2
digits = []
while True:

# Build up value string during iteration of key inputs
    if len(digits) > 0:
        print("build digits")
        number = build_number(digits)
# Scan for depressed key
    key=scanKeypad()
    #key = None

    if key is not None:
# When a key is depressed...
        print("key not none")
        if key == "*":
            print("Reset deviation generator to defaults")
            base = 19600000
            audio = 1500
            dev = 2500
            update_audio(audio)
            update_deviation(base,dev)
            update_display(str(audio),str(dev), str(base))
            print("parameters:", base, audio, dev)
            utime.sleep(debounce)	# Debounce timer
            digits = [0]

        elif key == "#":
            print("Updating audio and deviation settings")
            update_audio(audio)
            update_deviation(base,dev)
            update_display(str(audio),str(dev), str(base))
            print("parameters:", base, audio, dev)
            utime.sleep(debounce)	# Debounce timer

        elif key == "A":
            print("set Audio frequency")
            print("     audio frequency = {} Hz.".format(number))
            audio = number
            digits = [0]
            utime.sleep(debounce)	# Debounce timer
            
        elif key == "B":
            print("Invoke Bessel setting function")
            print("     deviation frequency = {} Hz.".format(number))
            audio = dev / 2.405
            print("Setting Bessel Null.  Deviation:", dev, "Audio:", audio)
            update_audio(audio)
            update_deviation(base, dev)
            update_display(str(audio),str(dev), str(base))
            digits = [0]
            utime.sleep(debounce)	# Debounce timer
             
        elif key == "C":
            print("set Carrier frequency")
            print("     carrier frequency = {} Hz.".format(number))
            base = number
            digits = [0]
            utime.sleep(debounce)	# Debounce timer
            
        elif key == "D":
            print("set Deviation frequency")
            print("     deviation frequency = {} Hz.".format(number))
            dev = number
            digits = [0]
            utime.sleep(debounce)	# Debounce timer
            
        elif key >= 0 and key <= 9:
            utime.sleep(debounce)	# Debounce timer
            print("Key pressed is {}".format(key))	# Echo key value
            digits.append(key)	# Add to value string
 

