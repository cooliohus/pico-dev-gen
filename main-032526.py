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
#	Designed by: K3JSE - Andy														#
#####################################################################################

#################################################################################
#
# High level concept of operation
#
# The application uses an AD9850 DDS chip module to output sine waves that create an
# FM modulated signal.  Each set of sine waves comprisinmg the modulated signal is
# composed of 64 samples which the pio block outputs to the AD9850 to create
# one full modulated audio cycle.  The sine wave table is centered around a
# carrier frequency and the 64 samples vary + / - around that carrier frequency.
# The amount of variance determines the deviatiion.  E.G. varying the modulated 
# sine wave frequency by +/- 2500Hz will create an FM deviation of 2500Hz.
#
# The modulated audio frequency is determined by how rapidly the sine wave table
# is output to the AD9850.  The AD9850 pio block clock is configured to vary
# the rate and therefore the audio frequency.
#
# The AD9850 module has two outputs.  One is unfiltered and passes the fundamental plus all
# images and the other has a 70MHz low pass filter which supppresses the images. The
# unfilterd output has an image at a frequency of the fundamental + 125MHz so a 20 Mhz
# fundamental will have an image at 145MHz in the VHF band.  This image is
# potentially useful providing the receiving VHF radio has adequate front end filtering.
# Note: never feed the output directly into the receivers antenna jack without 80dB or more
# of attenuation.  Even though the 145MHz image has a low amplitude, the 20MHz fundamental
# could be "deadly"
#
# The filtered output with suitable attenuation may be connected to a transverter to create
# a potentially cleaner VHF signal.  Note that a transverter will likely provide low pass
# filtering so the unfiltered output may also be adequate.
#
# The "user interface" is implemented as a simple virtual machine with ten
# virtual registers and some primitive commands.  The virtual machine fetches
# instructions from the RS232 port instead of memory which allows "programs"
# to run on a separate PC / Laptop.  The default operation when powered on is
# a 1500Hz audio tone, 2500Hz deviation with a 19,600,00Hz carrier / center
# frequency.  The VM can be driven through the PICO serial port or a 16 key keypad
#
# Default register usage is
#   r0 - carrier frequency in hertz
#   r1 - modulated audio frequency in hertz
#   r2 - modulated deviation in hertz
#
#
# VM Commands:
#	l<cr>						List register contents
#	r,<r#>,<data><cr>			Load register r# with data (see defaults)
#                                 e.g. r,0,19600000
#	u<cr>						Update modulation using values from the default registers
#	m,<ra>,<rb>,<rc><cr>	    Update the modulated signal using r# registers
#                               in this order, all values in Hertz
#                                  ra - register containing carrier frequency
#                                  rb - register containing audio tone / pitch
#                                  rc - register containing deviation
#                                 e.g. m,0,1,2  is the same as u
#	b,<r#>,<deviation><cr>	    Create a Bessel null modulated signal
#                                  r#        - register containing carrier frequency
#                                  deviation - deviation in Hertz
#                                 e.g. b,0,2500
#	h<cr>						Halt state machine
#	s<cr>						Start state machine
#
# Example program (from serial port)
#
# Alternate between two modulated signals then create a Bessel null signal
# Commands are sent from a PC / Laptop via the serial port with appropriate delays.  Users
# may send commands manually using a terminal emulator (e.g. putty, minicom) or from 
# scripts written in Python, bash, powershell, or the users favorite language :-)
#
#     r,0,20000000<cr>    # set registers r0 - r4
#     r,1,1500<cr>
#     r,2,2500<cr>
#     r,3,1000<cr>
#     r,4,2000<cr>
#     m,0,1,2<cr>         # update modulated signal
#                         # carrier = 20,000,000Hz, audio tone 1500Hz, deviation 2500Hz
#     .... delay ....
#     m,0,3,4<cr>         # update modulated signal
#                         # carrier = 20,000,000Hz, audio tone 1000Hz, deviation 2000Hz
#     .... delay ....
#     b,0,3000<cr>        # create Bessel null signal
#                         # carrier = 20,000,000, deviation 3000Hz, audio = 1247Hz (calculated)
#     .... delay ....
#     u<cr>               # update modulated signal
#                         # return to "default" using r0, r1, and r2
#
# Keypad Commands
# ---------------
#
# Most keypad commands begin with a command key, parameter, then the # key as a terminator
#
#   A - change audio frequency
#   D - change deviation
#   C - Change "carrier" frequency
#   B - set audio frequency for Bessel null at current deviation setting
#   * - reset to defaults
#   # - toggle modulation on / off
#
# Keypad Examples
#
#   A1500#      set audio modulation to 1500 Hertz
#   D2500#      set deviation to 2500 Hertz
#   C20000000#  set carrier frequency to 20,000,000 Hertz
#   B#          set audio frequency to create a Bessel null at curent deviation
#                  e.g. 1039.5 Hertz if deviation is 2500 Hertz (default)
#   *           reset dto efaults C=19,600,000 A=1500 D=2500
#   #           toggle modulation on / off
#
#####################################################################################


import select
import sys, framebuf
from machine import Pin, mem32, freq, I2C
from ssd1306 import SSD1306_I2C
import time
import rp2
import onewire
# import uctypes

###############################################################################
# Initialize the SSD1306 OLED display if present.
#
# The variable have_oled is set to false if there is no display present and the
# display routines will not attempt to update (a non-existant display)
#
pix_res_x = 128  # SSD1306 horizontal resolution
pix_res_y = 64   # SSD1306 vertical resolution

i2c_dev = I2C(1,scl=Pin(27),sda=Pin(26),freq=200000)  # start I2C on I2C1 (GPIO 26/27)
i2c_addr = [hex(ii) for ii in i2c_dev.scan()]         # get I2C address in hex format
if i2c_addr==[]:
    print('No I2C Display Found') 
    have_oled = False
    #sys.exit() # exit routine if no dev found
else:
    print("I2C Address      : {}".format(i2c_addr[0])) # I2C device address
    print("I2C Configuration: {}".format(i2c_dev))     # print I2C params
    oled = SSD1306_I2C(pix_res_x, pix_res_y, i2c_dev)  # oled controller
    have_oled = True


#########################################################

# Virtual machine registers - default assignments are
#   R0: carrier frequency
#   R1: audio frequency
#   R2: deviation
regs = [19_600_000, 1500, 2500, 3, 4, 5, 6, 7, 8, 9]
#regs = [10_000_000, 1500, 2500, 3, 4, 5, 6, 7, 8, 9]


SEEK_START = 0
SEEK_CURRENT = 1
SEEK_END = 2

#streambuff = bytearray(10)
#streambuff = "buffer"
key_buff = ""     # capture input from keyopad
got_pad = False   # keypad command complete - received #

#sfile = StringIO(streambuff)

# rows must be gpio 0,1,2,3
# Columns  gpio 4,5,6,7
rowPins = [0,1,2,3]
colPins = [4,5,6,7]

# Set up pin definitions for pins connected to keypad
for item in rowPins:
    Pin(item,Pin.OUT)
for item in colPins:
    Pin(item,Pin.IN,Pin.PULL_DOWN)

# Need to access the state machine read fifo directly as the Micropython method only returns 8 bits
# These constants are the PIO 0 base and SM2 RX FIFO offset
PIO0_base = 0x50200000   # PIO 0 base
RXF2      = 0x28         # SM2 RX FIFO offset

# Kludge to map powers of 2 to translate column bitmap
# returned from the pio block into columns 0-3
colmap = {1:0,2:1,4:2,8:3}

# Translate keypad matrix into ascii characters.  Define in
# the outer block to save a few clocks on each key press :-) 
newkeyMatrix = [
    [ "1","2","3","A" ],
    [ "4","5","6","B" ],
    [ "7","8","9","C" ],
    [ "*","0","#","D" ]
]

def get_key():
    # called from the keypad pio block when a new keypress is ready
    global key_buff
    global got_pad
    keyval = mem32[PIO0_base+RXF2]    # return 32 bits from the RX FIFO
    for row in range(4):
      if colx := (keyval & 0xf):
        break
      else:
        keyval >>= 4

    col = colmap[colx]
    ch = newkeyMatrix[row][col]
    #print("row: ",row,"  col: ",col,"  colx: ",colx,"  keypressed: ",ch)
    #print("keypressed: ",ch)
    # add the new character to the keypad input buffer
    key_buff += ch
    # if the new character is # or * then signal the outer loop to take action
    if (ch == "#") or (ch == "*"):
        got_pad = True

   

#pio block to read keypad
@rp2.asm_pio(set_init=[rp2.PIO.OUT_LOW] *4, in_shiftdir=0, autopush=False,autopull=False)

def keypad_getkey():
    wrap_target()
    label("loop")
    set(pins, 0b1000)[31]    # set 1000 on the 4 row output pins (activate first row) and wait for the signal to stabilize
    in_(pins, 4)             # shift the column input pins into the ISR
    set(pins, 0b0100)[31]    # rinse and repeat for all four rows
    in_(pins, 4)             # Note that the ISR register shifts left four bits on each input so 16 possible bit values
    set(pins, 0b0010)[31] 
    in_(pins, 4)
    set(pins, 0b0001)[31]
    in_(pins, 4)
    mov(x, isr)              # copy the ISR into the x scratch register
    jmp(not_x, "loop")       # if x contains 0, no key was pressed, start over
    push(noblock)            # a key was pressed, push the ISR into the RX FIFO
    irq(0)                   # generate "key available" interrupt

    # debounce routine, wait for all keys up. The side effect is key roll-over and key repeat are inhibited... which
    # is probably a good thing :-)
    set(y,0)                 # use y register for constant 0
    set(pins,0b1111)[31]     # set ALL row lines high add 31 clk delays to extend debounce time
    label("debounce")
    mov(isr,y)[31]           # clear ISR register
    in_(pins,4)[31]          # input four column lines
    mov(x,isr)               # move to x register
    jmp(x_dec,"debounce")    # if not zero, jump to start of debounce loop
    wrap()                   # loop back to the beginning (wrap_target)and wait for the next keypress


def sm_getkey_irq(sm):
    #print("flags: ",hex(sm.irq().flags()), sm)
    # Don't bother checking which interrupt as there is only one
    # Call get_key to read the new lkey from the state machine FIFO and process it
    get_key()


# Create state machine 2 in PIO 0 with a modest clock speed
# note that row pins are assumed to be contiguous atsrting at 0 and column pins starting at 4
sm_getkey = rp2.StateMachine(2, keypad_getkey, freq=50_000, set_base=Pin(0), in_base=Pin(4))
# Set the PIO zero interrupt handler to sm_getkey_irq
rp2.PIO(0).irq(handler=sm_getkey_irq,hard=False)
# Enable / start the get_key state machine
sm_getkey.active(1)

# pio block to output sine table to AD9850
#   pull command gets data from DMA
@rp2.asm_pio(
    out_shiftdir=1,
    out_init=rp2.PIO.OUT_LOW,
    set_init=rp2.PIO.OUT_LOW,
    sideset_init=[rp2.PIO.OUT_LOW] * 2,
    autopull=False,
    fifo_join=rp2.PIO.JOIN_TX,
)

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
# put kicks off the ad9850 state machine by satisfying the pull(block) command
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
# The table is scaled according to the required deviation then added to the
# carrier value when populating the DMA buffer
sines1000_64 = [
    0, 3368, 6703, 9974, 13149, 16197, 19089, 21798,
    24296, 26560, 28569, 30303, 31744, 32880, 33700, 34194,
    34360, 34194, 33700, 32880, 31744, 30303, 28569, 26560,
    24296, 21798, 19089, 16197, 13149, 9974, 6703, 3368,
]

# DMA buffer containg AD9850 frequency word values for sine wave
# with the desired deviation and carrier offset
src_data = bytearray(64 * 4)

# DMA interrupt function to restart each DMA buffer after chaining to the next one
# dma0 and dma1 ping / pong for continuity
def dma_handler(dm):
    dm.read = src_data

# Initialize the DMA buffer with values from the sine table scaled by required deviation (in Hz)
def init_deviation(carrier, deviation):
    global src_data
    #print("initialize Sine Table (deviation)")
    # adjust for TXCO error on AD9850 board
    #ad9850_txco_calib = -2800
    ad9850_txco_calib = 150
    base_freq = int(carrier / ((125_000_000 + ad9850_txco_calib) / pow(2, 32)))
    dev1000 = deviation / 1000
    for j in range(32):
        for i in range(4):
            src_data[j * 4 + i] = ((base_freq + int(sines1000_64[j] * dev1000)) >> i * 8) & 0xFF
            src_data[(j + 32) * 4 + i] = ((base_freq - int(sines1000_64[j] * dev1000)) >> i * 8) & 0xFF
    #for i in range(32):
    #    print (int(src_data[i]))
    #sys.exit()



# This value is the number of pio clocks in a complete 64 AD9850 frequency word cycle
# The value is scaled by the desired audio frequency and used to calculate the pio clock speed
PIO_CYCLE_COUNT = 5504

def start_modulation(carrier, audio, deviation):
    global src_data   # DMA buffer
    global sm_freq    # AD9850 state machine
    global dma_0      # DMA 0 block, ping / pongs with dma_1
    global dma_1      # DMA 1 block

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
        sideset_base=Pin(12)  # sideset is used to toggle W_CLK, FQ_UD pins
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


def vm(st):
    # "virtual machine" implementing core functionality
    #print("entering vm")
    #print("st: ",st)
    cmdstr = st.split(",")
    cmd = cmdstr[0]
    if cmdstr[0] =='':
        pass
    elif cmd == "r":
       # Set new register value
                    #   st[1] is register to update
                    #   st[2] is new register value
        if len(cmdstr) < 3:
            print("Not enough parameters for register comand")
        else:
            try:
                print("opcode:", cmdstr[0], int(cmdstr[1]), int(cmdstr[2]))
                # if (int(st[1])!=0) and ((int(st[2]) < 500) or (int(st[2]) > 5000)):
                #    print("Parameter out of range")
                #    break
                print("Writing Registers")
                regs[int(cmdstr[1])] = int(cmdstr[2])
            except:
                print("Parameters not numeric")
    elif cmd == "m":
        # update modulation based on three virtual register values
        # No registers are changed
        #   st[1] is register holding base frequency
        #   st[2] is register holding audio frequency
        #   st[3] is register holding deviation
        if len(cmdstr) < 4:
            print("not enough parameters for modulate comand")
            return(-1)
        print("opcode:", cmdstr[0], int(cmdstr[1]), int(cmdstr[2]), int(cmdstr[3]))
        try:
            base = int(cmdstr[1])
            audio = int(cmdstr[2])
            dev = int(cmdstr[3])
            print("parameters:", base, audio, dev)
            # if ((regs[audio]<500 or regs[audio]> 5000) or \
            #    (regs[dev]<500 or regs[dev]>5000) \
            #   ):
            #    print("Paramater error")
            # else:
            print("updating modulation")
            update_deviation(regs[base], regs[dev])
            update_audio(regs[audio])
            update_display(str(regs[audio]),str(regs[dev]), str(regs[base]))
        except:
            print("Parameters not numeric")
    elif cmd == "f":
        print("Frequency command - not implemented")
    elif cmd == "b":
        # calculate audio frequency value for a Bessel null at the desired deviation
        # The first paramenter is the register containing the carrier frequency and the 
        # second parameter is the register for deviation
        # Register R[1] is changed by this command
        if len(cmdstr) < 3:
            print("not enough parameters for bessel comand")
            return(-1)
        try:
            base = int(cmdstr[1])
            dev = regs[int(cmdstr[2])]
            print("opcode:", cmdstr[0], base, dev)
            audio = dev / 2.405
            regs[1] = audio
            print("Setting Bessel Null.  Deviation:", dev, "Audio:", audio)
        except:
            print("Deviation not numeric")
    elif cmd == "l":
        # list the values contained in all 10 virtual registers
        print("list registers")
        print(regs)
    elif cmd == "u":
        # Change modulation (carrier, audio, deviation) using values from the default registers
        # r0, r1, and r2.  No registers are changed by this command
        print("Updating audio and deviation settings")
        update_audio(regs[1])
        update_deviation(regs[0], regs[2])
        update_display(str(round(regs[1])),str(regs[2]), str(regs[0]))
    elif cmd == "h":
        print("Halting state machine SM0")
        sm_freq.active(0)
    elif cmd == "s":
        print("Starting state machine SM0")
        sm_freq.active(1)
        sm_freq.restart()
    else:
        print("Unknown Command:", cmd)


update_display(str(regs[1]),str(regs[2]), str(regs[0]))

# Create a polling object instance
poll_obj = select.poll()

# Register sys.stdin (standard input) for monitoring read events with priority 1
poll_obj.register(sys.stdin, select.POLLIN)

try:
    print("starting outer loop")
    start_modulation(regs[0], regs[1], regs[2])
    print("modulation started")
    #st = sys.stdin.read(1)
    while True:
        # Check if there is any data available on sys.stdin and block
        #ps = poll_obj.poll(-1)
        if ps := poll_obj.poll(100):
          for (o, e) in ps:
            if o == sys.stdin and e == select.POLLIN:
                #st = sys.stdin.readline().strip().lower().split(",")
                st = sys.stdin.readline().strip().lower()
                print("serial cmd: ",st)
                vm(st)
                
        elif got_pad:
            print("keybuff: ",key_buff)
            #if len(key_buff) > 1:
            #    param = int(key_buff[1:-1])
            if key_buff[0] == "A":
                cmd = "r,1," + key_buff[1:-1]
                print("send cmd: ",cmd)
                print("send cmd: ","u")
                vm(cmd)
                vm('u')
            elif key_buff[0] == "B":
                #print("entering bessel")
                #print("regs[2]: ",regs[2])
                cmd = 'b,0,2'
                print("send cmd: ",cmd)
                print("send cmd: ","u")
                vm(cmd)
                vm('u')
            elif key_buff[0] == "C":
                cmd = "r,0," + key_buff[1:-1]
                print("cmd: ",cmd)
                print("cmd: ","u")
                vm(cmd)
                vm('u')
            elif key_buff[0] == "D":
                cmd = "r,2," + key_buff[1:-1]
                print("cmd: ",cmd)
                print("cmd: ","u")
                vm(cmd)
                vm('u')
            elif key_buff[0] == "#":
                # toggle state mahine
                if sm_freq.active():
                    print("Stopping state machine SM0")
                    sm_freq.active(0)
                else:
                    print("Starting state machine SM0")
                    sm_freq.active(1)
                    sm_freq.restart()
            elif key_buff[0] == "*":
                # reset to default
                vm('r,0,19600000')
                vm('r,1,1500')
                vm('r,2,2500')
                vm('u')
            key_buff = ""
            got_pad = False

except KeyboardInterrupt:
    print("caught exception")
finally:
    print("Shutting down....")
    #sm_freq.active(0)

    dma_0.close()
    dma_1.close()

    sm_reset.put(1)

    print("All done")
    # Delay to allow graceful shutdown
    time.sleep(0.5)
    sm_reset.active(0)
    # reset()
    sys.exit()
