############################################################
#
# Deviation Generator by K3JSE
vers = 1.00
#
############################################################

import select
import sys, framebuf
from machine import Pin, mem32, freq, I2C
from ssd1306 import SSD1306_I2C
import time
import rp2
import onewire
# import uctypes

##########################################################
# test code, replace with actual stuff later
pix_res_x  = 128 # SSD1306 horizontal resolution
pix_res_y = 64   # SSD1306 vertical resolution

i2c_dev = I2C(1,scl=Pin(27),sda=Pin(26),freq=200000)  # start I2C on I2C1 (GPIO 26/27)
i2c_addr = [hex(ii) for ii in i2c_dev.scan()] # get I2C address in hex format
if i2c_addr==[]:
    print('No I2C Display Found') 
    have_oled = False
    #sys.exit() # exit routine if no dev found
else:
    print("I2C Address      : {}".format(i2c_addr[0])) # I2C device address
    print("I2C Configuration: {}".format(i2c_dev)) # print I2C params
    oled = SSD1306_I2C(pix_res_x, pix_res_y, i2c_dev) # oled controller
    have_oled = True
    #oled.text("  Dev:",5,5)
    #oled.text("Audio:",5,25)
    #oled.text("by K3JSE",5,45)
    #oled.show() # show the new text


#########################################################

# Virtual machine registers - default assignments are
#   R0: carrier frequency
#   R1: audio frequency
#   R2: deviation
regs = [19_600_000, 1500, 2500, 3, 4, 5, 6, 7, 8, 9]
#regs = [10_000_000, 1500, 2500, 3, 4, 5, 6, 7, 8, 9]

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

# AD9850 frequency word values for a 1 KHz deviation sine wave.
# The table is scaled according to the required deviation
sines1000_64 = [
    0, 3368, 6703, 9974, 13149, 16197, 19089, 21798,
    24296, 26560, 28569, 30303, 31744, 32880, 33700, 34194,
    34360, 34194, 33700, 32880, 31744, 30303, 28569, 26560,
    24296, 21798, 19089, 16197, 13149, 9974, 6703, 3368,
]

# AD9850 values for sine wave with the desired deviation
# DMA feeds this to the pio block
src_data = bytearray(64 * 4)

# restart each DMA buffer after chaining to the next one
# dma0 and dma1 ping / pong for continuity
def dma_handler(dm):
    dm.read = src_data

# Initialize the sine table with values scaled by required deviation (in Hz)
def init_deviation(carrier, deviation):
    global src_data
    print("initialize Sine Table (deviation)")
    # adjust for TXCO error on AD9850 board
    ad9850_txco_calib = 16500
    base_freq = int(carrier / (125_000_000 / pow(2, 32))) + ad9850_txco_calib
    dev1000 = deviation / 1000
    for j in range(32):
        for i in range(4):
            src_data[j * 4 + i] = (
                (base_freq + int(sines1000_64[j] * dev1000)) >> i * 8
            ) & 0xFF
            src_data[(j + 32) * 4 + i] = (
                (base_freq - int(sines1000_64[j] * dev1000)) >> i * 8
            ) & 0xFF

# This value scales the pio for correct audio frequencies
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
    # Initialize the sine table scaled by deviation
    
    init_deviation(carrier, deviation)
    #print("end init deviation")
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

    DATA_REQUEST_INDEX = 0  # pio0, sm 0, tx fifo
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
        oled.text("Audio: "+audio+"Hz",5,5)
        oled.text("  Dev: "+ dev+"Hz",5,25)
        oled.text("f: "+ freq+"Hz",5,45)
        oled.show() # show the new text

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
        ps = poll_obj.poll(-1)
        for (o, e) in ps:
            if o == sys.stdin and e == select.POLLIN:
                # print('\r\nin loop')
                st = sys.stdin.readline().strip().lower().split(",")
                # print("Reafline:",st)
                #print('st:',st[0])
                cmd = st[0]
                if st[0] =='':
                    #print("Empty String)")
                    pass
                elif cmd == "r":
                    # Set new register value
                    #   st[1] is register to update
                    #   st[2] is new register value
                    if len(st) < 3:
                        print("Not enough parameters for register comand")
                    else:
                        try:
                            print("opcode:", st[0], int(st[1]), int(st[2]))
                            # if (int(st[1])!=0) and ((int(st[2]) < 500) or (int(st[2]) > 5000)):
                            #    print("Parameter out of range")
                            #    break
                            print("Writing Registers")
                            regs[int(st[1])] = int(st[2])
                            # update_deviation(regs[0],regs[2])
                            # update_audio(regs[1])
                        except:
                            print("Parameters not numeric")
                elif cmd == "m":
                    # update modulation parameters
                    #   st[1] is register holding base frequency
                    #   st[2] is register holding audio frequency
                    #   st[3] is register holding deviation
                    if len(st) < 4:
                        print("not enough parameters for modulate comand")
                        break
                    print("opcode:", st[0], int(st[1]), int(st[2]), int(st[3]))
                    try:
                        base = int(st[1])
                        audio = int(st[2])
                        dev = int(st[3])
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
                    if len(st) < 3:
                        print("not enough parameters for bessel comand")
                        break
                    try:
                        base = int(st[1])
                        dev = int(st[2])
                        print("opcode:", st[0], base, dev)
                        if dev < 500 or dev > 5000:
                            print("Parameter error", dev)
                        else:
                            audio = dev / 2.405
                            print(
                                "Setting Bessel Null.  Deviation:", dev, "Audio:", audio
                            )
                            update_audio(audio)
                            update_deviation(regs[base], dev)
                            update_display(str(int(audio)),str(dev), str(regs[base]))
                    except:
                        print("Deviation not numeric")
                elif cmd == "l":
                    print("list registers")
                    print(regs)
                elif cmd == "u":
                    print("Updating audio and deviation settings")
                    update_audio(regs[1])
                    update_deviation(regs[0], regs[2])
                    update_display(str(regs[1]),str(regs[2]), str(regs[0]))
                    

                elif cmd == "h":
                    print("Halting state machine SM0")
                    sm_freq.active(0)
                elif cmd == "s":
                    print("Starting state machine SM0")
                    sm_freq.active(1)
                    sm_freq.restart()
                else:
                    print("Unknown Command:", cmd)
                

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
