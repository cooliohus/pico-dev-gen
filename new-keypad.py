# lots of hard codeing but it seems to work
#
# Next step is to create pio interrupt and [try to] turn the key presses into a stream 
# to seamlessly integrate with the serrial data stream - fingers crossed :-)
#
# And of course make it more pythonic!

from machine import Pin, mem32
import rp2
from io import StringIO
import sys, select


SEEK_START = 0
SEEK_CURRENT = 1
SEEK_END = 2

streambuff = bytearray(10)
streambuff = "buffer"
key_buff = ""

sfile = StringIO(streambuff)

# rows must be gpio 0,1,2,3
# Columns  gpio 4,5,6,7
rowPins = [0,1,2,3]
colPins = [4,5,6,7]

# Set up pin definitions for pins connected to keypad
for item in rowPins:
    Pin(item,Pin.OUT)
for item in colPins:
    Pin(item,Pin.IN,Pin.PULL_DOWN)

# Need to access the state machine read fifo directly as Micropython method only returns 8 bits
# These consta\nts are the PIO 0 base and SM2 RX FIFO offset
PIO0_base = 0x50200000   # PIO 0 base
RXF2      = 0x28         # SM2 RX FIFO offset

row = 0
col = 0

# Kludge to map powers of 2 to translate column bitmap returned
# from pio block into columns 0-3
colmap = {1:0,2:1,4:2,8:3}

newkeyMatrix = [
    [ "1","2","3","A" ],
    [ "4","5","6","B" ],
    [ "7","8","9","C" ],
    [ "*","0","#","D" ]
]

def get_key():
      global key_buff
    #if sm_getkey.rx_fifo():              # The RX FIFO isn't empty
       #sm_getkey.get(keyval,0)          # only returns 8 bits, I spentg a couple of hours on  this!
      keyval = mem32[PIO0_base+RXF2]    # return 32 bits from the RX FIFO

      #print("keyval: ",keyval)
      for row in range(4):
        if colx := (keyval & 0xf):
          break
        else:
          keyval >>= 4

      col = colmap[colx]
      ch = newkeyMatrix[row][col]
      #print("row: ",row,"  col: ",col,"  colx: ",colx,"  keypressed: ",ch)
      print("keypressed: ",ch)
      key_buff += ch
      
      if ch == "#":
          print("keybuff: ",key_buff)
          param = int(key_buff[1:-1])
          if key_buff[0] == "A":
              cmd = "r,1," + key_buff[1:-1]
              print("cmd: ",cmd)
              print("cmd: ","u")
          elif key_buff[0] == "B":
              cmd = "r,2," + key_buff[1:-1]
              print("cmd: ",cmd)
              cmd = "r,1," + "calc audio"
              print("cmd: ",cmd)
              print("cmd: ","u")         
          elif key_buff[0] == "C":
              cmd = "r,0," + key_buff[1:-1]
              print("param: {:,}".format(param))
              print("cmd: ",cmd)
              print("cmd: ","u")
          elif key_buff[0] == "D":
              cmd = "r,2," + key_buff[1:-1]
              print("cmd: ",cmd)
              print("cmd: ","u")              
          key_buff = ""

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
    irq(0)

    # debounce routine, wait for all keys up. This also inhibits key roll-over and key repeat which
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
    global sfile
    #print("flags: ",hex(sm.irq().flags()), sm)
    get_key()


# Create state machine 2 in PIO 0 with a modest clock speed
# note that row pins are assumed to be contiguous atsrting at 0 and column pins starting at 4
sm_getkey = rp2.StateMachine(2, keypad_getkey, freq=50_000, set_base=Pin(0), in_base=Pin(4))
#sm_getkey.irq(sm_getkey_irq)
rp2.PIO(0).irq(handler=sm_getkey_irq,hard=False)
sm_getkey.active(1)


# Create a polling object instance
poll_obj = select.poll()

# Register sys.stdin (standard input) for monitoring read events with priority 1
poll_obj.register(sys.stdin, select.POLLIN)

while True:
    if ps := poll_obj.poll(100):
        # check for serial line character 
        for (o, e) in ps:
            if o == sys.stdin and e == select.POLLIN:
                st = sys.stdin.readline().strip().lower().split(",")
            print("stream", st)

