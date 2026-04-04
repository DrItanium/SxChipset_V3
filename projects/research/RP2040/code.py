import digitalio
import board
import time
import busio
import sys
import rp2pio
import adafruit_pioasm
import array




# now that we have a successful setup for feedback testing, it is necessary to
# setup a state machine for processing a series of independent requests for the
# basic bus state machine of the i960

# the first thing to do is actually setup a simple blinking state machine
blink = adafruit_pioasm.assemble(
        """
.program blink
    pull block ; these two instructions take the blink duration
    out y, 32  ; and store it in y
forever:
    mov x, y
    set pins, 1 ; Turn LED on
lp1:
    jmp x-- lp1 ; delay for (x + 1) cycles, x is a 32-bit number
    mov x, y
    set pins, 0 ; turn led off
lp2:
    jmp x-- lp2 ; delay for the same number of cycles again
    jmp forever
"""
        )
blinkSM = rp2pio.StateMachine(
        blink,
        frequency = 125_000_000 ,
        first_set_pin=board.LED,
        wait_for_txstall = False,
        )
# this blinking state machine will persist for the entire time we are running
# everything else
data = array.array("I", [blinkSM.frequency // 5])
blinkSM.write(data)



# The first state machine is the detector for an address transaction:
transactionStartSM = adafruit_pioasm.assemble(
        """
.program transaction_start
    wait 0 pin 0 ; wait for ADS to go low
    wait 1 pin 0 ; then wait for it to go high again
    wait 0 pin 1 ; then wait for DEN to go low
    set pins 0   ; The falling edge becomes the interrupt source
    wait 1 pin 1 ; wait until DEN goes high again
    set pins 1      ; Then pull it high
    """
    )

adsDetect = rp2pio.StateMachine(
        transactionStartSM,
        frequency = 125_000_000,
        first_in_pin = board.D9,
        in_pin_count = 2,
        first_set_pin = board.D11,
        set_pin_count = 1,
        initial_set_pin_state = 1,
        initial_set_pin_direction = 1,
        )

readyTranslatorSM = adafruit_pioasm.assemble(
        """
.program readyTranslator
    wait 0 pin 0 ; wait for READY OUT to go low
    wait 1 pin 0 ; wait for READY OUT to go high
    set pins 0   ; make READY low
    wait 0 pin 0 ; wait for READY OUT to go low
    wait 1 pin 0 ; wait for READY OUT to go high
    set pins 1   ; toggle READY
    """
    )

readyTranslator = rp2pio.StateMachine(
        readyTranslatorSM,
        frequency = 125_000_000,
        first_in_pin = board.D5,
        in_pin_count = 1,
        first_set_pin = board.D6,
        set_pin_count = 1,
        initial_set_pin_state = 1,
        initial_set_pin_direction = 1,
        )

# the next state machine will only start executing once the interrupt from the
# adsDetect has triggered the interrupt. The responsibility of this second
# state machine is to pull in the address contents from the shift registers and
# pass it to the microcontroller cores

# after that, the responsibility will be to either get or recieve up to
# 128-bits of data. In the case of read operations, it is just up to 128-bits
# of data that we ignore the enable lines from.
#
# For write operations, we need to capture the byte enable lines as well as the
# data itself. We can apply optimizations where only the first and last bytes
# need to have their byte enable lines checked. Or... we just create
# transaction packets...
#
# the address needs to be sent to the microcontroller cores
# this will allow for either read or write operations to take place!
while True:
    time.sleep(1)
