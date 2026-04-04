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
# we reserve D9, D10, and D11 for this state machine
# D9 and D10 are the inputs
# D11 is the resultant output
# We run at 125MHz to detect this as fast as possible
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

# this SM is responsible for detecting the READY pulse generated from the
# AVR128DB64. It converts it into a level signal instead. This eliminates some
# overhead from firing an interrupt. Instead, it is positional code.
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

while True:
    time.sleep(1)
