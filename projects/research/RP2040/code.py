import digitalio
import board
import time
import busio
import sys
import rp2pio
import adafruit_pioasm
import array

# D12 is the System Up pin
systemWarmUp = digitalio.DigitalInOut(board.A0)
systemWarmUp.switch_to_output(True)

# now that we have a successful setup for feedback testing, it is necessary to
# setup a state machine for processing a series of independent requests for the
# basic bus state machine of the i960

# The first state machine is the detector for an address transaction:
transactionStartSM = adafruit_pioasm.assemble(
        """
.program transaction_start
    wait 0 pin 0 ; wait for ADS to go low
    set pins 0b101 ; address state enter
    wait 1 pin 0 ; then wait for it to go high again
    wait 0 pin 1 ; then wait for DEN to go low
    set pins 0b110   ; The falling edge becomes the interrupt source
    wait 1 pin 1 ; wait until DEN goes high again
    set pins 0b011      ; Then pull it high
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
        set_pin_count = 3,
        initial_set_pin_state = 0b011,
        initial_set_pin_direction = 0b111,
        )

# this SM is responsible for detecting the READY pulse generated from the
# AVR128DB64. It converts it into a level signal instead. This eliminates some
# overhead from firing an interrupt. Instead, it is positional code.

# it is important to note that we "toggle" the output pin after detecting the
# falling edge of the output pulse from the AVR's single shot generator.
# Previously, it was after waiting for it to go high. But that actually impacts
# performance quite a bit. 
readyTranslatorSM = adafruit_pioasm.assemble(
        """
.program readyTranslator
    wait 0 pin 0 ; wait for READY OUT to go low
    set pins 0   ; Denote that the pulse has been sent off
    wait 1 pin 0 ; wait for READY OUT to go high
    wait 0 pin 0 ; wait for READY OUT to go low
    set pins 1   ; Denote that the pulse has been sent off
    wait 1 pin 0 ; wait for READY OUT to go high
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
# reserved Pins on the RP2040 Feather
# D5: Ready Input
# D6: Ready Output
# D9: ADS Input
# D10: DEN Input
# D11: In Transaction
# D12: In Address State
# D13: In Idle State
# A0:  System Warm Up

# Since CircuitPython is very slow to boot we need to denote that it is up and
# running!
systemWarmUp.value = False
while True:
    time.sleep(1)
