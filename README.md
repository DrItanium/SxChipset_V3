This repo contains the code for the second version (really third or fourth
generation) of the i960Sx Chipset. 



# Initial Setup

After cloning the repo do the following from the root of the repo

1. ` python -m venv .venv `
2. ` . .venv/bin/activate `
3. ` pip install --upgrade platformio `

After this point the initial setup is done. After this all you need to do on a
new terminal is run

` . .venv/bin/activate `



# Projects
- Chipset16: Primary transaction handler that uses a far simpler 16-bit data bus (teensy uses this code)
- ManagementEngine: Handles a bunch of the timing related aspects when interfacing with the i960Sx
- PLD: Various PLD programs that I have written over the lifetime of this iteration of the chipset and previous ones. I use a tool called galette to compile them. The chips I use both the 5V (ATF22V10) and 3.3v (ATF22LV10) versions of the GAL22V10 plds. 
- Chipset: Primary transaction handler (uses an 8-bit data bus and is _deprecated_)
- research: contains code and sketches for different ideas I've had throughout the lifetime of this project
  - Fun fact: the code.py was actually used in my design since PIO programs are not affected by circuitpython slowness (I use FlexIO now)

# Goal

The goal of this project is to use a Teensy 4.1 and AVR128DB64 (running at 24MHz @ 3.3v) plus some other chips to bring an 80960SB-16/10 (running at 12MHz) to life.
This is a constantly evolving thing full of research code and advanced uses of the C++ programming language (like concepts in the Chipset project). 


