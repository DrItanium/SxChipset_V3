This repo contains the code for the second version (really third or fourth
generation) of the i960Sx Chipset. 



# Initial Setup

After cloning the repo do the following from the root of the repo

1. ` python -m venv . `
2. ` . bin/activate `
3. ` pip install --upgrade platformio `

After this point the initial setup is done. After this all you need to do on a
new terminal is run

` . bin/activate `



# Projects

- ManagementEngine: Handles a bunch of the timing related aspects when interfacing with the i960Sx
- Chipset: Primary transaction handler
