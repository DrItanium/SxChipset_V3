This chip is an AVR128DB64 which does the following:

- Generate 24/12/6 MHz clock signals with the i960 getting a 12/6mhz combo by default
- Use handle the synchronization of the ready signal and pulse generation plus feedback
- Provide an extra set of GPIOs in the style of a seesaw (but much more powerful) for whatever is needed
- Use the MVIO for automatic voltage translation as needed (if needed...)
- Allow for a second "sensor" chip if desired


The original management engine abstracted the entire memory transaction process
of the i960 from much faster chips. This new design handles the aspects that
would require external circuitry which may be too inflexible. 

I do a similar thing with the 2560/4808 combo for the second generation i960
chipset. 

